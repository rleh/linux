// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX586 cameras.
 * Copyright (C) 2020, Raphael Lehmann
 *
 * Based on Sony IMX586 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx258 camera driver
 * Copyright (C) 2018 Intel Corporation
 *
 * DT / fwnode changes, and regulator / GPIO control taken from ov5640.c
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

// Fixes IDE fuckup:
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


/* Exposure control */
//#define IMX586_REG_EXPOSURE		0x015a
#define IMX586_EXPOSURE_MIN		4
#define IMX586_EXPOSURE_STEP		1
#define IMX586_EXPOSURE_DEFAULT		0x640
#define IMX586_EXPOSURE_MAX		65535

/* Analog gain control */
//#define IMX586_REG_ANALOG_GAIN		0x0157
#define IMX586_ANA_GAIN_MIN		0
#define IMX586_ANA_GAIN_MAX		232
#define IMX586_ANA_GAIN_STEP		1
#define IMX586_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
//#define IMX586_REG_DIGITAL_GAIN		0x0158
#define IMX586_DGTL_GAIN_MIN		0x0100
#define IMX586_DGTL_GAIN_MAX		0x0fff
#define IMX586_DGTL_GAIN_DEFAULT	0x0100
#define IMX586_DGTL_GAIN_STEP		1


struct imx586_reg {
	u16 address;
	u8 val;
};

struct imx586_reg_list {
	u32 num_of_regs;
	const struct imx586_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx586_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;

	/* V-timing */
	u32 vts_def;

	/* Default register values */
	struct imx586_reg_list reg_list;
};

/*
 * Register sets lifted off the i2C interface from the Raspberry Pi firmware
 * driver.
 */
static const struct imx586_reg mode_3280x2464_regs[] = {
	{0x0100, 0x00},
	// ...
};
// ...

/* Mode configs */
static const struct imx586_mode supported_modes[] = {
	{
		/* 8MPix 15fps mode */
		.width = 8000,
		.height = 6000,
		.vts_def = 0, // ...
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3280x2464_regs),
			.regs = mode_3280x2464_regs,
		},
	},
};

struct imx586 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to IMX586 */
	u32 xclk_freq;

	struct gpio_desc *xclr_gpio;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;

	/* Current mode */
	const struct imx586_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	int power_count;
	/* Streaming on/off */
	bool streaming;
};

static inline struct imx586 *to_imx586(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx586, sd);
}

/* Read registers up to 2 at a time */
static int imx586_read_reg(struct imx586 *imx586, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx586_write_reg(struct imx586 *imx586, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx586_write_regs(struct imx586 *imx586,
			     const struct imx586_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx586_write_reg(imx586, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Power/clock management functions */
static void imx586_power(struct imx586 *imx586, bool enable)
{
	gpiod_set_value_cansleep(imx586->xclr_gpio, enable ? 1 : 0);
}

static int imx586_set_power_on(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	int ret;

	ret = clk_prepare_enable(imx586->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	// ...

	imx586_power(imx586, true);
	//msleep(IMX586_XCLR_DELAY_MS);

	return 0;
}

static void imx586_set_power_off(struct imx586 *imx586)
{
	imx586_power(imx586, false);
}

static int imx586_set_power(struct imx586 *imx586, bool on)
{
	int ret = 0;

	if (on) {
		ret = imx586_set_power_on(imx586);
		if (ret)
			return ret;
	} else {
		imx586_set_power_off(imx586);
	}

	return 0;
}

/* Open sub-device */
static int imx586_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx586 *imx586 = to_imx586(sd);
	int ret = 0;

	mutex_lock(&imx586->mutex);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (imx586->power_count == !on) {
		ret = imx586_set_power(imx586, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	imx586->power_count += on ? 1 : -1;
	WARN_ON(imx586->power_count < 0);
out:
	mutex_unlock(&imx586->mutex);

	return ret;
}

static int imx586_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);

	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int imx586_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx586 *imx586 =
		container_of(ctrl->handler, struct imx586, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	int ret = 0;

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		// ...
		break;
	case V4L2_CID_EXPOSURE:
		// ...
		break;
	case V4L2_CID_DIGITAL_GAIN:
		// ...
		break;
	case V4L2_CID_TEST_PATTERN:
		// ...
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx586_ctrl_ops = {
	.s_ctrl = imx586_set_ctrl,
};

static int imx586_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	/* Only one bayer order(GRBG) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int imx586_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx586_update_pad_format(const struct imx586_mode *mode,
				     struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __imx586_get_pad_format(struct imx586 *imx586,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_get_try_format(&imx586->sd, cfg,
							  fmt->pad);
	else
		imx586_update_pad_format(imx586->mode, fmt);

	return 0;
}

static int imx586_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx586 *imx586 = to_imx586(sd);
	int ret;

	mutex_lock(&imx586->mutex);
	ret = __imx586_get_pad_format(imx586, cfg, fmt);
	mutex_unlock(&imx586->mutex);

	return ret;
}

static int imx586_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx586 *imx586 = to_imx586(sd);
	const struct imx586_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;

	mutex_lock(&imx586->mutex);

	/* Only one raw bayer(BGGR) order is supported */
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	imx586_update_pad_format(mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*framefmt = fmt->format;
	} else {
		imx586->mode = mode;
	}

	mutex_unlock(&imx586->mutex);

	return 0;
}

/* Start streaming */
static int imx586_start_streaming(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	const struct imx586_reg_list *reg_list;

	// ...
}

/* Stop streaming */
static int imx586_stop_streaming(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);

	// ...
}

static int imx586_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx586 *imx586 = to_imx586(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx586->mutex);
	if (imx586->streaming == enable) {
		mutex_unlock(&imx586->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx586_start_streaming(imx586);
		if (ret) {
			pm_runtime_put(&client->dev);
			goto err_unlock;
		}
	} else {
		imx586_stop_streaming(imx586);
		pm_runtime_put(&client->dev);
	}

	imx586->streaming = enable;
	mutex_unlock(&imx586->mutex);

	return ret;

err_unlock:
	mutex_unlock(&imx586->mutex);

	return ret;
}

static int __maybe_unused imx586_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);

	if (imx586->streaming)
		imx586_stop_streaming(imx586);

	return 0;
}

static int __maybe_unused imx586_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);
	int ret;

	if (imx586->streaming) {
		ret = imx586_start_streaming(imx586);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx586_stop_streaming(imx586);
	imx586->streaming = 0;
	return ret;
}


/* Verify chip ID */
static int imx586_identify_module(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	
	// ...
	return 0;
}

static const struct v4l2_subdev_core_ops imx586_core_ops = {
	.s_power = imx586_s_power,
};

static const struct v4l2_subdev_video_ops imx586_video_ops = {
	.s_stream = imx586_set_stream,
};

static const struct v4l2_subdev_pad_ops imx586_pad_ops = {
	.enum_mbus_code = imx586_enum_mbus_code,
	.get_fmt = imx586_get_pad_format,
	.set_fmt = imx586_set_pad_format,
	.enum_frame_size = imx586_enum_frame_size,
};

static const struct v4l2_subdev_ops imx586_subdev_ops = {
	.core = &imx586_core_ops,
	.video = &imx586_video_ops,
	.pad = &imx586_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx586_internal_ops = {
	.open = imx586_open,
};

/* Initialize control handlers */
static int imx586_init_controls(struct imx586 *imx586)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx586->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &imx586->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	mutex_init(&imx586->mutex);
	ctrl_hdlr->lock = &imx586->mutex;

	imx586->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX586_EXPOSURE_MIN,
					     IMX586_EXPOSURE_MAX,
					     IMX586_EXPOSURE_STEP,
					     IMX586_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX586_ANA_GAIN_MIN, IMX586_ANA_GAIN_MAX,
			  IMX586_ANA_GAIN_STEP, IMX586_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx586_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX586_DGTL_GAIN_MIN, IMX586_DGTL_GAIN_MAX,
			  IMX586_DGTL_GAIN_STEP, IMX586_DGTL_GAIN_DEFAULT);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	imx586->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx586->mutex);

	return ret;
}

static void imx586_free_controls(struct imx586 *imx586)
{
	v4l2_ctrl_handler_free(imx586->sd.ctrl_handler);
	mutex_destroy(&imx586->mutex);
}

static int imx586_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct imx586 *imx586;
	int ret;

	imx586 = devm_kzalloc(&client->dev, sizeof(*imx586), GFP_KERNEL);
	if (!imx586)
		return -ENOMEM;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx586->sd, client, &imx586_subdev_ops);

	/* Get CSI2 bus config */
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &imx586->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	/* Get system clock (xclk) */
	imx586->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx586->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx586->xclk);
	}

	imx586->xclk_freq = clk_get_rate(imx586->xclk);
	if (imx586->xclk_freq != 24000000) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx586->xclk_freq);
		return -EINVAL;
	}

	/* request optional power down pin */
	imx586->xclr_gpio = devm_gpiod_get_optional(dev, "xclr",
						    GPIOD_OUT_HIGH);

	/* Check module identity */
	ret = imx586_identify_module(imx586);
	if (ret)
		return ret;

	/* Set default mode to max resolution */
	imx586->mode = &supported_modes[0];

	ret = imx586_init_controls(imx586);
	if (ret)
		return ret;

	/* Initialize subdev */
	imx586->sd.internal_ops = &imx586_internal_ops;
	imx586->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx586->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx586->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx586->sd.entity, 1, &imx586->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor_common(&imx586->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx586->sd.entity);

error_handler_free:
	imx586_free_controls(imx586);

	return ret;
}

static int imx586_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx586 *imx586 = to_imx586(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx586_free_controls(imx586);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id imx586_dt_ids[] = {
	{ .compatible = "sony,imx586" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx586_dt_ids);

static struct i2c_driver imx586_i2c_driver = {
	.driver = {
		.name = "imx586",
		.of_match_table	= imx586_dt_ids,
	},
	.probe = imx586_probe,
	.remove = imx586_remove,
};

module_i2c_driver(imx586_i2c_driver);

MODULE_AUTHOR("Raphael Lehmann <raphael+kernel@rleh.de");
MODULE_DESCRIPTION("Sony IMX586 sensor driver");
MODULE_LICENSE("GPL v2");
