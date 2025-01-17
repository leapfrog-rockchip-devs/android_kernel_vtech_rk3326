// SPDX-License-Identifier: GPL-2.0
/*
 * gc02m2b driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define gc02m2b_LANES			1
#define gc02m2b_BITS_PER_SAMPLE		10
#define gc02m2b_LINK_FREQ_MHZ	336000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define gc02m2b_PIXEL_RATE		(gc02m2b_LINK_FREQ_MHZ * 2 * 1 / 10)
#define gc02m2b_XVCLK_FREQ		24000000

#define CHIP_ID				0x02f0
#define gc02m2b_REG_CHIP_ID_H		0xf0
#define gc02m2b_REG_CHIP_ID_L		0xf1

#define gc02m2b_REG_SET_PAGE		0xfe
#define gc02m2b_SET_PAGE_ONE		0x00

#define gc02m2b_REG_CTRL_MODE		0x3e
#define gc02m2b_MODE_SW_STANDBY		0x00
#define gc02m2b_MODE_STREAMING		0x90

#define gc02m2b_REG_EXPOSURE_H		0x03
#define gc02m2b_REG_EXPOSURE_L		0x04
#define gc02m2b_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0x3F)	/* 6 Bits */
#define gc02m2b_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 8 Bits */
#define	gc02m2b_EXPOSURE_MIN		4
#define	gc02m2b_EXPOSURE_STEP		1
#define gc02m2b_VTS_MAX			0x3fff

#define gc02m2b_REG_AGAIN		0xb6
#define gc02m2b_REG_DGAIN_INT		0xb1
#define gc02m2b_REG_DGAIN_FRAC		0xb2
#define gc02m2b_GAIN_MIN		0x400
#define gc02m2b_GAIN_MAX		(12 * gc02m2b_GAIN_MIN)
#define gc02m2b_GAIN_STEP		1
#define gc02m2b_GAIN_DEFAULT		0x400
#define gc02m2b_SENSOR_DGAIN_BASE	0x400

#define gc02m2b_REG_VTS_H			0x41
#define gc02m2b_REG_VTS_L			0x42

#define REG_NULL			0xFF

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define gc02m2b_NAME			"gc02m2b"
#define gc02m2b_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SRGGB10_1X10

/* SENSOR MIRROR FLIP INFO */
#define gc02m2b_MIRROR_NORMAL    1
#define gc02m2b_MIRROR_H         0
#define gc02m2b_MIRROR_V         0
#define gc02m2b_MIRROR_HV        0

#if gc02m2b_MIRROR_NORMAL
#define gc02m2b_MIRROR	        0x80
#elif gc02m2b_MIRROR_H
#define gc02m2b_MIRROR	        0x81
#elif gc02m2b_MIRROR_V
#define gc02m2b_MIRROR	        0x82
#elif gc02m2b_MIRROR_HV
#define gc02m2b_MIRROR	        0x83
#else
#define gc02m2b_MIRROR	        0x80
#endif

static const char * const gc02m2b_supply_names[] = {
	"dvdd",		/* Digital core power 1.2*/
	"dovdd",	/* Digital I/O power 1.8*/
	"avdd",		/* Analog power 2.8*/
};

#define gc02m2b_NUM_SUPPLIES ARRAY_SIZE(gc02m2b_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct gc02m2b_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct gc02m2b {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[gc02m2b_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct gc02m2b_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_gc02m2b(sd) container_of(sd, struct gc02m2b, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval gc02m2b_global_regs[] = {
	/*system*/
	{0xfc, 0x01},
	{0xf4, 0x41},
	{0xf5, 0xc0},
	{0xf6, 0x44},
	{0xf8, 0x38},
	{0xf9, 0x82},
	{0xfa, 0x00},
	{0xfd, 0x80},
	{0xfc, 0x81},
	{0xfe, 0x03},
	{0x01, 0x0b},
	{0xf7, 0x01},
	{0xfc, 0x80},
	{0xfc, 0x80},
	{0xfc, 0x80},
	{0xfc, 0x8e},

	/*CISCTL*/
	{0xfe, 0x00},
	{0x87, 0x09},
	{0xee, 0x72},
	{0xfe, 0x01},
	{0x8c, 0x90},
	{0xfe, 0x00},
	{0x90, 0x00},
	{0x03, 0x04},
	{0x04, 0x7d},
	{0x41, 0x04},
	{0x42, 0xf4},
	{0x05, 0x04},
	{0x06, 0x48},
	{0x07, 0x00},
	{0x08, 0x18},
	{0x9d, 0x18},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0d, 0x04},
	{0x0e, 0xbc},
	{0x17, gc02m2b_MIRROR},
	{0x19, 0x04},
	{0x24, 0x00},
	{0x56, 0x20},
	{0x5b, 0x00},
	{0x5e, 0x01},

	/*analog Register width*/
	{0x21, 0x3c},
	{0x44, 0x20},
	{0xcc, 0x01},

	/*analog mode*/
	{0x1a, 0x04},
	{0x1f, 0x11},
	{0x27, 0x30},
	{0x2b, 0x00},
	{0x33, 0x00},
	{0x53, 0x90},
	{0xe6, 0x50},

	/*analog voltage*/
	{0x39, 0x07},
	{0x43, 0x04},
	{0x46, 0x2a},
	{0x7c, 0xa0},
	{0xd0, 0xbe},
	{0xd1, 0x40},
	{0xd2, 0x40},
	{0xd3, 0xb3},
	{0xde, 0x1c},

	/*analog current*/
	{0xcd, 0x06},
	{0xce, 0x6f},

	/*CISCTL RESET*/
	{0xfc, 0x88},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfc, 0x8e},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfe, 0x00},
	{0xfc, 0x88},
	{0xfe, 0x10},
	{0xfe, 0x00},
	{0xfc, 0x8e},
	{0xfe, 0x04},
	{0xe0, 0x01},
	{0xfe, 0x00},

	/*ISP*/
	{0xfe, 0x01},
	{0x53, 0x54},
	{0x87, 0x53},
	{0x89, 0x03},

	/*Gain*/
	{0xfe, 0x00},
	{0xb0, 0x74},
	{0xb1, 0x04},
	{0xb2, 0x00},
	{0xb6, 0x00},
	{0xfe, 0x04},
	{0xd8, 0x00},

	{0xc0, 0x40},  //1x
	{0xc0, 0x00},
	{0xc0, 0x00},
	{0xc0, 0x00},

	{0xc0, 0x60},  //1.5x
	{0xc0, 0x00},
	{0xc0, 0xc0},
	{0xc0, 0x2a},

	{0xc0, 0x80},  //2x
	{0xc0, 0x00},
	{0xc0, 0x00},
	{0xc0, 0x40},

	{0xc0, 0xa0},  //2.5x
	{0xc0, 0x00},
	{0xc0, 0x90},
	{0xc0, 0x19},

	{0xc0, 0xc0},  //3x
	{0xc0, 0x00},
	{0xc0, 0xD0},
	{0xc0, 0x2F},

	{0xc0, 0xe0},  //3.5x
	{0xc0, 0x00},
	{0xc0, 0x90},
	{0xc0, 0x39},

	{0xc0, 0x00},  //4x
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x04},

	{0xc0, 0x20},  //4.5x
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x0f},

	{0xc0, 0x40},  //5x
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x1a},

	{0xc0, 0x60},  //5.5x
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x25},

	{0xc0, 0x80},  //6x
	{0xc0, 0x01},
	{0xc0, 0xa0},
	{0xc0, 0x2c},

	{0xc0, 0xa0},  //6.5x
	{0xc0, 0x01},
	{0xc0, 0xe0},
	{0xc0, 0x32},

	{0xc0, 0xc0},  //7x
	{0xc0, 0x01},
	{0xc0, 0x20},
	{0xc0, 0x38},

	{0xc0, 0xe0},  //7.5x
	{0xc0, 0x01},
	{0xc0, 0x60},
	{0xc0, 0x3c},

	{0xc0, 0x00},  //8x
	{0xc0, 0x02},
	{0xc0, 0xa0},
	{0xc0, 0x40},

	{0xc0, 0x80},  //10x
	{0xc0, 0x02},
	{0xc0, 0x18},
	{0xc0, 0x5c},

	{0xfe, 0x00},
	{0x9f, 0x10},

	/*BLK*/
	{0xfe, 0x00},
	{0x26, 0x20},
	{0xfe, 0x01},
	{0x40, 0x22},
	{0x46, 0x7f},
	{0x49, 0x0f},
	{0x4a, 0xf0},
	{0xfe, 0x04},
	{0x14, 0x80},
	{0x15, 0x80},
	{0x16, 0x80},
	{0x17, 0x80},

	/*anti_blooming*/
	{0xfe, 0x01},
	{0x41, 0x20},
	{0x4c, 0x00},
	{0x4d, 0x0c},
	{0x44, 0x08},
	{0x48, 0x03},

	/*Window 1600X1200*/
	{0xfe, 0x01},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x06},
	{0x93, 0x00},
	{0x94, 0x06},
	{0x95, 0x04},
	{0x96, 0xb0},
	{0x97, 0x06},
	{0x98, 0x40},

	/*mipi*/
	{0xfe, 0x03},
	{0x01, 0x23},
	{0x03, 0xce},
	{0x04, 0x48},
	{0x15, 0x00},
	{0x21, 0x10},
	{0x22, 0x05},
	{0x23, 0x20},
	{0x25, 0x20},
	{0x26, 0x08},
	{0x29, 0x06},
	{0x2a, 0x0a},
	{0x2b, 0x08},

	/*out*/
	{0xfe, 0x01},
	{0x8c, 0x10},
	{0xfe, 0x00},
	{0x3e, 0x00},

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 656Mbps
 */
static const struct regval gc02m2b_1600x1200_regs[] = {
	{REG_NULL, 0x00},
};

static const struct gc02m2b_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0480,
		.hts_def = 0x0890,
		.vts_def = 0x04fc,
		.reg_list = gc02m2b_1600x1200_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	gc02m2b_LINK_FREQ_MHZ
};

/* Write registers up to 4 at a time */
static int gc02m2b_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"gc02m2b write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int gc02m2b_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc02m2b_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int gc02m2b_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev,
		"gc02m2b read reg:0x%x failed !\n", reg);

	return ret;
}

static int gc02m2b_get_reso_dist(const struct gc02m2b_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gc02m2b_mode *
gc02m2b_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc02m2b_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc02m2b_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	const struct gc02m2b_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc02m2b->mutex);

	mode = gc02m2b_find_best_fit(fmt);
	fmt->format.code = gc02m2b_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc02m2b->mutex);
		return -ENOTTY;
#endif
	} else {
		gc02m2b->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc02m2b->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc02m2b->vblank, vblank_def,
					 gc02m2b_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc02m2b->mutex);

	return 0;
}

static int gc02m2b_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	const struct gc02m2b_mode *mode = gc02m2b->cur_mode;

	mutex_lock(&gc02m2b->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc02m2b->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = gc02m2b_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc02m2b->mutex);

	return 0;
}

static int gc02m2b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = gc02m2b_MEDIA_BUS_FMT;

	return 0;
}

static int gc02m2b_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != gc02m2b_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int gc02m2b_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	const struct gc02m2b_mode *mode = gc02m2b->cur_mode;

	mutex_lock(&gc02m2b->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc02m2b->mutex);

	return 0;
}

static void gc02m2b_get_module_inf(struct gc02m2b *gc02m2b,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, gc02m2b_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gc02m2b->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gc02m2b->len_name, sizeof(inf->base.lens));
}

static long gc02m2b_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc02m2b_get_module_inf(gc02m2b, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc02m2b_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc02m2b_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = gc02m2b_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __gc02m2b_start_stream(struct gc02m2b *gc02m2b)
{
	int ret;

	ret = gc02m2b_write_array(gc02m2b->client, gc02m2b->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc02m2b->mutex);
	ret = v4l2_ctrl_handler_setup(&gc02m2b->ctrl_handler);
	mutex_lock(&gc02m2b->mutex);
	if (ret)
		return ret;
	ret = gc02m2b_write_reg(gc02m2b->client,
				 gc02m2b_REG_SET_PAGE,
				 gc02m2b_SET_PAGE_ONE);
	ret |= gc02m2b_write_reg(gc02m2b->client,
				 gc02m2b_REG_CTRL_MODE,
				 gc02m2b_MODE_STREAMING);
	return ret;
}

static int __gc02m2b_stop_stream(struct gc02m2b *gc02m2b)
{
	int ret;

	ret = gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_SET_PAGE, gc02m2b_SET_PAGE_ONE);
	ret |= gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_CTRL_MODE, gc02m2b_MODE_SW_STANDBY);
	return ret;
}

static int gc02m2b_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	struct i2c_client *client = gc02m2b->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				gc02m2b->cur_mode->width,
				gc02m2b->cur_mode->height,
				gc02m2b->cur_mode->max_fps.denominator);
	mutex_lock(&gc02m2b->mutex);
	on = !!on;
	if (on == gc02m2b->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc02m2b_start_stream(gc02m2b);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc02m2b_stop_stream(gc02m2b);
		pm_runtime_put(&client->dev);
	}

	gc02m2b->streaming = on;

unlock_and_return:
	mutex_unlock(&gc02m2b->mutex);

	return ret;
}

static int gc02m2b_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	struct i2c_client *client = gc02m2b->client;
	int ret = 0;

	mutex_lock(&gc02m2b->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc02m2b->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = gc02m2b_write_array(gc02m2b->client, gc02m2b_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc02m2b->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc02m2b->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc02m2b->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc02m2b_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, gc02m2b_XVCLK_FREQ / 1000 / 1000);
}

static int __gc02m2b_power_on(struct gc02m2b *gc02m2b)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc02m2b->client->dev;

	if (!IS_ERR_OR_NULL(gc02m2b->pins_default)) {
		ret = pinctrl_select_state(gc02m2b->pinctrl,
					   gc02m2b->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(gc02m2b->xvclk, gc02m2b_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc02m2b->xvclk) != gc02m2b_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc02m2b->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(gc02m2b->reset_gpio))
		gpiod_set_value_cansleep(gc02m2b->reset_gpio, 1);

	ret = regulator_bulk_enable(gc02m2b_NUM_SUPPLIES, gc02m2b->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(1000, 1100);
	if (!IS_ERR(gc02m2b->reset_gpio))
		gpiod_set_value_cansleep(gc02m2b->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(gc02m2b->pwdn_gpio))
		gpiod_set_value_cansleep(gc02m2b->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc02m2b_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(gc02m2b->xvclk);

	return ret;
}

static void __gc02m2b_power_off(struct gc02m2b *gc02m2b)
{
	int ret = 0;

	if (!IS_ERR(gc02m2b->pwdn_gpio))
		gpiod_set_value_cansleep(gc02m2b->pwdn_gpio, 0);
	clk_disable_unprepare(gc02m2b->xvclk);
	if (!IS_ERR(gc02m2b->reset_gpio))
		gpiod_set_value_cansleep(gc02m2b->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(gc02m2b->pins_sleep)) {
		ret = pinctrl_select_state(gc02m2b->pinctrl,
					   gc02m2b->pins_sleep);
		if (ret < 0)
			dev_dbg(&gc02m2b->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(gc02m2b_NUM_SUPPLIES, gc02m2b->supplies);
}

static int gc02m2b_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);

	return __gc02m2b_power_on(gc02m2b);
}

static int gc02m2b_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);

	__gc02m2b_power_off(gc02m2b);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc02m2b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc02m2b_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc02m2b->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = gc02m2b_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc02m2b->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int gc02m2b_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != gc02m2b_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops gc02m2b_pm_ops = {
	SET_RUNTIME_PM_OPS(gc02m2b_runtime_suspend,
			   gc02m2b_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc02m2b_internal_ops = {
	.open = gc02m2b_open,
};
#endif

static const struct v4l2_subdev_core_ops gc02m2b_core_ops = {
	.s_power = gc02m2b_s_power,
	.ioctl = gc02m2b_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc02m2b_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc02m2b_video_ops = {
	.s_stream = gc02m2b_s_stream,
	.g_frame_interval = gc02m2b_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc02m2b_pad_ops = {
	.enum_mbus_code = gc02m2b_enum_mbus_code,
	.enum_frame_size = gc02m2b_enum_frame_sizes,
	.enum_frame_interval = gc02m2b_enum_frame_interval,
	.get_fmt = gc02m2b_get_fmt,
	.set_fmt = gc02m2b_set_fmt,
};

static const struct v4l2_subdev_ops gc02m2b_subdev_ops = {
	.core	= &gc02m2b_core_ops,
	.video	= &gc02m2b_video_ops,
	.pad	= &gc02m2b_pad_ops,
};

static u32 gc02m2b_AGC_Param[16][2] = {
	{1024,  0},
	{1536,  1},
	{2035,  2},
	{2519,  3},
	{3165,  4},
	{3626,  5},
	{4147,  6},
	{4593,  7},
	{5095,  8},
	{5697,  9},
	{6270, 10},
	{6714, 11},
	{7210, 12},
	{7686, 13},
	{8214, 14},
	{10337, 15},
};

static int gc02m2b_set_gain_reg(struct gc02m2b *gc02m2b, u32 a_gain)
{
	struct device *dev = &gc02m2b->client->dev;
	int ret = 0, gain_index = 0;
	u32 temp_gain = 0;

	dev_info(dev, "%s(%d) a_gain(0x%08x)!\n", __func__, __LINE__, a_gain);
	if (a_gain < 0x400)
		a_gain = 0x400;
	else if (a_gain > 0x3000)
		a_gain = 0x3000;
	for (gain_index = 15; gain_index >= 0; gain_index--) {
		if (a_gain >= gc02m2b_AGC_Param[gain_index][0])
			break;
	}

	ret = gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_SET_PAGE,
		gc02m2b_SET_PAGE_ONE);
	ret |= gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_AGAIN, gc02m2b_AGC_Param[gain_index][1]);
	temp_gain = a_gain * gc02m2b_SENSOR_DGAIN_BASE /
			    gc02m2b_AGC_Param[gain_index][0];

	dev_info(dev, "AGC_Param[%d][0](%d) temp_gain is(0x%08x)!\n",
				gain_index, gc02m2b_AGC_Param[gain_index][0], temp_gain);
	ret |= gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_DGAIN_INT,
		(temp_gain >> 8) & 0x1f);
	ret |= gc02m2b_write_reg(gc02m2b->client,
		gc02m2b_REG_DGAIN_FRAC,
		temp_gain & 0xff);
	return ret;
}

static int gc02m2b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m2b *gc02m2b = container_of(ctrl->handler,
					     struct gc02m2b, ctrl_handler);
	struct i2c_client *client = gc02m2b->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc02m2b->cur_mode->height + ctrl->val - 16;
		__v4l2_ctrl_modify_range(gc02m2b->exposure,
					 gc02m2b->exposure->minimum, max,
					 gc02m2b->exposure->step,
					 gc02m2b->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_SET_PAGE,
					gc02m2b_SET_PAGE_ONE);
		ret |= gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_EXPOSURE_H,
					gc02m2b_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_EXPOSURE_L,
					gc02m2b_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc02m2b_set_gain_reg(gc02m2b, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_SET_PAGE,
					gc02m2b_SET_PAGE_ONE);
		ret |= gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_VTS_H,
					((gc02m2b->cur_mode->height + ctrl->val) >> 8) & 0x3f);
		ret |= gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_VTS_L,
					(gc02m2b->cur_mode->height + ctrl->val) & 0xff);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc02m2b_ctrl_ops = {
	.s_ctrl = gc02m2b_set_ctrl,
};

static int gc02m2b_initialize_controls(struct gc02m2b *gc02m2b)
{
	const struct gc02m2b_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc02m2b->ctrl_handler;
	mode = gc02m2b->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc02m2b->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, gc02m2b_PIXEL_RATE, 1, gc02m2b_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	gc02m2b->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc02m2b->hblank)
		gc02m2b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc02m2b->vblank = v4l2_ctrl_new_std(handler, &gc02m2b_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				gc02m2b_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc02m2b->exposure = v4l2_ctrl_new_std(handler, &gc02m2b_ctrl_ops,
				V4L2_CID_EXPOSURE, gc02m2b_EXPOSURE_MIN,
				exposure_max, gc02m2b_EXPOSURE_STEP,
				mode->exp_def);

	gc02m2b->anal_gain = v4l2_ctrl_new_std(handler, &gc02m2b_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, gc02m2b_GAIN_MIN,
				gc02m2b_GAIN_MAX, gc02m2b_GAIN_STEP,
				gc02m2b_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&gc02m2b->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc02m2b->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc02m2b_check_sensor_id(struct gc02m2b *gc02m2b,
				   struct i2c_client *client)
{
	struct device *dev = &gc02m2b->client->dev;
	u16 id = 0;
	u8 reg_H = 0;
	u8 reg_L = 0;
	int ret;

	ret = gc02m2b_write_reg(gc02m2b->client,
					gc02m2b_REG_SET_PAGE,
					gc02m2b_SET_PAGE_ONE);
	ret |= gc02m2b_read_reg(client, gc02m2b_REG_CHIP_ID_H, &reg_H);
	ret |= gc02m2b_read_reg(client, gc02m2b_REG_CHIP_ID_L, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	
	dev_info(dev, "detected gc%04x sensor\n", id);
	return ret;
}

static int gc02m2b_configure_regulators(struct gc02m2b *gc02m2b)
{
	unsigned int i;

	for (i = 0; i < gc02m2b_NUM_SUPPLIES; i++)
		gc02m2b->supplies[i].supply = gc02m2b_supply_names[i];

	return devm_regulator_bulk_get(&gc02m2b->client->dev,
				       gc02m2b_NUM_SUPPLIES,
				       gc02m2b->supplies);
}

static int gc02m2b_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc02m2b *gc02m2b;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc02m2b = devm_kzalloc(dev, sizeof(*gc02m2b), GFP_KERNEL);
	if (!gc02m2b)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc02m2b->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gc02m2b->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gc02m2b->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gc02m2b->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	gc02m2b->client = client;
	gc02m2b->cur_mode = &supported_modes[0];

	gc02m2b->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc02m2b->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc02m2b->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc02m2b->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc02m2b->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc02m2b->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = gc02m2b_configure_regulators(gc02m2b);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	gc02m2b->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc02m2b->pinctrl)) {
		gc02m2b->pins_default =
			pinctrl_lookup_state(gc02m2b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc02m2b->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc02m2b->pins_sleep =
			pinctrl_lookup_state(gc02m2b->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc02m2b->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&gc02m2b->mutex);

	sd = &gc02m2b->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc02m2b_subdev_ops);
	ret = gc02m2b_initialize_controls(gc02m2b);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc02m2b_power_on(gc02m2b);
	if (ret)
		goto err_free_handler;

	ret = gc02m2b_check_sensor_id(gc02m2b, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc02m2b_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc02m2b->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc02m2b->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc02m2b->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc02m2b->module_index, facing,
		 gc02m2b_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
  mdelay(25);
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__gc02m2b_power_off(gc02m2b);

err_free_handler:
	v4l2_ctrl_handler_free(&gc02m2b->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc02m2b->mutex);
		mdelay(25);
	return ret;
}

static int gc02m2b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m2b *gc02m2b = to_gc02m2b(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc02m2b->ctrl_handler);
	mutex_destroy(&gc02m2b->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc02m2b_power_off(gc02m2b);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc02m2b_of_match[] = {
	{ .compatible = "galaxycore,gc02m2b" },
	{},
};
MODULE_DEVICE_TABLE(of, gc02m2b_of_match);
#endif

static const struct i2c_device_id gc02m2b_match_id[] = {
	{ "galaxycore,gc02m2b", 0 },
	{ },
};

static struct i2c_driver gc02m2b_i2c_driver = {
	.driver = {
		.name = gc02m2b_NAME,
		.pm = &gc02m2b_pm_ops,
		.of_match_table = of_match_ptr(gc02m2b_of_match),
	},
	.probe		= &gc02m2b_probe,
	.remove		= &gc02m2b_remove,
	.id_table	= gc02m2b_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc02m2b_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc02m2b_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GalaxyCore gc02m2b sensor driver");
MODULE_LICENSE("GPL v2");
