// SPDX-License-Identifier: GPL-2.0
/*
 * s5k5e9 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 add enum_frame_interval function.
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
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* 45Mhz * 4 Binning */
#define S5K5E9_PIXEL_RATE		(190 * 1000 * 1000)
#define S5K5E9_XVCLK_FREQ		24000000

#define CHIP_ID				0x559B
#define S5K5E9_REG_CHIP_ID		0x0000

#define S5K5E9_REG_CTRL_MODE		0x0100
#define S5K5E9_MODE_SW_STANDBY		0x0
#define S5K5E9_MODE_STREAMING		BIT(0)

#define S5K5E9_REG_ORIGENTATION 0x0101
#define S5K5E9_FILP  0x02
#define S5K5E9_MIRROR 0x01

#define S5K5E9_REG_Reset 0x0103
#define S5K5E9_reset  0x01


#define S5K5E9_REG_EXPOSURE		0x0202
#define	S5K5E9_EXPOSURE_MIN		4
#define	S5K5E9_EXPOSURE_STEP		1
#define S5K5E9_VTS_MAX			0x7fff

#define S5K5E9_REG_ANALOG_GAIN		0x0204
#define	ANALOG_GAIN_MIN			0x20
#define	ANALOG_GAIN_MAX			0x200
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x20

#define S5K5E9_REG_DIGI_GAIN_H		0x350a
#define S5K5E9_REG_DIGI_GAIN_L		0x350b
#define S5K5E9_DIGI_GAIN_L_MASK		0x3f
#define S5K5E9_DIGI_GAIN_H_SHIFT	6
#define S5K5E9_DIGI_GAIN_MIN		0
#define S5K5E9_DIGI_GAIN_MAX		(0x4000 - 1)
#define S5K5E9_DIGI_GAIN_STEP		1
#define S5K5E9_DIGI_GAIN_DEFAULT	1024

#define S5K5E9_REG_TEST_PATTERN		0x0600
#define	S5K5E9_TEST_PATTERN_ENABLE	0x02
#define	S5K5E9_TEST_PATTERN_DISABLE	0x00

#define S5K5E9_REG_VTS			0x0340

#define REG_NULL			0xFFFF

#define S5K5E9_REG_VALUE_08BIT		1
#define S5K5E9_REG_VALUE_16BIT		2
#define S5K5E9_REG_VALUE_24BIT		3

#define S5K5E9_LANES			2
#define S5K5E9_BITS_PER_SAMPLE		10

#define I2C_M_WR			0
#define I2C_MSG_MAX			300
#define I2C_DATA_MAX			(I2C_MSG_MAX * 3)

#define S5K5E9_NAME			"s5k5e9"
struct i2c_client *s5k5e9_client=NULL; 
 extern int front_back_switch1_level;
extern int front_back_switch1_flag;
int s5k5e9power_on=0;
int s5k5e9power_on_flag=0;
int s5k5e9power_on_flag1=0;
int Check_micro_distance=0;
static const char * const s5k5e9_supply_names[] = {
	//"vcc1v5_dvp",	/* Digital core power */
	//"vdd1v8_dvp",	/* Digital I/O power */
	//"vcc2v8_dvp",	/* Analog power */
	"dvdd",		/* Digital core 1.2power */
	"avdd",		/* Analog power 2.8*/
	"dovdd",	/* Digital I/O power 1.8*/
	
};

#define S5K5E9_NUM_SUPPLIES ARRAY_SIZE(s5k5e9_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct s5k5e9_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct s5k5e9 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[S5K5E9_NUM_SUPPLIES];

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct s5k5e9_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};
static const  int s5k5e9_data_format[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,

};
#define to_s5k5e9(sd) container_of(sd, struct s5k5e9, subdev)

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1296
 * grabwindow_height 972
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_global_regs[] = {
  {0x0100, 0x00},
  {0x0000, 0x04},
  {0x0000, 0x55},
  {0x0A02, 0x3F},
  {0x3B45, 0x01},
  {0x0B05, 0x01},
  {0x392F, 0x01},
  {0x3930, 0x00},
  {0x3924, 0x7F},
  {0x3925, 0xFD},
  {0x3C08, 0xFF},
  {0x3C09, 0xFF},
  {0x3C31, 0xFF},
  {0x3C32, 0xFF},
  {0x3290, 0x10},
  {0x3200, 0x01},
  {0x3074, 0x06},
  {0x3075, 0x2F},
  {0x308A, 0x20},
  {0x308B, 0x08},
  {0x308C, 0x0B},
  {0x3081, 0x07},
  {0x307B, 0x85},
  {0x307A, 0x0A},
  {0x3079, 0x0A},
  {0x306E, 0x71},
  {0x306F, 0x28},
  {0x301F, 0x20},
  {0x3012, 0x4E},
  {0x306B, 0x9A},
  {0x3091, 0x16},
  {0x30C4, 0x06},
  {0x306A, 0x79},
  {0x30B0, 0xFF},
  {0x306D, 0x08},
  {0x3084, 0x16},
  {0x3070, 0x0F},
  {0x30C2, 0x05},
  {0x3069, 0x87},
  {0x3C0F, 0x00},
  {0x3083, 0x14},
  {0x3080, 0x08},
  {0x3C34, 0xEA},
  {0x3C35, 0x5C},
  {REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 740(0x2e4)
 * framelength 2024(0x7e8)
 * grabwindow_width 2592
 * grabwindow_height 1944
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_2592x1944_regs[] = {
  {0x0100, 0x00},
  {0x0136, 0x18},
  {0x0137, 0x00},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x5F},
  {0x030D, 0x04},
  {0x030E, 0x00},
  {0x030F, 0x92},
  {0x3C1F, 0x00},
  {0x3C17, 0x00},
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x0114, 0x01},
  {0x0820, 0x03},
  {0x0821, 0x6C},
  {0x0822, 0x00},
  {0x0823, 0x00},
  {0x3929, 0x0f},
  {0x0344, 0x00},
  {0x0345, 0x08},
  {0x0346, 0x00},
  {0x0347, 0x08},
  {0x0348, 0x0A},
  {0x0349, 0x27},
  {0x034A, 0x07},
  {0x034B, 0x9f},
  {0x034C, 0x0A},
  {0x034D, 0x20},
  {0x034E, 0x07},
  {0x034F, 0x98},
  {0x0900, 0x00},
  {0x0901, 0x00},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
//  {0x0101, 0x00},
  {0x0340, 0x07},
  {0x0341, 0xEE},
  {0x0342, 0x0C},
  {0x0343, 0x28},
  {0x0200, 0x0B},
  {0x0201, 0x9C},
  {0x0202, 0x00},
  {0x0203, 0x02},
  {0x30B8, 0x2E},
  {0x30BA, 0x36},
  {0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1920
 * grabwindow_height 1080
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_1920x1080_regs[] = {
  {0x0100, 0x00},
  {0x0136, 0x18},
  {0x0137, 0x00},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x5F},
  {0x030D, 0x04},
  {0x030E, 0x00},
  {0x030F, 0x7D},
  {0x3C1F, 0x00},
  {0x3C17, 0x00},
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x0114, 0x01},
  {0x0820, 0x02},
  {0x0821, 0xEE},
  {0x0822, 0x00},
  {0x0823, 0x00},
  {0x3929, 0x0F},
  {0x0344, 0x01},
  {0x0345, 0x58},
  {0x0346, 0x01},
  {0x0347, 0xB8},
  {0x0348, 0x08},
  {0x0349, 0xD7},
  {0x034A, 0x05},
  {0x034B, 0xEF},
  {0x034C, 0x07},
  {0x034D, 0x80},
  {0x034E, 0x04},
  {0x034F, 0x38},
  {0x0900, 0x00},
  {0x0901, 0x00},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
  //{0x0101, 0x00},
  {0x0340, 0x06},
  {0x0341, 0xDD},
  {0x0342, 0x0E},
  {0x0343, 0x14},
  //{0x0200, 0x0D},
  //{0x0201, 0x90},
  //{0x0202, 0x00},
  //{0x0203, 0x02},
  {0x30B8, 0x2E},
  {0x30BA, 0x36},
  {0x0100, 0x01},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 740(0x02e4)
 * framelength 1012(0x03f4)
 * grabwindow_width 1296
 * grabwindow_height 972
 * max_framerate 60fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_1296x972_regs[] = {
  {0x0100, 0x00},
  {0x0136, 0x18},
  {0x0137, 0x00},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x5F},
  {0x030D, 0x04},
  {0x030E, 0x00},
  {0x030F, 0x92},
  {0x3C1F, 0x00},
  {0x3C17, 0x00},
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x0114, 0x01},
  {0x0820, 0x03},
  {0x0821, 0x6C},
  {0x0822, 0x00},
  {0x0823, 0x00},
  {0x3929, 0x0F},
  {0x0344, 0x00},
  {0x0345, 0x08},
  {0x0346, 0x00},
  {0x0347, 0x08},
  {0x0348, 0x0A},
  {0x0349, 0x27},
  {0x034A, 0x07},
  {0x034B, 0x9F},
  {0x034C, 0x05},
  {0x034D, 0x10},
  {0x034E, 0x03},
  {0x034F, 0xCC},
  {0x0900, 0x01},
  {0x0901, 0x22},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x03},
  //{0x0101, 0x00},
  {0x0340, 0x07},
  {0x0341, 0xEE},
  {0x0342, 0x0C},
  {0x0343, 0x28},
  //{0x0200, 0x0B},
  //{0x0201, 0x9C},
  //{0x0202, 0x00},
  //{0x0203, 0x02},
  {0x30B8, 0x2A},
  {0x30BA, 0x2E},
  {0x0100, 0x01},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 2232(0x8b8)
 * grabwindow_width 1280
 * grabwindow_height 720
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_1280x720_regs[] = {
  {0x0100, 0x00},
  {0x0136, 0x18},
  {0x0137, 0x00},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x5F},
  {0x030D, 0x04},
  {0x030E, 0x00},
  {0x030F, 0x92},
  {0x3C1F, 0x00},
  {0x3C17, 0x00},
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x0114, 0x01},
  {0x0820, 0x03},
  {0x0821, 0x6C},
  {0x0822, 0x00},
  {0x0823, 0x00},
  {0x3929, 0x0F},
  {0x0344, 0x00},
  {0x0345, 0x18},
  {0x0346, 0x01},
  {0x0347, 0x04},
  {0x0348, 0x0A},
  {0x0349, 0x17},
  {0x034A, 0x06},
  {0x034B, 0xA3},
  {0x034C, 0x05},
  {0x034D, 0x00},
  {0x034E, 0x02},
  {0x034F, 0xD0},
  {0x0900, 0x01},
  {0x0901, 0x22},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x03},
  //{0x0101, 0x00},
  {0x0340, 0x03},
  {0x0341, 0xF9},
  {0x0342, 0x0C},
  {0x0343, 0x28},
  //{0x0200, 0x0B},
  //{0x0201, 0x9C},
  //{0x0202, 0x00},
  //{0x0203, 0x02},
  {0x30B8, 0x2A},
  {0x30BA, 0x2E},
  {0x0100, 0x01},
	{REG_NULL, 0x00}
};

/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 672(0x2a0)
 * framelength 558(0x22e)
 * grabwindow_width 640
 * grabwindow_height 480
 * max_framerate 120fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval s5k5e9_640x480_regs[] = {
  {0x0100, 0x00},
  {0x0136, 0x18},
  {0x0137, 0x00},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x5F},
  {0x030D, 0x04},
  {0x030E, 0x00},
  {0x030F, 0x92},
  {0x3C1F, 0x00},
  {0x3C17, 0x00},
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x0114, 0x01},
  {0x0820, 0x03},
  {0x0821, 0x6C},
  {0x0822, 0x00},
  {0x0823, 0x00},
  {0x3929, 0x0F},
  {0x0344, 0x00},
  {0x0345, 0x18},
  {0x0346, 0x00},
  {0x0347, 0x14},
  {0x0348, 0x0A},
  {0x0349, 0x17},
  {0x034A, 0x07},
  {0x034B, 0x93},
  {0x034C, 0x02},
  {0x034D, 0x80},
  {0x034E, 0x01},
  {0x034F, 0xE0},
  {0x0900, 0x01},
  {0x0901, 0x44},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x07},
  //{0x0101, 0x00},
  {0x0340, 0x02},
  {0x0341, 0xA6},
  {0x0342, 0x0C},
  {0x0343, 0x28},
  //{0x0200, 0x0B},
  //{0x0201, 0x9C},
  //{0x0202, 0x00},
  //{0x0203, 0x02},
  {0x30B8, 0x2E},
  {0x30BA, 0x36},
  {0x0100, 0x01},
	{REG_NULL, 0x00}
};

static const struct s5k5e9_mode supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x03F2,
		.hts_def = 0x0C28,
		.vts_def = 0x07EE,
		.reg_list = s5k5e9_2592x1944_regs,
	},
/*	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x03F2,
		.hts_def = 0x0E14,
		.vts_def = 0x06D0,
		.reg_list = s5k5e9_1920x1080_regs,
	},*/
	{
		.width = 1296,
		.height = 972,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x03F2,
		.hts_def = 0x0C28,
		.vts_def = 0x07EE,
		.reg_list = s5k5e9_1296x972_regs,
	},
	/*{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x03F2,
		.hts_def = 0x0C28,
		.vts_def = 0x03F9,
		.reg_list = s5k5e9_1280x720_regs,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 900000,
		},
		.exp_def = 0x03F2,
		.hts_def = 0x0C28,
		.vts_def = 0x02A6,
		.reg_list = s5k5e9_640x480_regs,
	},*/
};

#define S5K5E9_LINK_FREQ_420MHZ		420000000
static const s64 link_freq_menu_items[] = {
	S5K5E9_LINK_FREQ_420MHZ
};

static const char * const s5k5e9_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int s5k5e9_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int s5k5e9_write_array(struct i2c_client *client,
			      const struct regval *regs)
{/*
	u8 *data;
	u32 i, j = 0, k = 0;
	int ret = 0;
	struct i2c_msg *msg;

	msg = kmalloc((sizeof(struct i2c_msg) * I2C_MSG_MAX),
		GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	data = kmalloc((sizeof(unsigned char) * I2C_DATA_MAX),
		GFP_KERNEL);
	if (!data) {
		kfree(msg);
		return -ENOMEM;
	}

	for (i = 0; regs[i].addr != REG_NULL; i++) {
		(msg + j)->addr = client->addr;
		(msg + j)->flags = I2C_M_WR;
		(msg + j)->buf = (data + k);

		data[k + 0] = (u8)(regs[i].addr >> 8);
		data[k + 1] = (u8)(regs[i].addr & 0xFF);
		data[k + 2] = (u8)(regs[i].val & 0xFF);
		k = k + 3;
		(msg + j)->len = 3;

		if (j++ == (I2C_MSG_MAX - 1)) {
			ret = i2c_transfer(client->adapter, msg, j);
			if (ret < 0) {
				kfree(msg);
				kfree(data);
				printk("i2c error\n");
				return ret;
			}
			j = 0;
			k = 0;
		}
		udelay(50);
	}
   printk("i2c error1\n");
	if (j != 0) {
		
		ret = i2c_transfer(client->adapter, msg, j);
		if (ret < 0) {
			kfree(msg);
			kfree(data);
			printk("i2c error1\n");
			return ret;
		}
	}
	kfree(msg);
	kfree(data);
*/
	u32 i;
	int ret = 0;
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = s5k5e9_write_reg(client, regs[i].addr,
					S5K5E9_REG_VALUE_08BIT,
					regs[i].val);
	return ret;

	return 0;
}

/* Read registers up to 4 at a time */
static int s5k5e9_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

#define PATTERN_MASK 0xffffffc7
#define BAYER_PAT_RGGB        (0 << 3)
#define BAYER_PAT_GRBG        (1 << 3)
#define BAYER_PAT_GBRG        (2 << 3)
#define BAYER_PAT_BGGR        (3 << 3)

int Adjusts5k5e9direction(int rotate)
{
	u32 val;
	
	phys_addr_t write_addr_phy = 0xff4a0404;
	 void __iomem *write_addr = ioremap(write_addr_phy, 4);
//	 printk("frank debug rotate111 =%d\n\n",rotate);
		if(s5k5e9_client==NULL) 
	{
		//printk("frank debug rotate111 =%d\n\n",rotate);
		return 0;
	}
	if(s5k5e9power_on==0) 
	{
		//printk("frank debug rotate222 =%d\n\n",rotate);
		return 0;
	}
	if(rotate==1)
	{
			s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_ORIGENTATION,S5K5E9_REG_VALUE_08BIT, 0x01);
			//printk("frank debug rotate =%d\n\n",rotate);
	}
	else
	{
			s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_ORIGENTATION,S5K5E9_REG_VALUE_08BIT, 0x00);
			//printk("frank debug rotate =%d\n\n",rotate);
	}	
	udelay(50);
	//printk("frank debug rotate =%d\n\n",rotate);
#if 1 /* rk3326 isp set GRF_SOC_CON16[15] to 1 */
    val = readl(write_addr);
    val = val && 0xffffffc7;
    if(s5k5e9_data_format[rotate] == MEDIA_BUS_FMT_SGRBG10_1X10)
    	writel(val | BAYER_PAT_GRBG, write_addr);
    else
        writel(val | BAYER_PAT_RGGB, write_addr);
   // printk(KERN_ERR " set GRF_SOC_CON16[15] to 1 ok\n");
#endif
	return 1;
}
int Set_S5k5e9Dataformat(int rotate)
{
		if(rotate==1)
	{
			s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_ORIGENTATION,S5K5E9_REG_VALUE_08BIT, 0x01);
			//printk("frank debug rotate =%d\n\n",rotate);
	}
	else
	{
			s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_ORIGENTATION,S5K5E9_REG_VALUE_08BIT, 0x00);
			//printk("frank debug rotate =%d\n\n",rotate);
	}
	return 1;
}
static int s5k5e9_get_reso_dist(const struct s5k5e9_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}
EXPORT_SYMBOL(s5k5e9_read_reg);
static const struct s5k5e9_mode *
s5k5e9_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = s5k5e9_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int s5k5e9_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	const struct s5k5e9_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&s5k5e9->mutex);

	mode = s5k5e9_find_best_fit(fmt);
	fmt->format.code = s5k5e9_data_format[front_back_switch1_level];
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&s5k5e9->mutex);
		return -ENOTTY;
#endif
	} else {
		s5k5e9->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(s5k5e9->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5k5e9->vblank, vblank_def,
					 S5K5E9_VTS_MAX - mode->height,
					 1, vblank_def);
	}
	Set_S5k5e9Dataformat(front_back_switch1_level);
	mutex_unlock(&s5k5e9->mutex);
	return 0;
}

static int s5k5e9_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	const struct s5k5e9_mode *mode = s5k5e9->cur_mode;

	mutex_lock(&s5k5e9->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		 // printk("frank debug fmt->format.code3 =%d\n\n",fmt->format.code);
#else
		mutex_unlock(&s5k5e9->mutex);
		//  printk("frank debug fmt->format.code2 =%d\n\n",fmt->format.code);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = s5k5e9_data_format[front_back_switch1_level];
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&s5k5e9->mutex);
  //printk("frank debug fmt->format.code1 =%d\n\n",fmt->format.code);
	return 0;
}

static int s5k5e9_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = s5k5e9_data_format[front_back_switch1_level];
	//printk("frank debug s5k5e9_enum_mbus_code\n\n");
	return 0;
}

static int s5k5e9_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != s5k5e9_data_format[front_back_switch1_level])
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
//	printk("frank debug s5k5e9_enum_frame_sizes\n\n");
	return 0;
}

static int s5k5e9_enable_test_pattern(struct s5k5e9 *s5k5e9, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | S5K5E9_TEST_PATTERN_ENABLE;
	else
		val = S5K5E9_TEST_PATTERN_DISABLE;

	return s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_TEST_PATTERN,
				S5K5E9_REG_VALUE_08BIT, val);
}

static int s5k5e9_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	const struct s5k5e9_mode *mode = s5k5e9->cur_mode;

	mutex_lock(&s5k5e9->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&s5k5e9->mutex);

	return 0;
}

static void s5k5e9_get_module_inf(struct s5k5e9 *s5k5e9,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, S5K5E9_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, s5k5e9->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, s5k5e9->len_name, sizeof(inf->base.lens));
}

static long s5k5e9_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		s5k5e9_get_module_inf(s5k5e9, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long s5k5e9_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = s5k5e9_ioctl(sd, cmd, inf);
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
			ret = s5k5e9_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __s5k5e9_start_stream(struct s5k5e9 *s5k5e9)
{
	int ret;

	ret = s5k5e9_write_array(s5k5e9->client, s5k5e9->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&s5k5e9->mutex);
	ret = v4l2_ctrl_handler_setup(&s5k5e9->ctrl_handler);
	mutex_lock(&s5k5e9->mutex);
	if (ret)
		return ret;

	return s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_CTRL_MODE,
				S5K5E9_REG_VALUE_08BIT, S5K5E9_MODE_STREAMING);
}

static int __s5k5e9_stop_stream(struct s5k5e9 *s5k5e9)
{
	return s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_CTRL_MODE,
				S5K5E9_REG_VALUE_08BIT, S5K5E9_MODE_SW_STANDBY);
}

static int s5k5e9_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	struct i2c_client *client = s5k5e9->client;
	int ret = 0;

	mutex_lock(&s5k5e9->mutex);
	on = !!on;
	if (on == s5k5e9->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		//Adjusts5k5e9direction(front_back_switch1_level);
		//	mdelay(10);
		ret = __s5k5e9_start_stream(s5k5e9);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__s5k5e9_stop_stream(s5k5e9);
		pm_runtime_put(&client->dev);
	}

	s5k5e9->streaming = on;

unlock_and_return:
	mutex_unlock(&s5k5e9->mutex);

	return ret;
}

static int s5k5e9_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	struct i2c_client *client = s5k5e9->client;
	int ret = 0;

	mutex_lock(&s5k5e9->mutex);

	/* If the power state is not modified - no work to do. */
	if (s5k5e9->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		printk("s5k5e9->power_on start \n");	
		front_back_switch1_flag=10;
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			front_back_switch1_flag=0;
			goto unlock_and_return;
		}
		s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_Reset,S5K5E9_REG_VALUE_08BIT, 0x01);
		mdelay(5);
		ret = s5k5e9_write_array(s5k5e9->client, s5k5e9_global_regs);
		if (ret) {
			pm_runtime_put(&client->dev);
			mdelay(50);
			pm_runtime_get_sync(&client->dev);
			mdelay(20);
			s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_Reset,S5K5E9_REG_VALUE_08BIT, 0x01);
		  mdelay(8);
			ret = s5k5e9_write_array(s5k5e9->client, s5k5e9_global_regs);//
			if(ret)
			{
					v4l2_err(sd, "could not set init registers\n");
					//	pm_runtime_put_noidle(&client->dev);
					pm_runtime_put(&client->dev);
					front_back_switch1_flag=0;
				  goto unlock_and_return;
			 }
		}
		Adjusts5k5e9direction(front_back_switch1_level);
		s5k5e9->power_on = true;
		s5k5e9power_on=1;
		s5k5e9power_on_flag=1;
		s5k5e9power_on_flag1=1;
		Check_micro_distance=1;
		printk("s5k5e9->power_on end\n");	
	} else {
		printk("s5k5e9->power_off\n");
		pm_runtime_put(&client->dev);
		s5k5e9->power_on = false;
		s5k5e9power_on=0;
		front_back_switch1_flag=0;
		s5k5e9power_on_flag1=0;
		Check_micro_distance=0;
		//	printk("s5k5e9->power_off end1 \n");
	}

unlock_and_return:
	mutex_unlock(&s5k5e9->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 s5k5e9_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, S5K5E9_XVCLK_FREQ / 1000 / 1000);
}

static int __s5k5e9_power_on(struct s5k5e9 *s5k5e9)
{
	int ret;
	u32 delay_us;
	struct device *dev = &s5k5e9->client->dev;

	ret = clk_set_rate(s5k5e9->xvclk, S5K5E9_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(s5k5e9->xvclk) != S5K5E9_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(s5k5e9->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(s5k5e9->reset_gpio))
		gpiod_set_value_cansleep(s5k5e9->reset_gpio, 1);

	ret = regulator_bulk_enable(S5K5E9_NUM_SUPPLIES, s5k5e9->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(s5k5e9->reset_gpio))
		gpiod_set_value_cansleep(s5k5e9->reset_gpio, 0);

	if (!IS_ERR(s5k5e9->pwdn_gpio))
		gpiod_set_value_cansleep(s5k5e9->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = s5k5e9_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
//	dev_err(dev, "frank debug to enable xvclk\n");
	return 0;

disable_clk:
	clk_disable_unprepare(s5k5e9->xvclk);

	return ret;
}

static void __s5k5e9_power_off(struct s5k5e9 *s5k5e9)
{
	if (!IS_ERR(s5k5e9->pwdn_gpio))
		gpiod_set_value_cansleep(s5k5e9->pwdn_gpio, 0);
	clk_disable_unprepare(s5k5e9->xvclk);
	if (!IS_ERR(s5k5e9->reset_gpio))
		gpiod_set_value_cansleep(s5k5e9->reset_gpio, 1);
		
	
	regulator_bulk_disable(S5K5E9_NUM_SUPPLIES, s5k5e9->supplies);
}

static int s5k5e9_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
//	dev_info(dev, "frank12 debug s5k5e9_runtime_resume\n");
	return __s5k5e9_power_on(s5k5e9);
}

static int s5k5e9_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
 // dev_info(dev, "frank123 debug s5k5e9_runtime_suspend\n");
	__s5k5e9_power_off(s5k5e9);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int s5k5e9_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct s5k5e9_mode *def_mode = &supported_modes[0];

	mutex_lock(&s5k5e9->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = s5k5e9_data_format[front_back_switch1_level];
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&s5k5e9->mutex);
	if(s5k5e9power_on_flag==1)
	{
		if(s5k5e9power_on_flag1>6)
		{
			s5k5e9power_on_flag=0;
			s5k5e9power_on_flag1=0;
		}
		else
		{
			s5k5e9power_on_flag1++;
		}
		printk("frank debug s5k5e9open\n\n");
		Adjusts5k5e9direction(front_back_switch1_level);
  }
	/* No crop or compose */
		printk("frank debug s5k5e9open\n\n");
	return 0;
}
#endif

static int s5k5e9_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	//printk("frank debug s5k5e9_enum_frame_interval1111ß \n\n");
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != s5k5e9_data_format[front_back_switch1_level])
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	//	printk("frank debug s5k5e9_enum_frame_interval \n\n");
	return 0;
}

static const struct dev_pm_ops s5k5e9_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k5e9_runtime_suspend,
			   s5k5e9_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops s5k5e9_internal_ops = {
	.open = s5k5e9_open,
};
#endif

static const struct v4l2_subdev_core_ops s5k5e9_core_ops = {
	.s_power = s5k5e9_s_power,
	.ioctl = s5k5e9_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = s5k5e9_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops s5k5e9_video_ops = {
	.s_stream = s5k5e9_s_stream,
	.g_frame_interval = s5k5e9_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops s5k5e9_pad_ops = {
	.enum_mbus_code = s5k5e9_enum_mbus_code,
	.enum_frame_size = s5k5e9_enum_frame_sizes,
	.enum_frame_interval = s5k5e9_enum_frame_interval,
	.get_fmt = s5k5e9_get_fmt,
	.set_fmt = s5k5e9_set_fmt,
};

static const struct v4l2_subdev_ops s5k5e9_subdev_ops = {
	.core	= &s5k5e9_core_ops,
	.video	= &s5k5e9_video_ops,
	.pad	= &s5k5e9_pad_ops,
};

static int s5k5e9_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e9 *s5k5e9 = container_of(ctrl->handler,
					     struct s5k5e9, ctrl_handler);
	struct i2c_client *client = s5k5e9->client;
	s64 max;
	int ret = 0;
//dev_info(&client->dev, "frankadsfadf debug to enable xvclk\n");
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = s5k5e9->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(s5k5e9->exposure,
					 s5k5e9->exposure->minimum, max,
					 s5k5e9->exposure->step,
					 s5k5e9->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
	{
		dev_info(&client->dev, "frankadsfadf debug to s5k5e9_set_ctrl pm_runtime_get\n");
		pm_runtime_put(&client->dev);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_EXPOSURE,
				       S5K5E9_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_ANALOG_GAIN,
				       S5K5E9_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
	break;
		ret = s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_DIGI_GAIN_L,
				       S5K5E9_REG_VALUE_08BIT,
				       ctrl->val & S5K5E9_DIGI_GAIN_L_MASK);
		ret |= s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_DIGI_GAIN_H,
				       S5K5E9_REG_VALUE_08BIT,
				       ctrl->val >> S5K5E9_DIGI_GAIN_H_SHIFT);
		break;
	case V4L2_CID_VBLANK:
		ret = s5k5e9_write_reg(s5k5e9->client, S5K5E9_REG_VTS,
				       S5K5E9_REG_VALUE_16BIT,
				       ctrl->val + s5k5e9->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = s5k5e9_enable_test_pattern(s5k5e9, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);
	if(s5k5e9power_on_flag==1)
	{
		//s5k5e9power_on_flag=0;
	//	printk("frank debug 1111\n\n");
		Adjusts5k5e9direction(front_back_switch1_level);
  }
  //printk("frank debug s5k5e9_set_ctrl123 \n\n");
	return ret;
}

static const struct v4l2_ctrl_ops s5k5e9_ctrl_ops = {
	.s_ctrl = s5k5e9_set_ctrl,
};

static int s5k5e9_initialize_controls(struct s5k5e9 *s5k5e9)
{
	const struct s5k5e9_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &s5k5e9->ctrl_handler;
	mode = s5k5e9->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &s5k5e9->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, S5K5E9_PIXEL_RATE, 1, S5K5E9_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	s5k5e9->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (s5k5e9->hblank)
		s5k5e9->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5k5e9->vblank = v4l2_ctrl_new_std(handler, &s5k5e9_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				S5K5E9_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	s5k5e9->exposure = v4l2_ctrl_new_std(handler, &s5k5e9_ctrl_ops,
				V4L2_CID_EXPOSURE, S5K5E9_EXPOSURE_MIN,
				exposure_max, S5K5E9_EXPOSURE_STEP,
				mode->exp_def);

	s5k5e9->anal_gain = v4l2_ctrl_new_std(handler, &s5k5e9_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	/* Digital gain */
	s5k5e9->digi_gain = v4l2_ctrl_new_std(handler, &s5k5e9_ctrl_ops,
				V4L2_CID_DIGITAL_GAIN, S5K5E9_DIGI_GAIN_MIN,
				S5K5E9_DIGI_GAIN_MAX, S5K5E9_DIGI_GAIN_STEP,
				S5K5E9_DIGI_GAIN_DEFAULT);

	s5k5e9->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&s5k5e9_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(s5k5e9_test_pattern_menu) - 1,
				0, 0, s5k5e9_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&s5k5e9->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	s5k5e9->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5k5e9_check_sensor_id(struct s5k5e9 *s5k5e9,
				  struct i2c_client *client)
{
	struct device *dev = &s5k5e9->client->dev;
	u32 id = 0;
	int ret;

	ret = s5k5e9_read_reg(client, S5K5E9_REG_CHIP_ID,
			      S5K5E9_REG_VALUE_16BIT, &id);
			      
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected s5k5e9333 %04x sensor\n", CHIP_ID);

	return 0;
}

static int s5k5e9_configure_regulators(struct s5k5e9 *s5k5e9)
{
	int i;
	//struct device *dev = &s5k5e9->client->dev;
	for (i = 0; i < S5K5E9_NUM_SUPPLIES; i++)
	{
		s5k5e9->supplies[i].supply = s5k5e9_supply_names[i];
	//	dev_info(dev, "s5k5e9_configure_regulators '%s'\n",s5k5e9->supplies[i].supply);	
}
	return devm_regulator_bulk_get(&s5k5e9->client->dev,
				       S5K5E9_NUM_SUPPLIES,
				       s5k5e9->supplies);
}

static int s5k5e9_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct s5k5e9 *s5k5e9;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	s5k5e9 = devm_kzalloc(dev, sizeof(*s5k5e9), GFP_KERNEL);
	if (!s5k5e9)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &s5k5e9->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &s5k5e9->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &s5k5e9->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &s5k5e9->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	s5k5e9power_on=0;
	s5k5e9power_on_flag=0;
	s5k5e9power_on_flag1=0;
	Check_micro_distance=0;
	s5k5e9->client = client;
	s5k5e9_client= client;
	s5k5e9->cur_mode = &supported_modes[0];

	s5k5e9->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(s5k5e9->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	s5k5e9->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5k5e9->reset_gpio)) {
		dev_warn(dev, "Failed to get reset-gpios\n");
	}

	s5k5e9->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(s5k5e9->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = s5k5e9_configure_regulators(s5k5e9);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&s5k5e9->mutex);

	sd = &s5k5e9->subdev;
	v4l2_i2c_subdev_init(sd, client, &s5k5e9_subdev_ops);
	ret = s5k5e9_initialize_controls(s5k5e9);
	if (ret)
		goto err_destroy_mutex;

	ret = __s5k5e9_power_on(s5k5e9);
	if (ret)
		goto err_free_handler;
		
  mdelay(20);
	ret = s5k5e9_check_sensor_id(s5k5e9, client);
	if (ret)
	{
		s5k5e9_write_reg(s5k5e9_client, S5K5E9_REG_Reset,S5K5E9_REG_VALUE_08BIT, 0x01);
		mdelay(30);
		dev_err(dev, "s5k5e9_check_sensor_id erorr\n");
	  ret = s5k5e9_check_sensor_id(s5k5e9, client);
	  if (ret)
	  {
	  	dev_err(dev, "s5k5e9_check_sensor_id erorr1\n");
			goto err_power_off;
		}
	}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &s5k5e9_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	s5k5e9->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &s5k5e9->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(s5k5e9->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 s5k5e9->module_index, facing,
		 S5K5E9_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	//printk("frank debug prob sucsessful\n\n");
	//Adjusts5k5e9direction(front_back_switch1_level);
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__s5k5e9_power_off(s5k5e9);
err_free_handler:
	v4l2_ctrl_handler_free(&s5k5e9->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5k5e9->mutex);

	return ret;
}

static int s5k5e9_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k5e9 = to_s5k5e9(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&s5k5e9->ctrl_handler);
	mutex_destroy(&s5k5e9->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__s5k5e9_power_off(s5k5e9);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s5k5e9_of_match[] = {
	{ .compatible = "ovti,s5k5e9" },
	{},
};
MODULE_DEVICE_TABLE(of, s5k5e9_of_match);
#endif

static const struct i2c_device_id s5k5e9_match_id[] = {
	{ "ovti,s5k5e9", 0 },
	{ },
};

static struct i2c_driver s5k5e9_i2c_driver = {
	.driver = {
		.name = S5K5E9_NAME,
		.pm = &s5k5e9_pm_ops,
		.of_match_table = of_match_ptr(s5k5e9_of_match),
	},
	.probe		= &s5k5e9_probe,
	.remove		= &s5k5e9_remove,
	.id_table	= s5k5e9_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&s5k5e9_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&s5k5e9_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision s5k5e9 sensor driver");
MODULE_LICENSE("GPL v2");
