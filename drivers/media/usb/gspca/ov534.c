/*
 * ov534-ov7xxx gspca driver
 *
 * Copyright (C) 2008 Antonio Ospite <ospite@studenti.unina.it>
 * Copyright (C) 2008 Jim Paris <jim@jtan.com>
 * Copyright (C) 2009 Jean-Francois Moine http://moinejf.free.fr
 *
 * Based on a prototype written by Mark Ferrell <majortrips@gmail.com>
 * USB protocol reverse engineered by Jim Paris <jim@jtan.com>
 * https://jim.sh/svn/jim/devl/playstation/ps3/eye/test/
 *
 * PS3 Eye camera enhanced by Richard Kaswy http://kaswy.free.fr
 * PS3 Eye camera - brightness, contrast, awb, agc, aec controls
 *                  added by Max Thrun <bear24rw@gmail.com>
 * PS3 Eye camera - FPS range extended by Joseph Howse
 *                  <josephhowse@nummist.com> http://nummist.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define BAYER_OUTPUT

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define MODULE_NAME "ov534"

#include "gspca.h"

#include <linux/fixp-arith.h>
#include <media/v4l2-ctrls.h>

#define OV534_REG_ADDRESS	0xf1	/* sensor address */
#define OV534_REG_SUBADDR	0xf2
#define OV534_REG_WRITE		0xf3
#define OV534_REG_READ		0xf4
#define OV534_REG_OPERATION	0xf5
#define OV534_REG_STATUS	0xf6

#define OV534_OP_WRITE_3	0x37
#define OV534_OP_WRITE_2	0x33
#define OV534_OP_READ_2		0xf9

#define CTRL_TIMEOUT 500
#define DEFAULT_FRAME_RATE 30

MODULE_AUTHOR("Antonio Ospite <ospite@studenti.unina.it>");
MODULE_DESCRIPTION("GSPCA/OV534 USB Camera Driver");
MODULE_LICENSE("GPL");

/* specific webcam descriptor */
struct sd {
	struct gspca_dev gspca_dev;	/* !! must be the first item */

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *contrast;
	struct { /* gain control cluster */
		struct v4l2_ctrl *autogain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *autowhitebalance;
	struct { /* exposure control cluster */
		struct v4l2_ctrl *autoexposure;
		struct v4l2_ctrl *exposure;
	};
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *plfreq;
	struct v4l2_ctrl *gamma;

	__u32 last_pts;
	u16 last_fid;
	__u32 frame_rate_numerator;
	__u32 frame_rate_denominator;

	u8 sensor;
};
enum sensors {
	SENSOR_OV767x,
	SENSOR_OV772x,
	NSENSORS
};

static int sd_start(struct gspca_dev *gspca_dev);
static void sd_stopN(struct gspca_dev *gspca_dev);

#ifdef BAYER_OUTPUT
static const struct v4l2_pix_format ov772x_mode[] = {
	{320, 240, V4L2_PIX_FMT_SGRBG8, V4L2_FIELD_NONE,
	 .bytesperline = 320,
	 .sizeimage = 320 * 240,
	 .colorspace = V4L2_COLORSPACE_RAW,
	 .priv = 1},
	{640, 480, V4L2_PIX_FMT_SGRBG8, V4L2_FIELD_NONE,
	 .bytesperline = 640,
	 .sizeimage = 640 * 480,
	 .colorspace = V4L2_COLORSPACE_RAW,
	 .priv = 0},
};
#else
static const struct v4l2_pix_format ov772x_mode[] = {
	{320, 240, V4L2_PIX_FMT_YUYV, V4L2_FIELD_NONE,
	 .bytesperline = 320 * 2,
	 .sizeimage = 320 * 240 * 2,
	 .colorspace = V4L2_COLORSPACE_SRGB,
	 .priv = 1},
	{640, 480, V4L2_PIX_FMT_YUYV, V4L2_FIELD_NONE,
	 .bytesperline = 640 * 2,
	 .sizeimage = 640 * 480 * 2,
	 .colorspace = V4L2_COLORSPACE_SRGB,
	 .priv = 0},
};
#endif

static const struct v4l2_pix_format ov767x_mode[] = {
	{320, 240, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 240 * 3 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG},
	{640, 480, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 640,
		.sizeimage = 640 * 480 * 3 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG},
};

static const u8 qvga_rates[] = {187, 150, 137, 125, 100, 90, 75, 60, 50, 40, 37, 30, 17, 15, 12, 10, 7, 5, 3, 2};
static const u8 vga_rates[] = {75, 60, 50, 40, 30, 25, 20, 15, 10, 8, 5, 3, 2, 1};
static const struct v4l2_fract vga_fract_rates[] = {{10,9}, {10,8}, {10,7}, {10,6}, {10,5}, {10,4}, {10,3}, {10,2}, {10,1}};

static const struct framerates ov772x_framerates[] = {
	{ /* 320x240 */
		.rates = qvga_rates,
		.nrates = ARRAY_SIZE(qvga_rates),
	},
	{ /* 640x480 */
		.rates = vga_rates,
		.nrates = ARRAY_SIZE(vga_rates),
		.fract_rates = vga_fract_rates,
		.nfract_rates = ARRAY_SIZE(vga_fract_rates),
	},
};

struct reg_array {
	const u8 (*val)[2];
	int len;
};

static const u8 bridge_init_767x[][2] = {
/* comments from the ms-win file apollo7670.set */
/* str1 */
	{0xf1, 0x42},
	{0x88, 0xf8},
	{0x89, 0xff},
	{0x76, 0x03},
	{0x92, 0x03},
	{0x95, 0x10},
	{0xe2, 0x00},
	{0xe7, 0x3e},
	{0x8d, 0x1c},
	{0x8e, 0x00},
	{0x8f, 0x00},
	{0x1f, 0x00},
	{0xc3, 0xf9},
	{0x89, 0xff},
	{0x88, 0xf8},
	{0x76, 0x03},
	{0x92, 0x01},
	{0x93, 0x18},
	{0x1c, 0x00},
	{0x1d, 0x48},
	{0x1d, 0x00},
	{0x1d, 0xff},
	{0x1d, 0x02},
	{0x1d, 0x58},
	{0x1d, 0x00},
	{0x1c, 0x0a},
	{0x1d, 0x0a},
	{0x1d, 0x0e},
	{0xc0, 0x50},	/* HSize 640 */
	{0xc1, 0x3c},	/* VSize 480 */
	{0x34, 0x05},	/* enable Audio Suspend mode */
	{0xc2, 0x0c},	/* Input YUV */
	{0xc3, 0xf9},	/* enable PRE */
	{0x34, 0x05},	/* enable Audio Suspend mode */
	{0xe7, 0x2e},	/* this solves failure of "SuspendResumeTest" */
	{0x31, 0xf9},	/* enable 1.8V Suspend */
	{0x35, 0x02},	/* turn on JPEG */
	{0xd9, 0x10},
	{0x25, 0x42},	/* GPIO[8]:Input */
	{0x94, 0x11},	/* If the default setting is loaded when
			 * system boots up, this flag is closed here */
};
static const u8 sensor_init_767x[][2] = {
	{0x12, 0x80},
	{0x11, 0x03},
	{0x3a, 0x04},
	{0x12, 0x00},
	{0x17, 0x13},
	{0x18, 0x01},
	{0x32, 0xb6},
	{0x19, 0x02},
	{0x1a, 0x7a},
	{0x03, 0x0a},
	{0x0c, 0x00},
	{0x3e, 0x00},
	{0x70, 0x3a},
	{0x71, 0x35},
	{0x72, 0x11},
	{0x73, 0xf0},
	{0xa2, 0x02},
	{0x7a, 0x2a},	/* set Gamma=1.6 below */
	{0x7b, 0x12},
	{0x7c, 0x1d},
	{0x7d, 0x2d},
	{0x7e, 0x45},
	{0x7f, 0x50},
	{0x80, 0x59},
	{0x81, 0x62},
	{0x82, 0x6b},
	{0x83, 0x73},
	{0x84, 0x7b},
	{0x85, 0x8a},
	{0x86, 0x98},
	{0x87, 0xb2},
	{0x88, 0xca},
	{0x89, 0xe0},
	{0x13, 0xe0},
	{0x00, 0x00},
	{0x10, 0x00},
	{0x0d, 0x40},
	{0x14, 0x38},	/* gain max 16x */
	{0xa5, 0x05},
	{0xab, 0x07},
	{0x24, 0x95},
	{0x25, 0x33},
	{0x26, 0xe3},
	{0x9f, 0x78},
	{0xa0, 0x68},
	{0xa1, 0x03},
	{0xa6, 0xd8},
	{0xa7, 0xd8},
	{0xa8, 0xf0},
	{0xa9, 0x90},
	{0xaa, 0x94},
	{0x13, 0xe5},
	{0x0e, 0x61},
	{0x0f, 0x4b},
	{0x16, 0x02},
	{0x21, 0x02},
	{0x22, 0x91},
	{0x29, 0x07},
	{0x33, 0x0b},
	{0x35, 0x0b},
	{0x37, 0x1d},
	{0x38, 0x71},
	{0x39, 0x2a},
	{0x3c, 0x78},
	{0x4d, 0x40},
	{0x4e, 0x20},
	{0x69, 0x00},
	{0x6b, 0x4a},
	{0x74, 0x10},
	{0x8d, 0x4f},
	{0x8e, 0x00},
	{0x8f, 0x00},
	{0x90, 0x00},
	{0x91, 0x00},
	{0x96, 0x00},
	{0x9a, 0x80},
	{0xb0, 0x84},
	{0xb1, 0x0c},
	{0xb2, 0x0e},
	{0xb3, 0x82},
	{0xb8, 0x0a},
	{0x43, 0x0a},
	{0x44, 0xf0},
	{0x45, 0x34},
	{0x46, 0x58},
	{0x47, 0x28},
	{0x48, 0x3a},
	{0x59, 0x88},
	{0x5a, 0x88},
	{0x5b, 0x44},
	{0x5c, 0x67},
	{0x5d, 0x49},
	{0x5e, 0x0e},
	{0x6c, 0x0a},
	{0x6d, 0x55},
	{0x6e, 0x11},
	{0x6f, 0x9f},
	{0x6a, 0x40},
	{0x01, 0x40},
	{0x02, 0x40},
	{0x13, 0xe7},
	{0x4f, 0x80},
	{0x50, 0x80},
	{0x51, 0x00},
	{0x52, 0x22},
	{0x53, 0x5e},
	{0x54, 0x80},
	{0x58, 0x9e},
	{0x41, 0x08},
	{0x3f, 0x00},
	{0x75, 0x04},
	{0x76, 0xe1},
	{0x4c, 0x00},
	{0x77, 0x01},
	{0x3d, 0xc2},
	{0x4b, 0x09},
	{0xc9, 0x60},
	{0x41, 0x38},	/* jfm: auto sharpness + auto de-noise  */
	{0x56, 0x40},
	{0x34, 0x11},
	{0x3b, 0xc2},
	{0xa4, 0x8a},	/* Night mode trigger point */
	{0x96, 0x00},
	{0x97, 0x30},
	{0x98, 0x20},
	{0x99, 0x20},
	{0x9a, 0x84},
	{0x9b, 0x29},
	{0x9c, 0x03},
	{0x9d, 0x4c},
	{0x9e, 0x3f},
	{0x78, 0x04},
	{0x79, 0x01},
	{0xc8, 0xf0},
	{0x79, 0x0f},
	{0xc8, 0x00},
	{0x79, 0x10},
	{0xc8, 0x7e},
	{0x79, 0x0a},
	{0xc8, 0x80},
	{0x79, 0x0b},
	{0xc8, 0x01},
	{0x79, 0x0c},
	{0xc8, 0x0f},
	{0x79, 0x0d},
	{0xc8, 0x20},
	{0x79, 0x09},
	{0xc8, 0x80},
	{0x79, 0x02},
	{0xc8, 0xc0},
	{0x79, 0x03},
	{0xc8, 0x20},
	{0x79, 0x26},
};
static const u8 bridge_start_vga_767x[][2] = {
/* str59 JPG */
	{0x94, 0xaa},
	{0xf1, 0x42},
	{0xe5, 0x04},
	{0xc0, 0x50},
	{0xc1, 0x3c},
	{0xc2, 0x0c},
	{0x35, 0x02},	/* turn on JPEG */
	{0xd9, 0x10},
	{0xda, 0x00},	/* for higher clock rate(30fps) */
	{0x34, 0x05},	/* enable Audio Suspend mode */
	{0xc3, 0xf9},	/* enable PRE */
	{0x8c, 0x00},	/* CIF VSize LSB[2:0] */
	{0x8d, 0x1c},	/* output YUV */
/*	{0x34, 0x05},	 * enable Audio Suspend mode (?) */
	{0x50, 0x00},	/* H/V divider=0 */
	{0x51, 0xa0},	/* input H=640/4 */
	{0x52, 0x3c},	/* input V=480/4 */
	{0x53, 0x00},	/* offset X=0 */
	{0x54, 0x00},	/* offset Y=0 */
	{0x55, 0x00},	/* H/V size[8]=0 */
	{0x57, 0x00},	/* H-size[9]=0 */
	{0x5c, 0x00},	/* output size[9:8]=0 */
	{0x5a, 0xa0},	/* output H=640/4 */
	{0x5b, 0x78},	/* output V=480/4 */
	{0x1c, 0x0a},
	{0x1d, 0x0a},
	{0x94, 0x11},
};
static const u8 sensor_start_vga_767x[][2] = {
	{0x11, 0x01},
	{0x1e, 0x04},
	{0x19, 0x02},
	{0x1a, 0x7a},
};
static const u8 bridge_start_qvga_767x[][2] = {
/* str86 JPG */
	{0x94, 0xaa},
	{0xf1, 0x42},
	{0xe5, 0x04},
	{0xc0, 0x80},
	{0xc1, 0x60},
	{0xc2, 0x0c},
	{0x35, 0x02},	/* turn on JPEG */
	{0xd9, 0x10},
	{0xc0, 0x50},	/* CIF HSize 640 */
	{0xc1, 0x3c},	/* CIF VSize 480 */
	{0x8c, 0x00},	/* CIF VSize LSB[2:0] */
	{0x8d, 0x1c},	/* output YUV */
	{0x34, 0x05},	/* enable Audio Suspend mode */
	{0xc2, 0x4c},	/* output YUV and Enable DCW */
	{0xc3, 0xf9},	/* enable PRE */
	{0x1c, 0x00},	/* indirect addressing */
	{0x1d, 0x48},	/* output YUV422 */
	{0x50, 0x89},	/* H/V divider=/2; plus DCW AVG */
	{0x51, 0xa0},	/* DCW input H=640/4 */
	{0x52, 0x78},	/* DCW input V=480/4 */
	{0x53, 0x00},	/* offset X=0 */
	{0x54, 0x00},	/* offset Y=0 */
	{0x55, 0x00},	/* H/V size[8]=0 */
	{0x57, 0x00},	/* H-size[9]=0 */
	{0x5c, 0x00},	/* DCW output size[9:8]=0 */
	{0x5a, 0x50},	/* DCW output H=320/4 */
	{0x5b, 0x3c},	/* DCW output V=240/4 */
	{0x1c, 0x0a},
	{0x1d, 0x0a},
	{0x94, 0x11},
};
static const u8 sensor_start_qvga_767x[][2] = {
	{0x11, 0x01},
	{0x1e, 0x04},
	{0x19, 0x02},
	{0x1a, 0x7a},
};

#ifdef BAYER_OUTPUT
static const u8 bridge_init_772x[][2] = {
    { 0xe7, 0x3a },

    { OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

    { 0x92, 0x01 },
    { 0x93, 0x18 },
    { 0x94, 0x10 },
    { 0x95, 0x10 },
    { 0xE2, 0x00 },
    { 0xE7, 0x3E },
    
    { 0x96, 0x00 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x0A },
    { 0x97, 0x3F },
    { 0x97, 0x4A },
    { 0x97, 0x20 },
    { 0x97, 0x15 },
    { 0x97, 0x0B },

    { 0x8E, 0x40 },
    { 0x1F, 0x81 },
    { 0xC0, 0x50 },
    { 0xC1, 0x3C },
    { 0xC2, 0x01 },
    { 0xC3, 0x01 },
    { 0x50, 0x89 },
    { 0x88, 0x08 },
    { 0x8D, 0x00 },
    { 0x8E, 0x00 },

    { 0x1C, 0x00 },     /* video data start (V_FMT) */

    { 0x1D, 0x00 },     /* RAW8 mode */
    { 0x1D, 0x02 },     /* payload size 0x0200 * 4 = 2048 bytes */
    { 0x1D, 0x00 },     /* payload size */

    { 0x1D, 0x01 },     /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    { 0x1D, 0x2C },     /* frame size */
    { 0x1D, 0x00 },     /* frame size */

    { 0x1C, 0x0A },     /* video data start (V_CNTL0) */
    { 0x1D, 0x08 },     /* turn on UVC header */
    { 0x1D, 0x0E },

    { 0x34, 0x05 },
    { 0xE3, 0x04 },
    { 0x89, 0x00 },
    { 0x76, 0x00 },
    { 0xE7, 0x2E },
    { 0x31, 0xF9 },
    { 0x25, 0x42 },
    { 0x21, 0xF0 },
    { 0xE5, 0x04 }  
};
static const u8 sensor_init_772x[][2] = {

    { 0x12, 0x80 },     /* reset */
    { 0x3D, 0x00 },

    { 0x12, 0x01 },     /* Processed Bayer RAW (8bit) */

    { 0x11, 0x01 },
    { 0x14, 0x40 },
    { 0x15, 0x00 },
    { 0x63, 0xAA },     // AWB  
    { 0x64, 0x87 },
    { 0x66, 0x00 },
    { 0x67, 0x02 },
    { 0x17, 0x26 },
    { 0x18, 0xA0 },
    { 0x19, 0x07 },
    { 0x1A, 0xF0 },
    { 0x29, 0xA0 },
    { 0x2A, 0x00 },
    { 0x2C, 0xF0 },
    { 0x20, 0x10 },
    { 0x4E, 0x0F },
    { 0x3E, 0xF3 },
    { 0x0D, 0x41 },
    { 0x32, 0x00 },
    { 0x13, 0xF0 },     // COM8  - jfrancois 0xf0   orig x0f7
    { 0x22, 0x7F },
    { 0x23, 0x03 },
    { 0x24, 0x40 },
    { 0x25, 0x30 },
    { 0x26, 0xA1 },
    { 0x2A, 0x00 },
    { 0x2B, 0x00 },
    { 0x13, 0xF7 },
    { 0x0C, 0xC0 },

    { 0x11, 0x00 },
    { 0x0D, 0x41 },

    { 0x8E, 0x00 },     // De-noise threshold - jfrancois 0x00 - orig 0x04
};
static const u8 bridge_start_vga_772x[][2] = {
    {0x1c, 0x00},
    {0x1d, 0x00},
    {0x1d, 0x02},
    {0x1d, 0x00},
    {0x1d, 0x01},   /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    {0x1d, 0x2C},   /* frame size */
    {0x1d, 0x00},   /* frame size */
    {0xc0, 0x50},
    {0xc1, 0x3c},
};
static const u8 sensor_start_vga_772x[][2] = {
    {0x12, 0x01},
    {0x17, 0x26},
    {0x18, 0xa0},
    {0x19, 0x07},
    {0x1a, 0xf0},
    {0x29, 0xa0},
    {0x2c, 0xf0},
    {0x65, 0x20},
};
static const u8 bridge_start_qvga_772x[][2] = {
    {0x1c, 0x00},
    {0x1d, 0x00},
    {0x1d, 0x02},
    {0x1d, 0x00},   
    {0x1d, 0x00},   /* frame size = 0x004B00 * 4 = 76800 bytes (320 * 240 @ 8bpp) */
    {0x1d, 0x4b},   /* frame size */
    {0x1d, 0x00},   /* frame size */
    {0xc0, 0x28},
    {0xc1, 0x1e},
};
static const u8 sensor_start_qvga_772x[][2] = {
    {0x12, 0x41},
    {0x17, 0x3f},
    {0x18, 0x50},
    {0x19, 0x03},
    {0x1a, 0x78},
    {0x29, 0x50},
    {0x2c, 0x78},
    {0x65, 0x2f},
};

#else

static const u8 bridge_init_772x[][2] = {
	{ 0xc2, 0x0c },
	{ 0x88, 0xf8 },
	{ 0xc3, 0x69 },
	{ 0x89, 0xff },
	{ 0x76, 0x03 },
	{ 0x92, 0x01 },
	{ 0x93, 0x18 },
	{ 0x94, 0x10 },
	{ 0x95, 0x10 },
	{ 0xe2, 0x00 },
	{ 0xe7, 0x3e },

	{ 0x96, 0x00 },

	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x0a },
	{ 0x97, 0x3f },
	{ 0x97, 0x4a },
	{ 0x97, 0x20 },
	{ 0x97, 0x15 },
	{ 0x97, 0x0b },

	{ 0x8e, 0x40 },
	{ 0x1f, 0x81 },
	{ 0x34, 0x05 },
	{ 0xe3, 0x04 },
	{ 0x88, 0x00 },
	{ 0x89, 0x00 },
	{ 0x76, 0x00 },
	{ 0xe7, 0x2e },
	{ 0x31, 0xf9 },
	{ 0x25, 0x42 },
	{ 0x21, 0xf0 },

	{ 0x1c, 0x00 },
	{ 0x1d, 0x40 },
	{ 0x1d, 0x02 }, /* payload size 0x0200 * 4 = 2048 bytes */
	{ 0x1d, 0x00 }, /* payload size */

	{ 0x1d, 0x02 }, /* frame size 0x025800 * 4 = 614400 */
	{ 0x1d, 0x58 }, /* frame size */
	{ 0x1d, 0x00 }, /* frame size */

	{ 0x1c, 0x0a },
	{ 0x1d, 0x08 }, /* turn on UVC header */
	{ 0x1d, 0x0e }, /* .. */

	{ 0x8d, 0x1c },
	{ 0x8e, 0x80 },
	{ 0xe5, 0x04 },

	{ 0xc0, 0x50 },
	{ 0xc1, 0x3c },
	{ 0xc2, 0x0c },
};
static const u8 sensor_init_772x[][2] = {
	{ 0x12, 0x80 },
	{ 0x11, 0x01 },
/*fixme: better have a delay?*/
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },
	{ 0x11, 0x01 },

	{ 0x3d, 0x03 },
	{ 0x17, 0x26 },
	{ 0x18, 0xa0 },
	{ 0x19, 0x07 },
	{ 0x1a, 0xf0 },
	{ 0x32, 0x00 },
	{ 0x29, 0xa0 },
	{ 0x2c, 0xf0 },
	{ 0x65, 0x20 },
	{ 0x11, 0x01 },
	{ 0x42, 0x7f },
	{ 0x63, 0xaa },		/* AWB - was e0 */
	{ 0x64, 0xff },
	{ 0x66, 0x00 },
	{ 0x13, 0xf0 },		/* com8 */
	{ 0x0d, 0x41 },
	{ 0x0f, 0xc5 },
	{ 0x14, 0x11 },

	{ 0x22, 0x7f },
	{ 0x23, 0x03 },
	{ 0x24, 0x40 },
	{ 0x25, 0x30 },
	{ 0x26, 0xa1 },
	{ 0x2a, 0x00 },
	{ 0x2b, 0x00 },
	{ 0x6b, 0xaa },
	{ 0x13, 0xff },		/* AWB */

	{ 0x90, 0x05 },
	{ 0x91, 0x01 },
	{ 0x92, 0x03 },
	{ 0x93, 0x00 },
	{ 0x94, 0x60 },
	{ 0x95, 0x3c },
	{ 0x96, 0x24 },
	{ 0x97, 0x1e },
	{ 0x98, 0x62 },
	{ 0x99, 0x80 },
	{ 0x9a, 0x1e },
	{ 0x9b, 0x08 },
	{ 0x9c, 0x20 },
	{ 0x9e, 0x81 },

	{ 0xa6, 0x07 },
	{ 0x7e, 0x0c },
	{ 0x7f, 0x16 },
	{ 0x80, 0x2a },
	{ 0x81, 0x4e },
	{ 0x82, 0x61 },
	{ 0x83, 0x6f },
	{ 0x84, 0x7b },
	{ 0x85, 0x86 },
	{ 0x86, 0x8e },
	{ 0x87, 0x97 },
	{ 0x88, 0xa4 },
	{ 0x89, 0xaf },
	{ 0x8a, 0xc5 },
	{ 0x8b, 0xd7 },
	{ 0x8c, 0xe8 },
	{ 0x8d, 0x20 },

	{ 0x0c, 0x90 },

	{ 0x2b, 0x00 },
	{ 0x22, 0x7f },
	{ 0x23, 0x03 },
	{ 0x11, 0x01 },
	{ 0x0c, 0xd0 },
	{ 0x64, 0xff },
	{ 0x0d, 0x41 },

	{ 0x14, 0x41 },
	{ 0x0e, 0xcd },
	{ 0xac, 0xbf },
	{ 0x8e, 0x00 },		/* De-noise threshold */
	{ 0x0c, 0xd0 }
};
static const u8 bridge_start_vga_772x[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x40},
	{0x1d, 0x02},
	{0x1d, 0x00},
	{0x1d, 0x02},
	{0x1d, 0x58},
	{0x1d, 0x00},
	{0xc0, 0x50},
	{0xc1, 0x3c},
};
static const u8 sensor_start_vga_772x[][2] = {
	{0x12, 0x00},
	{0x17, 0x26},
	{0x18, 0xa0},
	{0x19, 0x07},
	{0x1a, 0xf0},
	{0x29, 0xa0},
	{0x2c, 0xf0},
	{0x65, 0x20},
};
static const u8 bridge_start_qvga_772x[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x40},
	{0x1d, 0x02},
	{0x1d, 0x00},
	{0x1d, 0x00},
	{0x1d, 0x96},
	{0x1d, 0x00},
	{0xc0, 0x28},
	{0xc1, 0x1e},
};
static const u8 sensor_start_qvga_772x[][2] = {
	{0x12, 0x40},
	{0x17, 0x3f},
	{0x18, 0x50},
	{0x19, 0x03},
	{0x1a, 0x78},
	{0x29, 0x50},
	{0x2c, 0x78},
	{0x65, 0x2f},
};
#endif

static void ov534_reg_write(struct gspca_dev *gspca_dev, u16 reg, u8 val)
{
	struct usb_device *udev = gspca_dev->dev;
	int ret;

	if (gspca_dev->usb_err < 0)
		return;

	PDEBUG(D_USBO, "SET 01 0000 %04x %02x", reg, val);
	gspca_dev->usb_buf[0] = val;
	ret = usb_control_msg(udev,
			      usb_sndctrlpipe(udev, 0),
			      0x01,
			      USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      0x00, reg, gspca_dev->usb_buf, 1, CTRL_TIMEOUT);
	if (ret < 0) {
		pr_err("write failed %d\n", ret);
		gspca_dev->usb_err = ret;
	}
}

static u8 ov534_reg_read(struct gspca_dev *gspca_dev, u16 reg)
{
	struct usb_device *udev = gspca_dev->dev;
	int ret;

	if (gspca_dev->usb_err < 0)
		return 0;
	ret = usb_control_msg(udev,
			      usb_rcvctrlpipe(udev, 0),
			      0x01,
			      USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			      0x00, reg, gspca_dev->usb_buf, 1, CTRL_TIMEOUT);
	PDEBUG(D_USBI, "GET 01 0000 %04x %02x", reg, gspca_dev->usb_buf[0]);
	if (ret < 0) {
		pr_err("read failed %d\n", ret);
		gspca_dev->usb_err = ret;
	}
	return gspca_dev->usb_buf[0];
}

/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
static void ov534_set_led(struct gspca_dev *gspca_dev, int status)
{
	u8 data;

	PDEBUG(D_CONF, "led status: %d", status);

	data = ov534_reg_read(gspca_dev, 0x21);
	data |= 0x80;
	ov534_reg_write(gspca_dev, 0x21, data);

	data = ov534_reg_read(gspca_dev, 0x23);
	if (status)
		data |= 0x80;
	else
		data &= ~0x80;

	ov534_reg_write(gspca_dev, 0x23, data);

	if (!status) {
		data = ov534_reg_read(gspca_dev, 0x21);
		data &= ~0x80;
		ov534_reg_write(gspca_dev, 0x21, data);
	}
}

static int sccb_check_status(struct gspca_dev *gspca_dev)
{
	u8 data;
	int i;

	for (i = 0; i < 5; i++) {
		msleep(10);
		data = ov534_reg_read(gspca_dev, OV534_REG_STATUS);

		switch (data) {
		case 0x00:
			return 1;
		case 0x04:
			return 0;
		case 0x03:
			break;
		default:
			PERR("sccb status 0x%02x, attempt %d/5",
			       data, i + 1);
		}
	}
	return 0;
}

static void sccb_reg_write(struct gspca_dev *gspca_dev, u8 reg, u8 val)
{
	PDEBUG(D_USBO, "sccb write: %02x %02x", reg, val);
	ov534_reg_write(gspca_dev, OV534_REG_SUBADDR, reg);
	ov534_reg_write(gspca_dev, OV534_REG_WRITE, val);
	ov534_reg_write(gspca_dev, OV534_REG_OPERATION, OV534_OP_WRITE_3);

	if (!sccb_check_status(gspca_dev)) {
		pr_err("sccb_reg_write failed\n");
		gspca_dev->usb_err = -EIO;
	}
}

static u8 sccb_reg_read(struct gspca_dev *gspca_dev, u16 reg)
{
	ov534_reg_write(gspca_dev, OV534_REG_SUBADDR, reg);
	ov534_reg_write(gspca_dev, OV534_REG_OPERATION, OV534_OP_WRITE_2);
	if (!sccb_check_status(gspca_dev))
		pr_err("sccb_reg_read failed 1\n");

	ov534_reg_write(gspca_dev, OV534_REG_OPERATION, OV534_OP_READ_2);
	if (!sccb_check_status(gspca_dev))
		pr_err("sccb_reg_read failed 2\n");

	return ov534_reg_read(gspca_dev, OV534_REG_READ);
}

/* output a bridge sequence (reg - val) */
static void reg_w_array(struct gspca_dev *gspca_dev,
			const u8 (*data)[2], int len)
{
	while (--len >= 0) {
		ov534_reg_write(gspca_dev, (*data)[0], (*data)[1]);
		data++;
	}
}

/* output a sensor sequence (reg - val) */
static void sccb_w_array(struct gspca_dev *gspca_dev,
			const u8 (*data)[2], int len)
{
	while (--len >= 0) {
		if ((*data)[0] != 0xff) {
			sccb_reg_write(gspca_dev, (*data)[0], (*data)[1]);
		} else {
			sccb_reg_read(gspca_dev, (*data)[1]);
			sccb_reg_write(gspca_dev, 0xff, 0x00);
		}
		data++;
	}
}

/* ov772x specific controls */
static void set_frame_rate(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i;
	struct rate_s {
		double fps;
		__u32 fps_numerator;
		__u32 fps_denominator;
		u8 r11;
		u8 r0d;
		u8 re5;
	};
	const struct rate_s *r;
	static const struct rate_s rate_0[] = {	/* 640x480 */
		{75, 1, 75, 0x01, 0x81, 0x02}, /* 75 FPS or below: video is valid */
		{60, 1, 60, 0x00, 0x41, 0x04},
		{50, 1, 50, 0x01, 0x41, 0x02},
		{40, 1, 40, 0x02, 0xc1, 0x04},
		{30, 1, 30, 0x04, 0x81, 0x02},
		{25, 1, 25, 0x00, 0x01, 0x02},
		{20, 1, 20, 0x04, 0x41, 0x02},
		{15, 1, 15, 0x09, 0x81, 0x02},
		{10, 1, 10, 0x09, 0x41, 0x02},
		{8, 1, 8, 0x02, 0x01, 0x02},
		{5, 1, 5, 0x04, 0x01, 0x02},
		{3, 1, 3, 0x06, 0x01, 0x02},
		{2, 1, 2, 0x09, 0x01, 0x02},
		{1, 1, 1, 0x18, 0x01, 0x02},
		{0.9, 10, 9, 0x31, 0x81, 0x09},
		{0.8, 10, 8, 0x2e, 0x41, 0x07},
		{0.7, 10, 7, 0x3d, 0x41, 0x06},
		{0.6, 10, 6, 0x18, 0x01, 0x04},
		{0.5, 10, 5, 0x31, 0x01, 0x02},
		{0.4, 10, 4, 0x2e, 0x01, 0x03},
		{0.3, 10, 3, 0x31, 0x01, 0x04},
		{0.2, 10, 2, 0x2b, 0x01, 0x08},
		{0.1, 10, 1, 0x3f, 0x01, 0x0a},
	};
	static const struct rate_s rate_1[] = {	/* 320x240 */
		{187, 1, 187, 0x01, 0x81, 0x02}, /* 187 FPS or below: video is valid */
		{150, 1, 150, 0x00, 0x41, 0x04},
		{137, 1, 137, 0x02, 0xc1, 0x02},
		{125, 1, 125, 0x01, 0x41, 0x02},
		{100, 1, 100, 0x02, 0xc1, 0x04},
		{90, 1, 90, 0x03, 0x81, 0x02},
		{75, 1, 75, 0x04, 0x81, 0x02},
		{60, 1, 60, 0x04, 0xc1, 0x04},
		{50, 1, 50, 0x04, 0x41, 0x02},
		{40, 1, 40, 0x06, 0x81, 0x03},
		{37, 1, 37, 0x00, 0x01, 0x04},
		{30, 1, 30, 0x04, 0x41, 0x04},
		{17, 1, 17, 0x18, 0xc1, 0x02},
		{15, 1, 15, 0x18, 0x81, 0x02},
		{12, 1, 12, 0x02, 0x01, 0x04},
		{10, 1, 10, 0x18, 0x41, 0x02},
		{7, 1, 7, 0x04, 0x01, 0x04},
		{5, 1, 5, 0x06, 0x01, 0x04},
		{3, 1, 3, 0x09, 0x01, 0x04},
		{2, 1, 2, 0x18, 0x01, 0x02},
	};

	if (sd->sensor != SENSOR_OV772x)
		return;
	if (gspca_dev->cam.cam_mode[gspca_dev->curr_mode].priv == 0) {
		r = rate_0;
		i = ARRAY_SIZE(rate_0);
	} else {
		r = rate_1;
		i = ARRAY_SIZE(rate_1);
	}
	while (--i > 0) {
		if (sd->frame_rate_numerator == r->fps_numerator &&
			sd->frame_rate_denominator == r->fps_denominator)
			break;
		r++;
	}

	sccb_reg_write(gspca_dev, 0x11, r->r11);
	sccb_reg_write(gspca_dev, 0x0d, r->r0d);
	ov534_reg_write(gspca_dev, 0xe5, r->re5);

	PDEBUG(D_PROBE, "frame_rate: %f", r->fps);
}

static void sethue(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x) {
		/* TBD */
	} else {
		s16 huesin;
		s16 huecos;

		/* According to the datasheet the registers expect HUESIN and
		 * HUECOS to be the result of the trigonometric functions,
		 * scaled by 0x80.
		 *
		 * The 0x7fff here represents the maximum absolute value
		 * returned byt fixp_sin and fixp_cos, so the scaling will
		 * consider the result like in the interval [-1.0, 1.0].
		 */
		huesin = fixp_sin16(val) * 0x80 / 0x7fff;
		huecos = fixp_cos16(val) * 0x80 / 0x7fff;

		if (huesin < 0) {
			sccb_reg_write(gspca_dev, 0xab,
				sccb_reg_read(gspca_dev, 0xab) | 0x2);
			huesin = -huesin;
		} else {
			sccb_reg_write(gspca_dev, 0xab,
				sccb_reg_read(gspca_dev, 0xab) & ~0x2);

		}
		sccb_reg_write(gspca_dev, 0xa9, (u8)huecos);
		sccb_reg_write(gspca_dev, 0xaa, (u8)huesin);
	}
}

static void setsaturation(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x) {
		int i;
		static u8 color_tb[][6] = {
			{0x42, 0x42, 0x00, 0x11, 0x30, 0x41},
			{0x52, 0x52, 0x00, 0x16, 0x3c, 0x52},
			{0x66, 0x66, 0x00, 0x1b, 0x4b, 0x66},
			{0x80, 0x80, 0x00, 0x22, 0x5e, 0x80},
			{0x9a, 0x9a, 0x00, 0x29, 0x71, 0x9a},
			{0xb8, 0xb8, 0x00, 0x31, 0x87, 0xb8},
			{0xdd, 0xdd, 0x00, 0x3b, 0xa2, 0xdd},
		};

		for (i = 0; i < ARRAY_SIZE(color_tb[0]); i++)
			sccb_reg_write(gspca_dev, 0x4f + i, color_tb[val][i]);
	} else {
		sccb_reg_write(gspca_dev, 0xa7, val); /* U saturation */
		sccb_reg_write(gspca_dev, 0xa8, val); /* V saturation */
	}
}

static void setbrightness(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x) {
		if (val < 0)
			val = 0x80 - val;
		sccb_reg_write(gspca_dev, 0x55, val);	/* bright */
	} else {
		sccb_reg_write(gspca_dev, 0x9b, val);
	}
}

static void setcontrast(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x)
		sccb_reg_write(gspca_dev, 0x56, val);	/* contras */
	else
		sccb_reg_write(gspca_dev, 0x9c, val);
}

static void setgain(struct gspca_dev *gspca_dev, s32 val)
{
	switch (val & 0x30) {
	case 0x00:
		val &= 0x0f;
		break;
	case 0x10:
		val &= 0x0f;
		val |= 0x30;
		break;
	case 0x20:
		val &= 0x0f;
		val |= 0x70;
		break;
	default:
/*	case 0x30: */
		val &= 0x0f;
		val |= 0xf0;
		break;
	}
	sccb_reg_write(gspca_dev, 0x00, val);
}

static s32 getgain(struct gspca_dev *gspca_dev)
{
	return sccb_reg_read(gspca_dev, 0x00);
}

static void setexposure(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x) {

		/* set only aec[9:2] */
		sccb_reg_write(gspca_dev, 0x10, val);	/* aech */
	} else {

		/* 'val' is one byte and represents half of the exposure value
		 * we are going to set into registers, a two bytes value:
		 *
		 *    MSB: ((u16) val << 1) >> 8   == val >> 7
		 *    LSB: ((u16) val << 1) & 0xff == val << 1
		 */
		sccb_reg_write(gspca_dev, 0x08, val >> 7);
		sccb_reg_write(gspca_dev, 0x10, val << 1);
	}
}

static s32 getexposure(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor == SENSOR_OV767x) {
		/* get only aec[9:2] */
		return sccb_reg_read(gspca_dev, 0x10);	/* aech */
	} else {
		u8 hi = sccb_reg_read(gspca_dev, 0x08);
		u8 lo = sccb_reg_read(gspca_dev, 0x10);
		return (hi << 8 | lo) >> 1;
	}
}

static void setagc(struct gspca_dev *gspca_dev, s32 val)
{
	if (val) {
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) | 0x04);
		sccb_reg_write(gspca_dev, 0x64,
				sccb_reg_read(gspca_dev, 0x64) | 0x03);
	} else {
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) & ~0x04);
		sccb_reg_write(gspca_dev, 0x64,
				sccb_reg_read(gspca_dev, 0x64) & ~0x03);
	}
}

static void setawb(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (val) {
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) | 0x02);
		if (sd->sensor == SENSOR_OV772x)
			sccb_reg_write(gspca_dev, 0x63,
				sccb_reg_read(gspca_dev, 0x63) | 0xc0);
	} else {
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) & ~0x02);
		if (sd->sensor == SENSOR_OV772x)
			sccb_reg_write(gspca_dev, 0x63,
				sccb_reg_read(gspca_dev, 0x63) & ~0xc0);
	}
}

static void setaec(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 data;

	data = sd->sensor == SENSOR_OV767x ?
			0x05 :		/* agc + aec */
			0x01;		/* agc */
	switch (val) {
	case V4L2_EXPOSURE_AUTO:
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) | data);
		break;
	case V4L2_EXPOSURE_MANUAL:
		sccb_reg_write(gspca_dev, 0x13,
				sccb_reg_read(gspca_dev, 0x13) & ~data);
		break;
	}
}

static void setsharpness(struct gspca_dev *gspca_dev, s32 val)
{
	sccb_reg_write(gspca_dev, 0x91, val);	/* Auto de-noise threshold */
	sccb_reg_write(gspca_dev, 0x8e, val);	/* De-noise threshold */
}

static void sethvflip(struct gspca_dev *gspca_dev, s32 hflip, s32 vflip)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 val;

	if (sd->sensor == SENSOR_OV767x) {
		val = sccb_reg_read(gspca_dev, 0x1e);	/* mvfp */
		val &= ~0x30;
		if (hflip)
			val |= 0x20;
		if (vflip)
			val |= 0x10;
		sccb_reg_write(gspca_dev, 0x1e, val);
	} else {
		val = sccb_reg_read(gspca_dev, 0x0c);
		val &= ~0xc0;
		if (hflip == 0)
			val |= 0x40;
		if (vflip == 0)
			val |= 0x80;
		sccb_reg_write(gspca_dev, 0x0c, val);
	}
}

static void setlightfreq(struct gspca_dev *gspca_dev, s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	val = val ? 0x9e : 0x00;
	if (sd->sensor == SENSOR_OV767x) {
		sccb_reg_write(gspca_dev, 0x2a, 0x00);
		if (val)
			val = 0x9d;	/* insert dummy to 25fps for 50Hz */
	}
	sccb_reg_write(gspca_dev, 0x2b, val);
}

static void setgamma(struct gspca_dev *gspca_dev, s32 val)
{
static const u8 gamma_table[][16] = {
{0,   0,   0,   0,   0,   0,   1,   1,   2,   2,   5,   9,   26,  58,  113, 190},
{0,   0,   0,   0,   0,   0,   1,   1,   2,   3,   6,   11,  29,  63,  118, 184},
{0,   0,   0,   0,   0,   1,   1,   2,   3,   4,   8,   13,  33,  68,  123, 177},
{0,   0,   0,   0,   0,   1,   1,   2,   3,   5,   9,   16,  37,  72,  128, 170},
{0,   0,   0,   0,   1,   1,   2,   3,   4,   6,   11,  18,  40,  77,  132, 165},
{0,   0,   0,   0,   1,   1,   2,   4,   5,   7,   12,  20,  44,  81,  136, 160},
{0,   0,   0,   1,   1,   2,   3,   4,   6,   8,   14,  22,  47,  85,  140, 154},
{0,   0,   0,   1,   1,   2,   3,   5,   7,   10,  16,  25,  51,  89,  143, 150},
{0,   0,   0,   1,   2,   3,   4,   6,   8,   11,  18,  27,  54,  93,  147, 145},
{0,   0,   0,   1,   2,   3,   5,   7,   9,   12,  20,  30,  57,  97,  150, 141},
{0,   0,   0,   1,   2,   4,   6,   8,   11,  14,  22,  32,  60,  100, 153, 137},
{0,   0,   0,   2,   3,   4,   6,   9,   12,  15,  24,  35,  64,  104, 155, 134},
{0,   0,   0,   2,   3,   5,   7,   10,  13,  17,  26,  37,  67,  107, 158, 130},
{0,   0,   0,   2,   4,   6,   8,   11,  15,  18,  28,  40,  70,  110, 161, 126},
{0,   0,   1,   3,   4,   7,   9,   12,  16,  20,  30,  42,  73,  113, 163, 124},
{0,   0,   1,   3,   5,   7,   10,  14,  17,  22,  32,  44,  76,  116, 165, 121},
{0,   0,   1,   4,   6,   8,   11,  15,  19,  23,  34,  47,  79,  119, 168, 117},
{0,   0,   1,   4,   6,   9,   12,  16,  20,  25,  36,  49,  81,  121, 170, 114},
{0,   0,   1,   5,   7,   10,  13,  17,  22,  27,  38,  52,  84,  124, 172, 112},
{0,   0,   1,   5,   8,   11,  15,  19,  23,  29,  40,  54,  87,  127, 174, 109},
{0,   0,   2,   6,   8,   12,  16,  20,  25,  30,  42,  56,  89,  129, 175, 108},
{0,   1,   2,   6,   9,   13,  17,  21,  27,  32,  44,  58,  92,  131, 177, 105},
{0,   1,   2,   7,   10,  14,  18,  23,  28,  34,  46,  61,  94,  134, 179, 102},
{0,   1,   2,   7,   11,  15,  19,  24,  30,  36,  48,  63,  97,  136, 180, 101},
{0,   1,   3,   8,   12,  16,  21,  26,  31,  37,  50,  65,  99,  138, 182, 98},
{0,   1,   3,   9,   13,  17,  22,  27,  33,  39,  52,  67,  101, 140, 183, 97},
{0,   1,   3,   10,  14,  18,  23,  29,  35,  41,  54,  69,  103, 142, 185, 94},
{0,   1,   4,   10,  15,  19,  24,  30,  36,  43,  56,  72,  105, 144, 186, 93},
{0,   1,   4,   11,  16,  20,  26,  32,  38,  44,  58,  74,  108, 146, 187, 92},
{1,   2,   4,   12,  16,  22,  27,  33,  39,  46,  60,  76,  110, 147, 189, 89},
{1,   2,   5,   13,  17,  23,  28,  35,  41,  48,  62,  78,  112, 149, 190, 88},
{1,   2,   5,   13,  18,  24,  30,  36,  43,  49,  64,  79,  113, 151, 191, 86},
{1,   2,   5,   14,  20,  25,  31,  37,  44,  51,  66,  81,  115, 152, 192, 85},
{1,   2,   6,   15,  21,  26,  32,  39,  46,  53,  68,  83,  117, 154, 193, 84},
{1,   3,   6,   16,  22,  28,  34,  40,  47,  54,  69,  85,  119, 156, 194, 82},
{1,   3,   7,   17,  23,  29,  35,  42,  49,  56,  71,  87,  121, 157, 195, 81},
{1,   3,   7,   18,  24,  30,  36,  43,  50,  58,  73,  89,  122, 158, 196, 80},
{1,   3,   8,   19,  25,  31,  38,  45,  52,  59,  75,  90,  124, 160, 197, 78},
{1,   4,   8,   20,  26,  32,  39,  46,  53,  61,  76,  92,  126, 161, 198, 77},
{2,   4,   9,   21,  27,  34,  40,  48,  55,  62,  78,  94,  127, 163, 199, 76},
{2,   4,   9,   21,  28,  35,  42,  49,  56,  64,  80,  96,  129, 164, 200, 74},
{2,   4,   10,  22,  29,  36,  43,  50,  58,  66,  81,  97,  130, 165, 201, 73},
{2,   5,   10,  23,  30,  37,  44,  52,  59,  67,  83,  99,  132, 166, 202, 72},
{2,   5,   11,  24,  31,  38,  46,  53,  61,  69,  84,  100, 133, 168, 202, 72},
{2,   5,   12,  25,  32,  40,  47,  55,  62,  70,  86,  102, 135, 169, 203, 70},
{3,   6,   12,  26,  33,  41,  48,  56,  64,  72,  87,  103, 136, 170, 204, 69},
{3,   6,   13,  27,  35,  42,  50,  57,  65,  73,  89,  105, 138, 171, 205, 68},
{3,   6,   13,  28,  36,  43,  51,  59,  67,  74,  90,  106, 139, 172, 205, 68},
{3,   7,   14,  29,  37,  44,  52,  60,  68,  76,  92,  108, 140, 173, 206, 66},
{4,   7,   15,  30,  38,  46,  53,  61,  69,  77,  93,  109, 142, 174, 207, 65},
{4,   8,   15,  31,  39,  47,  55,  63,  71,  79,  95,  111, 143, 175, 207, 65},
{4,   8,   16,  32,  40,  48,  56,  64,  72,  80,  96,  112, 144, 176, 208, 64},
{4,   8,   17,  33,  41,  49,  57,  65,  73,  81,  97,  113, 145, 177, 209, 62},
{5,   9,   17,  34,  42,  50,  58,  67,  75,  83,  99,  115, 146, 178, 209, 62},
{5,   9,   18,  35,  43,  52,  60,  68,  76,  84,  100, 116, 148, 179, 210, 61},
{5,   10,  19,  36,  44,  53,  61,  69,  77,  85,  101, 117, 149, 180, 210, 61},
{5,   10,  19,  37,  45,  54,  62,  70,  79,  87,  103, 118, 150, 181, 211, 60},
{6,   11,  20,  38,  46,  55,  63,  72,  80,  88,  104, 120, 151, 181, 211, 60},
{6,   11,  21,  39,  48,  56,  65,  73,  81,  89,  105, 121, 152, 182, 212, 58},
{6,   12,  21,  40,  49,  57,  66,  74,  82,  90,  106, 122, 153, 183, 213, 57},
{7,   12,  22,  41,  50,  58,  67,  75,  83,  92,  108, 123, 154, 184, 213, 57},
{7,   12,  23,  42,  51,  59,  68,  76,  85,  93,  109, 124, 155, 185, 214, 56},
{7,   13,  24,  43,  52,  61,  69,  78,  86,  94,  110, 126, 156, 185, 214, 56},
{7,   13,  24,  44,  53,  62,  70,  79,  87,  95,  111, 127, 157, 186, 214, 56},
{8,   14,  25,  45,  54,  63,  71,  80,  88,  96,  112, 128, 158, 187, 215, 54},
{8,   14,  26,  46,  55,  64,  73,  81,  89,  98,  113, 129, 159, 188, 215, 54},
{8,   15,  26,  47,  56,  65,  74,  82,  90,  99,  115, 130, 160, 188, 216, 53},
{9,   15,  27,  48,  57,  66,  75,  83,  92,  100, 116, 131, 161, 189, 216, 53},
{9,   16,  28,  48,  58,  67,  76,  84,  93,  101, 117, 132, 161, 190, 217, 52},
{10,  17,  29,  49,  59,  68,  77,  85,  94,  102, 118, 133, 162, 190, 217, 52},
{10,  17,  29,  50,  60,  69,  78,  87,  95,  103, 119, 134, 163, 191, 217, 52},
{10,  18,  30,  51,  61,  70,  79,  88,  96,  104, 120, 135, 164, 191, 218, 50},
{11,  18,  31,  52,  62,  71,  80,  89,  97,  105, 121, 136, 165, 192, 218, 50},
{11,  19,  31,  53,  63,  72,  81,  90,  98,  106, 122, 137, 166, 193, 219, 49},
{11,  19,  32,  54,  64,  73,  82,  91,  99,  107, 123, 138, 166, 193, 219, 49},
{12,  20,  33,  55,  65,  74,  83,  92,  100, 108, 124, 139, 167, 194, 219, 49},
{12,  20,  34,  56,  66,  75,  84,  93,  101, 109, 125, 140, 168, 194, 220, 48},
{13,  21,  34,  57,  67,  76,  85,  94,  102, 110, 126, 141, 169, 195, 220, 48},
{13,  21,  35,  58,  68,  77,  86,  95,  103, 111, 127, 142, 169, 196, 220, 48},
{13,  22,  36,  59,  69,  78,  87,  96,  104, 112, 128, 142, 170, 196, 221, 46},
{14,  23,  37,  60,  70,  79,  88,  97,  105, 113, 129, 143, 171, 197, 221, 46},
{14,  23,  37,  60,  71,  80,  89,  98,  106, 114, 129, 144, 172, 197, 221, 46},
{15,  24,  38,  61,  71,  81,  90,  99,  107, 115, 130, 145, 172, 198, 222, 45},
{15,  24,  39,  62,  72,  82,  91,  100, 108, 116, 131, 146, 173, 198, 222, 45},
{16,  25,  40,  63,  73,  83,  92,  101, 109, 117, 132, 147, 174, 199, 222, 45},
{16,  25,  40,  64,  74,  84,  93,  101, 110, 118, 133, 147, 174, 199, 223, 44},
{16,  26,  41,  65,  75,  85,  94,  102, 111, 119, 134, 148, 175, 200, 223, 44},
{17,  27,  42,  66,  76,  86,  95,  103, 112, 119, 135, 149, 175, 200, 223, 44},
{17,  27,  42,  66,  77,  86,  96,  104, 112, 120, 135, 150, 176, 201, 223, 44},
{18,  28,  43,  67,  78,  87,  96,  105, 113, 121, 136, 150, 177, 201, 224, 42},
{18,  28,  44,  68,  79,  88,  97,  106, 114, 122, 137, 151, 177, 201, 224, 42},
{19,  29,  45,  69,  79,  89,  98,  107, 115, 123, 138, 152, 178, 202, 224, 42},
{19,  29,  45,  70,  80,  90,  99,  108, 116, 124, 139, 153, 179, 202, 225, 41},
{20,  30,  46,  71,  81,  91,  100, 108, 117, 125, 139, 153, 179, 203, 225, 41},
{20,  31,  47,  72,  82,  92,  101, 109, 118, 125, 140, 154, 180, 203, 225, 41},
{20,  31,  47,  72,  83,  93,  102, 110, 118, 126, 141, 155, 180, 204, 225, 41},
{21,  32,  48,  73,  84,  93,  102, 111, 119, 127, 142, 155, 181, 204, 226, 40},
{21,  32,  49,  74,  84,  94,  103, 112, 120, 128, 142, 156, 181, 204, 226, 40},
{22,  33,  50,  75,  85,  95,  104, 113, 121, 128, 143, 157, 182, 205, 226, 40},
{22,  34,  50,  76,  86,  96,  105, 113, 122, 129, 144, 157, 182, 205, 226, 40},
{23,  34,  51,  76,  87,  97,  106, 114, 122, 130, 145, 158, 183, 206, 227, 38},
{23,  35,  52,  77,  88,  97,  106, 115, 123, 131, 145, 159, 183, 206, 227, 38},
{24,  35,  52,  78,  88,  98,  107, 116, 124, 131, 146, 159, 184, 206, 227, 38},
{24,  36,  53,  79,  89,  99,  108, 117, 125, 132, 147, 160, 184, 207, 227, 38},
{25,  36,  54,  79,  90,  100, 109, 117, 125, 133, 147, 161, 185, 207, 227, 38},
{25,  37,  54,  80,  91,  101, 110, 118, 126, 134, 148, 161, 185, 207, 228, 37},
{26,  38,  55,  81,  92,  101, 110, 119, 127, 134, 149, 162, 186, 208, 228, 37},
{26,  38,  56,  82,  92,  102, 111, 119, 127, 135, 149, 162, 186, 208, 228, 37},
{27,  39,  57,  82,  93,  103, 112, 120, 128, 136, 150, 163, 187, 208, 228, 37},
{27,  39,  57,  83,  94,  104, 113, 121, 129, 136, 151, 164, 187, 209, 228, 37},
{28,  40,  58,  84,  95,  104, 113, 122, 130, 137, 151, 164, 188, 209, 229, 36},
{28,  41,  59,  85,  95,  105, 114, 122, 130, 138, 152, 165, 188, 209, 229, 36},
{29,  41,  59,  85,  96,  106, 115, 123, 131, 138, 152, 165, 189, 210, 229, 36},
{29,  42,  60,  86,  97,  106, 115, 124, 132, 139, 153, 166, 189, 210, 229, 36},
{30,  42,  61,  87,  97,  107, 116, 124, 132, 140, 154, 166, 190, 210, 229, 36},
{30,  43,  61,  88,  98,  108, 117, 125, 133, 140, 154, 167, 190, 211, 230, 34},
{30,  43,  62,  88,  99,  109, 117, 126, 134, 141, 155, 167, 190, 211, 230, 34},
{31,  44,  63,  89,  100, 109, 118, 126, 134, 142, 155, 168, 191, 211, 230, 34},
{31,  45,  63,  90,  100, 110, 119, 127, 135, 142, 156, 168, 191, 212, 230, 34},
{32,  45,  64,  90,  101, 111, 119, 128, 135, 143, 156, 169, 192, 212, 230, 34},
{32,  46,  65,  91,  102, 111, 120, 128, 136, 143, 157, 170, 192, 212, 230, 34},
{33,  46,  65,  92,  102, 112, 121, 129, 137, 144, 158, 170, 192, 212, 231, 33},
{33,  47,  66,  92,  103, 113, 121, 130, 137, 145, 158, 171, 193, 213, 231, 33},
{34,  47,  66,  93,  104, 113, 122, 130, 138, 145, 159, 171, 193, 213, 231, 33},
{34,  48,  67,  94,  104, 114, 123, 131, 139, 146, 159, 171, 194, 213, 231, 33},
{35,  49,  68,  94,  105, 115, 123, 132, 139, 146, 160, 172, 194, 214, 231, 33},
{35,  49,  68,  95,  106, 115, 124, 132, 140, 147, 160, 172, 194, 214, 231, 33},
{36,  50,  69,  96,  106, 116, 125, 133, 140, 148, 161, 173, 195, 214, 232, 32},
{36,  50,  70,  96,  107, 117, 125, 133, 141, 148, 161, 173, 195, 214, 232, 32},
{37,  51,  70,  97,  108, 117, 126, 134, 141, 149, 162, 174, 195, 215, 232, 32},
{37,  51,  71,  98,  108, 118, 126, 135, 142, 149, 162, 174, 196, 215, 232, 32},
{38,  52,  71,  98,  109, 118, 127, 135, 143, 150, 163, 175, 196, 215, 232, 32},
{38,  53,  72,  99,  109, 119, 128, 136, 143, 150, 163, 175, 196, 215, 232, 32},
{39,  53,  73,  100, 110, 120, 128, 136, 144, 151, 164, 176, 197, 216, 233, 30},
{39,  54,  73,  100, 111, 120, 129, 137, 144, 151, 164, 176, 197, 216, 233, 30},
{40,  54,  74,  101, 111, 121, 129, 137, 145, 152, 165, 176, 197, 216, 233, 30},
{40,  55,  74,  101, 112, 121, 130, 138, 145, 152, 165, 177, 198, 216, 233, 30},
{41,  55,  75,  102, 113, 122, 131, 138, 146, 153, 166, 177, 198, 216, 233, 30},
{41,  56,  76,  103, 113, 123, 131, 139, 146, 153, 166, 178, 198, 217, 233, 30},
{42,  56,  76,  103, 114, 123, 132, 140, 147, 154, 167, 178, 199, 217, 233, 30},
{42,  57,  77,  104, 114, 124, 132, 140, 147, 154, 167, 179, 199, 217, 233, 30},
{43,  57,  77,  104, 115, 124, 133, 141, 148, 155, 167, 179, 199, 217, 234, 29},
{43,  58,  78,  105, 115, 125, 133, 141, 148, 155, 168, 179, 200, 218, 234, 29},
{44,  59,  79,  106, 116, 125, 134, 142, 149, 156, 168, 180, 200, 218, 234, 29},
{44,  59,  79,  106, 117, 126, 134, 142, 149, 156, 169, 180, 200, 218, 234, 29},
{45,  60,  80,  107, 117, 127, 135, 143, 150, 157, 169, 181, 201, 218, 234, 29},
{45,  60,  80,  107, 118, 127, 135, 143, 150, 157, 170, 181, 201, 218, 234, 29},
{46,  61,  81,  108, 118, 128, 136, 144, 151, 158, 170, 181, 201, 219, 234, 29},
{46,  61,  81,  108, 119, 128, 137, 144, 151, 158, 170, 182, 201, 219, 234, 29},
{46,  62,  82,  109, 119, 129, 137, 145, 152, 159, 171, 182, 202, 219, 235, 28},
{47,  62,  83,  110, 120, 129, 138, 145, 152, 159, 171, 182, 202, 219, 235, 28},
{47,  63,  83,  110, 120, 130, 138, 146, 153, 160, 172, 183, 202, 219, 235, 28},
{48,  63,  84,  111, 121, 130, 139, 146, 153, 160, 172, 183, 203, 220, 235, 28},
{48,  64,  84,  111, 122, 131, 139, 147, 154, 160, 173, 183, 203, 220, 235, 28},
{49,  64,  85,  112, 122, 131, 140, 147, 154, 161, 173, 184, 203, 220, 235, 28},
{49,  65,  85,  112, 123, 132, 140, 148, 155, 161, 173, 184, 203, 220, 235, 28},
{50,  65,  86,  113, 123, 132, 141, 148, 155, 162, 174, 185, 204, 220, 235, 28},
{50,  66,  86,  113, 124, 133, 141, 149, 156, 162, 174, 185, 204, 221, 235, 28},
{51,  66,  87,  114, 124, 133, 141, 149, 156, 163, 174, 185, 204, 221, 236, 26},
{51,  67,  87,  114, 125, 134, 142, 149, 156, 163, 175, 186, 204, 221, 236, 26},
{52,  67,  88,  115, 125, 134, 142, 150, 157, 163, 175, 186, 205, 221, 236, 26},
{52,  68,  89,  115, 126, 135, 143, 150, 157, 164, 176, 186, 205, 221, 236, 26},
{53,  68,  89,  116, 126, 135, 143, 151, 158, 164, 176, 187, 205, 221, 236, 26},
{53,  69,  90,  116, 127, 136, 144, 151, 158, 165, 176, 187, 205, 222, 236, 26},
{54,  69,  90,  117, 127, 136, 144, 152, 159, 165, 177, 187, 206, 222, 236, 26},
{54,  70,  91,  117, 128, 137, 145, 152, 159, 165, 177, 188, 206, 222, 236, 26},
{54,  70,  91,  118, 128, 137, 145, 153, 159, 166, 177, 188, 206, 222, 236, 26},
{55,  71,  92,  118, 129, 138, 146, 153, 160, 166, 178, 188, 206, 222, 237, 25},
{55,  71,  92,  119, 129, 138, 146, 153, 160, 167, 178, 188, 207, 223, 237, 25},
{56,  72,  93,  119, 130, 138, 147, 154, 161, 167, 178, 189, 207, 223, 237, 25},
{56,  72,  93,  120, 130, 139, 147, 154, 161, 167, 179, 189, 207, 223, 237, 25},
{57,  73,  94,  120, 130, 139, 147, 155, 161, 168, 179, 189, 207, 223, 237, 25},
{57,  73,  94,  121, 131, 140, 148, 155, 162, 168, 179, 190, 208, 223, 237, 25},
{58,  74,  95,  121, 131, 140, 148, 155, 162, 168, 180, 190, 208, 223, 237, 25},
{58,  74,  95,  122, 132, 141, 149, 156, 163, 169, 180, 190, 208, 223, 237, 25},
{59,  75,  96,  122, 132, 141, 149, 156, 163, 169, 180, 191, 208, 224, 237, 25},
{59,  75,  96,  123, 133, 142, 149, 157, 163, 169, 181, 191, 208, 224, 237, 25},
{59,  76,  97,  123, 133, 142, 150, 157, 164, 170, 181, 191, 209, 224, 237, 25},
{60,  76,  97,  124, 134, 142, 150, 157, 164, 170, 181, 191, 209, 224, 238, 24},
{60,  77,  98,  124, 134, 143, 151, 158, 164, 171, 182, 192, 209, 224, 238, 24},
{61,  77,  98,  125, 135, 143, 151, 158, 165, 171, 182, 192, 209, 224, 238, 24},
{61,  78,  99,  125, 135, 144, 152, 159, 165, 171, 182, 192, 210, 225, 238, 24},
{62,  78,  99,  125, 135, 144, 152, 159, 166, 172, 183, 193, 210, 225, 238, 24},
{62,  79,  99,  126, 136, 145, 152, 159, 166, 172, 183, 193, 210, 225, 238, 24},
{63,  79,  100, 126, 136, 145, 153, 160, 166, 172, 183, 193, 210, 225, 238, 24},
{63,  80,  100, 127, 137, 145, 153, 160, 167, 173, 184, 193, 210, 225, 238, 24},
{63,  80,  101, 127, 137, 146, 153, 160, 167, 173, 184, 194, 211, 225, 238, 24},
{64,  80,  101, 128, 138, 146, 154, 161, 167, 173, 184, 194, 211, 225, 238, 24},
{64,  81,  102, 128, 138, 147, 154, 161, 168, 174, 184, 194, 211, 225, 238, 24},
{65,  81,  102, 129, 138, 147, 155, 162, 168, 174, 185, 194, 211, 226, 238, 24},
{65,  82,  103, 129, 139, 147, 155, 162, 168, 174, 185, 195, 211, 226, 238, 24},
{66,  82,  103, 129, 139, 148, 155, 162, 169, 175, 185, 195, 212, 226, 239, 22},
{66,  83,  104, 130, 140, 148, 156, 163, 169, 175, 186, 195, 212, 226, 239, 22},
{66,  83,  104, 130, 140, 148, 156, 163, 169, 175, 186, 195, 212, 226, 239, 22},
{67,  84,  104, 131, 140, 149, 156, 163, 170, 176, 186, 196, 212, 226, 239, 22},
{67,  84,  105, 131, 141, 149, 157, 164, 170, 176, 186, 196, 212, 226, 239, 22},
{68,  84,  105, 131, 141, 150, 157, 164, 170, 176, 187, 196, 212, 227, 239, 22},
{68,  85,  106, 132, 142, 150, 158, 164, 171, 176, 187, 196, 213, 227, 239, 22},
{69,  85,  106, 132, 142, 150, 158, 165, 171, 177, 187, 197, 213, 227, 239, 22},
{69,  86,  107, 133, 142, 151, 158, 165, 171, 177, 187, 197, 213, 227, 239, 22},
{69,  86,  107, 133, 143, 151, 159, 165, 172, 177, 188, 197, 213, 227, 239, 22},
{70,  87,  108, 133, 143, 151, 159, 166, 172, 178, 188, 197, 213, 227, 239, 22},
{70,  87,  108, 134, 143, 152, 159, 166, 172, 178, 188, 198, 214, 227, 239, 22},
{71,  87,  108, 134, 144, 152, 160, 166, 172, 178, 189, 198, 214, 227, 239, 22},
{71,  88,  109, 135, 144, 153, 160, 167, 173, 178, 189, 198, 214, 228, 240, 21},
{71,  88,  109, 135, 145, 153, 160, 167, 173, 179, 189, 198, 214, 228, 240, 21},
{72,  89,  110, 135, 145, 153, 161, 167, 173, 179, 189, 198, 214, 228, 240, 21},
{72,  89,  110, 136, 145, 154, 161, 168, 174, 179, 190, 199, 214, 228, 240, 21},
{73,  90,  110, 136, 146, 154, 161, 168, 174, 180, 190, 199, 215, 228, 240, 21},
{73,  90,  111, 137, 146, 154, 162, 168, 174, 180, 190, 199, 215, 228, 240, 21},
{73,  90,  111, 137, 146, 155, 162, 169, 175, 180, 190, 199, 215, 228, 240, 21},
{74,  91,  112, 137, 147, 155, 162, 169, 175, 180, 191, 200, 215, 228, 240, 21},
{74,  91,  112, 138, 147, 155, 163, 169, 175, 181, 191, 200, 215, 228, 240, 21},
{75,  92,  112, 138, 147, 156, 163, 169, 175, 181, 191, 200, 215, 229, 240, 21},
{75,  92,  113, 138, 148, 156, 163, 170, 176, 181, 191, 200, 216, 229, 240, 21},
{75,  92,  113, 139, 148, 156, 164, 170, 176, 182, 192, 200, 216, 229, 240, 21},
{76,  93,  114, 139, 149, 157, 164, 170, 176, 182, 192, 201, 216, 229, 240, 21},
{76,  93,  114, 140, 149, 157, 164, 171, 177, 182, 192, 201, 216, 229, 240, 21},
{77,  94,  114, 140, 149, 157, 164, 171, 177, 182, 192, 201, 216, 229, 240, 21},
{77,  94,  115, 140, 150, 158, 165, 171, 177, 183, 192, 201, 216, 229, 240, 21},
{77,  94,  115, 141, 150, 158, 165, 172, 177, 183, 193, 201, 216, 229, 241, 20},
{78,  95,  116, 141, 150, 158, 165, 172, 178, 183, 193, 202, 217, 229, 241, 20},
{78,  95,  116, 141, 151, 159, 166, 172, 178, 183, 193, 202, 217, 229, 241, 20},
{79,  96,  116, 142, 151, 159, 166, 172, 178, 184, 193, 202, 217, 230, 241, 20},
{79,  96,  117, 142, 151, 159, 166, 173, 178, 184, 194, 202, 217, 230, 241, 20},
{79,  96,  117, 142, 152, 159, 167, 173, 179, 184, 194, 202, 217, 230, 241, 20},
{80,  97,  118, 143, 152, 160, 167, 173, 179, 184, 194, 203, 217, 230, 241, 20},
{80,  97,  118, 143, 152, 160, 167, 173, 179, 185, 194, 203, 217, 230, 241, 20},
{80,  98,  118, 143, 152, 160, 167, 174, 180, 185, 194, 203, 218, 230, 241, 20},
{81,  98,  119, 144, 153, 161, 168, 174, 180, 185, 195, 203, 218, 230, 241, 20},
{81,  98,  119, 144, 153, 161, 168, 174, 180, 185, 195, 203, 218, 230, 241, 20},
{82,  99,  119, 144, 153, 161, 168, 175, 180, 186, 195, 204, 218, 230, 241, 20},
{82,  99,  120, 145, 154, 162, 169, 175, 181, 186, 195, 204, 218, 230, 241, 20},
{82,  99,  120, 145, 154, 162, 169, 175, 181, 186, 195, 204, 218, 231, 241, 20},
{83,  100, 120, 145, 154, 162, 169, 175, 181, 186, 196, 204, 218, 231, 241, 20},
{83,  100, 121, 146, 155, 162, 169, 176, 181, 187, 196, 204, 219, 231, 241, 20},
{83,  101, 121, 146, 155, 163, 170, 176, 182, 187, 196, 204, 219, 231, 241, 20},
{84,  101, 122, 146, 155, 163, 170, 176, 182, 187, 196, 205, 219, 231, 241, 20},
{84,  101, 122, 147, 156, 163, 170, 176, 182, 187, 197, 205, 219, 231, 242, 18},
{85,  102, 122, 147, 156, 164, 170, 177, 182, 187, 197, 205, 219, 231, 242, 18},
{85,  102, 123, 147, 156, 164, 171, 177, 182, 188, 197, 205, 219, 231, 242, 18},
{85,  102, 123, 148, 156, 164, 171, 177, 183, 188, 197, 205, 219, 231, 242, 18},
{86,  103, 123, 148, 157, 164, 171, 177, 183, 188, 197, 205, 219, 231, 242, 18},
{86,  103, 124, 148, 157, 165, 172, 178, 183, 188, 198, 206, 220, 231, 242, 18},
{86,  103, 124, 148, 157, 165, 172, 178, 183, 189, 198, 206, 220, 232, 242, 18},
{87,  104, 124, 149, 158, 165, 172, 178, 184, 189, 198, 206, 220, 232, 242, 18},
{87,  104, 125, 149, 158, 166, 172, 178, 184, 189, 198, 206, 220, 232, 242, 18},
{87,  105, 125, 149, 158, 166, 173, 179, 184, 189, 198, 206, 220, 232, 242, 18},
{88,  105, 125, 150, 159, 166, 173, 179, 184, 189, 198, 206, 220, 232, 242, 18},
{88,  105, 126, 150, 159, 166, 173, 179, 185, 190, 199, 207, 220, 232, 242, 18},
{89,  106, 126, 150, 159, 167, 173, 179, 185, 190, 199, 207, 220, 232, 242, 18},
{89,  106, 126, 151, 159, 167, 174, 180, 185, 190, 199, 207, 221, 232, 242, 18},
{89,  106, 127, 151, 160, 167, 174, 180, 185, 190, 199, 207, 221, 232, 242, 18},
{90,  107, 127, 151, 160, 167, 174, 180, 185, 190, 199, 207, 221, 232, 242, 18},
{90,  107, 127, 151, 160, 168, 174, 180, 186, 191, 200, 207, 221, 232, 242, 18},
{90,  107, 128, 152, 160, 168, 175, 180, 186, 191, 200, 208, 221, 232, 242, 18},
};
	static const u8 registers[] = {0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d};
	struct sd *sd = (struct sd *) gspca_dev;
	u8 i;

	if (sd->sensor == SENSOR_OV767x) return;
	for (i=0; i<16; ++i)
	{
		sccb_reg_write(gspca_dev, registers[i], gamma_table[val][i]);
	}
	return;
}


/* this function is called at probe time */
static int sd_config(struct gspca_dev *gspca_dev,
		     const struct usb_device_id *id)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam;

	cam = &gspca_dev->cam;

	cam->cam_mode = ov772x_mode;
	cam->nmodes = ARRAY_SIZE(ov772x_mode);

	sd->frame_rate_numerator = 1;
	sd->frame_rate_denominator = DEFAULT_FRAME_RATE;

	return 0;
}

static int ov534_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sd *sd = container_of(ctrl->handler, struct sd, ctrl_handler);
	struct gspca_dev *gspca_dev = &sd->gspca_dev;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		gspca_dev->usb_err = 0;
		if (ctrl->val && sd->gain && gspca_dev->streaming)
			sd->gain->val = getgain(gspca_dev);
		return gspca_dev->usb_err;

	case V4L2_CID_EXPOSURE_AUTO:
		gspca_dev->usb_err = 0;
		if (ctrl->val == V4L2_EXPOSURE_AUTO && sd->exposure &&
		    gspca_dev->streaming)
			sd->exposure->val = getexposure(gspca_dev);
		return gspca_dev->usb_err;
	}
	return -EINVAL;
}

static int ov534_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sd *sd = container_of(ctrl->handler, struct sd, ctrl_handler);
	struct gspca_dev *gspca_dev = &sd->gspca_dev;

	gspca_dev->usb_err = 0;
	if (!gspca_dev->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HUE:
		sethue(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		setsaturation(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_BRIGHTNESS:
		setbrightness(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		setcontrast(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_AUTOGAIN:
	/* case V4L2_CID_GAIN: */
		setagc(gspca_dev, ctrl->val);
		if (!gspca_dev->usb_err && !ctrl->val && sd->gain)
			setgain(gspca_dev, sd->gain->val);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		setawb(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
	/* case V4L2_CID_EXPOSURE: */
		setaec(gspca_dev, ctrl->val);
		if (!gspca_dev->usb_err && ctrl->val == V4L2_EXPOSURE_MANUAL &&
		    sd->exposure)
			setexposure(gspca_dev, sd->exposure->val);
		break;
	case V4L2_CID_SHARPNESS:
		setsharpness(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		sethvflip(gspca_dev, ctrl->val, sd->vflip->val);
		break;
	case V4L2_CID_VFLIP:
		sethvflip(gspca_dev, sd->hflip->val, ctrl->val);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		setlightfreq(gspca_dev, ctrl->val);
		break;
	case V4L2_CID_GAMMA:
		setgamma(gspca_dev, ctrl->val);
		break;
	}
	return gspca_dev->usb_err;
}

static const struct v4l2_ctrl_ops ov534_ctrl_ops = {
	.g_volatile_ctrl = ov534_g_volatile_ctrl,
	.s_ctrl = ov534_s_ctrl,
};

static int sd_init_controls(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct v4l2_ctrl_handler *hdl = &sd->ctrl_handler;
	/* parameters with different values between the supported sensors */
	int saturation_min;
	int saturation_max;
	int saturation_def;
	int brightness_min;
	int brightness_max;
	int brightness_def;
	int contrast_max;
	int contrast_def;
	int exposure_min;
	int exposure_max;
	int exposure_def;
	int hflip_def;
	int gamma;

	if (sd->sensor == SENSOR_OV767x) {
		saturation_min = 0,
		saturation_max = 6,
		saturation_def = 3,
		brightness_min = -127;
		brightness_max = 127;
		brightness_def = 0;
		contrast_max = 0x80;
		contrast_def = 0x40;
		exposure_min = 0x08;
		exposure_max = 0x60;
		exposure_def = 0x13;
		hflip_def = 1;
	} else {
		saturation_min = 0,
		saturation_max = 255,
		saturation_def = 64,
		brightness_min = 0;
		brightness_max = 255;
		brightness_def = 0;
		contrast_max = 255;
		contrast_def = 32;
		exposure_min = 0;
		exposure_max = 255;
		exposure_def = 120;
		hflip_def = 0;
		gamma = 128;
	}

	gspca_dev->vdev.ctrl_handler = hdl;

	v4l2_ctrl_handler_init(hdl, 13);

	if (sd->sensor == SENSOR_OV772x)
		sd->hue = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
				V4L2_CID_HUE, -90, 90, 1, 0);

	sd->saturation = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_SATURATION, saturation_min, saturation_max, 1,
			saturation_def);
	sd->brightness = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_BRIGHTNESS, brightness_min, brightness_max, 1,
			brightness_def);
	sd->contrast = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_CONTRAST, 0, contrast_max, 1, contrast_def);

	if (sd->sensor == SENSOR_OV772x) {
		sd->autogain = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
				V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
		sd->gain = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
				V4L2_CID_GAIN, 0, 63, 1, 20);
	}

	sd->autoexposure = v4l2_ctrl_new_std_menu(hdl, &ov534_ctrl_ops,
			V4L2_CID_EXPOSURE_AUTO,
			V4L2_EXPOSURE_MANUAL, 0,
			V4L2_EXPOSURE_AUTO);
	sd->exposure = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_EXPOSURE, exposure_min, exposure_max, 1,
			exposure_def);

	sd->autowhitebalance = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);

	if (sd->sensor == SENSOR_OV772x)
		sd->sharpness = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
				V4L2_CID_SHARPNESS, 0, 63, 1, 0);

	sd->hflip = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, hflip_def);
	sd->vflip = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	sd->plfreq = v4l2_ctrl_new_std_menu(hdl, &ov534_ctrl_ops,
			V4L2_CID_POWER_LINE_FREQUENCY,
			V4L2_CID_POWER_LINE_FREQUENCY_50HZ, 0,
			V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);
	sd->gamma = v4l2_ctrl_new_std(hdl, &ov534_ctrl_ops,
			V4L2_CID_GAMMA, 0, 255, 1, 128);

	if (hdl->error) {
		pr_err("Could not initialize controls\n");
		return hdl->error;
	}

	if (sd->sensor == SENSOR_OV772x)
		v4l2_ctrl_auto_cluster(2, &sd->autogain, 0, true);

	v4l2_ctrl_auto_cluster(2, &sd->autoexposure, V4L2_EXPOSURE_MANUAL,
			       true);

	return 0;
}

/* this function is called at probe and resume time */
static int sd_init(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u16 sensor_id;
	static const struct reg_array bridge_init[NSENSORS] = {
	[SENSOR_OV767x] = {bridge_init_767x, ARRAY_SIZE(bridge_init_767x)},
	[SENSOR_OV772x] = {bridge_init_772x, ARRAY_SIZE(bridge_init_772x)},
	};
	static const struct reg_array sensor_init[NSENSORS] = {
	[SENSOR_OV767x] = {sensor_init_767x, ARRAY_SIZE(sensor_init_767x)},
	[SENSOR_OV772x] = {sensor_init_772x, ARRAY_SIZE(sensor_init_772x)},
	};

	/* reset bridge */
	ov534_reg_write(gspca_dev, 0xe7, 0x3a);
	ov534_reg_write(gspca_dev, 0xe0, 0x08);
	msleep(100);

	/* initialize the sensor address */
	ov534_reg_write(gspca_dev, OV534_REG_ADDRESS, 0x42);

	/* reset sensor */
	sccb_reg_write(gspca_dev, 0x12, 0x80);
	msleep(10);

	/* probe the sensor */
	sccb_reg_read(gspca_dev, 0x0a);
	sensor_id = sccb_reg_read(gspca_dev, 0x0a) << 8;
	sccb_reg_read(gspca_dev, 0x0b);
	sensor_id |= sccb_reg_read(gspca_dev, 0x0b);
	PDEBUG(D_PROBE, "Sensor ID: %04x", sensor_id);

	if ((sensor_id & 0xfff0) == 0x7670) {
		sd->sensor = SENSOR_OV767x;
		gspca_dev->cam.cam_mode = ov767x_mode;
		gspca_dev->cam.nmodes = ARRAY_SIZE(ov767x_mode);
	} else {
		sd->sensor = SENSOR_OV772x;
		gspca_dev->cam.bulk = 1;
		gspca_dev->cam.bulk_size = 16384;
		gspca_dev->cam.bulk_nurbs = 2;
		gspca_dev->cam.mode_framerates = ov772x_framerates;
	}

	/* initialize */
	reg_w_array(gspca_dev, bridge_init[sd->sensor].val,
			bridge_init[sd->sensor].len);
	ov534_set_led(gspca_dev, 1);
	sccb_w_array(gspca_dev, sensor_init[sd->sensor].val,
			sensor_init[sd->sensor].len);

	sd_stopN(gspca_dev);
/*	set_frame_rate(gspca_dev);	*/

	return gspca_dev->usb_err;
}

static int sd_start(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int mode;
	static const struct reg_array bridge_start[NSENSORS][2] = {
	[SENSOR_OV767x] = {{bridge_start_qvga_767x,
					ARRAY_SIZE(bridge_start_qvga_767x)},
			{bridge_start_vga_767x,
					ARRAY_SIZE(bridge_start_vga_767x)}},
	[SENSOR_OV772x] = {{bridge_start_qvga_772x,
					ARRAY_SIZE(bridge_start_qvga_772x)},
			{bridge_start_vga_772x,
					ARRAY_SIZE(bridge_start_vga_772x)}},
	};
	static const struct reg_array sensor_start[NSENSORS][2] = {
	[SENSOR_OV767x] = {{sensor_start_qvga_767x,
					ARRAY_SIZE(sensor_start_qvga_767x)},
			{sensor_start_vga_767x,
					ARRAY_SIZE(sensor_start_vga_767x)}},
	[SENSOR_OV772x] = {{sensor_start_qvga_772x,
					ARRAY_SIZE(sensor_start_qvga_772x)},
			{sensor_start_vga_772x,
					ARRAY_SIZE(sensor_start_vga_772x)}},
	};

	/* (from ms-win trace) */
	if (sd->sensor == SENSOR_OV767x)
		sccb_reg_write(gspca_dev, 0x1e, 0x04);
					/* black sun enable ? */

	mode = gspca_dev->curr_mode;	/* 0: 320x240, 1: 640x480 */
	reg_w_array(gspca_dev, bridge_start[sd->sensor][mode].val,
				bridge_start[sd->sensor][mode].len);
	sccb_w_array(gspca_dev, sensor_start[sd->sensor][mode].val,
				sensor_start[sd->sensor][mode].len);

	set_frame_rate(gspca_dev);

	if (sd->hue)
		sethue(gspca_dev, v4l2_ctrl_g_ctrl(sd->hue));
	setsaturation(gspca_dev, v4l2_ctrl_g_ctrl(sd->saturation));
	if (sd->autogain)
		setagc(gspca_dev, v4l2_ctrl_g_ctrl(sd->autogain));
	setawb(gspca_dev, v4l2_ctrl_g_ctrl(sd->autowhitebalance));
	setaec(gspca_dev, v4l2_ctrl_g_ctrl(sd->autoexposure));
	if (sd->gain)
		setgain(gspca_dev, v4l2_ctrl_g_ctrl(sd->gain));
	setexposure(gspca_dev, v4l2_ctrl_g_ctrl(sd->exposure));
	setbrightness(gspca_dev, v4l2_ctrl_g_ctrl(sd->brightness));
	setcontrast(gspca_dev, v4l2_ctrl_g_ctrl(sd->contrast));
	if (sd->sharpness)
		setsharpness(gspca_dev, v4l2_ctrl_g_ctrl(sd->sharpness));
	sethvflip(gspca_dev, v4l2_ctrl_g_ctrl(sd->hflip),
		  v4l2_ctrl_g_ctrl(sd->vflip));
	setlightfreq(gspca_dev, v4l2_ctrl_g_ctrl(sd->plfreq));
	setgamma(gspca_dev, v4l2_ctrl_g_ctrl(sd->gamma));

	ov534_set_led(gspca_dev, 1);
	ov534_reg_write(gspca_dev, 0xe0, 0x00);
	return gspca_dev->usb_err;
}

static void sd_stopN(struct gspca_dev *gspca_dev)
{
	ov534_reg_write(gspca_dev, 0xe0, 0x09);
	ov534_set_led(gspca_dev, 0);
}

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

static void sd_pkt_scan(struct gspca_dev *gspca_dev,
			u8 *data, int len)
{
	struct sd *sd = (struct sd *) gspca_dev;
	__u32 this_pts;
	u16 this_fid;
	int remaining_len = len;
	int payload_len;

	payload_len = gspca_dev->cam.bulk ? 2048 : 2040;
	do {
		len = min(remaining_len, payload_len);

		/* Payloads are prefixed with a UVC-style header.  We
		   consider a frame to start when the FID toggles, or the PTS
		   changes.  A frame ends when EOF is set, and we've received
		   the correct number of bytes. */

		/* Verify UVC header.  Header length is always 12 */
		if (data[0] != 12 || len < 12) {
			PDEBUG(D_PACK, "bad header");
			goto discard;
		}

		/* Check errors */
		if (data[1] & UVC_STREAM_ERR) {
			PDEBUG(D_PACK, "payload error");
			goto discard;
		}

		/* Extract PTS and FID */
		if (!(data[1] & UVC_STREAM_PTS)) {
			PDEBUG(D_PACK, "PTS not present");
			goto discard;
		}
		this_pts = (data[5] << 24) | (data[4] << 16)
						| (data[3] << 8) | data[2];
		this_fid = (data[1] & UVC_STREAM_FID) ? 1 : 0;

		/* If PTS or FID has changed, start a new frame. */
		if (this_pts != sd->last_pts || this_fid != sd->last_fid) {
			if (gspca_dev->last_packet_type == INTER_PACKET)
				gspca_frame_add(gspca_dev, LAST_PACKET,
						NULL, 0);
			sd->last_pts = this_pts;
			sd->last_fid = this_fid;
			gspca_frame_add(gspca_dev, FIRST_PACKET,
					data + 12, len - 12);
		/* If this packet is marked as EOF, end the frame */
		} else if (data[1] & UVC_STREAM_EOF) {
			sd->last_pts = 0;
			if (gspca_dev->pixfmt.pixelformat == V4L2_PIX_FMT_YUYV
			 && gspca_dev->image_len + len - 12 !=
				   gspca_dev->pixfmt.width *
					gspca_dev->pixfmt.height * 2) {
				PDEBUG(D_PACK, "wrong sized frame");
				goto discard;
			}
			gspca_frame_add(gspca_dev, LAST_PACKET,
					data + 12, len - 12);
		} else {

			/* Add the data from this payload */
			gspca_frame_add(gspca_dev, INTER_PACKET,
					data + 12, len - 12);
		}

		/* Done this payload */
		goto scan_next;

discard:
		/* Discard data until a new frame starts. */
		gspca_dev->last_packet_type = DISCARD_PACKET;

scan_next:
		remaining_len -= len;
		data += len;
	} while (remaining_len > 0);
}

/* get stream parameters (framerate) */
static void sd_get_streamparm(struct gspca_dev *gspca_dev,
			     struct v4l2_streamparm *parm)
{
	struct v4l2_captureparm *cp = &parm->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sd *sd = (struct sd *) gspca_dev;

	cp->capability |= V4L2_CAP_TIMEPERFRAME;
	tpf->numerator = sd->frame_rate_numerator;
	tpf->denominator = sd->frame_rate_denominator;
}

/* set stream parameters (framerate) */
static void sd_set_streamparm(struct gspca_dev *gspca_dev,
			     struct v4l2_streamparm *parm)
{
	struct v4l2_captureparm *cp = &parm->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sd *sd = (struct sd *) gspca_dev;

	if (tpf->numerator == 0 || tpf->denominator == 0)
	{
		sd->frame_rate_numerator = 1;
		sd->frame_rate_denominator = DEFAULT_FRAME_RATE;
	}
	else
	{
		sd->frame_rate_numerator = tpf->numerator;
		sd->frame_rate_denominator = tpf->denominator;
	}

	if (gspca_dev->streaming)
		set_frame_rate(gspca_dev);

	/* Return the actual framerate */
	tpf->numerator = sd->frame_rate_numerator;
	tpf->denominator = sd->frame_rate_denominator;
}

/* sub-driver description */
static const struct sd_desc sd_desc = {
	.name     = MODULE_NAME,
	.config   = sd_config,
	.init     = sd_init,
	.init_controls = sd_init_controls,
	.start    = sd_start,
	.stopN    = sd_stopN,
	.pkt_scan = sd_pkt_scan,
	.get_streamparm = sd_get_streamparm,
	.set_streamparm = sd_set_streamparm,
};

/* -- module initialisation -- */
static const struct usb_device_id device_table[] = {
	{USB_DEVICE(0x1415, 0x2000)},
	{USB_DEVICE(0x06f8, 0x3002)},
	{}
};

MODULE_DEVICE_TABLE(usb, device_table);

/* -- device connect -- */
static int sd_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	return gspca_dev_probe(intf, id, &sd_desc, sizeof(struct sd),
				THIS_MODULE);
}

static struct usb_driver sd_driver = {
	.name       = MODULE_NAME,
	.id_table   = device_table,
	.probe      = sd_probe,
	.disconnect = gspca_disconnect,
#ifdef CONFIG_PM
	.suspend    = gspca_suspend,
	.resume     = gspca_resume,
	.reset_resume = gspca_resume,
#endif
};

module_usb_driver(sd_driver);
