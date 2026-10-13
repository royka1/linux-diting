// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5KHP1 (ISOCELL HP1) 200 MP image sensor driver
 *
 * Based on the s5k4h7/s5kjn1 drivers (s5kjn1: Copyright (c) 2025 Linaro
 * Ltd). The sensor initialisation (including a sensor firmware patch) and
 * the mode register tables are module specific and are loaded from a
 * "camregs" firmware file built from the camera module's own data:
 *
 *   char magic[8] = "CAMREGS1" or "CAMREGS2"; u32 n_tables;
 *   n_tables x { char name[16]; u32 n_entries;
 *                n_entries x { u16 addr; u16 value; u8 width; u8 pad;
 *                              u16 delay_us } }
 *
 * width 1/2 is a register write; width 0 (CAMREGS2) a burst of `value`
 * bytes to consecutive addresses, following the entry, padded to 4 bytes.
 * Tables used: "init", then one of the "modeN" tables.
 *
 * The sensor output is MIPI C-PHY with three trios.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5KHP1_MCLK_FREQ		19200000
#define S5KHP1_DATA_LANES		3

#define S5KHP1_NATIVE_WIDTH		16384
#define S5KHP1_NATIVE_HEIGHT		12288

/* Register map is similar to MIPI CCS compliant camera sensors */
#define S5KHP1_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5KHP1_CHIP_ID			0x1b71

#define S5KHP1_REG_PAGE			CCI_REG16(0xfcfc)
#define S5KHP1_PAGE_4000		0x4000

#define S5KHP1_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5KHP1_MODE_STREAMING		BIT(0)

#define S5KHP1_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5KHP1_VFLIP			BIT(1)
#define S5KHP1_HFLIP			BIT(0)
/* The module data streams with the readout mirrored and flipped */
#define S5KHP1_ORIENTATION_DEFAULT	(S5KHP1_VFLIP | S5KHP1_HFLIP)

#define S5KHP1_REG_FRAME_COUNT		CCI_REG8(0x0005)

#define S5KHP1_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5KHP1_EXPOSURE_MIN		8
#define S5KHP1_EXPOSURE_STEP		1
#define S5KHP1_EXPOSURE_MARGIN		22

/* Analogue gain code: 32 = 1x ... 512 = 16x */
#define S5KHP1_REG_AGAIN		CCI_REG16(0x0204)
#define S5KHP1_AGAIN_MIN		32
#define S5KHP1_AGAIN_MAX		512
#define S5KHP1_AGAIN_STEP		1
#define S5KHP1_AGAIN_DEFAULT		64

#define S5KHP1_REG_VTS			CCI_REG16(0x0340)
#define S5KHP1_VTS_MAX			0xffff

#define S5KHP1_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define S5KHP1_FW_DEFAULT		"s5khp1.camregs"

#define to_s5khp1(_sd)			container_of(_sd, struct s5khp1, sd)

/*
 * C-PHY link frequencies (V4L2: pixel rate = link_freq * 2 * trios * 16 /
 * 7 / bpp) of the module data's output pixel clocks: 1163.6928, 1429.5552
 * and 1638.7584 Mpixel/s at RAW10.
 */
static const s64 s5khp1_link_freq_menu[] = {
	848526000,
	1042384000,
	1194928000,
};

/* Native GRBG; indexed by (vflip << 1 | hflip) relative to the sensor */
static const u32 s5khp1_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5khp1_reg {
	u16 addr;
	u16 value;		/* register value, or burst length */
	u8 width;		/* 1, 2, or 0 for a burst */
	u16 delay_us;
	const u8 *data;		/* burst bytes */
};

struct s5khp1_table {
	const struct s5khp1_reg *regs;
	unsigned int num_regs;
};

struct s5khp1_mode {
	u32 width;			/* Frame width in pixels */
	u32 height;			/* Frame height in pixels */
	u32 hts;			/* Horizontal timing size */
	u32 vts;			/* Default vertical timing size */
	u32 exposure;			/* Default exposure value */
	u32 pixel_rate;			/* Pixel array rate: hts * vts * fps */
	unsigned int link_freq_index;
	struct v4l2_rect crop;		/* Analogue crop in native pixels */
	const char *table;		/* Register table in the firmware */
};

static const char * const s5khp1_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Fade to grey color bars",
	"PN9",
};

static const char * const s5khp1_supply_names[] = {
	"vdda2",	/* WL2866D AVDD2 */
	"vddio",	/* 1.8 V I/O */
	"vdda",		/* analogue */
	"vddd",		/* digital core */
};

struct s5khp1 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5khp1_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	unsigned long link_freq_bitmap;

	struct s5khp1_table init_table;
	struct s5khp1_table mode_tables[5];

	const struct s5khp1_mode *mode;
};

/* Mode table names are the module data's resolution indices */
static const struct s5khp1_mode s5khp1_supported_modes[] = {
	{
		/* 4x4 binned, 30 fps */
		.width = 4096,
		.height = 3072,
		.hts = 9520,
		.vts = 4760,
		.exposure = 4000,
		.pixel_rate = 1359456000,
		.link_freq_index = 0,
		.crop = { 0, 0, 16384, 12288 },
		.table = "mode2",
	},
	{
		/* 2x2 binned, 22 fps */
		.width = 8192,
		.height = 6144,
		.hts = 9728,
		.vts = 6360,
		.exposure = 6000,
		.pixel_rate = 1361141760,
		.link_freq_index = 1,
		.crop = { 0, 0, 16384, 12288 },
		.table = "mode1",
	},
	{
		/* full resolution, 6.5 fps */
		.width = 16384,
		.height = 12288,
		.hts = 19200,
		.vts = 12698,
		.exposure = 12000,
		.pixel_rate = 1584710400,
		.link_freq_index = 2,
		.crop = { 0, 0, 16384, 12288 },
		.table = "mode0",
	},
	{
		/* 4x4 binned 16:9, 60 fps */
		.width = 4096,
		.height = 2304,
		.hts = 9280,
		.vts = 2440,
		.exposure = 2300,
		.pixel_rate = 1358515200,
		.link_freq_index = 1,
		.crop = { 0, 1536, 16384, 9216 },
		.table = "mode5",
	},
	{
		/* binned 16:9, 120 fps */
		.width = 1920,
		.height = 1080,
		.hts = 4352,
		.vts = 2606,
		.exposure = 2500,
		.pixel_rate = 1360957440,
		.link_freq_index = 0,
		.crop = { 0, 1536, 16384, 9216 },
		.table = "mode6",
	},
};

static int s5khp1_write_table(struct s5khp1 *s5khp1,
			      const struct s5khp1_table *table)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < table->num_regs; i++) {
		const struct s5khp1_reg *r = &table->regs[i];

		if (r->width == 0) {
			ret = regmap_bulk_write(s5khp1->regmap, r->addr,
						r->data, r->value);
		} else {
			u32 reg = r->width == 2 ? CCI_REG16(r->addr) :
						  CCI_REG8(r->addr);

			cci_write(s5khp1->regmap, reg, r->value, &ret);
		}
		if (ret) {
			dev_err(s5khp1->dev, "write to 0x%04x failed: %d\n",
				r->addr, ret);
			return ret;
		}
		if (r->delay_us)
			fsleep(r->delay_us);
	}

	return 0;
}

static void s5khp1_update_mode_ctrls(struct s5khp1 *s5khp1,
				     const struct s5khp1_mode *mode)
{
	s64 hblank, vblank, exposure_max;

	__v4l2_ctrl_s_ctrl(s5khp1->link_freq, mode->link_freq_index);
	__v4l2_ctrl_modify_range(s5khp1->pixel_rate, mode->pixel_rate,
				 mode->pixel_rate, 1, mode->pixel_rate);

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5khp1->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5khp1->vblank, vblank,
				 S5KHP1_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5khp1->vblank, vblank);

	exposure_max = mode->vts - S5KHP1_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5khp1->exposure, S5KHP1_EXPOSURE_MIN,
				 exposure_max, S5KHP1_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5khp1->exposure, mode->exposure);
}

static int s5khp1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5khp1 *s5khp1 = container_of(ctrl->handler, struct s5khp1,
					     ctrl_handler);
	const struct s5khp1_mode *mode = s5khp1->mode;
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->height + ctrl->val - S5KHP1_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5khp1->exposure,
					 s5khp1->exposure->minimum,
					 exposure_max,
					 s5khp1->exposure->step,
					 s5khp1->exposure->default_value);
		break;
	}

	/* V4L2 controls are applied, when sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(s5khp1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5khp1->regmap, S5KHP1_REG_AGAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5khp1->regmap, S5KHP1_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5khp1->regmap, S5KHP1_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5khp1->regmap, S5KHP1_REG_ORIENTATION,
				S5KHP1_ORIENTATION_DEFAULT ^
				((s5khp1->vflip->val ? S5KHP1_VFLIP : 0) |
				 (s5khp1->hflip->val ? S5KHP1_HFLIP : 0)), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5khp1->regmap, S5KHP1_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = 0;
		break;
	}

	pm_runtime_put(s5khp1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5khp1_ctrl_ops = {
	.s_ctrl = s5khp1_set_ctrl,
};

static int s5khp1_init_controls(struct s5khp1 *s5khp1)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5khp1->ctrl_handler;
	const struct s5khp1_mode *mode = s5khp1->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 11);

	s5khp1->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5khp1_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5khp1_link_freq_menu) - 1,
					mode->link_freq_index,
					s5khp1_link_freq_menu);
	if (s5khp1->link_freq)
		s5khp1->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5khp1->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       mode->pixel_rate,
					       mode->pixel_rate, 1,
					       mode->pixel_rate);

	hblank = mode->hts - mode->width;
	s5khp1->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5khp1->hblank)
		s5khp1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5khp1->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5KHP1_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5KHP1_AGAIN_MIN, S5KHP1_AGAIN_MAX,
			  S5KHP1_AGAIN_STEP, S5KHP1_AGAIN_DEFAULT);

	exposure_max = mode->vts - S5KHP1_EXPOSURE_MARGIN;
	s5khp1->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5KHP1_EXPOSURE_MIN,
					     exposure_max,
					     S5KHP1_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5khp1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5khp1_test_pattern_menu) - 1,
				     0, 0, s5khp1_test_pattern_menu);

	s5khp1->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5khp1->hflip)
		s5khp1->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5khp1->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5khp1_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5khp1->vflip)
		s5khp1->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(s5khp1->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5khp1_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	s5khp1->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5khp1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	unsigned int idx = s5khp1->mode - s5khp1_supported_modes;
	int ret;

	ret = pm_runtime_resume_and_get(s5khp1->dev);
	if (ret)
		return ret;

	ret = s5khp1_write_table(s5khp1, &s5khp1->init_table);
	if (!ret)
		ret = s5khp1_write_table(s5khp1, &s5khp1->mode_tables[idx]);
	if (ret)
		goto error;

	cci_write(s5khp1->regmap, S5KHP1_REG_PAGE, S5KHP1_PAGE_4000, &ret);
	if (ret)
		goto error;

	/* Also writes the orientation register */
	ret = __v4l2_ctrl_handler_setup(s5khp1->sd.ctrl_handler);

	cci_write(s5khp1->regmap, S5KHP1_REG_CTRL_MODE,
		  S5KHP1_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5khp1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5khp1->dev);

	return ret;
}

static int s5khp1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	unsigned int i;
	u64 val;
	int ret;

	ret = cci_write(s5khp1->regmap, S5KHP1_REG_CTRL_MODE, 0x0, NULL);
	if (ret)
		dev_err(s5khp1->dev, "failed to stop streaming: %d\n", ret);

	/* The frame counter reads 0xff once the sensor is in software standby */
	for (i = 0; !ret && i < 30; i++) {
		ret = cci_read(s5khp1->regmap, S5KHP1_REG_FRAME_COUNT, &val, NULL);
		if (ret || val == 0xff)
			break;
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	pm_runtime_put_autosuspend(s5khp1->dev);

	return ret;
}

static u32 s5khp1_get_format_code(struct s5khp1 *s5khp1)
{
	unsigned int i;

	/* The Bayer order follows the readout orientation of the sensor */
	i = S5KHP1_ORIENTATION_DEFAULT ^
	    ((s5khp1->vflip->val ? S5KHP1_VFLIP : 0) |
	     (s5khp1->hflip->val ? S5KHP1_HFLIP : 0));

	return s5khp1_mbus_formats[i];
}

static void s5khp1_update_pad_format(struct s5khp1 *s5khp1,
				     const struct s5khp1_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5khp1_get_format_code(s5khp1);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5khp1_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	const struct s5khp1_mode *mode;

	mode = v4l2_find_nearest_size(s5khp1_supported_modes,
				      ARRAY_SIZE(s5khp1_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5khp1_update_pad_format(s5khp1, mode, &fmt->format);

	/* Format code could be updated with respect to flip controls */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5khp1->mode == mode)
		goto set_format;

	if (!(s5khp1->link_freq_bitmap & BIT(mode->link_freq_index)))
		return -EINVAL;

	/* Update limits and set FPS and exposure to default values */
	s5khp1_update_mode_ctrls(s5khp1, mode);
	if (s5khp1->sd.ctrl_handler->error)
		return s5khp1->sd.ctrl_handler->error;

	s5khp1->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5khp1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);

	/* Media bus code index is constant, but code formats are not */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5khp1_get_format_code(s5khp1);

	return 0;
}

static int s5khp1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);

	if (fse->index >= ARRAY_SIZE(s5khp1_supported_modes))
		return -EINVAL;

	if (fse->code != s5khp1_get_format_code(s5khp1))
		return -EINVAL;

	fse->min_width = s5khp1_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5khp1_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5khp1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = s5khp1->mode->crop;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5KHP1_NATIVE_WIDTH;
		sel->r.height = S5KHP1_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5khp1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			/* Media bus code depends on current flip controls */
			.width = s5khp1->mode->width,
			.height = s5khp1->mode->height,
		},
	};

	s5khp1_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5khp1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5khp1_pad_ops = {
	.set_fmt = s5khp1_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5khp1_get_selection,
	.enum_mbus_code = s5khp1_enum_mbus_code,
	.enum_frame_size = s5khp1_enum_frame_size,
	.enable_streams = s5khp1_enable_streams,
	.disable_streams = s5khp1_disable_streams,
};

static const struct v4l2_subdev_ops s5khp1_subdev_ops = {
	.video = &s5khp1_video_ops,
	.pad = &s5khp1_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5khp1_internal_ops = {
	.init_state = s5khp1_init_state,
};

static const struct media_entity_operations s5khp1_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Copy one table of the camregs firmware into driver memory */
static int s5khp1_parse_table(struct s5khp1 *s5khp1, const u8 *data,
			      size_t size, size_t *pos, bool bursts,
			      char *name, struct s5khp1_table *out)
{
	struct s5khp1_reg *regs;
	unsigned int i, n;
	const u8 *e;
	u8 *copy;

	if (*pos + 20 > size)
		return -EINVAL;
	memcpy(name, data + *pos, 16);
	name[15] = '\0';
	n = get_unaligned_le32(data + *pos + 16);
	*pos += 20;
	if (n > 8192)
		return -EINVAL;

	regs = devm_kcalloc(s5khp1->dev, n, sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	for (i = 0; i < n; i++) {
		if (*pos + 8 > size)
			return -EINVAL;
		e = data + *pos;
		regs[i].addr = get_unaligned_le16(e);
		regs[i].value = get_unaligned_le16(e + 2);
		regs[i].width = e[4];
		regs[i].delay_us = get_unaligned_le16(e + 6);
		*pos += 8;

		if (regs[i].width == 0 && bursts) {
			size_t len = regs[i].value;

			if (!len || *pos + ALIGN(len, 4) > size)
				return -EINVAL;
			copy = devm_kmemdup(s5khp1->dev, data + *pos, len,
					    GFP_KERNEL);
			if (!copy)
				return -ENOMEM;
			regs[i].data = copy;
			*pos += ALIGN(len, 4);
		} else if (regs[i].width != 1 && regs[i].width != 2) {
			return -EINVAL;
		} else if (regs[i].width == 1 && regs[i].value > 0xff) {
			return -EINVAL;
		}
	}

	out->regs = regs;
	out->num_regs = n;

	return 0;
}

static int s5khp1_load_tables(struct s5khp1 *s5khp1)
{
	const char *fw_name = S5KHP1_FW_DEFAULT;
	const struct firmware *fw;
	struct s5khp1_table table;
	unsigned int i, j, n;
	char name[16];
	bool bursts;
	size_t pos;
	int ret;

	device_property_read_string(s5khp1->dev, "firmware-name", &fw_name);

	ret = request_firmware(&fw, fw_name, s5khp1->dev);
	if (ret)
		return dev_err_probe(s5khp1->dev, ret,
				     "failed to load register tables %s\n",
				     fw_name);

	ret = -EINVAL;
	if (fw->size < 12)
		goto out;
	if (!memcmp(fw->data, "CAMREGS2", 8))
		bursts = true;
	else if (!memcmp(fw->data, "CAMREGS1", 8))
		bursts = false;
	else
		goto out;

	n = get_unaligned_le32(fw->data + 8);
	pos = 12;
	for (i = 0; i < n; i++) {
		ret = s5khp1_parse_table(s5khp1, fw->data, fw->size, &pos,
					 bursts, name, &table);
		if (ret)
			goto out;

		if (!strcmp(name, "init"))
			s5khp1->init_table = table;
		for (j = 0; j < ARRAY_SIZE(s5khp1_supported_modes); j++)
			if (!strcmp(name, s5khp1_supported_modes[j].table))
				s5khp1->mode_tables[j] = table;
	}

	ret = 0;
	if (!s5khp1->init_table.num_regs)
		ret = -EINVAL;
	for (j = 0; j < ARRAY_SIZE(s5khp1_supported_modes); j++)
		if (!s5khp1->mode_tables[j].num_regs)
			ret = -EINVAL;

out:
	release_firmware(fw);
	if (ret)
		dev_err(s5khp1->dev, "invalid register tables in %s\n", fw_name);

	return ret;
}

static int s5khp1_identify_sensor(struct s5khp1 *s5khp1)
{
	u64 val;
	int ret;

	ret = cci_read(s5khp1->regmap, S5KHP1_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5khp1->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5KHP1_CHIP_ID) {
		dev_err(s5khp1->dev, "chip id mismatch: %x!=%llx\n",
			S5KHP1_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5khp1_check_hwcfg(struct s5khp1 *s5khp1)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5khp1->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_CPHY,
	};
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5KHP1_DATA_LANES) {
		dev_err(s5khp1->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5khp1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5khp1_link_freq_menu,
				       ARRAY_SIZE(s5khp1_link_freq_menu),
				       &s5khp1->link_freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Power-up sequence of the module data: supplies, reset release, MCLK */
static int s5khp1_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(s5khp1->supplies); i++) {
		ret = regulator_enable(s5khp1->supplies[i].consumer);
		if (ret)
			goto disable_supplies;
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	gpiod_set_value_cansleep(s5khp1->reset_gpio, 0);
	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	ret = clk_prepare_enable(s5khp1->mclk);
	if (ret)
		goto assert_reset;
	usleep_range(25 * USEC_PER_MSEC, 26 * USEC_PER_MSEC);

	return 0;

assert_reset:
	gpiod_set_value_cansleep(s5khp1->reset_gpio, 1);
disable_supplies:
	while (i--)
		regulator_disable(s5khp1->supplies[i].consumer);

	return ret;
}

static int s5khp1_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5khp1 *s5khp1 = to_s5khp1(sd);
	unsigned int i;

	clk_disable_unprepare(s5khp1->mclk);
	gpiod_set_value_cansleep(s5khp1->reset_gpio, 1);
	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	for (i = ARRAY_SIZE(s5khp1->supplies); i--; )
		regulator_disable(s5khp1->supplies[i].consumer);

	return 0;
}

static int s5khp1_probe(struct i2c_client *client)
{
	struct s5khp1 *s5khp1;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5khp1 = devm_kzalloc(&client->dev, sizeof(*s5khp1), GFP_KERNEL);
	if (!s5khp1)
		return -ENOMEM;

	s5khp1->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5khp1->sd, client, &s5khp1_subdev_ops);

	s5khp1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5khp1->regmap))
		return dev_err_probe(s5khp1->dev, PTR_ERR(s5khp1->regmap),
				     "failed to init CCI\n");

	s5khp1->mclk = devm_v4l2_sensor_clk_get(s5khp1->dev, NULL);
	if (IS_ERR(s5khp1->mclk))
		return dev_err_probe(s5khp1->dev, PTR_ERR(s5khp1->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5khp1->mclk);
	if (freq != S5KHP1_MCLK_FREQ)
		return dev_err_probe(s5khp1->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5khp1_check_hwcfg(s5khp1);
	if (ret)
		return dev_err_probe(s5khp1->dev, ret,
				     "failed to check HW configuration\n");

	s5khp1->reset_gpio = devm_gpiod_get(s5khp1->dev, "reset",
					    GPIOD_OUT_HIGH);
	if (IS_ERR(s5khp1->reset_gpio))
		return dev_err_probe(s5khp1->dev, PTR_ERR(s5khp1->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < ARRAY_SIZE(s5khp1->supplies); i++)
		s5khp1->supplies[i].supply = s5khp1_supply_names[i];
	ret = devm_regulator_bulk_get(s5khp1->dev, ARRAY_SIZE(s5khp1->supplies),
				      s5khp1->supplies);
	if (ret)
		return dev_err_probe(s5khp1->dev, ret,
				     "failed to get supplies\n");

	ret = s5khp1_load_tables(s5khp1);
	if (ret)
		return ret;

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5khp1_power_on(s5khp1->dev);
	if (ret)
		return ret;

	ret = s5khp1_identify_sensor(s5khp1);
	if (ret) {
		dev_err_probe(s5khp1->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5khp1->mode = &s5khp1_supported_modes[0];
	if (!(s5khp1->link_freq_bitmap & BIT(s5khp1->mode->link_freq_index))) {
		ret = dev_err_probe(s5khp1->dev, -EINVAL,
				    "default mode link frequency missing\n");
		goto power_off;
	}

	ret = s5khp1_init_controls(s5khp1);
	if (ret) {
		dev_err_probe(s5khp1->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5khp1->sd.state_lock = s5khp1->ctrl_handler.lock;
	s5khp1->sd.internal_ops = &s5khp1_internal_ops;
	s5khp1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5khp1->sd.entity.ops = &s5khp1_subdev_entity_ops;
	s5khp1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5khp1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5khp1->sd.entity, 1, &s5khp1->pad);
	if (ret) {
		dev_err_probe(s5khp1->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5khp1->sd);
	if (ret < 0) {
		dev_err_probe(s5khp1->dev, ret,
			      "failed to init media entity pads\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5khp1->dev);
	pm_runtime_enable(s5khp1->dev);

	ret = v4l2_async_register_subdev_sensor(&s5khp1->sd);
	if (ret < 0) {
		dev_err_probe(s5khp1->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5khp1->dev, 1000);
	pm_runtime_use_autosuspend(s5khp1->dev);
	pm_runtime_idle(s5khp1->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5khp1->sd);
	pm_runtime_disable(s5khp1->dev);
	pm_runtime_set_suspended(s5khp1->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5khp1->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5khp1->sd.ctrl_handler);

power_off:
	s5khp1_power_off(s5khp1->dev);

	return ret;
}

static void s5khp1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5khp1 *s5khp1 = to_s5khp1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5khp1->dev);

	if (!pm_runtime_status_suspended(s5khp1->dev)) {
		s5khp1_power_off(s5khp1->dev);
		pm_runtime_set_suspended(s5khp1->dev);
	}
}

static const struct dev_pm_ops s5khp1_pm_ops = {
	SET_RUNTIME_PM_OPS(s5khp1_power_off, s5khp1_power_on, NULL)
};

static const struct of_device_id s5khp1_of_match[] = {
	{ .compatible = "samsung,s5khp1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5khp1_of_match);

static struct i2c_driver s5khp1_i2c_driver = {
	.driver = {
		.name = "s5khp1",
		.pm = &s5khp1_pm_ops,
		.of_match_table = s5khp1_of_match,
	},
	.probe = s5khp1_probe,
	.remove = s5khp1_remove,
};

module_i2c_driver(s5khp1_i2c_driver);

MODULE_DESCRIPTION("Samsung S5KHP1 image sensor driver");
MODULE_LICENSE("GPL");
