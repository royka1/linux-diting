// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4H7 8 MP image sensor driver
 *
 * Based on the s5kjn1 driver (Copyright (c) 2025 Linaro Ltd). The sensor
 * initialisation and mode register tables are not part of the driver: they
 * are loaded from a "camregs" firmware file, built from the camera module's
 * own data (the sensor needs module specific settings).
 *
 * camregs format (little endian):
 *   char magic[8] = "CAMREGS1"; u32 n_tables;
 *   n_tables x { char name[16]; u32 n_entries;
 *                n_entries x { u16 addr; u16 value; u8 width; u8 pad;
 *                              u16 delay_us } }
 * Tables used: "init", then one of "mode0", "mode1", ...
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <linux/unaligned.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K4H7_LINK_FREQ_330MHZ		(330ULL * HZ_PER_MHZ)
#define S5K4H7_PIXEL_RATE		(280 * HZ_PER_MHZ)
#define S5K4H7_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5K4H7_DATA_LANES		4

/* Register map is similar to MIPI CCS compliant camera sensors */
#define S5K4H7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K4H7_CHIP_ID			0x487b

#define S5K4H7_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5K4H7_MODE_STREAMING		BIT(0)

#define S5K4H7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K4H7_VFLIP			BIT(1)
#define S5K4H7_HFLIP			BIT(0)
/* The module is mounted with the sensor read out mirrored and flipped */
#define S5K4H7_ORIENTATION_DEFAULT	(S5K4H7_VFLIP | S5K4H7_HFLIP)

#define S5K4H7_REG_FRAME_COUNT		CCI_REG8(0x0005)

#define S5K4H7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H7_EXPOSURE_MIN		2
#define S5K4H7_EXPOSURE_STEP		1
#define S5K4H7_EXPOSURE_MARGIN		8

/* Analogue gain code: 32 = 1x ... 512 = 16x */
#define S5K4H7_REG_AGAIN		CCI_REG16(0x0204)
#define S5K4H7_AGAIN_MIN		32
#define S5K4H7_AGAIN_MAX		512
#define S5K4H7_AGAIN_STEP		1
#define S5K4H7_AGAIN_DEFAULT		64

#define S5K4H7_REG_VTS			CCI_REG16(0x0340)
#define S5K4H7_VTS_MAX			0xffff

#define S5K4H7_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define S5K4H7_FW_MAGIC			"CAMREGS1"
#define S5K4H7_FW_DEFAULT		"s5k4h7.camregs"

#define to_s5k4h7(_sd)			container_of(_sd, struct s5k4h7, sd)

static const s64 s5k4h7_link_freq_menu[] = {
	S5K4H7_LINK_FREQ_330MHZ,
};

/* Native GRBG; indexed by (vflip << 1 | hflip) relative to the sensor */
static const u32 s5k4h7_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5k4h7_reg {
	u16 addr;
	u16 value;
	u8 width;
	u16 delay_us;
};

struct s5k4h7_table {
	const struct s5k4h7_reg *regs;
	unsigned int num_regs;
};

struct s5k4h7_mode {
	u32 width;			/* Frame width in pixels */
	u32 height;			/* Frame height in pixels */
	u32 hts;			/* Horizontal timing size */
	u32 vts;			/* Default vertical timing size */
	u32 exposure;			/* Default exposure value */
	const char *table;		/* Register table in the firmware */
};

static const char * const s5k4h7_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Fade to grey color bars",
	"PN9",
};

struct s5k4h7 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *vdda;		/* Analog power */
	struct regulator *vddd;		/* Digital core power */
	struct regulator *vddio;	/* Digital I/O power */

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

	struct s5k4h7_table init_table;
	struct s5k4h7_table mode_tables[2];

	const struct s5k4h7_mode *mode;
};

static const struct s5k4h7_mode s5k4h7_supported_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.hts = 3688,
		.vts = 2530,
		.exposure = 2400,
		.table = "mode0",
	},
	{
		.width = 3264,
		.height = 1836,
		.hts = 3688,
		.vts = 2530,
		.exposure = 2400,
		.table = "mode1",
	},
};

static int s5k4h7_write_table(struct s5k4h7 *s5k4h7,
			      const struct s5k4h7_table *table)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < table->num_regs; i++) {
		const struct s5k4h7_reg *r = &table->regs[i];
		u32 reg = r->width == 2 ? CCI_REG16(r->addr) : CCI_REG8(r->addr);

		cci_write(s5k4h7->regmap, reg, r->value, &ret);
		if (ret)
			return ret;
		if (r->delay_us)
			fsleep(r->delay_us);
	}

	return 0;
}

static int s5k4h7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7 *s5k4h7 = container_of(ctrl->handler, struct s5k4h7,
					     ctrl_handler);
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max = mode->height + ctrl->val - S5K4H7_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k4h7->exposure,
					 s5k4h7->exposure->minimum,
					 exposure_max,
					 s5k4h7->exposure->step,
					 s5k4h7->exposure->default_value);
		break;
	}

	/* V4L2 controls are applied, when sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(s5k4h7->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_AGAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_ORIENTATION,
				S5K4H7_ORIENTATION_DEFAULT ^
				((s5k4h7->vflip->val ? S5K4H7_VFLIP : 0) |
				 (s5k4h7->hflip->val ? S5K4H7_HFLIP : 0)), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k4h7->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7_ctrl_ops = {
	.s_ctrl = s5k4h7_set_ctrl,
};

static int s5k4h7_init_controls(struct s5k4h7 *s5k4h7)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k4h7->ctrl_handler;
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	s64 hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 11);

	s5k4h7->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k4h7_ctrl_ops,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5k4h7_link_freq_menu) - 1,
					0, s5k4h7_link_freq_menu);
	if (s5k4h7->link_freq)
		s5k4h7->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k4h7->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K4H7_PIXEL_RATE,
					       S5K4H7_PIXEL_RATE, 1,
					       S5K4H7_PIXEL_RATE);

	hblank = mode->hts - mode->width;
	s5k4h7->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5k4h7->hblank)
		s5k4h7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k4h7->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5K4H7_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K4H7_AGAIN_MIN, S5K4H7_AGAIN_MAX,
			  S5K4H7_AGAIN_STEP, S5K4H7_AGAIN_DEFAULT);

	exposure_max = mode->vts - S5K4H7_EXPOSURE_MARGIN;
	s5k4h7->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4H7_EXPOSURE_MIN,
					     exposure_max,
					     S5K4H7_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k4h7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h7_test_pattern_menu) - 1,
				     0, 0, s5k4h7_test_pattern_menu);

	s5k4h7->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k4h7->hflip)
		s5k4h7->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k4h7->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k4h7->vflip)
		s5k4h7->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ret = v4l2_fwnode_device_parse(s5k4h7->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k4h7_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	s5k4h7->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k4h7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	unsigned int idx = s5k4h7->mode - s5k4h7_supported_modes;
	int ret;

	ret = pm_runtime_resume_and_get(s5k4h7->dev);
	if (ret)
		return ret;

	ret = s5k4h7_write_table(s5k4h7, &s5k4h7->init_table);
	if (!ret)
		ret = s5k4h7_write_table(s5k4h7, &s5k4h7->mode_tables[idx]);
	if (ret)
		goto error;

	/* Also writes the orientation register */
	ret = __v4l2_ctrl_handler_setup(s5k4h7->sd.ctrl_handler);

	cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE,
		  S5K4H7_MODE_STREAMING, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5k4h7->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static int s5k4h7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	unsigned int i;
	u64 val;
	int ret;

	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE, 0x0, NULL);
	if (ret)
		dev_err(s5k4h7->dev, "failed to stop streaming: %d\n", ret);

	/* The frame counter reads 0xff once the sensor is in software standby */
	for (i = 0; !ret && i < 50; i++) {
		ret = cci_read(s5k4h7->regmap, S5K4H7_REG_FRAME_COUNT, &val, NULL);
		if (ret || val == 0xff)
			break;
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
	}

	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static u32 s5k4h7_get_format_code(struct s5k4h7 *s5k4h7)
{
	unsigned int i;

	/* The Bayer order follows the readout orientation of the sensor */
	i = S5K4H7_ORIENTATION_DEFAULT ^
	    ((s5k4h7->vflip->val ? S5K4H7_VFLIP : 0) |
	     (s5k4h7->hflip->val ? S5K4H7_HFLIP : 0));

	return s5k4h7_mbus_formats[i];
}

static void s5k4h7_update_pad_format(struct s5k4h7 *s5k4h7,
				     const struct s5k4h7_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k4h7_get_format_code(s5k4h7);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k4h7_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	s64 hblank, vblank, exposure_max;
	const struct s5k4h7_mode *mode;

	mode = v4l2_find_nearest_size(s5k4h7_supported_modes,
				      ARRAY_SIZE(s5k4h7_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k4h7_update_pad_format(s5k4h7, mode, &fmt->format);

	/* Format code could be updated with respect to flip controls */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k4h7->mode == mode)
		goto set_format;

	/* Update limits and set FPS and exposure to default values */
	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k4h7->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k4h7->vblank, vblank,
				 S5K4H7_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k4h7->vblank, vblank);

	exposure_max = mode->vts - S5K4H7_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k4h7->exposure, S5K4H7_EXPOSURE_MIN,
				 exposure_max, S5K4H7_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5k4h7->exposure, mode->exposure);

	if (s5k4h7->sd.ctrl_handler->error)
		return s5k4h7->sd.ctrl_handler->error;

	s5k4h7->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k4h7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	/* Media bus code index is constant, but code formats are not */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4h7_get_format_code(s5k4h7);

	return 0;
}

static int s5k4h7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	if (fse->index >= ARRAY_SIZE(s5k4h7_supported_modes))
		return -EINVAL;

	if (fse->code != s5k4h7_get_format_code(s5k4h7))
		return -EINVAL;

	fse->min_width = s5k4h7_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k4h7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = (2448 - s5k4h7->mode->height) / 2;
		sel->r.width = s5k4h7->mode->width;
		sel->r.height = s5k4h7->mode->height;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = 3264;
		sel->r.height = 2448;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k4h7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			/* Media bus code depends on current flip controls */
			.width = s5k4h7->mode->width,
			.height = s5k4h7->mode->height,
		},
	};

	s5k4h7_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k4h7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h7_pad_ops = {
	.set_fmt = s5k4h7_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k4h7_get_selection,
	.enum_mbus_code = s5k4h7_enum_mbus_code,
	.enum_frame_size = s5k4h7_enum_frame_size,
	.enable_streams = s5k4h7_enable_streams,
	.disable_streams = s5k4h7_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h7_subdev_ops = {
	.video = &s5k4h7_video_ops,
	.pad = &s5k4h7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7_internal_ops = {
	.init_state = s5k4h7_init_state,
};

static const struct media_entity_operations s5k4h7_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* Copy one table of the camregs firmware into driver memory */
static int s5k4h7_parse_table(struct s5k4h7 *s5k4h7, const u8 *data,
			      size_t size, size_t *pos, char *name,
			      struct s5k4h7_table *out)
{
	struct s5k4h7_reg *regs;
	unsigned int i, n;
	const u8 *e;

	if (*pos + 20 > size)
		return -EINVAL;
	memcpy(name, data + *pos, 16);
	name[15] = '\0';
	n = get_unaligned_le32(data + *pos + 16);
	*pos += 20;
	if (n > 4096 || *pos + 8 * (size_t)n > size)
		return -EINVAL;

	regs = devm_kcalloc(s5k4h7->dev, n, sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	for (i = 0; i < n; i++) {
		e = data + *pos + 8 * i;
		regs[i].addr = get_unaligned_le16(e);
		regs[i].value = get_unaligned_le16(e + 2);
		regs[i].width = e[4];
		regs[i].delay_us = get_unaligned_le16(e + 6);
		if (regs[i].width != 1 && regs[i].width != 2)
			return -EINVAL;
		if (regs[i].width == 1 && regs[i].value > 0xff)
			return -EINVAL;
	}
	*pos += 8 * (size_t)n;

	out->regs = regs;
	out->num_regs = n;

	return 0;
}

static int s5k4h7_load_tables(struct s5k4h7 *s5k4h7)
{
	const char *fw_name = S5K4H7_FW_DEFAULT;
	const struct firmware *fw;
	struct s5k4h7_table table;
	unsigned int i, j, n;
	char name[16];
	size_t pos;
	int ret;

	device_property_read_string(s5k4h7->dev, "firmware-name", &fw_name);

	ret = request_firmware(&fw, fw_name, s5k4h7->dev);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				     "failed to load register tables %s\n",
				     fw_name);

	ret = -EINVAL;
	if (fw->size < 12 || memcmp(fw->data, S5K4H7_FW_MAGIC, 8))
		goto out;

	n = get_unaligned_le32(fw->data + 8);
	pos = 12;
	for (i = 0; i < n; i++) {
		ret = s5k4h7_parse_table(s5k4h7, fw->data, fw->size, &pos,
					 name, &table);
		if (ret)
			goto out;

		if (!strcmp(name, "init"))
			s5k4h7->init_table = table;
		for (j = 0; j < ARRAY_SIZE(s5k4h7_supported_modes); j++)
			if (!strcmp(name, s5k4h7_supported_modes[j].table))
				s5k4h7->mode_tables[j] = table;
	}

	ret = 0;
	if (!s5k4h7->init_table.num_regs)
		ret = -EINVAL;
	for (j = 0; j < ARRAY_SIZE(s5k4h7_supported_modes); j++)
		if (!s5k4h7->mode_tables[j].num_regs)
			ret = -EINVAL;

out:
	release_firmware(fw);
	if (ret)
		dev_err(s5k4h7->dev, "invalid register tables in %s\n", fw_name);

	return ret;
}

static int s5k4h7_identify_sensor(struct s5k4h7 *s5k4h7)
{
	u64 val;
	int ret;

	ret = cci_read(s5k4h7->regmap, S5K4H7_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K4H7_CHIP_ID) {
		dev_err(s5k4h7->dev, "chip id mismatch: %x!=%llx\n",
			S5K4H7_CHIP_ID, val);
		return -ENODEV;
	}

	return 0;
}

static int s5k4h7_check_hwcfg(struct s5k4h7 *s5k4h7)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k4h7->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5K4H7_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K4H7_DATA_LANES) {
		dev_err(s5k4h7->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k4h7->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k4h7_link_freq_menu,
				       ARRAY_SIZE(s5k4h7_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k4h7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	if (s5k4h7->vddio) {
		ret = regulator_enable(s5k4h7->vddio);
		if (ret)
			return ret;
	}

	if (s5k4h7->vdda) {
		ret = regulator_enable(s5k4h7->vdda);
		if (ret)
			goto disable_vddio;
	}

	if (s5k4h7->vddd) {
		ret = regulator_enable(s5k4h7->vddd);
		if (ret)
			goto disable_vdda;
	}

	usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);

	ret = clk_prepare_enable(s5k4h7->mclk);
	if (ret)
		goto disable_vddd;

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	return 0;

disable_vddd:
	if (s5k4h7->vddd)
		regulator_disable(s5k4h7->vddd);

disable_vdda:
	if (s5k4h7->vdda)
		regulator_disable(s5k4h7->vdda);

disable_vddio:
	if (s5k4h7->vddio)
		regulator_disable(s5k4h7->vddio);

	return ret;
}

static int s5k4h7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);

	clk_disable_unprepare(s5k4h7->mclk);

	if (s5k4h7->vddd)
		regulator_disable(s5k4h7->vddd);

	if (s5k4h7->vdda)
		regulator_disable(s5k4h7->vdda);

	if (s5k4h7->vddio)
		regulator_disable(s5k4h7->vddio);

	return 0;
}

static struct regulator *s5k4h7_get_supply(struct s5k4h7 *s5k4h7,
					   const char *name)
{
	struct regulator *reg;

	reg = devm_regulator_get_optional(s5k4h7->dev, name);
	if (IS_ERR(reg) && PTR_ERR(reg) == -ENODEV)
		return NULL;

	return reg;
}

static int s5k4h7_probe(struct i2c_client *client)
{
	struct s5k4h7 *s5k4h7;
	unsigned long freq;
	int ret;

	s5k4h7 = devm_kzalloc(&client->dev, sizeof(*s5k4h7), GFP_KERNEL);
	if (!s5k4h7)
		return -ENOMEM;

	s5k4h7->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k4h7->sd, client, &s5k4h7_subdev_ops);

	s5k4h7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k4h7->regmap))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->regmap),
				     "failed to init CCI\n");

	s5k4h7->mclk = devm_v4l2_sensor_clk_get(s5k4h7->dev, NULL);
	if (IS_ERR(s5k4h7->mclk))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k4h7->mclk);
	if (freq != S5K4H7_MCLK_FREQ_24MHZ)
		return dev_err_probe(s5k4h7->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5k4h7_check_hwcfg(s5k4h7);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				     "failed to check HW configuration\n");

	s5k4h7->reset_gpio = devm_gpiod_get_optional(s5k4h7->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k4h7->reset_gpio))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->reset_gpio),
				     "cannot get reset GPIO\n");

	s5k4h7->vdda = s5k4h7_get_supply(s5k4h7, "vdda");
	if (IS_ERR(s5k4h7->vdda))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->vdda),
				     "Failed to get 'vdda' regulator\n");

	s5k4h7->vddd = s5k4h7_get_supply(s5k4h7, "vddd");
	if (IS_ERR(s5k4h7->vddd))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->vddd),
				     "Failed to get 'vddd' regulator\n");

	s5k4h7->vddio = s5k4h7_get_supply(s5k4h7, "vddio");
	if (IS_ERR(s5k4h7->vddio))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->vddio),
				     "Failed to get 'vddio' regulator\n");

	ret = s5k4h7_load_tables(s5k4h7);
	if (ret)
		return ret;

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5k4h7_power_on(s5k4h7->dev);
	if (ret)
		return ret;

	ret = s5k4h7_identify_sensor(s5k4h7);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	s5k4h7->mode = &s5k4h7_supported_modes[0];
	ret = s5k4h7_init_controls(s5k4h7);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret, "failed to init controls\n");
		goto power_off;
	}

	s5k4h7->sd.state_lock = s5k4h7->ctrl_handler.lock;
	s5k4h7->sd.internal_ops = &s5k4h7_internal_ops;
	s5k4h7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k4h7->sd.entity.ops = &s5k4h7_subdev_entity_ops;
	s5k4h7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k4h7->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k4h7->sd.entity, 1, &s5k4h7->pad);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to init media entity pads\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k4h7->dev);
	pm_runtime_enable(s5k4h7->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k4h7->dev, 1000);
	pm_runtime_use_autosuspend(s5k4h7->dev);
	pm_runtime_idle(s5k4h7->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&s5k4h7->sd);
	pm_runtime_disable(s5k4h7->dev);
	pm_runtime_set_suspended(s5k4h7->dev);

media_entity_cleanup:
	media_entity_cleanup(&s5k4h7->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k4h7->sd.ctrl_handler);

power_off:
	s5k4h7_power_off(s5k4h7->dev);

	return ret;
}

static void s5k4h7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k4h7->dev);

	if (!pm_runtime_status_suspended(s5k4h7->dev)) {
		s5k4h7_power_off(s5k4h7->dev);
		pm_runtime_set_suspended(s5k4h7->dev);
	}
}

static const struct dev_pm_ops s5k4h7_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h7_power_off, s5k4h7_power_on, NULL)
};

static const struct of_device_id s5k4h7_of_match[] = {
	{ .compatible = "samsung,s5k4h7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7_of_match);

static struct i2c_driver s5k4h7_i2c_driver = {
	.driver = {
		.name = "s5k4h7",
		.pm = &s5k4h7_pm_ops,
		.of_match_table = s5k4h7_of_match,
	},
	.probe = s5k4h7_probe,
	.remove = s5k4h7_remove,
};

module_i2c_driver(s5k4h7_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K4H7 image sensor driver");
MODULE_LICENSE("GPL");
