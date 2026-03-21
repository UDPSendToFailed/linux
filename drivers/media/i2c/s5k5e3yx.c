// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K5E3YX CMOS Image Sensor driver
 *
 * 5MP (2576x1932) front camera sensor with 2-lane MIPI CSI-2 output.
 * Uses standard SMIA register set with 16-bit register addresses.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Chip identification */
#define S5K5E3YX_CHIP_ID		CCI_REG16(0x0000)
#define S5K5E3YX_CHIP_ID_VAL		0x5e30

/* Streaming control */
#define S5K5E3YX_MODE_SELECT		CCI_REG8(0x0100)
#define   S5K5E3YX_MODE_STANDBY	0x00
#define   S5K5E3YX_MODE_STREAMING	0x01

/* Pixel array */
#define S5K5E3YX_PIXEL_ARRAY_WIDTH	2592U
#define S5K5E3YX_PIXEL_ARRAY_HEIGHT	1944U
#define S5K5E3YX_PIXEL_ARRAY_LEFT	8U
#define S5K5E3YX_PIXEL_ARRAY_TOP	6U
#define S5K5E3YX_ACTIVE_WIDTH		2576U
#define S5K5E3YX_ACTIVE_HEIGHT		1932U

/* Frame timing */
#define S5K5E3YX_REG_FLL		CCI_REG16(0x0340)
#define S5K5E3YX_REG_LLP		CCI_REG16(0x0342)
#define S5K5E3YX_FLL_MAX		0xffff

/* Image crop */
#define S5K5E3YX_REG_X_START		CCI_REG16(0x0344)
#define S5K5E3YX_REG_Y_START		CCI_REG16(0x0346)
#define S5K5E3YX_REG_X_END		CCI_REG16(0x0348)
#define S5K5E3YX_REG_Y_END		CCI_REG16(0x034a)
#define S5K5E3YX_REG_X_OUTPUT		CCI_REG16(0x034c)
#define S5K5E3YX_REG_Y_OUTPUT		CCI_REG16(0x034e)

/* Exposure */
#define S5K5E3YX_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5E3YX_EXPOSURE_MIN		1
#define S5K5E3YX_EXPOSURE_OFFSET	8
#define S5K5E3YX_EXPOSURE_STEP		1

/* Analog gain */
#define S5K5E3YX_REG_ANALOG_GAIN	CCI_REG16(0x0204)
#define S5K5E3YX_ANA_GAIN_MIN		0x0020
#define S5K5E3YX_ANA_GAIN_MAX		0x0200
#define S5K5E3YX_ANA_GAIN_STEP		1
#define S5K5E3YX_ANA_GAIN_DEFAULT	0x0020

/* Digital gain */
#define S5K5E3YX_REG_DIG_GAIN		CCI_REG16(0x020e)
#define S5K5E3YX_DGTL_GAIN_MIN		0x0100
#define S5K5E3YX_DGTL_GAIN_MAX		0x1000
#define S5K5E3YX_DGTL_GAIN_STEP	1
#define S5K5E3YX_DGTL_GAIN_DEFAULT	0x0100

/* Test pattern */
#define S5K5E3YX_REG_TEST_PATTERN	CCI_REG16(0x0600)

/* Image orientation */
#define S5K5E3YX_REG_ORIENTATION	CCI_REG8(0x0101)

/* PLL registers */
#define S5K5E3YX_REG_VT_PIX_CLK_DIV	CCI_REG16(0x0300)
#define S5K5E3YX_REG_VT_SYS_CLK_DIV	CCI_REG16(0x0302)
#define S5K5E3YX_REG_PRE_PLL_CLK_DIV	CCI_REG16(0x0304)
#define S5K5E3YX_REG_PLL_MULTIPLIER	CCI_REG16(0x0306)
#define S5K5E3YX_REG_OP_PIX_CLK_DIV	CCI_REG16(0x0308)
#define S5K5E3YX_REG_OP_SYS_CLK_DIV	CCI_REG16(0x030a)

#define S5K5E3YX_MCLK_FREQ		24000000UL
#define S5K5E3YX_VBLANK_MIN		16

static const char * const s5k5e3yx_supply_names[] = {
	"avdd",
	"dvdd",
	"vio",
};

#define S5K5E3YX_NUM_SUPPLIES ARRAY_SIZE(s5k5e3yx_supply_names)

struct s5k5e3yx_mode {
	u32 width;
	u32 height;
	u32 fll_def;
	u32 llp;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

struct s5k5e3yx {
	struct regmap *regmap;
	struct clk *extclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k5e3yx_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint bus_cfg;

	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	const struct s5k5e3yx_mode *cur_mode;
	s64 link_freq_val;
	u64 pixel_rate_val;
};

static inline struct s5k5e3yx *sd_to_s5k5e3yx(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e3yx, sd);
}

static inline struct s5k5e3yx *ctrl_to_s5k5e3yx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct s5k5e3yx, hdl);
}

/*
 * Global init: software reset, basic analog tuning, PLL configuration.
 *
 * PLL (init defaults, overridden by mode table):
 *   MCLK=24MHz, pre_div=4, mult=140 → VCO=840MHz
 *   vt_pix_div=5, vt_sys_div=1  → pixel_clk=168MHz
 *   op_pix_div=10, op_sys_div=1
 *
 * Mode table overrides to: pre_div=6, mult=224 → VCO=896MHz
 *   pixel_clk=179.2MHz, link_freq=448MHz
 */
static const struct cci_reg_sequence s5k5e3yx_global_init[] = {
	/* Software standby */
	{ CCI_REG8(0x0100), 0x00 },

	/* PLL configuration */
	{ CCI_REG16(0x0300), 0x0005 },	/* vt_pix_clk_div = 5 */
	{ CCI_REG16(0x0302), 0x0001 },	/* vt_sys_clk_div = 1 */
	{ CCI_REG16(0x0304), 0x0004 },	/* pre_pll_clk_div = 4 */
	{ CCI_REG16(0x0306), 0x008c },	/* pll_multiplier = 140 */
	{ CCI_REG16(0x0308), 0x000a },	/* op_pix_clk_div = 10 */
	{ CCI_REG16(0x030a), 0x0001 },	/* op_sys_clk_div = 1 */

	/* MIPI configuration: 10-bit */
	{ CCI_REG16(0x0112), 0x0a0a },

	/*
	 * Analog tuning from downstream blob (rev >= 3).
	 * Extracted from libmmcamera_s5k5e3yx.so at offset 0x177ac0.
	 */
	{ CCI_REG8(0x3000), 0x04 },
	{ CCI_REG8(0x3002), 0x03 },
	{ CCI_REG8(0x3003), 0x04 },
	{ CCI_REG8(0x3004), 0x05 },
	{ CCI_REG8(0x3005), 0x00 },
	{ CCI_REG8(0x3006), 0x10 },
	{ CCI_REG8(0x3007), 0x0a },
	{ CCI_REG8(0x3008), 0x55 },
	{ CCI_REG8(0x3039), 0x00 },
	{ CCI_REG8(0x303a), 0x00 },
	{ CCI_REG8(0x303b), 0x00 },
	{ CCI_REG8(0x3009), 0x05 },
	{ CCI_REG8(0x300a), 0x55 },
	{ CCI_REG8(0x300b), 0x38 },
	{ CCI_REG8(0x300c), 0x10 },
	{ CCI_REG8(0x3012), 0x14 },
	{ CCI_REG8(0x3013), 0x00 },
	{ CCI_REG8(0x3014), 0x22 },
	{ CCI_REG8(0x300e), 0x79 },
	{ CCI_REG8(0x3010), 0x68 },
	{ CCI_REG8(0x3019), 0x03 },
	{ CCI_REG8(0x301a), 0x00 },
	{ CCI_REG8(0x301b), 0x06 },
	{ CCI_REG8(0x301c), 0x00 },
	{ CCI_REG8(0x301d), 0x22 },
	{ CCI_REG8(0x301e), 0x00 },
	{ CCI_REG8(0x301f), 0x10 },
	{ CCI_REG8(0x3020), 0x00 },
	{ CCI_REG8(0x3021), 0x00 },
	{ CCI_REG8(0x3022), 0x0a },
	{ CCI_REG8(0x3023), 0x1e },
	{ CCI_REG8(0x3024), 0x00 },
	{ CCI_REG8(0x3025), 0x00 },
	{ CCI_REG8(0x3026), 0x00 },
	{ CCI_REG8(0x3027), 0x00 },
	{ CCI_REG8(0x3028), 0x1a },
	{ CCI_REG8(0x3015), 0x00 },
	{ CCI_REG8(0x3016), 0x84 },
	{ CCI_REG8(0x3017), 0x00 },
	{ CCI_REG8(0x3018), 0xa0 },
	{ CCI_REG8(0x302b), 0x10 },
	{ CCI_REG8(0x302c), 0x0a },
	{ CCI_REG8(0x302d), 0x06 },
	{ CCI_REG8(0x302e), 0x05 },
	{ CCI_REG8(0x302f), 0x0e },
	{ CCI_REG8(0x3030), 0x2f },
	{ CCI_REG8(0x3031), 0x08 },
	{ CCI_REG8(0x3032), 0x05 },
	{ CCI_REG8(0x3033), 0x09 },
	{ CCI_REG8(0x3034), 0x05 },
	{ CCI_REG8(0x3035), 0x00 },
	{ CCI_REG8(0x3036), 0x00 },
	{ CCI_REG8(0x3037), 0x00 },
	{ CCI_REG8(0x3038), 0x00 },
	{ CCI_REG8(0x3088), 0x06 },
	{ CCI_REG8(0x308a), 0x08 },
	{ CCI_REG8(0x308c), 0x05 },
	{ CCI_REG8(0x308e), 0x07 },
	{ CCI_REG8(0x3090), 0x06 },
	{ CCI_REG8(0x3092), 0x08 },
	{ CCI_REG8(0x3094), 0x05 },
	{ CCI_REG8(0x3096), 0x21 },
	{ CCI_REG8(0x3055), 0x9e },
	{ CCI_REG8(0x3099), 0x06 },
	{ CCI_REG8(0x3070), 0x10 },
	{ CCI_REG8(0x3085), 0x31 },
	{ CCI_REG8(0x3086), 0x01 },
	{ CCI_REG8(0x3064), 0x00 },
	{ CCI_REG8(0x3062), 0x08 },
	{ CCI_REG8(0x3061), 0x15 },
	{ CCI_REG8(0x307b), 0x20 },
	{ CCI_REG8(0x3068), 0x01 },
	{ CCI_REG8(0x3074), 0x00 },
	{ CCI_REG8(0x307d), 0x05 },
	{ CCI_REG8(0x3045), 0x01 },
	{ CCI_REG8(0x3046), 0x05 },
	{ CCI_REG8(0x3047), 0x78 },
	{ CCI_REG8(0x307f), 0xb1 },
	{ CCI_REG8(0x3098), 0x01 },
	{ CCI_REG8(0x305c), 0xf6 },
	{ CCI_REG8(0x3063), 0x2f },
	{ CCI_REG8(0x3400), 0x01 },
	{ CCI_REG8(0x3235), 0x49 },
	{ CCI_REG8(0x3233), 0x00 },
	{ CCI_REG8(0x3234), 0x00 },
	{ CCI_REG8(0x3300), 0x0c },
	{ CCI_REG8(0x3203), 0x45 },
	{ CCI_REG8(0x3205), 0x4d },
	{ CCI_REG8(0x320b), 0x40 },
	{ CCI_REG8(0x320c), 0x06 },
	{ CCI_REG8(0x320d), 0xc0 },
	{ CCI_REG8(0x3244), 0x00 },
	{ CCI_REG8(0x3245), 0x00 },
	{ CCI_REG8(0x3246), 0x01 },
	{ CCI_REG8(0x3247), 0x00 },
	{ CCI_REG8(0x3268), 0x88 },
	{ CCI_REG8(0x3269), 0x01 },
};

/*
 * Full-resolution mode: 2576x1932 @ ~30fps
 * line_length_pck = 2950, frame_length_lines = 2025
 *
 * PLL override: pre_div=6, mult=224 → VCO=896MHz
 *   vt_pix_div=5 (from init), vt_sys_div=1 → pixel_clk=179.2MHz
 *   MIPI data rate = 896Mbps, link_freq = 448MHz
 * fps = 179.2MHz / (2950 * 2025) ≈ 30fps
 *
 * Register list from downstream CCI dump (45 registers).
 */
static const struct cci_reg_sequence s5k5e3yx_mode_2576x1932[] = {
	/* PLL override (change VCO from init 840MHz to 896MHz) */
	{ CCI_REG8(0x0305), 0x06 },	/* pre_pll_clk_div = 6 */
	{ CCI_REG8(0x0306), 0x00 },	/* pll_multiplier [15:8] */
	{ CCI_REG8(0x0307), 0xe0 },	/* pll_multiplier [7:0] = 224 */

	/* Samsung timing */
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c1c), 0x58 },

	/* MIPI data rate = 896 Mbps */
	{ CCI_REG8(0x0820), 0x03 },	/* data rate [15:8] */
	{ CCI_REG8(0x0821), 0x80 },	/* data rate [7:0] = 896 */
	{ CCI_REG8(0x0114), 0x01 },	/* CSI-2 lane count = 2 */

	/* Frame geometry */
	{ S5K5E3YX_REG_FLL,     2025 },
	{ S5K5E3YX_REG_LLP,     2950 },
	{ S5K5E3YX_REG_X_START, 0x0000 },
	{ S5K5E3YX_REG_Y_START, 0x0002 },
	{ S5K5E3YX_REG_X_END,   0x0a0f },
	{ S5K5E3YX_REG_Y_END,   0x078d },
	{ S5K5E3YX_REG_X_OUTPUT, 2576 },
	{ S5K5E3YX_REG_Y_OUTPUT, 1932 },

	/* No binning */
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },

	/* AEC defaults */
	{ CCI_REG8(0x0204), 0x00 },	/* analog gain [15:8] */
	{ CCI_REG8(0x0205), 0x20 },	/* analog gain [7:0] = 32 */
	{ CCI_REG8(0x0202), 0x02 },	/* coarse integration [15:8] */
	{ CCI_REG8(0x0203), 0x00 },	/* coarse integration [7:0] */
	{ CCI_REG8(0x0200), 0x04 },	/* fine integration [15:8] */
	{ CCI_REG8(0x0201), 0x98 },	/* fine integration [7:0] */

	/* Samsung analog tuning */
	{ CCI_REG8(0x3941), 0x04 },
	{ CCI_REG8(0x3942), 0xb5 },
	{ CCI_REG8(0x3924), 0x2f },
	{ CCI_REG8(0x3925), 0x10 },

	/* Samsung timing tuning */
	{ CCI_REG8(0x3c31), 0x54 },
	{ CCI_REG8(0x3c32), 0x60 },
	{ CCI_REG8(0x3c08), 0x5d },
	{ CCI_REG8(0x3c09), 0xc0 },

	{ CCI_REG8(0x3320), 0x01 },

	/* CSI data format: RAW10 */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
};

/*
 * 2x2 binned mode: 1288x966 @ ~60fps
 * Same PLL overrides as full-resolution mode.
 * fps = 179.2MHz / (2950 * 1013) ≈ 60fps
 */
static const struct cci_reg_sequence s5k5e3yx_mode_1288x966[] = {
	/* PLL override (same as full-res) */
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xe0 },

	/* Samsung timing */
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c1c), 0x58 },

	/* MIPI data rate = 896 Mbps */
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x80 },
	{ CCI_REG8(0x0114), 0x01 },

	/* Frame geometry */
	{ S5K5E3YX_REG_FLL,     1013 },
	{ S5K5E3YX_REG_LLP,     2950 },
	{ S5K5E3YX_REG_X_START, 0x0000 },
	{ S5K5E3YX_REG_Y_START, 0x0002 },
	{ S5K5E3YX_REG_X_END,   0x0a0f },
	{ S5K5E3YX_REG_Y_END,   0x078d },
	{ S5K5E3YX_REG_X_OUTPUT, 1288 },
	{ S5K5E3YX_REG_Y_OUTPUT, 966 },

	/* 2x2 binning */
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },

	/* AEC defaults */
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x20 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x00 },
	{ CCI_REG8(0x0200), 0x04 },
	{ CCI_REG8(0x0201), 0x98 },

	/* Samsung analog tuning */
	{ CCI_REG8(0x3941), 0x04 },
	{ CCI_REG8(0x3942), 0xb5 },
	{ CCI_REG8(0x3924), 0x2f },
	{ CCI_REG8(0x3925), 0x10 },

	/* Samsung timing tuning */
	{ CCI_REG8(0x3c31), 0x54 },
	{ CCI_REG8(0x3c32), 0x60 },
	{ CCI_REG8(0x3c08), 0x5d },
	{ CCI_REG8(0x3c09), 0xc0 },

	{ CCI_REG8(0x3320), 0x01 },

	/* CSI data format: RAW10 */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
};

static const struct s5k5e3yx_mode s5k5e3yx_modes[] = {
	{
		.width = 2576,
		.height = 1932,
		.fll_def = 2025,
		.llp = 2950,
		.regs = s5k5e3yx_mode_2576x1932,
		.num_regs = ARRAY_SIZE(s5k5e3yx_mode_2576x1932),
	},
	{
		.width = 1288,
		.height = 966,
		.fll_def = 1013,
		.llp = 2950,
		.regs = s5k5e3yx_mode_1288x966,
		.num_regs = ARRAY_SIZE(s5k5e3yx_mode_1288x966),
	},
};

static const char * const s5k5e3yx_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade To Grey",
	"PN9",
};

/* ------------------------------------------------------------------ */
/* V4L2 Controls                                                       */
/* ------------------------------------------------------------------ */

static int s5k5e3yx_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e3yx *sensor = ctrl_to_s5k5e3yx(ctrl);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = sensor->cur_mode->height + ctrl->val -
			  S5K5E3YX_EXPOSURE_OFFSET;
		__v4l2_ctrl_modify_range(sensor->exposure,
					S5K5E3YX_EXPOSURE_MIN, max,
					S5K5E3YX_EXPOSURE_STEP, max);
	}

	if (!pm_runtime_get_if_in_use(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, S5K5E3YX_REG_ANALOG_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(sensor->regmap, S5K5E3YX_REG_DIG_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, S5K5E3YX_REG_EXPOSURE,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, S5K5E3YX_REG_FLL,
			  sensor->cur_mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, S5K5E3YX_REG_ORIENTATION,
			  (sensor->hflip->val ? 1 : 0) |
			  (sensor->vflip->val ? 2 : 0), &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, S5K5E3YX_REG_TEST_PATTERN,
			  ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k5e3yx_ctrl_ops = {
	.s_ctrl = s5k5e3yx_set_ctrl,
};

static int s5k5e3yx_init_controls(struct s5k5e3yx *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5k5e3yx_mode *mode = sensor->cur_mode;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *hdl = &sensor->hdl;
	int ret;

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(hdl, 12);

	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, NULL,
					       V4L2_CID_PIXEL_RATE,
					       sensor->pixel_rate_val,
					       sensor->pixel_rate_val, 1,
					       sensor->pixel_rate_val);

	sensor->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL,
						   V4L2_CID_LINK_FREQ,
						   0, 0,
						   &sensor->link_freq_val);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K5E3YX_ANA_GAIN_MIN, S5K5E3YX_ANA_GAIN_MAX,
			  S5K5E3YX_ANA_GAIN_STEP, S5K5E3YX_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  S5K5E3YX_DGTL_GAIN_MIN, S5K5E3YX_DGTL_GAIN_MAX,
			  S5K5E3YX_DGTL_GAIN_STEP, S5K5E3YX_DGTL_GAIN_DEFAULT);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K5E3YX_EXPOSURE_MIN,
					     mode->fll_def -
					     S5K5E3YX_EXPOSURE_OFFSET,
					     S5K5E3YX_EXPOSURE_STEP,
					     mode->fll_def -
					     S5K5E3YX_EXPOSURE_OFFSET);

	sensor->vblank = v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops,
					   V4L2_CID_VBLANK,
					   S5K5E3YX_VBLANK_MIN,
					   S5K5E3YX_FLL_MAX - mode->height,
					   1,
					   mode->fll_def - mode->height);

	sensor->hblank = v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->llp - mode->width,
					   mode->llp - mode->width,
					   1,
					   mode->llp - mode->width);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->hflip = v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(hdl, &s5k5e3yx_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(hdl, &s5k5e3yx_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k5e3yx_test_pattern_menu) - 1,
				     0, 0, s5k5e3yx_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &s5k5e3yx_ctrl_ops, &props);

	if (hdl->error)
		return hdl->error;

	sensor->sd.ctrl_handler = hdl;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Streaming                                                           */
/* ------------------------------------------------------------------ */

static int s5k5e3yx_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5k5e3yx_mode *mode = sensor->cur_mode;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = cci_multi_reg_write(sensor->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret < 0) {
		dev_err(dev, "failed to apply mode settings\n");
		goto err_rpm_put;
	}

	ret = __v4l2_ctrl_handler_setup(&sensor->hdl);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(sensor->regmap, S5K5E3YX_MODE_SELECT,
			S5K5E3YX_MODE_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	__v4l2_ctrl_grab(sensor->vflip, true);
	__v4l2_ctrl_grab(sensor->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int s5k5e3yx_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    u32 pad, u64 streams_mask)
{
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret;

	ret = cci_write(sensor->regmap, S5K5E3YX_MODE_SELECT,
			S5K5E3YX_MODE_STANDBY, NULL);

	__v4l2_ctrl_grab(sensor->vflip, false);
	__v4l2_ctrl_grab(sensor->hflip, false);

	pm_runtime_put_autosuspend(dev);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Pad operations                                                      */
/* ------------------------------------------------------------------ */

static int s5k5e3yx_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int s5k5e3yx_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k5e3yx_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = s5k5e3yx_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k5e3yx_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k5e3yx_set_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);
	const struct s5k5e3yx_mode *mode;
	struct v4l2_mbus_framefmt *fmt;

	mode = v4l2_find_nearest_size(s5k5e3yx_modes,
				      ARRAY_SIZE(s5k5e3yx_modes),
				      width, height,
				      format->format.width,
				      format->format.height);

	fmt = v4l2_subdev_state_get_format(state, format->pad);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->cur_mode = mode;

		__v4l2_ctrl_modify_range(sensor->vblank,
					 S5K5E3YX_VBLANK_MIN,
					 S5K5E3YX_FLL_MAX - mode->height,
					 1, mode->fll_def - mode->height);
		__v4l2_ctrl_s_ctrl(sensor->vblank,
				   mode->fll_def - mode->height);
		__v4l2_ctrl_modify_range(sensor->hblank,
					 mode->llp - mode->width,
					 mode->llp - mode->width,
					 1, mode->llp - mode->width);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 S5K5E3YX_EXPOSURE_MIN,
					 mode->fll_def -
					 S5K5E3YX_EXPOSURE_OFFSET,
					 S5K5E3YX_EXPOSURE_STEP,
					 mode->fll_def -
					 S5K5E3YX_EXPOSURE_OFFSET);
	}

	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->field = V4L2_FIELD_NONE;

	format->format = *fmt;

	/* Update the crop rectangle to match the active pixel area. */
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, format->pad);
	crop->left = S5K5E3YX_PIXEL_ARRAY_LEFT;
	crop->top = S5K5E3YX_PIXEL_ARRAY_TOP;
	crop->width = S5K5E3YX_ACTIVE_WIDTH;
	crop->height = S5K5E3YX_ACTIVE_HEIGHT;

	return 0;
}

static int s5k5e3yx_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_state_get_format(state, 0);
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->width = sensor->cur_mode->width;
	fmt->height = sensor->cur_mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->left = S5K5E3YX_PIXEL_ARRAY_LEFT;
	crop->top = S5K5E3YX_PIXEL_ARRAY_TOP;
	crop->width = S5K5E3YX_ACTIVE_WIDTH;
	crop->height = S5K5E3YX_ACTIVE_HEIGHT;

	return 0;
}

static int s5k5e3yx_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = S5K5E3YX_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K5E3YX_PIXEL_ARRAY_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = S5K5E3YX_PIXEL_ARRAY_TOP;
		sel->r.left = S5K5E3YX_PIXEL_ARRAY_LEFT;
		sel->r.width = S5K5E3YX_ACTIVE_WIDTH;
		sel->r.height = S5K5E3YX_ACTIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_video_ops s5k5e3yx_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5e3yx_pad_ops = {
	.enum_mbus_code = s5k5e3yx_enum_mbus_code,
	.enum_frame_size = s5k5e3yx_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k5e3yx_set_format,
	.get_selection = s5k5e3yx_get_selection,
	.enable_streams = s5k5e3yx_enable_streams,
	.disable_streams = s5k5e3yx_disable_streams,
};

static const struct v4l2_subdev_ops s5k5e3yx_ops = {
	.video = &s5k5e3yx_video_ops,
	.pad = &s5k5e3yx_pad_ops,
};

static const struct media_entity_operations s5k5e3yx_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops s5k5e3yx_internal_ops = {
	.init_state = s5k5e3yx_init_state,
};

/* ------------------------------------------------------------------ */
/* Power management                                                    */
/* ------------------------------------------------------------------ */

/*
 * Power sequence from downstream vendor binary (libmmcamera_s5k5e3yx.so):
 *   Power-on:  AVDD → DVDD → VIO → RESET(high) → MCLK
 *   Power-off: MCLK → RESET(low) → VIO → DVDD → AVDD
 */
static int s5k5e3yx_power_on(struct s5k5e3yx *sensor)
{
	int ret, i;

	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);

	/* Enable supplies one-by-one: AVDD → DVDD → VIO */
	for (i = 0; i < S5K5E3YX_NUM_SUPPLIES; i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret < 0)
			goto err_reg_disable;
		usleep_range(1000, 1500);
	}

	usleep_range(1000, 1500);

	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 0);

	usleep_range(2000, 2500);

	ret = clk_prepare_enable(sensor->extclk);
	if (ret < 0)
		goto err_reset;

	/* Wait for sensor PLL to lock and become responsive */
	usleep_range(10000, 12000);

	return 0;

err_reset:
	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);
err_reg_disable:
	while (--i >= 0)
		regulator_disable(sensor->supplies[i].consumer);
	return ret;
}

static void s5k5e3yx_power_off(struct s5k5e3yx *sensor)
{
	int i;

	clk_disable_unprepare(sensor->extclk);

	usleep_range(1000, 1500);

	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);

	usleep_range(1000, 1500);

	/* Disable supplies in reverse: VIO → DVDD → AVDD */
	for (i = S5K5E3YX_NUM_SUPPLIES - 1; i >= 0; i--) {
		regulator_disable(sensor->supplies[i].consumer);
		usleep_range(1000, 1500);
	}
}

static int s5k5e3yx_initialize(struct s5k5e3yx *sensor)
{
	return cci_multi_reg_write(sensor->regmap, s5k5e3yx_global_init,
				  ARRAY_SIZE(s5k5e3yx_global_init), NULL);
}

static int __maybe_unused s5k5e3yx_pm_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);
	int ret;

	ret = s5k5e3yx_power_on(sensor);
	if (ret)
		return ret;

	ret = s5k5e3yx_initialize(sensor);
	if (ret) {
		s5k5e3yx_power_off(sensor);
		return ret;
	}

	return 0;
}

static int __maybe_unused s5k5e3yx_pm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	s5k5e3yx_power_off(sd_to_s5k5e3yx(sd));
	return 0;
}

static const struct dev_pm_ops s5k5e3yx_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k5e3yx_pm_suspend, s5k5e3yx_pm_resume, NULL)
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int s5k5e3yx_identify(struct s5k5e3yx *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	u64 chip_id;
	int ret;

	ret = cci_read(sensor->regmap, S5K5E3YX_CHIP_ID, &chip_id, NULL);
	if (ret)
		return ret;

	if (chip_id != S5K5E3YX_CHIP_ID_VAL) {
		dev_err(dev, "chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			S5K5E3YX_CHIP_ID_VAL, chip_id);
		return -ENXIO;
	}

	dev_dbg(dev, "Samsung S5K5E3YX sensor found (id=0x%04llx)\n", chip_id);
	return 0;
}

static int s5k5e3yx_parse_dt(struct s5k5e3yx *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	struct fwnode_handle *ep;
	u32 ndata_lanes;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &sensor->bus_cfg);
	fwnode_handle_put(ep);
	if (ret < 0)
		return ret;

	sensor->bus_cfg.bus_type = V4L2_MBUS_CSI2_DPHY;
	ndata_lanes = sensor->bus_cfg.bus.mipi_csi2.num_data_lanes;

	if (ndata_lanes != 2) {
		dev_err(dev, "unsupported number of data lanes: %u (expected 2)\n",
			ndata_lanes);
		v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
		return -EINVAL;
	}

	/*
	 * PLL: Mode table overrides init PLL to VCO = 896 MHz.
	 *   pixel_clk = 896/5 = 179.2 MHz, link_freq = 896/2 = 448 MHz.
	 *   pixel_rate = 2 * link_freq * num_lanes / bpp
	 */
	sensor->link_freq_val = 448000000LL;
	sensor->pixel_rate_val = 2ULL * sensor->link_freq_val * ndata_lanes / 10;

	return 0;
}

static int s5k5e3yx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e3yx *sensor;
	int ret;

	dev_info(dev, "probe: start\n");

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to init CCI regmap\n");

	sensor->extclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->extclk))
		return dev_err_probe(dev, PTR_ERR(sensor->extclk),
				     "failed to get clock\n");

	ret = clk_set_rate(sensor->extclk, S5K5E3YX_MCLK_FREQ);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock rate\n");

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	dev_info(dev, "probe: clk/gpio OK, getting regulators\n");

	for (ret = 0; ret < S5K5E3YX_NUM_SUPPLIES; ret++)
		sensor->supplies[ret].supply = s5k5e3yx_supply_names[ret];

	ret = devm_regulator_bulk_get(dev, S5K5E3YX_NUM_SUPPLIES,
				     sensor->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	dev_info(dev, "probe: regulators OK, parsing DT\n");

	ret = s5k5e3yx_parse_dt(sensor);
	if (ret < 0)
		return ret;

	dev_info(dev, "probe: DT OK, powering on\n");

	ret = s5k5e3yx_power_on(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to power on\n");
		goto err_ep_free;
	}

	dev_info(dev, "probe: power on OK, reading chip ID\n");

	ret = s5k5e3yx_identify(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to identify sensor\n");
		goto err_power_off;
	}

	ret = s5k5e3yx_initialize(sensor);
	if (ret < 0)
		goto err_power_off;

	sensor->cur_mode = &s5k5e3yx_modes[0];

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5k5e3yx_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.internal_ops = &s5k5e3yx_internal_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &s5k5e3yx_entity_ops;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		goto err_power_off;

	ret = s5k5e3yx_init_controls(sensor);
	if (ret < 0)
		goto err_entity;

	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_ctrl;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register V4L2 subdev: %d\n", ret);
		goto err_pm;
	}

	dev_info(dev, "probe: async subdev registered, fwnode=%pfw\n",
		 dev_fwnode(dev));

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_pm:
	v4l2_subdev_cleanup(&sensor->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_ctrl:
	v4l2_ctrl_handler_free(&sensor->hdl);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_power_off:
	s5k5e3yx_power_off(sensor);
err_ep_free:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void s5k5e3yx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e3yx *sensor = sd_to_s5k5e3yx(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->hdl);
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k5e3yx_power_off(sensor);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id s5k5e3yx_of_match[] = {
	{ .compatible = "samsung,s5k5e3yx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k5e3yx_of_match);

static struct i2c_driver s5k5e3yx_i2c_driver = {
	.driver = {
		.name = "s5k5e3yx",
		.of_match_table = s5k5e3yx_of_match,
		.pm = &s5k5e3yx_pm_ops,
	},
	.probe = s5k5e3yx_probe,
	.remove = s5k5e3yx_remove,
};
module_i2c_driver(s5k5e3yx_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K5E3YX CMOS Image Sensor driver");
MODULE_LICENSE("GPL");
