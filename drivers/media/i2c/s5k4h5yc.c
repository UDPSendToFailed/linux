// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K4H5YC CMOS Image Sensor driver
 *
 * 8MP (3264x2448) rear camera sensor with 4-lane MIPI CSI-2 output.
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
#define S5K4H5YC_CHIP_ID		CCI_REG16(0x0001)
#define S5K4H5YC_CHIP_ID_VAL		0x5b03

/* Streaming control */
#define S5K4H5YC_MODE_SELECT		CCI_REG8(0x0100)
#define   S5K4H5YC_MODE_STANDBY	0x00
#define   S5K4H5YC_MODE_STREAMING	0x01

/* Pixel array */
#define S5K4H5YC_PIXEL_ARRAY_WIDTH	3280U
#define S5K4H5YC_PIXEL_ARRAY_HEIGHT	2464U
#define S5K4H5YC_PIXEL_ARRAY_LEFT	8U
#define S5K4H5YC_PIXEL_ARRAY_TOP	8U
#define S5K4H5YC_ACTIVE_WIDTH		3264U
#define S5K4H5YC_ACTIVE_HEIGHT		2448U

/* Frame timing */
#define S5K4H5YC_REG_FLL		CCI_REG16(0x0340)
#define S5K4H5YC_REG_LLP		CCI_REG16(0x0342)
#define S5K4H5YC_FLL_MAX		0xffff

/* Image crop */
#define S5K4H5YC_REG_X_START		CCI_REG16(0x0344)
#define S5K4H5YC_REG_Y_START		CCI_REG16(0x0346)
#define S5K4H5YC_REG_X_END		CCI_REG16(0x0348)
#define S5K4H5YC_REG_Y_END		CCI_REG16(0x034a)
#define S5K4H5YC_REG_X_OUTPUT		CCI_REG16(0x034c)
#define S5K4H5YC_REG_Y_OUTPUT		CCI_REG16(0x034e)

/* Exposure */
#define S5K4H5YC_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H5YC_EXPOSURE_MIN		1
#define S5K4H5YC_EXPOSURE_OFFSET	8
#define S5K4H5YC_EXPOSURE_STEP		1

/* Analog gain */
#define S5K4H5YC_REG_ANALOG_GAIN	CCI_REG16(0x0204)
#define S5K4H5YC_ANA_GAIN_MIN		0x0020
#define S5K4H5YC_ANA_GAIN_MAX		0x0200
#define S5K4H5YC_ANA_GAIN_STEP		1
#define S5K4H5YC_ANA_GAIN_DEFAULT	0x0020

/* Digital gain */
#define S5K4H5YC_REG_DIG_GAIN		CCI_REG16(0x020e)
#define S5K4H5YC_DGTL_GAIN_MIN		0x0100
#define S5K4H5YC_DGTL_GAIN_MAX		0x1000
#define S5K4H5YC_DGTL_GAIN_STEP	1
#define S5K4H5YC_DGTL_GAIN_DEFAULT	0x0100

/* Test pattern */
#define S5K4H5YC_REG_TEST_PATTERN	CCI_REG16(0x0600)

/* Image orientation */
#define S5K4H5YC_REG_ORIENTATION	CCI_REG8(0x0101)

/* PLL registers */
#define S5K4H5YC_REG_VT_PIX_CLK_DIV	CCI_REG16(0x0300)
#define S5K4H5YC_REG_VT_SYS_CLK_DIV	CCI_REG16(0x0302)
#define S5K4H5YC_REG_PRE_PLL_CLK_DIV	CCI_REG16(0x0304)
#define S5K4H5YC_REG_PLL_MULTIPLIER	CCI_REG16(0x0306)
#define S5K4H5YC_REG_OP_PIX_CLK_DIV	CCI_REG16(0x0308)
#define S5K4H5YC_REG_OP_SYS_CLK_DIV	CCI_REG16(0x030a)

/* Binning */
#define S5K4H5YC_REG_BINNING_MODE	CCI_REG16(0x0900)
#define S5K4H5YC_REG_BINNING_TYPE	CCI_REG16(0x0901)

#define S5K4H5YC_MCLK_FREQ		24000000UL
#define S5K4H5YC_VBLANK_MIN		16

static const char * const s5k4h5yc_supply_names[] = {
	"avdd",
	"dvdd",
	"vio",
};

#define S5K4H5YC_NUM_SUPPLIES ARRAY_SIZE(s5k4h5yc_supply_names)

struct s5k4h5yc_mode {
	u32 width;
	u32 height;
	u32 fll_def;
	u32 llp;
	const struct cci_reg_sequence *regs;
	u32 num_regs;
};

struct s5k4h5yc {
	struct regmap *regmap;
	struct clk *extclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k4h5yc_supply_names)];

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

	const struct s5k4h5yc_mode *cur_mode;
	s64 link_freq_val;
	u64 pixel_rate_val;
};

static inline struct s5k4h5yc *sd_to_s5k4h5yc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4h5yc, sd);
}

static inline struct s5k4h5yc *ctrl_to_s5k4h5yc(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct s5k4h5yc, hdl);
}

/*
 * Global init: software reset, basic analog tuning, PLL configuration.
 *
 * PLL (init defaults, overridden by mode table):
 *   MCLK=24MHz, pre_div=6, mult=175 → VCO=700MHz
 *   vt_pix_div=5, vt_sys_div=1  → pixel_clk=140MHz
 *   op_pix_div=10, op_sys_div=1
 *
 * Mode table overrides to: mult=140, vt_pix_div=2 → VCO=560MHz
 *   pixel_clk=280MHz, link_freq=350MHz (for 4 lanes)
 */
static const struct cci_reg_sequence s5k4h5yc_global_init[] = {
	/* Software standby */
	{ CCI_REG8(0x0100), 0x00 },

	/* PLL configuration */
	{ CCI_REG16(0x0300), 0x0005 },	/* vt_pix_clk_div = 5 */
	{ CCI_REG16(0x0302), 0x0001 },	/* vt_sys_clk_div = 1 */
	{ CCI_REG16(0x0304), 0x0006 },	/* pre_pll_clk_div = 6 */
	{ CCI_REG16(0x0306), 0x00af },	/* pll_multiplier = 175 */
	{ CCI_REG16(0x0308), 0x000a },	/* op_pix_clk_div = 10 */
	{ CCI_REG16(0x030a), 0x0001 },	/* op_sys_clk_div = 1 */

	/* MIPI configuration: 10-bit */
	{ CCI_REG16(0x0112), 0x0a0a },	/* CSI data format: RAW10 */

	/*
	 * Analog tuning from downstream blob (rev >= 3).
	 * Extracted from libmmcamera_s5k4h5yc.so at offset 0x177ac0.
	 */
	{ CCI_REG8(0x3000), 0x07 },
	{ CCI_REG8(0x3001), 0x05 },
	{ CCI_REG8(0x3002), 0x03 },
	{ CCI_REG8(0x3069), 0x00 },
	{ CCI_REG8(0x306a), 0x04 },
	{ CCI_REG8(0x0200), 0x06 },	/* Fine integration time */
	{ CCI_REG8(0x0201), 0x44 },
	{ CCI_REG8(0x3005), 0x03 },
	{ CCI_REG8(0x3006), 0x03 },
	{ CCI_REG8(0x3009), 0x0d },
	{ CCI_REG8(0x300a), 0x03 },
	{ CCI_REG8(0x300c), 0x65 },
	{ CCI_REG8(0x300d), 0x54 },
	{ CCI_REG8(0x3010), 0x00 },
	{ CCI_REG8(0x3012), 0x14 },
	{ CCI_REG8(0x3014), 0x19 },
	{ CCI_REG8(0x3017), 0x0f },
	{ CCI_REG8(0x3018), 0x1a },
	{ CCI_REG8(0x3019), 0x6a },
	{ CCI_REG8(0x301a), 0x72 },
	{ CCI_REG8(0x306f), 0x00 },
	{ CCI_REG8(0x3070), 0x00 },
	{ CCI_REG8(0x3071), 0x00 },
	{ CCI_REG8(0x3072), 0x00 },
	{ CCI_REG8(0x3073), 0x00 },
	{ CCI_REG8(0x3074), 0x00 },
	{ CCI_REG8(0x3075), 0x00 },
	{ CCI_REG8(0x3076), 0x0a },
	{ CCI_REG8(0x3077), 0x03 },
	{ CCI_REG8(0x3078), 0x84 },
	{ CCI_REG8(0x3079), 0x00 },
	{ CCI_REG8(0x307a), 0x00 },
	{ CCI_REG8(0x307b), 0x00 },
	{ CCI_REG8(0x307c), 0x00 },
	{ CCI_REG8(0x3085), 0x00 },
	{ CCI_REG8(0x3086), 0x72 },
	{ CCI_REG8(0x30a6), 0x01 },
	{ CCI_REG8(0x30a7), 0x06 },
	{ CCI_REG8(0x3032), 0x01 },
	{ CCI_REG8(0x3037), 0x02 },
	{ CCI_REG8(0x304a), 0x01 },
	{ CCI_REG8(0x3054), 0xf0 },
	{ CCI_REG8(0x3044), 0x10 },
	{ CCI_REG8(0x3045), 0x20 },
	{ CCI_REG8(0x3047), 0x04 },
	{ CCI_REG8(0x3048), 0x14 },
	{ CCI_REG8(0x303d), 0x08 },
	{ CCI_REG8(0x304b), 0x44 },
	{ CCI_REG8(0x3063), 0x00 },
	{ CCI_REG8(0x302d), 0x7f },
	{ CCI_REG8(0x3039), 0x45 },
	{ CCI_REG8(0x3038), 0x10 },
	{ CCI_REG8(0x3097), 0x11 },
	{ CCI_REG8(0x3096), 0x03 },
	{ CCI_REG8(0x3042), 0x01 },
	{ CCI_REG8(0x3053), 0x01 },
	{ CCI_REG8(0x303a), 0x0b },
	{ CCI_REG8(0x320b), 0x40 },
	{ CCI_REG8(0x320c), 0x06 },
	{ CCI_REG8(0x320d), 0xc0 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x3d },
	{ CCI_REG8(0x3204), 0x00 },
	{ CCI_REG8(0x3205), 0x3d },
	{ CCI_REG8(0x3206), 0x00 },
	{ CCI_REG8(0x3207), 0x3d },
	{ CCI_REG8(0x3208), 0x00 },
	{ CCI_REG8(0x3209), 0x3d },
	{ CCI_REG8(0x3211), 0x02 },
	{ CCI_REG8(0x3212), 0x21 },
	{ CCI_REG8(0x3213), 0x02 },
	{ CCI_REG8(0x3214), 0x21 },
	{ CCI_REG8(0x3215), 0x02 },
	{ CCI_REG8(0x3216), 0x21 },
	{ CCI_REG8(0x3217), 0x02 },
	{ CCI_REG8(0x3218), 0x21 },
	{ CCI_REG8(0x3048), 0x14 },
	{ CCI_REG8(0x3244), 0x00 },
	{ CCI_REG8(0x3245), 0x00 },
	{ CCI_REG8(0x3246), 0x00 },
	{ CCI_REG8(0x3247), 0x00 },
	{ CCI_REG8(0x323f), 0x01 },
	{ CCI_REG8(0x3240), 0x01 },
	{ CCI_REG8(0x3241), 0x01 },
	{ CCI_REG8(0x3242), 0x01 },
	{ CCI_REG8(0x3264), 0x90 },
	{ CCI_REG8(0x3265), 0x90 },
	{ CCI_REG8(0x3266), 0x90 },
	{ CCI_REG8(0x3267), 0x90 },
	{ CCI_REG8(0x3269), 0x03 },
	{ CCI_REG8(0x3b29), 0x01 },
};

/*
 * Full-resolution mode: 3264x2448 @ ~30fps
 * line_length_pck = 3754, frame_length_lines = 2486
 * PLL override: pre_div=6, mult=140 → VCO=560MHz
 *   vt_pix_div=2 → pixel_clk=280MHz
 *   op_pix_div=2, op_sys_div=1
 * fps = 280MHz / (3754 * 2486) ≈ 30fps
 *
 * Register list from downstream CCI dump (50 registers).
 */
static const struct cci_reg_sequence s5k4h5yc_mode_3264x2448[] = {
	/* PLL1 overrides (change VCO from init 700MHz to 560MHz) */
	{ CCI_REG8(0x0301), 0x02 },	/* vt_pix_clk_div = 2 */
	{ CCI_REG8(0x0303), 0x01 },	/* vt_sys_clk_div = 1 */
	{ CCI_REG8(0x0305), 0x06 },	/* pre_pll_clk_div = 6 */
	{ CCI_REG8(0x0306), 0x00 },	/* pll_multiplier [15:8] */
	{ CCI_REG8(0x0307), 0x8c },	/* pll_multiplier [7:0] = 140 */
	{ CCI_REG8(0x0309), 0x02 },	/* op_pix_clk_div = 2 */
	{ CCI_REG8(0x030b), 0x01 },	/* op_sys_clk_div = 1 */

	/* PLL2 (secondary PLL for MIPI/DPHY timing) */
	{ CCI_REG8(0x030d), 0x06 },	/* pll2 pre_div = 6 */
	{ CCI_REG8(0x030e), 0x00 },	/* pll2 mult [15:8] */
	{ CCI_REG8(0x030f), 0xad },	/* pll2 mult [7:0] = 173 */
	{ CCI_REG8(0x0310), 0x01 },	/* pll2 op_sys_div = 1 */

	/* Samsung timing/analog tuning */
	{ CCI_REG8(0x3c59), 0x00 },
	{ CCI_REG8(0x3c5a), 0x00 },
	{ CCI_REG8(0x3c50), 0x53 },
	{ CCI_REG8(0x3c62), 0x02 },
	{ CCI_REG8(0x3c63), 0xb4 },
	{ CCI_REG8(0x3c64), 0x00 },
	{ CCI_REG8(0x3c65), 0x00 },
	{ CCI_REG8(0x3c1e), 0x00 },
	{ CCI_REG8(0x3c1a), 0xa8 },
	{ CCI_REG8(0x3500), 0x0c },

	/* Frame geometry */
	{ S5K4H5YC_REG_FLL,     2486 },
	{ S5K4H5YC_REG_LLP,     3754 },
	{ S5K4H5YC_REG_X_START, 0x0008 },
	{ S5K4H5YC_REG_Y_START, 0x0008 },
	{ S5K4H5YC_REG_X_END,   0x0cc7 },
	{ S5K4H5YC_REG_Y_END,   0x0997 },
	{ S5K4H5YC_REG_X_OUTPUT, 3264 },
	{ S5K4H5YC_REG_Y_OUTPUT, 2448 },

	/* Binning: off */
	{ CCI_REG8(0x0390), 0x00 },
	{ CCI_REG8(0x0391), 0x00 },
	{ CCI_REG8(0x0940), 0x00 },

	/* Subsampling: no skip */
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },

	/* AEC defaults */
	{ CCI_REG8(0x0204), 0x00 },	/* analog gain [15:8] */
	{ CCI_REG8(0x0205), 0x20 },	/* analog gain [7:0] = 32 */
	{ CCI_REG8(0x3030), 0x1b },
	{ CCI_REG8(0x0202), 0x04 },	/* coarse integration [15:8] */
	{ CCI_REG8(0x0203), 0xe2 },	/* coarse integration [7:0] */

	/* Image orientation */
	{ CCI_REG8(0x0101), 0x00 },
};

/*
 * 2x2 binned mode: 1632x1224 @ 30fps
 * Same PLL overrides as full-resolution mode.
 * fps = 280MHz / (3688 * 1266) ≈ 60fps
 */
static const struct cci_reg_sequence s5k4h5yc_mode_1632x1224[] = {
	/* PLL1 overrides (same as full-res) */
	{ CCI_REG8(0x0301), 0x02 },
	{ CCI_REG8(0x0303), 0x01 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x0309), 0x02 },
	{ CCI_REG8(0x030b), 0x01 },

	/* PLL2 */
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xad },
	{ CCI_REG8(0x0310), 0x01 },

	/* Samsung timing/analog tuning */
	{ CCI_REG8(0x3c59), 0x00 },
	{ CCI_REG8(0x3c5a), 0x00 },
	{ CCI_REG8(0x3c50), 0x53 },
	{ CCI_REG8(0x3c62), 0x02 },
	{ CCI_REG8(0x3c63), 0xb4 },
	{ CCI_REG8(0x3c64), 0x00 },
	{ CCI_REG8(0x3c65), 0x00 },
	{ CCI_REG8(0x3c1e), 0x00 },
	{ CCI_REG8(0x3c1a), 0xa8 },
	{ CCI_REG8(0x3500), 0x0c },

	/* Frame geometry */
	{ S5K4H5YC_REG_FLL,     1266 },
	{ S5K4H5YC_REG_LLP,     3688 },
	{ S5K4H5YC_REG_X_START, 0x0008 },
	{ S5K4H5YC_REG_Y_START, 0x0008 },
	{ S5K4H5YC_REG_X_END,   0x0cc7 },
	{ S5K4H5YC_REG_Y_END,   0x0997 },
	{ S5K4H5YC_REG_X_OUTPUT, 1632 },
	{ S5K4H5YC_REG_Y_OUTPUT, 1224 },

	/* Binning: 2x2 */
	{ S5K4H5YC_REG_BINNING_MODE, 0x0001 },
	{ S5K4H5YC_REG_BINNING_TYPE, 0x0022 },
	{ CCI_REG8(0x0940), 0x00 },

	/* Subsampling: 2x2 */
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },

	/* AEC defaults */
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x20 },
	{ CCI_REG8(0x3030), 0x1b },
	{ CCI_REG8(0x0202), 0x04 },
	{ CCI_REG8(0x0203), 0xe2 },

	/* Image orientation */
	{ CCI_REG8(0x0101), 0x00 },
};

static const struct s5k4h5yc_mode s5k4h5yc_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.fll_def = 2486,
		.llp = 3754,
		.regs = s5k4h5yc_mode_3264x2448,
		.num_regs = ARRAY_SIZE(s5k4h5yc_mode_3264x2448),
	},
	{
		.width = 1632,
		.height = 1224,
		.fll_def = 1266,
		.llp = 3688,
		.regs = s5k4h5yc_mode_1632x1224,
		.num_regs = ARRAY_SIZE(s5k4h5yc_mode_1632x1224),
	},
};

static const char * const s5k4h5yc_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade To Grey",
	"PN9",
};

/* ------------------------------------------------------------------ */
/* V4L2 Controls                                                       */
/* ------------------------------------------------------------------ */

static int s5k4h5yc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h5yc *sensor = ctrl_to_s5k4h5yc(ctrl);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = sensor->cur_mode->height + ctrl->val -
			  S5K4H5YC_EXPOSURE_OFFSET;
		__v4l2_ctrl_modify_range(sensor->exposure,
					S5K4H5YC_EXPOSURE_MIN, max,
					S5K4H5YC_EXPOSURE_STEP, max);
	}

	if (!pm_runtime_get_if_in_use(dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, S5K4H5YC_REG_ANALOG_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(sensor->regmap, S5K4H5YC_REG_DIG_GAIN,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, S5K4H5YC_REG_EXPOSURE,
			  ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, S5K4H5YC_REG_FLL,
			  sensor->cur_mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, S5K4H5YC_REG_ORIENTATION,
			  (sensor->hflip->val ? 1 : 0) |
			  (sensor->vflip->val ? 2 : 0), &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(sensor->regmap, S5K4H5YC_REG_TEST_PATTERN,
			  ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k4h5yc_ctrl_ops = {
	.s_ctrl = s5k4h5yc_set_ctrl,
};

static int s5k4h5yc_init_controls(struct s5k4h5yc *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5k4h5yc_mode *mode = sensor->cur_mode;
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

	v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K4H5YC_ANA_GAIN_MIN, S5K4H5YC_ANA_GAIN_MAX,
			  S5K4H5YC_ANA_GAIN_STEP, S5K4H5YC_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  S5K4H5YC_DGTL_GAIN_MIN, S5K4H5YC_DGTL_GAIN_MAX,
			  S5K4H5YC_DGTL_GAIN_STEP, S5K4H5YC_DGTL_GAIN_DEFAULT);

	sensor->exposure = v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4H5YC_EXPOSURE_MIN,
					     mode->fll_def -
					     S5K4H5YC_EXPOSURE_OFFSET,
					     S5K4H5YC_EXPOSURE_STEP,
					     mode->fll_def -
					     S5K4H5YC_EXPOSURE_OFFSET);

	sensor->vblank = v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops,
					   V4L2_CID_VBLANK,
					   S5K4H5YC_VBLANK_MIN,
					   S5K4H5YC_FLL_MAX - mode->height,
					   1,
					   mode->fll_def - mode->height);

	sensor->hblank = v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->llp - mode->width,
					   mode->llp - mode->width,
					   1,
					   mode->llp - mode->width);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->hflip = v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(hdl, &s5k4h5yc_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(hdl, &s5k4h5yc_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h5yc_test_pattern_menu) - 1,
				     0, 0, s5k4h5yc_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, &s5k4h5yc_ctrl_ops, &props);

	if (hdl->error)
		return hdl->error;

	sensor->sd.ctrl_handler = hdl;

	return 0;
}

/* ------------------------------------------------------------------ */
/* Streaming                                                           */
/* ------------------------------------------------------------------ */

static int s5k4h5yc_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	const struct s5k4h5yc_mode *mode = sensor->cur_mode;
	int ret;

	dev_info(dev, "s5k4h5yc enable_streams: pad=%u mask=0x%llx mode=%ux%u\n",
		 pad, streams_mask, mode->width, mode->height);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "s5k4h5yc: pm_runtime_resume_and_get failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "s5k4h5yc: writing %u mode registers\n", mode->num_regs);
	ret = cci_multi_reg_write(sensor->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret < 0) {
		dev_err(dev, "failed to apply mode settings: %d\n", ret);
		goto err_rpm_put;
	}

	ret = __v4l2_ctrl_handler_setup(&sensor->hdl);
	if (ret) {
		dev_err(dev, "s5k4h5yc: ctrl handler setup failed: %d\n", ret);
		goto err_rpm_put;
	}

	dev_info(dev, "s5k4h5yc: setting MODE_STREAMING\n");
	ret = cci_write(sensor->regmap, S5K4H5YC_MODE_SELECT,
			S5K4H5YC_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(dev, "s5k4h5yc: failed to set streaming mode: %d\n", ret);
		goto err_rpm_put;
	}

	dev_info(dev, "s5k4h5yc: streaming started successfully\n");

	__v4l2_ctrl_grab(sensor->vflip, true);
	__v4l2_ctrl_grab(sensor->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int s5k4h5yc_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    u32 pad, u64 streams_mask)
{
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);
	struct device *dev = regmap_get_device(sensor->regmap);
	int ret;

	ret = cci_write(sensor->regmap, S5K4H5YC_MODE_SELECT,
			S5K4H5YC_MODE_STANDBY, NULL);

	__v4l2_ctrl_grab(sensor->vflip, false);
	__v4l2_ctrl_grab(sensor->hflip, false);

	pm_runtime_put_autosuspend(dev);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Pad operations                                                      */
/* ------------------------------------------------------------------ */

static int s5k4h5yc_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int s5k4h5yc_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k4h5yc_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = s5k4h5yc_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h5yc_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k4h5yc_set_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);
	const struct s5k4h5yc_mode *mode;
	struct v4l2_mbus_framefmt *fmt;

	mode = v4l2_find_nearest_size(s5k4h5yc_modes,
				      ARRAY_SIZE(s5k4h5yc_modes),
				      width, height,
				      format->format.width,
				      format->format.height);

	fmt = v4l2_subdev_state_get_format(state, format->pad);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->cur_mode = mode;

		__v4l2_ctrl_modify_range(sensor->vblank,
					 S5K4H5YC_VBLANK_MIN,
					 S5K4H5YC_FLL_MAX - mode->height,
					 1, mode->fll_def - mode->height);
		__v4l2_ctrl_s_ctrl(sensor->vblank,
				   mode->fll_def - mode->height);
		__v4l2_ctrl_modify_range(sensor->hblank,
					 mode->llp - mode->width,
					 mode->llp - mode->width,
					 1, mode->llp - mode->width);
		__v4l2_ctrl_modify_range(sensor->exposure,
					 S5K4H5YC_EXPOSURE_MIN,
					 mode->fll_def -
					 S5K4H5YC_EXPOSURE_OFFSET,
					 S5K4H5YC_EXPOSURE_STEP,
					 mode->fll_def -
					 S5K4H5YC_EXPOSURE_OFFSET);
	}

	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->field = V4L2_FIELD_NONE;

	format->format = *fmt;

	/* Update the crop rectangle to match the active pixel area. */
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, format->pad);
	crop->left = S5K4H5YC_PIXEL_ARRAY_LEFT;
	crop->top = S5K4H5YC_PIXEL_ARRAY_TOP;
	crop->width = S5K4H5YC_ACTIVE_WIDTH;
	crop->height = S5K4H5YC_ACTIVE_HEIGHT;

	return 0;
}

static int s5k4h5yc_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);
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
	crop->left = S5K4H5YC_PIXEL_ARRAY_LEFT;
	crop->top = S5K4H5YC_PIXEL_ARRAY_TOP;
	crop->width = S5K4H5YC_ACTIVE_WIDTH;
	crop->height = S5K4H5YC_ACTIVE_HEIGHT;

	return 0;
}

static int s5k4h5yc_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = S5K4H5YC_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K4H5YC_PIXEL_ARRAY_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = S5K4H5YC_PIXEL_ARRAY_TOP;
		sel->r.left = S5K4H5YC_PIXEL_ARRAY_LEFT;
		sel->r.width = S5K4H5YC_ACTIVE_WIDTH;
		sel->r.height = S5K4H5YC_ACTIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_video_ops s5k4h5yc_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h5yc_pad_ops = {
	.enum_mbus_code = s5k4h5yc_enum_mbus_code,
	.enum_frame_size = s5k4h5yc_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k4h5yc_set_format,
	.get_selection = s5k4h5yc_get_selection,
	.enable_streams = s5k4h5yc_enable_streams,
	.disable_streams = s5k4h5yc_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h5yc_ops = {
	.video = &s5k4h5yc_video_ops,
	.pad = &s5k4h5yc_pad_ops,
};

static const struct media_entity_operations s5k4h5yc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops s5k4h5yc_internal_ops = {
	.init_state = s5k4h5yc_init_state,
};

/* ------------------------------------------------------------------ */
/* Power management                                                    */
/* ------------------------------------------------------------------ */

/*
 * Power sequence from downstream vendor binary (libmmcamera_s5k4h5yc.so):
 *   Power-on:  AVDD → DVDD → VIO → RESET(high) → MCLK
 *   Power-off: MCLK → RESET(low) → VIO → DVDD → AVDD
 */
static int s5k4h5yc_power_on(struct s5k4h5yc *sensor)
{
	int ret, i;

	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);

	/* Enable supplies one-by-one: AVDD → DVDD → VIO */
	for (i = 0; i < S5K4H5YC_NUM_SUPPLIES; i++) {
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

static void s5k4h5yc_power_off(struct s5k4h5yc *sensor)
{
	int i;

	clk_disable_unprepare(sensor->extclk);

	usleep_range(1000, 1500);

	if (sensor->reset)
		gpiod_set_value_cansleep(sensor->reset, 1);

	usleep_range(1000, 1500);

	/* Disable supplies in reverse: VIO → DVDD → AVDD */
	for (i = S5K4H5YC_NUM_SUPPLIES - 1; i >= 0; i--) {
		regulator_disable(sensor->supplies[i].consumer);
		usleep_range(1000, 1500);
	}
}

static int s5k4h5yc_initialize(struct s5k4h5yc *sensor)
{
	return cci_multi_reg_write(sensor->regmap, s5k4h5yc_global_init,
				  ARRAY_SIZE(s5k4h5yc_global_init), NULL);
}

static int __maybe_unused s5k4h5yc_pm_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);
	int ret;

	ret = s5k4h5yc_power_on(sensor);
	if (ret)
		return ret;

	ret = s5k4h5yc_initialize(sensor);
	if (ret) {
		s5k4h5yc_power_off(sensor);
		return ret;
	}

	return 0;
}

static int __maybe_unused s5k4h5yc_pm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);

	s5k4h5yc_power_off(sd_to_s5k4h5yc(sd));
	return 0;
}

static const struct dev_pm_ops s5k4h5yc_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h5yc_pm_suspend, s5k4h5yc_pm_resume, NULL)
};

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int s5k4h5yc_identify(struct s5k4h5yc *sensor)
{
	struct device *dev = regmap_get_device(sensor->regmap);
	u64 chip_id;
	int ret;

	ret = cci_read(sensor->regmap, S5K4H5YC_CHIP_ID, &chip_id, NULL);
	if (ret)
		return ret;

	if (chip_id != S5K4H5YC_CHIP_ID_VAL) {
		dev_err(dev, "chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			S5K4H5YC_CHIP_ID_VAL, chip_id);
		return -ENXIO;
	}

	dev_dbg(dev, "Samsung S5K4H5YC sensor found (id=0x%04llx)\n", chip_id);
	return 0;
}

static int s5k4h5yc_parse_dt(struct s5k4h5yc *sensor)
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

	if (ndata_lanes < 2 || ndata_lanes > 4) {
		dev_err(dev, "unsupported number of data lanes: %u\n",
			ndata_lanes);
		v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
		return -EINVAL;
	}

	/*
	 * Mode table overrides PLL: VCO=560MHz, pixel_clk=280MHz.
	 * link_freq = pixel_rate * bpp / (2 * lanes) = 280*10/8 = 350MHz.
	 * pixel_rate = 2 * link_freq * num_lanes / bpp
	 */
	sensor->link_freq_val = 350000000LL;
	sensor->pixel_rate_val = 2ULL * sensor->link_freq_val * ndata_lanes / 10;

	return 0;
}

static int s5k4h5yc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k4h5yc *sensor;
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

	ret = clk_set_rate(sensor->extclk, S5K4H5YC_MCLK_FREQ);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock rate\n");

	sensor->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	dev_info(dev, "probe: clk/gpio OK, getting regulators\n");

	for (ret = 0; ret < S5K4H5YC_NUM_SUPPLIES; ret++)
		sensor->supplies[ret].supply = s5k4h5yc_supply_names[ret];

	ret = devm_regulator_bulk_get(dev, S5K4H5YC_NUM_SUPPLIES,
				     sensor->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	dev_info(dev, "probe: regulators OK, parsing DT\n");

	ret = s5k4h5yc_parse_dt(sensor);
	if (ret < 0)
		return ret;

	dev_info(dev, "probe: DT OK, powering on\n");

	ret = s5k4h5yc_power_on(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to power on\n");
		goto err_ep_free;
	}

	dev_info(dev, "probe: power on OK, reading chip ID\n");

	ret = s5k4h5yc_identify(sensor);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to identify sensor\n");
		goto err_power_off;
	}

	ret = s5k4h5yc_initialize(sensor);
	if (ret < 0)
		goto err_power_off;

	sensor->cur_mode = &s5k4h5yc_modes[0];

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5k4h5yc_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.internal_ops = &s5k4h5yc_internal_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &s5k4h5yc_entity_ops;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		goto err_power_off;

	ret = s5k4h5yc_init_controls(sensor);
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
	s5k4h5yc_power_off(sensor);
err_ep_free:
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);
	return ret;
}

static void s5k4h5yc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h5yc *sensor = sd_to_s5k4h5yc(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->hdl);
	v4l2_fwnode_endpoint_free(&sensor->bus_cfg);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k4h5yc_power_off(sensor);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id s5k4h5yc_of_match[] = {
	{ .compatible = "samsung,s5k4h5yc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h5yc_of_match);

static struct i2c_driver s5k4h5yc_i2c_driver = {
	.driver = {
		.name = "s5k4h5yc",
		.of_match_table = s5k4h5yc_of_match,
		.pm = &s5k4h5yc_pm_ops,
	},
	.probe = s5k4h5yc_probe,
	.remove = s5k4h5yc_remove,
};
module_i2c_driver(s5k4h5yc_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K4H5YC CMOS Image Sensor driver");
MODULE_LICENSE("GPL");
