// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S2MU004 PMIC – MUIC switch & charger path init driver.
 *
 * The S2MU004 is a multi-function PMIC at I2C address 0x3D whose MUIC
 * block sits between the DWC3 controller and the USB Type-C connector.
 * Its internal analog switches default to open, so the USB D+/D- lines
 * are disconnected until the MUIC is explicitly told to close them.
 *
 * In addition to routing USB data lines, this driver puts the S2MU004's
 * internal charger block into buck mode so that VBUS power reaches the
 * system rail (the BQ25898S sub-charger handles actual battery charging).
 *
 * Two-phase approach:
 *  1) Register as a standard I2C driver (OF/DT matching).
 *  2) late_initcall fallback – if the normal probe path never ran
 *     (observed on some mainline DT setups), talk to the MUIC directly
 *     via i2c_smbus_xfer on the adapter.
 *
 * MUIC configuration:
 *  - Register 0xC7 (CTRL1): set SWITCH_OPEN + MANUAL_SW -> manual mode
 *    NOTE: despite its name, SWITCH_OPEN=1 means "normal operation"
 *    (switch follows manual/auto control).  SWITCH_OPEN=0 forces the
 *    analog switch disconnected.  Downstream keeps SWITCH_OPEN=1.
 *  - Register 0xCA (SW_CTRL): set 0x26 -> D+/D- to USB, VBUS to charger
 *
 * Charger configuration:
 *  - Register 0x10 (CHG_CTRL0): set bits [3:0] to BUCK_MODE (0x01)
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#define S2MU004_I2C_ADDR		0x3D
#define S2MU004_I2C_BUS			12	/* DT alias i2c12 */

/* Chip identification */
#define S2MU004_REG_REV_ID		0x82

/* Charger block registers */
#define S2MU004_CHG_CTRL0		0x10
#define  REG_MODE_MASK			0x0F
#define  CHARGER_OFF_MODE		0x00
#define  BUCK_MODE			0x01
#define  CHG_MODE			0x03

/* MUIC block registers */
#define S2MU004_MUIC_CTRL1		0xC7
#define  CTRL1_SWITCH_OPEN		BIT(4)
#define  CTRL1_RAW_DATA			BIT(3)
#define  CTRL1_MANUAL_SW		BIT(2)
#define  CTRL1_WAIT			BIT(1)
#define  CTRL1_INT_MASK			BIT(0)

#define S2MU004_MUIC_SW_CTRL		0xCA
/*
 * MANSW_USB: Route D+/D- to USB, VBUS to charger path.
 * SW_CTRL layout:[7:5] D- path | [4:2] D+ path | [1] VBUS | [0] OTG
 * USB = 001 for both D- and D+, VBUS = 1
 * -> (1<<5) | (1<<2) | (1<<1) = 0x26
 */
#define  MANSW_USB			0x26

/*
 * RID_CTRL (0xCC): controls MUIC RID/ADC detection.
 * Setting ADC_OFF (bit 1) disables the analogue RID/ADC engine
 * which otherwise monitors D+/D- and interferes with USB traffic.
 */
#define S2MU004_MUIC_RID_CTRL		0xCC
#define  RID_CTRL_ADC_OFF		BIT(1)

/*
 * AFC_OTP6 (0xDA): controls VBUS detection for charger (BC1.2).
 * When EN_VBUS_DET_MUIC (bit 4) is set the MUIC runs its internal
 * BC1.2 charger-detection state-machine on every VBUS change, which
 * actively drives D+/D- for DCD/SDP/CDP/DCP detection.  This MUST
 * be cleared before enabling the USB gadget path.
 */
#define S2MU004_MUIC_AFC_OTP6		0xDA
#define  AFC_OTP6_EN_VBUS_DET		BIT(4)

/* AFC OTP tuning registers used by downstream MUIC init sequence */
#define S2MU004_MUIC_AFC_OTP8		0xD4
#define S2MU004_MUIC_AFC_OTP9		0xD5
#define  AFC_OTP8_DOWNSTREAM_INIT	0x1F
#define  AFC_OTP9_DOWNSTREAM_INIT	0x5C

/* MUIC interrupt mask registers */
#define S2MU004_MUIC_INT1_MASK		0x08
#define S2MU004_MUIC_INT2_MASK		0x09
#define  INT_PDIC_MASK1			0xFC
#define  INT_PDIC_MASK2			0x7A

/* MUIC interrupt status registers */
#define S2MU004_MUIC_INT1		0x06
#define S2MU004_MUIC_INT2		0x07

/* MUIC detection/status registers */
#define S2MU004_MUIC_ADC		0x61
#define S2MU004_MUIC_DEVICE_TYPE1	0x62
#define S2MU004_MUIC_DEVICE_TYPE2	0x63
#define S2MU004_MUIC_DEVICE_TYPE3	0x64
#define S2MU004_MUIC_CHG_TYPE		0x68
#define S2MU004_MUIC_DEVICE_APPLE	0x69

/* MUIC INT1 bits */
#define INT1_DETACH			BIT(1)
#define INT1_ATTACH			BIT(0)

/* MUIC INT2 bits */
#define INT2_VBUS_OFF			BIT(7)
#define INT2_ADC_CHANGE			BIT(2)
#define INT2_VBUS_ON			BIT(0)

/* MUIC device/chg type bits used for USB attach detection */
#define DEV_TYPE1_USB_OTG		BIT(7)
#define DEV_TYPE1_CDP			BIT(5)
#define DEV_TYPE1_USB			BIT(2)
#define DEV_TYPE1_USB_TYPES		(DEV_TYPE1_USB_OTG | DEV_TYPE1_CDP | DEV_TYPE1_USB)
#define DEV_TYPE2_JIG_USB_OFF		BIT(1)
#define DEV_TYPE2_JIG_USB_ON		BIT(0)
#define DEV_TYPE2_JIG_USB_TYPES		(DEV_TYPE2_JIG_USB_OFF | DEV_TYPE2_JIG_USB_ON)
#define DEV_TYPE_APPLE_VBUS_WAKEUP	BIT(1)
#define CHG_TYPE_USB			BIT(2)
#define CHG_TYPE_CDP			BIT(1)

/*
 * State shared between probe and late_initcall fallback paths.
 * TODO: migrate to per-device drvdata once the dual-path init
 * (I2C driver + raw adapter fallback) is unified.
 */
static bool s2mu004_configured;
static bool s2mu004_usb_attached;
static struct i2c_client *s2mu004_saved_client;
static struct delayed_work s2mu004_debug_work;

enum s2mu004_attach_mode {
	S2MU004_NONE_CABLE = 0,
	S2MU004_FIRST_ATTACH,
	S2MU004_SECOND_ATTACH,
	S2MU004_MUIC_DETACH,
};

static enum s2mu004_attach_mode s2mu004_attach_mode = S2MU004_NONE_CABLE;

/* ---- raw SMBus helpers for fallback path (no i2c_client) ---- */

static int s2mu004_raw_read(struct i2c_adapter *adap, u8 reg, u8 *val)
{
	union i2c_smbus_data data;
	int ret;

	ret = i2c_smbus_xfer(adap, S2MU004_I2C_ADDR, 0,
			     I2C_SMBUS_READ, reg,
			     I2C_SMBUS_BYTE_DATA, &data);
	if (ret)
		return ret;

	*val = data.byte;
	return 0;
}

static int s2mu004_raw_write(struct i2c_adapter *adap, u8 reg, u8 val)
{
	union i2c_smbus_data data = { .byte = val };

	return i2c_smbus_xfer(adap, S2MU004_I2C_ADDR, 0,
			      I2C_SMBUS_WRITE, reg,
			      I2C_SMBUS_BYTE_DATA, &data);
}

/* ---- shared configuration sequences ---- */

static int s2mu004_enable_usb_path_smbus(
		int (*rd)(void *ctx, u8 reg, u8 *val),
		int (*wr)(void *ctx, u8 reg, u8 val),
		void *ctx, const char *tag)
{
	u8 ctrl1, reg;
	int ret;

	/*
	 * Step 1: Disable MUIC's internal VBUS / BC1.2 charger detection.
	 * If left enabled the MUIC actively drives D+/D- for DCD and
	 * charger-type detection, which prevents the DWC3 gadget from
	 * asserting a clean D+ pull-up to the host.
	 */
	ret = rd(ctx, S2MU004_MUIC_AFC_OTP6, &reg);
	if (ret) {
		pr_err("s2mu004: [%s] read AFC_OTP6 failed: %d\n", tag, ret);
		return ret;
	}
	reg &= ~AFC_OTP6_EN_VBUS_DET;
	ret = wr(ctx, S2MU004_MUIC_AFC_OTP6, reg);
	if (ret) {
		pr_err("s2mu004: [%s] write AFC_OTP6 failed: %d\n", tag, ret);
		return ret;
	}
	pr_info("s2mu004: [%s] VBUS detection disabled (AFC_OTP6=0x%02x)\n",
		tag, reg);

	/*
	 * Step 2: Temporarily disable the RID/ADC engine.  This mirrors
	 * downstream sequencing (off during switch programming, then on).
	 */
	ret = rd(ctx, S2MU004_MUIC_RID_CTRL, &reg);
	if (ret) {
		pr_err("s2mu004: [%s] read RID_CTRL failed: %d\n", tag, ret);
		return ret;
	}
	reg |= RID_CTRL_ADC_OFF;
	ret = wr(ctx, S2MU004_MUIC_RID_CTRL, reg);
	if (ret) {
		pr_err("s2mu004: [%s] write RID_CTRL failed: %d\n", tag, ret);
		return ret;
	}
	pr_info("s2mu004: [%s] RID/ADC disabled (RID_CTRL=0x%02x)\n",
		tag, reg);

	/* Step 3: Apply downstream PDIC interrupt masks. */
	ret = wr(ctx, S2MU004_MUIC_INT1_MASK, INT_PDIC_MASK1);
	if (ret) {
		pr_err("s2mu004: [%s] write INT1_MASK failed: %d\n", tag, ret);
		return ret;
	}

	ret = wr(ctx, S2MU004_MUIC_INT2_MASK, INT_PDIC_MASK2);
	if (ret) {
		pr_err("s2mu004: [%s] write INT2_MASK failed: %d\n", tag, ret);
		return ret;
	}

	/* Step 4: Apply downstream AFC OTP tuning values. */
	ret = wr(ctx, S2MU004_MUIC_AFC_OTP8, AFC_OTP8_DOWNSTREAM_INIT);
	if (ret) {
		pr_err("s2mu004: [%s] write AFC_OTP8 failed: %d\n", tag, ret);
		return ret;
	}

	ret = wr(ctx, S2MU004_MUIC_AFC_OTP9, AFC_OTP9_DOWNSTREAM_INIT);
	if (ret) {
		pr_err("s2mu004: [%s] write AFC_OTP9 failed: %d\n", tag, ret);
		return ret;
	}

	/* Downstream logs show this register being programmed twice. */
	ret = wr(ctx, S2MU004_MUIC_AFC_OTP9, AFC_OTP9_DOWNSTREAM_INIT);
	if (ret) {
		pr_err("s2mu004: [%s] re-write AFC_OTP9 failed: %d\n", tag, ret);
		return ret;
	}

	/*
	 * Step 5: Configure CTRL1 – manual switch mode, switches closed,
	 * interrupts masked.
	 */
	ret = rd(ctx, S2MU004_MUIC_CTRL1, &ctrl1);
	if (ret) {
		pr_err("s2mu004: [%s] read CTRL1 failed: %d\n", tag, ret);
		return ret;
	}
	pr_info("s2mu004: [%s] CTRL1=0x%02x (before)\n", tag, ctrl1);

	/*
	 * Despite its name, SWITCH_OPEN=1 means "normal operation" —
	 * the analog switch follows the manual/auto path selection.
	 * SWITCH_OPEN=0 forces the switch disconnected.  Downstream
	 * always keeps this bit set (CTRL1=0x17 during USB operation).
	 */
	ctrl1 |= CTRL1_SWITCH_OPEN;    /* enable switch (required!) */
	ctrl1 |= CTRL1_MANUAL_SW;      /* manual switch control */
	ctrl1 |= CTRL1_WAIT;           /* wait for detection */
	ctrl1 |= CTRL1_INT_MASK;       /* mask CTRL1 interrupt */
	ret = wr(ctx, S2MU004_MUIC_CTRL1, ctrl1);
	if (ret) {
		pr_err("s2mu004: [%s] write CTRL1 failed: %d\n", tag, ret);
		return ret;
	}

	/* Step 6: Route D+/D- to USB, VBUS to charger path */
	ret = wr(ctx, S2MU004_MUIC_SW_CTRL, MANSW_USB);
	if (ret) {
		pr_err("s2mu004: [%s] write SW_CTRL failed: %d\n", tag, ret);
		return ret;
	}

	/* Step 7: Re-enable RID/ADC after switch programming. */
	usleep_range(10000, 12000);
	ret = rd(ctx, S2MU004_MUIC_RID_CTRL, &reg);
	if (ret) {
		pr_err("s2mu004: [%s] read RID_CTRL(restore) failed: %d\n", tag, ret);
		return ret;
	}
	reg &= ~RID_CTRL_ADC_OFF;
	ret = wr(ctx, S2MU004_MUIC_RID_CTRL, reg);
	if (ret) {
		pr_err("s2mu004: [%s] write RID_CTRL(restore) failed: %d\n", tag, ret);
		return ret;
	}

	/* Readback verification */
	{
		u8 v_ctrl1, v_sw;

		rd(ctx, S2MU004_MUIC_CTRL1, &v_ctrl1);
		rd(ctx, S2MU004_MUIC_SW_CTRL, &v_sw);
		pr_info("s2mu004: [%s] USB data-path enabled (CTRL1=0x%02x SW=0x%02x)\n",
			tag, v_ctrl1, v_sw);
	}
	return 0;
}

static int s2mu004_init_charger_buck(
		int (*rd)(void *ctx, u8 reg, u8 *val),
		int (*wr)(void *ctx, u8 reg, u8 val),
		void *ctx, const char *tag)
{
	u8 ctrl0;
	int ret;

	ret = rd(ctx, S2MU004_CHG_CTRL0, &ctrl0);
	if (ret) {
		pr_err("s2mu004: [%s] read CHG_CTRL0 failed: %d\n", tag, ret);
		return ret;
	}

	pr_info("s2mu004: [%s] CHG_CTRL0=0x%02x (mode=%d)\n",
		tag, ctrl0, ctrl0 & REG_MODE_MASK);

	/* Set to BUCK_MODE for system power passthrough */
	ctrl0 = (ctrl0 & ~REG_MODE_MASK) | BUCK_MODE;
	ret = wr(ctx, S2MU004_CHG_CTRL0, ctrl0);
	if (ret) {
		pr_err("s2mu004: [%s] write CHG_CTRL0 failed: %d\n", tag, ret);
		return ret;
	}

	pr_info("s2mu004: [%s] charger set to buck mode\n", tag);
	return 0;
}

/* ---- I2C client wrappers ---- */

static int s2mu004_client_read(void *ctx, u8 reg, u8 *val)
{
	struct i2c_client *c = ctx;
	int ret = i2c_smbus_read_byte_data(c, reg);

	if (ret < 0)
		return ret;
	*val = ret;
	return 0;
}

static int s2mu004_client_write(void *ctx, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(ctx, reg, val);
}

/* ---- adapter (raw) wrappers ---- */

static int s2mu004_adap_read(void *ctx, u8 reg, u8 *val)
{
	return s2mu004_raw_read(ctx, reg, val);
}

static int s2mu004_adap_write(void *ctx, u8 reg, u8 val)
{
	return s2mu004_raw_write(ctx, reg, val);
}

/* ---- MUIC interrupt-driven cable detection ---- */

static void s2mu004_muic_dump_regs(struct i2c_client *client, const char *tag)
{
	u8 ctrl1 = 0, sw = 0, rid = 0, otp6 = 0;

	s2mu004_client_read(client, S2MU004_MUIC_CTRL1, &ctrl1);
	s2mu004_client_read(client, S2MU004_MUIC_SW_CTRL, &sw);
	s2mu004_client_read(client, S2MU004_MUIC_RID_CTRL, &rid);
	s2mu004_client_read(client, S2MU004_MUIC_AFC_OTP6, &otp6);

	dev_info(&client->dev,
		 "[%s] CTRL1=0x%02x SW_CTRL=0x%02x RID_CTRL=0x%02x AFC_OTP6=0x%02x\n",
		 tag, ctrl1, sw, rid, otp6);
}

static void s2mu004_debug_work_fn(struct work_struct *work)
{
	struct i2c_client *client = s2mu004_saved_client;
	u8 dev1 = 0, apple = 0, chg = 0;

	if (!client)
		return;

	dev_info(&client->dev, "=== MUIC DELAYED RE-CHECK (5s after probe) ===\n");
	s2mu004_muic_dump_regs(client, "delayed-recheck");

	s2mu004_client_read(client, S2MU004_MUIC_DEVICE_TYPE1, &dev1);
	s2mu004_client_read(client, S2MU004_MUIC_DEVICE_APPLE, &apple);
	s2mu004_client_read(client, S2MU004_MUIC_CHG_TYPE, &chg);

	dev_info(&client->dev,
		 "[delayed] DEV1=0x%02x APPLE=0x%02x CHG=0x%02x attached=%d mode=%d\n",
		 dev1, apple, chg, s2mu004_usb_attached, s2mu004_attach_mode);

	/* Schedule another re-check at 15s */
	schedule_delayed_work(&s2mu004_debug_work, msecs_to_jiffies(10000));
}

static void s2mu004_muic_detect_state(struct i2c_client *client,
				      u8 int1, u8 int2)
{
	u8 dev1 = 0, dev2 = 0, dev3 = 0, adc = 0, apple = 0, chg = 0;
	bool vbus_high, usb_detected;
	int ret;

	ret = s2mu004_client_read(client, S2MU004_MUIC_DEVICE_TYPE1, &dev1);
	if (ret)
		return;
	ret = s2mu004_client_read(client, S2MU004_MUIC_DEVICE_TYPE2, &dev2);
	if (ret)
		return;
	ret = s2mu004_client_read(client, S2MU004_MUIC_DEVICE_TYPE3, &dev3);
	if (ret)
		return;
	ret = s2mu004_client_read(client, S2MU004_MUIC_ADC, &adc);
	if (ret)
		return;
	ret = s2mu004_client_read(client, S2MU004_MUIC_DEVICE_APPLE, &apple);
	if (ret)
		return;
	ret = s2mu004_client_read(client, S2MU004_MUIC_CHG_TYPE, &chg);
	if (ret)
		return;

	vbus_high = !!(apple & DEV_TYPE_APPLE_VBUS_WAKEUP);
	usb_detected = !!(dev1 & DEV_TYPE1_USB_TYPES) ||
		       !!(dev2 & DEV_TYPE2_JIG_USB_TYPES) ||
		       !!(chg & (CHG_TYPE_USB | CHG_TYPE_CDP));

	if ((int2 & INT2_VBUS_OFF) && s2mu004_attach_mode == S2MU004_SECOND_ATTACH)
		s2mu004_attach_mode = S2MU004_MUIC_DETACH;

	if ((int1 & INT1_DETACH) && s2mu004_attach_mode == S2MU004_FIRST_ATTACH)
		s2mu004_attach_mode = S2MU004_MUIC_DETACH;

	if ((int1 & INT1_ATTACH) && !vbus_high && !s2mu004_usb_attached)
		s2mu004_attach_mode = S2MU004_FIRST_ATTACH;

	dev_info(&client->dev,
		 "MUIC detect: mode=%d dev[1:0x%02x 2:0x%02x 3:0x%02x] adc:0x%02x vbus:%d apple:0x%02x chg:0x%02x\n",
		 s2mu004_attach_mode, dev1, dev2, dev3, adc, vbus_high, apple, chg);

	if ((int1 & INT1_DETACH) && vbus_high && s2mu004_usb_attached) {
		dev_info(&client->dev,
			 "MUIC DETACH with VBUS high, skip detach handling\n");
		s2mu004_attach_mode = S2MU004_SECOND_ATTACH;
		return;
	}

	if (usb_detected ||
	    ((int2 & INT2_VBUS_ON) &&
	     (s2mu004_attach_mode == S2MU004_FIRST_ATTACH ||
	      s2mu004_attach_mode == S2MU004_MUIC_DETACH))) {
		s2mu004_enable_usb_path_smbus(s2mu004_client_read,
					      s2mu004_client_write,
					      client, "irq-detect");
		s2mu004_init_charger_buck(s2mu004_client_read,
					 s2mu004_client_write,
					 client, "irq-detect");
		s2mu004_usb_attached = true;
		s2mu004_attach_mode = S2MU004_SECOND_ATTACH;
		return;
	}

	if ((int1 & INT1_DETACH) && !vbus_high) {
		s2mu004_usb_attached = false;
		s2mu004_attach_mode = S2MU004_NONE_CABLE;
	}
}

static irqreturn_t s2mu004_muic_irq_thread(int irq, void *data)
{
	struct i2c_client *client = data;
	u8 int1 = 0, int2 = 0;
	int ret;

	ret = s2mu004_client_read(client, S2MU004_MUIC_INT1, &int1);
	if (ret)
		return IRQ_HANDLED;

	ret = s2mu004_client_read(client, S2MU004_MUIC_INT2, &int2);
	if (ret)
		return IRQ_HANDLED;

	dev_info(&client->dev, "MUIC IRQ: INT1=0x%02x INT2=0x%02x\n", int1, int2);

	if ((int1 & (INT1_ATTACH | INT1_DETACH)) ||
	    (int2 & (INT2_VBUS_ON | INT2_VBUS_OFF | INT2_ADC_CHANGE)))
		s2mu004_muic_detect_state(client, int1, int2);

	return IRQ_HANDLED;
}

/* ---- standard I2C driver path ---- */

static int s2mu004_muic_probe(struct i2c_client *client)
{
	u8 rev;
	int ret;

	dev_info(&client->dev, "s2mu004_muic_probe: enter (addr=0x%02x)\n",
		 client->addr);

	ret = s2mu004_client_read(client, S2MU004_REG_REV_ID, &rev);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read chip revision\n");

	dev_info(&client->dev, "S2MU004 rev=0x%02x at 0x%02x\n",
		 rev, client->addr);

	ret = s2mu004_enable_usb_path_smbus(
		s2mu004_client_read, s2mu004_client_write, client, "probe");
	if (ret)
		return ret;

	ret = s2mu004_init_charger_buck(
		s2mu004_client_read, s2mu004_client_write, client, "probe");
	if (ret)
		dev_warn(&client->dev, "charger buck init failed: %d\n", ret);

	s2mu004_usb_attached = false;
	s2mu004_attach_mode = S2MU004_NONE_CABLE;

	/* Dump register state after USB path configuration */
	s2mu004_muic_dump_regs(client, "post-probe");

	/* Force initial cable detection with explicit register reads */
	{
		u8 dev1 = 0, apple = 0, chg = 0;

		s2mu004_client_read(client, S2MU004_MUIC_DEVICE_TYPE1, &dev1);
		s2mu004_client_read(client, S2MU004_MUIC_DEVICE_APPLE, &apple);
		s2mu004_client_read(client, S2MU004_MUIC_CHG_TYPE, &chg);

		dev_info(&client->dev,
			 "probe initial state: DEV1=0x%02x APPLE=0x%02x CHG=0x%02x\n",
			 dev1, apple, chg);

		if ((dev1 & DEV_TYPE1_USB_TYPES) ||
		    (chg & (CHG_TYPE_USB | CHG_TYPE_CDP)) ||
		    (apple & DEV_TYPE_APPLE_VBUS_WAKEUP)) {
			dev_info(&client->dev,
				 "probe: USB or VBUS detected\n");
			s2mu004_usb_attached = true;
			s2mu004_attach_mode = S2MU004_SECOND_ATTACH;
		}
	}

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, s2mu004_muic_irq_thread,
						IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
						"s2mu004-muic", client);
		if (ret) {
			dev_warn(&client->dev,
				 "failed to request MUIC IRQ %d: %d\n",
				 client->irq, ret);
		} else {
			dev_info(&client->dev, "MUIC IRQ enabled on %d\n", client->irq);
			device_init_wakeup(&client->dev, true);
			enable_irq_wake(client->irq);
			s2mu004_muic_irq_thread(-1, client);
		}
	} else {
		dev_warn(&client->dev, "no IRQ in DT, runtime MUIC events unavailable\n");
	}

	s2mu004_configured = true;
	s2mu004_saved_client = client;
	INIT_DELAYED_WORK(&s2mu004_debug_work, s2mu004_debug_work_fn);
	schedule_delayed_work(&s2mu004_debug_work, msecs_to_jiffies(5000));
	return 0;
}

static const struct of_device_id s2mu004_muic_of_match[] = {
	{ .compatible = "samsung,s2mu004-muic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu004_muic_of_match);

static const struct i2c_device_id s2mu004_muic_id[] = {
	{ "s2mu004-muic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mu004_muic_id);

static struct i2c_driver s2mu004_muic_driver = {
	.driver = {
		.name		= "s2mu004-muic-usb",
		.of_match_table	= s2mu004_muic_of_match,
	},
	.probe		= s2mu004_muic_probe,
	.id_table	= s2mu004_muic_id,
};

/* ---- init: register I2C driver ---- */

static int __init s2mu004_muic_init(void)
{
	int ret;

	ret = i2c_add_driver(&s2mu004_muic_driver);
	if (ret)
		pr_err("s2mu004: i2c_add_driver failed: %d\n", ret);
	else
		pr_info("s2mu004: driver registered, waiting for probe\n");

	return ret;
}
module_init(s2mu004_muic_init);

/* ---- fallback: direct adapter access if probe never ran ---- */

static int __init s2mu004_muic_fallback(void)
{
	struct i2c_adapter *adap;
	int ret;

	if (s2mu004_configured)
		return 0;

	pr_warn("s2mu004: OF probe did not fire, trying direct I2C on bus %d\n",
		S2MU004_I2C_BUS);

	adap = i2c_get_adapter(S2MU004_I2C_BUS);
	if (!adap) {
		pr_err("s2mu004: I2C adapter %d not found\n", S2MU004_I2C_BUS);
		return -ENODEV;
	}

	ret = s2mu004_enable_usb_path_smbus(
		s2mu004_adap_read, s2mu004_adap_write, adap, "fallback");
	if (!ret) {
		s2mu004_init_charger_buck(
			s2mu004_adap_read, s2mu004_adap_write,
			adap, "fallback");
		s2mu004_configured = true;
	}

	i2c_put_adapter(adap);
	return ret;
}
late_initcall(s2mu004_muic_fallback);

static void __exit s2mu004_muic_exit(void)
{
	i2c_del_driver(&s2mu004_muic_driver);
}
module_exit(s2mu004_muic_exit);

MODULE_DESCRIPTION("Samsung S2MU004 MUIC USB switch & charger path init");
MODULE_LICENSE("GPL");