/*
 *  Setup code for SAMA5 Evaluation Kits with Device Tree support
 *
 *  Copyright (C) 2013 Atmel,
 *                2013 Ludovic Desroches <ludovic.desroches@atmel.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/micrel_phy.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/firmware.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <linux/of_address.h>

#include "at91_aic.h"
#include "generic.h"
#include "clock.h"

/************************************/
/* TEMPORARY NON-DT STUFF FOR MIURA */
/************************************/
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <video/atmel_lcdfb.h>
#include <mach/atmel_hlcdc.h>
#include <media/soc_camera.h>
#include <media/atmel-isi.h>

#define LINK_SENSOR_MODULE_TO_SOC_CAMERA(_sensor_name, _soc_camera_id, _i2c_adapter_id)	\
	static struct soc_camera_desc iclink_##_sensor_name = {		\
		.subdev_desc = {					\
			.power = i2c_camera_power,			\
			.query_bus_param = isi_camera_query_bus_param,	\
		},							\
		.host_desc = {						\
			.bus_id		= -1,				\
			.board_info	= &i2c_##_sensor_name,		\
			.i2c_adapter_id	= _i2c_adapter_id,		\
		},							\
	};								\
	static struct platform_device isi_##_sensor_name = {		\
		.name	= "soc-camera-pdrv",				\
		.id	= _soc_camera_id,				\
		.dev	= {						\
			.platform_data = &iclink_##_sensor_name,	\
		},							\
	};

/*
 * LCD Controller
 */
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name		= "LG",
		.refresh	= 60,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(33260),

		.left_margin	= 88,		.right_margin	= 168,
		.upper_margin	= 8,		.lower_margin	= 37,
		.hsync_len	= 128,		.vsync_len	= 2,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "LG",
	.monitor	= "LB043WQ1",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};

/* Default output mode is TFT 24 bit */
#define BPP_OUT_DEFAULT_LCDCFG5	(LCDC_LCDCFG5_MODE_OUTPUT_24BPP)

/* Driver datas */
static struct atmel_lcdfb_info __initdata ek_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.alpha_enabled			= false,
	.default_bpp			= 16,
	/* Reserve enough memory for 32bpp */
	.smem_len			= 800 * 480 * 4,
	/* default_lcdcon2 is used for LCDCFG5 */
	.default_lcdcon2		= BPP_OUT_DEFAULT_LCDCFG5,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time			= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

/*
 *  ISI
 */
static struct isi_platform_data isi_data = {
	.frate			= ISI_CFG1_FRATE_CAPTURE_ALL,
	/* to use codec and preview path simultaneously */
	.full_mode		= 1,
	.data_width_flags	= ISI_DATAWIDTH_8 | ISI_DATAWIDTH_10,
	/* ISI_MCK is provided by programmable clock or external clock */
	.mck_hz			= 25000000,
};

static struct clk_lookup isi_mck_lookups[] = {
	CLKDEV_CON_DEV_ID("isi_mck", "atmel_isi", NULL),
};

void __init at91_config_isi(bool use_pck_as_mck, const char *pck_id)
{
	struct clk *pck;
	struct clk *parent;

	if (use_pck_as_mck) {
		pck = clk_get(NULL, pck_id);
		parent = clk_get(NULL, "plla");

		BUG_ON(IS_ERR(pck) || IS_ERR(parent));

		if (clk_set_parent(pck, parent)) {
			pr_err("Failed to set PCK's parent\n");
		} else {
			/* Register PCK as ISI_MCK */
			isi_mck_lookups[0].clk = pck;
			clkdev_add_table(isi_mck_lookups,
				ARRAY_SIZE(isi_mck_lookups));
		}

		clk_put(pck);
		clk_put(parent);
	}
}

static unsigned int camera_reset_pin;
static unsigned int camera_power_pin;
static void camera_set_gpio_pins(uint reset_pin, uint power_pin)
{
	camera_reset_pin = reset_pin;
	camera_power_pin = power_pin;
}

/*
 * soc-camera OV2640
 */
static unsigned long isi_camera_query_bus_param(struct soc_camera_subdev_desc *link)
{
	/* ISI board for ek using default 8-bits connection */
	return SOCAM_DATAWIDTH_8;
}

static int i2c_camera_power(struct device *dev, int on)
{
	int res, ret = 0;

	pr_debug("%s: %s the camera\n", __func__, on ? "ENABLE" : "DISABLE");

	res = devm_gpio_request(dev, camera_power_pin, "ov2640_power");
	if (res < 0) {
		printk("can't request ov2640_power pin\n");
		return -1;
	}

	res = devm_gpio_request(dev, camera_reset_pin, "ov2640_reset");
	if (res < 0) {
		printk("can't request ov2640_reset pin\n");
		devm_gpio_free(dev, camera_power_pin);
		return -1;
	}

	/* enable or disable the camera */
	res = gpio_direction_output(camera_power_pin, !on);
	if (res < 0) {
		printk("can't request output direction for ov2640_power pin\n");
		ret = -1;
		goto out;
	}

	if (!on)
		goto out;

	/* If enabled, give a reset impulse */
	res = gpio_direction_output(camera_reset_pin, 0);
	if (res < 0) {
		printk("can't request output direction for ov2640_reset pin\n");
		ret = -1;
		goto out;
	}
	msleep(20);
	res = gpio_direction_output(camera_reset_pin, 1);
	if (res < 0) {
		printk("can't request output direction for ov2640_reset pin\n");
		ret = -1;
		goto out;
	}
	msleep(100);

out:
	devm_gpio_free(dev, camera_reset_pin);
	devm_gpio_free(dev, camera_power_pin);
	return ret;
}

static struct i2c_board_info i2c_ov2640 = {
	I2C_BOARD_INFO("ov2640", 0x30),
};
static struct i2c_board_info i2c_ov5642 = {
	I2C_BOARD_INFO("ov5642", 0x3c),
};
static struct i2c_board_info i2c_ov9740 = {
	I2C_BOARD_INFO("ov9740", 0x10),
};
static struct i2c_board_info i2c_ov7740 = {
	I2C_BOARD_INFO("ov7740", 0x21),
};

LINK_SENSOR_MODULE_TO_SOC_CAMERA(ov2640, 0, 1);
LINK_SENSOR_MODULE_TO_SOC_CAMERA(ov5642, 1, 1);
LINK_SENSOR_MODULE_TO_SOC_CAMERA(ov9740, 2, 1);
LINK_SENSOR_MODULE_TO_SOC_CAMERA(ov7740, 3, 1);

static struct platform_device *sensors[] __initdata = {
	&isi_ov2640,
	&isi_ov5642,	/* compatible for ov5640 */
	&isi_ov9740,
	&isi_ov7740,
};

static void at91_fixup_isi_sensor_bus(struct platform_device **sensors,
		unsigned int sensors_nbr, unsigned int fixed_i2c_id)
{
	int i;
	struct platform_device **s;
	struct soc_camera_desc *scd;

	s = sensors;
	for (i = 0; i < sensors_nbr; i++) {
		if (s[i]) {
			scd = s[i]->dev.platform_data;
			if (scd)
				scd->host_desc.i2c_adapter_id = fixed_i2c_id;
		} else {
			break;
		}
	}
}

struct of_dev_auxdata at91_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf8038000, "atmel_hlcdfb_base", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf8038100, "atmel_hlcdfb_ovl1", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030000, "atmel_hlcdfb_base", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030140, "atmel_hlcdfb_ovl1", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030240, "atmel_hlcdfb_ovl2", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9g45-isi", 0xf0034000, "atmel_isi", &isi_data),
	OF_DEV_AUXDATA("atmel,at91sam9g45-isi", 0xf0008000, "atmel_isi", &isi_data),
	/* SAMA5D4 */
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0000000, "atmel_hlcdfb_base", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0000140, "atmel_hlcdfb_ovl1", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0000240, "atmel_hlcdfb_ovl2", &ek_lcdc_data),
	{ /* sentinel */ }
};

static const struct of_device_id irq_of_match[] __initconst = {

	{ .compatible = "atmel,sama5d3-aic", .data = sama5d3_aic5_of_init },
	{ .compatible = "atmel,sama5d4-aic", .data = sama5d4_aic5_of_init },
	{ /*sentinel*/ }
};

static void __init at91_dt_init_irq(void)
{
	of_irq_init(irq_of_match);
}

static int ksz9021rn_phy_fixup(struct phy_device *phy)
{
	int value;

#define GMII_RCCPSR	260
#define GMII_RRDPSR	261
#define GMII_ERCR	11
#define GMII_ERDWR	12

	/* Set delay values */
	value = GMII_RCCPSR | 0x8000;
	phy_write(phy, GMII_ERCR, value);
	value = 0xF2F4;
	phy_write(phy, GMII_ERDWR, value);
	value = GMII_RRDPSR | 0x8000;
	phy_write(phy, GMII_ERCR, value);
	value = 0x2222;
	phy_write(phy, GMII_ERDWR, value);

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	/*
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);
	*/
	mmd_write_reg(dev, 2, 4, 0x84);
	mmd_write_reg(dev, 2, 5, 0x4444);
	mmd_write_reg(dev, 2, 8, 0x1ef);

	return 0;
}

static int ksz8081_phy_reset(struct phy_device *phy)
{
	int value;

	/*
	 * As disconnect the hardware reset, so use software reset
	 *
	 * The basic control (register 0) bit 15 is software reset
	 */
	value = phy_read(phy, 0);
	value |= (1 << 15);
	phy_write(phy, 0, value);

	return 0;
}

static void __iomem *l2cc_base;

void __iomem *at91_get_l2cc_base(void)
{
	return l2cc_base;
}
EXPORT_SYMBOL_GPL(at91_get_l2cc_base);

#ifdef CONFIG_CACHE_L2X0
static void __init at91_init_l2cache(void)
{
	struct device_node *np;
	u32 reg;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
	if (!np)
		return;

	l2cc_base = of_iomap(np, 0);
	if (!l2cc_base)
		panic("unable to map l2cc cpu registers\n");

	of_node_put(np);

	ret = call_firmware_op(l2x0_init);
	if (ret == -ENOSYS) {
		/* Disable cache if it hasn't been done yet */
		if (readl_relaxed(l2cc_base + L2X0_CTRL) & L2X0_CTRL_EN)
			writel_relaxed(~L2X0_CTRL_EN, l2cc_base + L2X0_CTRL);

		/* Prefetch Control */
		reg = readl_relaxed(l2cc_base + L2X0_PREFETCH_CTRL);
		reg &= ~L2X0_PCR_OFFSET_MASK;
		reg |= L2X0_PCR_OFFSET_(0x01);
		reg |= L2X0_PCR_IDLEN;
		reg |= L2X0_PCR_PDEN;
		reg |= L2X0_PCR_DATPEN;
		reg |= L2X0_PCR_INSPEN;
		reg |= L2X0_PCR_DLEN;
		writel_relaxed(reg, l2cc_base + L2X0_PREFETCH_CTRL);

		/* Power Control */
		reg = readl_relaxed(l2cc_base + L2X0_POWER_CTRL);
		reg |= L2X0_STNDBY_MODE_EN;
		reg |= L2X0_DYNAMIC_CLK_GATING_EN;
		writel_relaxed(reg, l2cc_base + L2X0_POWER_CTRL);

		/* Disable interrupts */
		writel_relaxed(0x00, l2cc_base + L2X0_INTR_MASK);
		writel_relaxed(0x01ff, l2cc_base + L2X0_INTR_CLEAR);

		l2x0_of_init(0, ~0UL);
	} else {
		outer_cache.disable = firmware_ops->l2x0_disable;
	}
}
#else
static inline void at91_init_l2cache(void) {}
#endif

static void __init sama5_dt_device_init(void)
{
	struct device_node *np;
	int resolution;

	at91_init_l2cache();

	if (of_machine_is_compatible("atmel,sama5d3xcm") &&
	    IS_ENABLED(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
			ksz9021rn_phy_fixup);
	} else if (of_machine_is_compatible("atmel,sama5d3-xplained") &&
	    IS_ENABLED(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
			ksz9031rn_phy_fixup);
	} else if (of_machine_is_compatible("atmel,sama5d4ek") &&
	    IS_ENABLED(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ8081, MICREL_PHY_ID_MASK,
			ksz8081_phy_reset);
	}

	np = of_find_compatible_node(NULL, NULL, "atmel,at91sam9g45-isi");
	if (np) {
		if (of_device_is_available(np)) {
			if (of_machine_is_compatible("atmel,sama5d3xmb")) {
				camera_set_gpio_pins(AT91_PIN_PE24, AT91_PIN_PE29);
				at91_config_isi(true, "pck1");
			} else if (of_machine_is_compatible("atmel,sama5d4ek")) {
				at91_fixup_isi_sensor_bus(sensors, ARRAY_SIZE(sensors), 0);
				camera_set_gpio_pins(AT91_PIN_PB11, AT91_PIN_PB5);
				at91_config_isi(true, "pck1");
			}
		}
	}
	of_node_put(np);

	/* Hack for PDA display modules to update lcd settings */
	if (of_machine_is_compatible("pda,tm70xx")) {
		__u8 manufacturer[4] = "PALM";
		__u8 monitor[14] = "AT07";

		/* set LCD configuration */
		at91_tft_vga_modes[0].name = "PALM";
		at91_tft_vga_modes[0].left_margin = 128;
		at91_tft_vga_modes[0].right_margin = 0;
		at91_tft_vga_modes[0].upper_margin = 23;
		at91_tft_vga_modes[0].lower_margin = 22;
		at91_tft_vga_modes[0].hsync_len = 5;
		at91_tft_vga_modes[0].vsync_len = 5;

		memcpy(at91fb_default_monspecs.manufacturer, manufacturer, 4);
		memcpy(at91fb_default_monspecs.monitor, monitor, 14);

		ek_lcdc_data.default_lcdcon2 = LCDC_LCDCFG5_MODE_OUTPUT_18BPP;

		printk("LCD parameters updated for PDA7 display module\n");
	}
	if (of_machine_is_compatible("pda,tm43xx")) {
		__u8 manufacturer[4] = "Inlx";
		__u8 monitor[14] = "AT043TN24";

		/* set LCD configuration */
		at91_tft_vga_modes[0].name = "Inlx";
		at91_tft_vga_modes[0].xres = 480;
		at91_tft_vga_modes[0].yres = 272;
		at91_tft_vga_modes[0].pixclock = KHZ2PICOS(9000);
		at91_tft_vga_modes[0].left_margin = 2;
		at91_tft_vga_modes[0].right_margin = 2;
		at91_tft_vga_modes[0].upper_margin = 2;
		at91_tft_vga_modes[0].lower_margin = 2;
		at91_tft_vga_modes[0].hsync_len = 41;
		at91_tft_vga_modes[0].vsync_len = 11;

		memcpy(at91fb_default_monspecs.manufacturer, manufacturer, 4);
		memcpy(at91fb_default_monspecs.monitor, monitor, 14);

		ek_lcdc_data.smem_len = 480 * 272 * 4;

		printk("LCD parameters updated for PDA4 display module\n");
	}

	if (of_machine_is_compatible("encoder-sii9022")) {
		np = of_find_compatible_node(NULL, NULL, "sii902x");
		if (np)
			if (of_device_is_available(np))
				of_property_read_u32(np, "resolution", &resolution);

		of_node_put(np);

		switch (resolution) {
		case 1080:
			/* set LCD configuration */
			at91_tft_vga_modes[0].xres = 1920;
			at91_tft_vga_modes[0].yres = 1080;
			at91_tft_vga_modes[0].pixclock = KHZ2PICOS(74250);
			at91_tft_vga_modes[0].left_margin = 148;
			at91_tft_vga_modes[0].right_margin = 88;
			at91_tft_vga_modes[0].upper_margin = 36;
			at91_tft_vga_modes[0].lower_margin = 4;
			at91_tft_vga_modes[0].hsync_len = 44;
			at91_tft_vga_modes[0].vsync_len = 5;
			at91_tft_vga_modes[0].sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,

			ek_lcdc_data.smem_len = 1920 * 1080 * 2;

			printk("LCD parameters updated for HDMI with 1080P30 resolution\n");

			break;
		case 900:
			/* set LCD configuration */
			at91_tft_vga_modes[0].xres = 1440;
			at91_tft_vga_modes[0].yres = 900;
			at91_tft_vga_modes[0].pixclock = KHZ2PICOS(88750);
			at91_tft_vga_modes[0].left_margin = 48;
			at91_tft_vga_modes[0].right_margin = 80;
			at91_tft_vga_modes[0].upper_margin = 3;
			at91_tft_vga_modes[0].lower_margin = 17;
			at91_tft_vga_modes[0].hsync_len = 32;
			at91_tft_vga_modes[0].vsync_len = 6;
			at91_tft_vga_modes[0].sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,

			ek_lcdc_data.smem_len = 1440 * 900 * 2;

			printk("LCD parameters updated for HDMI with 1440 x 900 resolution\n");

			break;
		default:
			/* set LCD configuration */
			at91_tft_vga_modes[0].xres = 1280;
			at91_tft_vga_modes[0].yres = 720;
			at91_tft_vga_modes[0].pixclock = KHZ2PICOS(74250);
			at91_tft_vga_modes[0].left_margin = 220;
			at91_tft_vga_modes[0].right_margin = 110;
			at91_tft_vga_modes[0].upper_margin = 20;
			at91_tft_vga_modes[0].lower_margin = 5;
			at91_tft_vga_modes[0].hsync_len = 40;
			at91_tft_vga_modes[0].vsync_len = 5;
			at91_tft_vga_modes[0].sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,

			ek_lcdc_data.smem_len = 1280 * 720 * 2;

			printk("LCD parameters updated for HDMI with 720P60 resolution\n");

			break;
		}
	}

	of_platform_populate(NULL, of_default_bus_match_table, at91_auxdata_lookup, NULL);
	platform_add_devices(sensors, ARRAY_SIZE(sensors));
}

static const char *sama5_dt_board_compat[] __initdata = {
	"atmel,sama5",
	NULL
};

DT_MACHINE_START(sama5_dt, "Atmel SAMA5 (Device Tree)")
	/* Maintainer: Atmel */
	.init_time	= at91sam926x_pit_init,
	.map_io		= at91_map_io,
	.handle_irq	= at91_aic5_handle_irq,
	.init_early	= at91_dt_initialize,
	.init_irq	= at91_dt_init_irq,
	.init_machine	= sama5_dt_device_init,
	.dt_compat	= sama5_dt_board_compat,
MACHINE_END

static const char *sama5_alt_dt_board_compat[] __initdata = {
	"atmel,sama5d4",
	NULL
};

DT_MACHINE_START(sama5_alt_dt, "Atmel SAMA5 (Device Tree)")
	/* Maintainer: Atmel */
	.init_time	= at91sam926x_pit_init,
	.map_io		= at91_alt_map_io,
	.handle_irq	= at91_aic5_handle_irq,
	.init_early	= at91_alt_dt_initialize,
	.init_irq	= at91_dt_init_irq,
	.init_machine	= sama5_dt_device_init,
	.dt_compat	= sama5_alt_dt_board_compat,
MACHINE_END
