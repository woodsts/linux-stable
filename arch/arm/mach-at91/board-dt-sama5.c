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
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

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
static struct i2c_board_info i2c_ov5640 = {
	I2C_BOARD_INFO("ov5642", 0x3c),
};

static struct soc_camera_desc iclink_ov2640 = {
	.subdev_desc = {
		.power 		= i2c_camera_power,
		.query_bus_param	= isi_camera_query_bus_param,
	},
	.host_desc = {
		.bus_id		= -1,
		.board_info	= &i2c_ov2640,
		.i2c_adapter_id	= 1,
	},
};
static struct soc_camera_desc iclink_ov5640 = {
	.subdev_desc = {
		.power		= i2c_camera_power,
		.query_bus_param	= isi_camera_query_bus_param,
	},
	.host_desc = {
		.bus_id			= -1,
		.board_info		= &i2c_ov5640,
		.i2c_adapter_id		= 1,
	},
};

static struct platform_device isi_ov2640 = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.platform_data = &iclink_ov2640,
	},
};
static struct platform_device isi_ov5640 = {
	.name	= "soc-camera-pdrv",
	.id	= 1,
	.dev	= {
		.platform_data = &iclink_ov5640,
	},
};

static struct platform_device *sensors[] __initdata = {
	&isi_ov2640,
	&isi_ov5640,
};

struct of_dev_auxdata at91_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf8038000, "atmel_hlcdfb_base", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf8038100, "atmel_hlcdfb_ovl1", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030000, "atmel_hlcdfb_base", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030140, "atmel_hlcdfb_ovl1", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9x5-lcd", 0xf0030240, "atmel_hlcdfb_ovl2", &ek_lcdc_data),
	OF_DEV_AUXDATA("atmel,at91sam9g45-isi", 0xf0034000, "atmel_isi", &isi_data),
	{ /* sentinel */ }
};

static const struct of_device_id irq_of_match[] __initconst = {

	{ .compatible = "atmel,sama5d3-aic", .data = at91_aic5_of_init },
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

static void __init sama5_dt_device_init(void)
{
	struct device_node *np;

	if (of_machine_is_compatible("atmel,sama5d3xcm") &&
	    IS_ENABLED(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
			ksz9021rn_phy_fixup);
	else if (of_machine_is_compatible("atmel,sama5d3-xplained") &&
	    IS_ENABLED(CONFIG_PHYLIB)) {
		printk(KERN_CRIT "NFE test ksz9031rn_phy_fixup\n");
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
			ksz9031rn_phy_fixup);
	}

	np = of_find_compatible_node(NULL, NULL, "atmel,at91sam9g45-isi");
	if (np) {
		if (of_device_is_available(np)) {
			camera_set_gpio_pins(AT91_PIN_PE24, AT91_PIN_PE29);
			at91_config_isi(true, "pck1");
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
