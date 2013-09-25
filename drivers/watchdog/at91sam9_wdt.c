/*
 * Watchdog driver for Atmel AT91SAM9x processors.
 *
 * Copyright (C) 2008 Renaud CERRATO r.cerrato@til-technologies.fr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

/*
 * The Watchdog Timer Mode Register can be only written to once. If the
 * timeout need to be set from Linux, be sure that the bootstrap or the
 * bootloader doesn't write to this register.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/of.h>

#include "at91sam9_wdt.h"

#define DRV_NAME "AT91SAM9 Watchdog"

/* AT91SAM9 watchdog runs a 12bit counter @ 256Hz,
 * use this to convert a watchdog
 * value from/to milliseconds.
 */
#define ms_to_ticks(t)	(((t << 8) / 1000) - 1)
#define ticks_to_ms(t)	(((t + 1) * 1000) >> 8)

/* Hardware timeout in seconds */
#define WDT_HW_TIMEOUT 2

/* Timer heartbeat (500ms) */
#define WDT_TIMEOUT	(HZ/2)

/* User land timeout */
#define MIN_HEARTBEAT 1
#define MAX_HEARTBEAT 16
#define WDT_HEARTBEAT 15
static int heartbeat;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. "
	"(default = " __MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
	"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct at91wdt_drvdata {
	void __iomem	*phybase;
	bool		is_enable;	/* indicate if the watchdog is eabled */
	unsigned long	next_heartbeat;	/* the next_heartbeat for the timer */
	struct timer_list	timer;	/* The timer that pings the watchdog */
};

/* ......................................................................... */

static inline unsigned int wdt_read(struct at91wdt_drvdata *driver_data,
					unsigned int field)
{
	return readl_relaxed(driver_data->phybase + field);
}

static inline void wdt_write(struct at91wdt_drvdata *driver_data,
				unsigned int field, unsigned int val)
{
	writel_relaxed((val), driver_data->phybase + field);
}

static inline bool watchdog_is_open(struct watchdog_device *wddev)
{
	return test_bit(WDOG_DEV_OPEN, &wddev->status);
}

/*
 * Reload the watchdog timer.  (ie, pat the watchdog)
 */
static inline void at91_wdt_reset(struct at91wdt_drvdata *driver_data)
{
	wdt_write(driver_data, AT91_WDT_CR, AT91_WDT_KEY | AT91_WDT_WDRSTT);
}

/*
 * Timer tick
 */
static void at91wdt_timer_tick(unsigned long data)
{
	struct watchdog_device *wddev = (struct watchdog_device *)data;
	struct at91wdt_drvdata *driver_data = watchdog_get_drvdata(wddev);

	if (time_before(jiffies, driver_data->next_heartbeat)) {
		at91_wdt_reset(driver_data);
		mod_timer(&driver_data->timer, jiffies + WDT_TIMEOUT);

		if (!watchdog_is_open(wddev))
			driver_data->next_heartbeat = jiffies
						+ wddev->timeout * HZ;
	} else
		pr_crit("I will reset your machine !\n");
}

static int at91wdt_enable(struct watchdog_device *wddev, unsigned int timeout)
{
	struct at91wdt_drvdata *driver_data = watchdog_get_drvdata(wddev);
	unsigned int reg;

	/* Check if the watchdog is disabled,
	 * if disabled, the reason is the bootstrap or the bootloader has
	 * written the Watchdog Timer Mode Register to disable the
	 * watchdog timer
	 */
	reg = wdt_read(driver_data, AT91_WDT_MR);
	if (reg & AT91_WDT_WDDIS) {
		driver_data->is_enable = false;
		pr_info("sorry, watchdog is disabled\n");
		return -1;
	}

	/*
	 * All counting occurs at SLOW_CLOCK / 128 = 256 Hz
	 *
	 * Since WDV is a 12-bit counter, the maximum period is
	 * 4096 / 256 = 16 seconds.
	 */
	reg = AT91_WDT_WDRSTEN	/* causes watchdog reset */
		/* | AT91_WDT_WDRPROC	causes processor reset only */
		| AT91_WDT_WDDBGHLT	/* disabled in debug mode */
		| AT91_WDT_WDD		/* restart at any time */
		| (timeout & AT91_WDT_WDV);  /* timer value */
	wdt_write(driver_data, AT91_WDT_MR, reg);

	driver_data->is_enable = true;

	return 0;
}

static const struct watchdog_info at91_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_KEEPALIVEPING,
};

static int at91wdt_start(struct watchdog_device *wddev)
{
	struct at91wdt_drvdata *driver_data = watchdog_get_drvdata(wddev);

	if (driver_data->is_enable)
		return 0;
	else
		return -EIO;
}

static int at91wdt_stop(struct watchdog_device *wddev)
{
	struct at91wdt_drvdata *driver_data = watchdog_get_drvdata(wddev);

	if (driver_data->is_enable)
		return -EIO;
	else
		return 0;
}

static int at91wdt_ping(struct watchdog_device *wddev)
{
	struct at91wdt_drvdata *driver_data = watchdog_get_drvdata(wddev);

	if (driver_data->is_enable) {
		driver_data->next_heartbeat = jiffies + wddev->timeout * HZ;
		return 0;
	} else
		return -EIO;
}

/* ......................................................................... */

static struct watchdog_ops at91wdt_ops = {
	.owner = THIS_MODULE,
	.start = at91wdt_start,
	.stop = at91wdt_stop,
	.ping = at91wdt_ping,
};

static struct watchdog_device at91wdt_wdd = {
	.timeout = WDT_HEARTBEAT,
	.min_timeout = MIN_HEARTBEAT,
	.max_timeout = MAX_HEARTBEAT,
	.info = &at91_wdt_info,
	.ops = &at91wdt_ops,
};

static int at91wdt_probe(struct platform_device *pdev)
{
	struct at91wdt_drvdata *driver_data;
	struct resource	*r;
	int ret;

	driver_data = devm_kzalloc(&pdev->dev,
				sizeof(*driver_data), GFP_KERNEL);
	if (!driver_data) {
		dev_err(&pdev->dev, "Unable to alloacate watchdog device\n");
		return -ENOMEM;
	}

	watchdog_set_drvdata(&at91wdt_wdd, driver_data);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENODEV;

	driver_data->phybase = ioremap(r->start, resource_size(r));
	if (!driver_data->phybase) {
		dev_err(&pdev->dev, "failed to map registers, aborting.\n");
		return -ENOMEM;
	}

	ret = watchdog_register_device(&at91wdt_wdd);
	if (ret) {
		dev_err(&pdev->dev, "cannot register watchdog (%d)\n", ret);
		return ret;
	}

	watchdog_set_nowayout(&at91wdt_wdd, nowayout);

	watchdog_init_timeout(&at91wdt_wdd, heartbeat, pdev->dev.of_node);

	ret = at91wdt_enable(&at91wdt_wdd, ms_to_ticks(WDT_HW_TIMEOUT * 1000));
	if (ret) {
		pr_info("the watchdog has been disabled\n");
		return 0;
	}

	driver_data->next_heartbeat = jiffies + at91wdt_wdd.timeout * HZ;
	setup_timer(&driver_data->timer, at91wdt_timer_tick,
					(unsigned long)&at91wdt_wdd);
	mod_timer(&driver_data->timer, jiffies + WDT_TIMEOUT);

	pr_info("enabled (heartbeat=%d sec, nowayout=%d)\n",
		at91wdt_wdd.timeout, nowayout);

	return 0;
}

static int __exit at91wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&at91wdt_wdd);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id at91_wdt_dt_ids[] = {
	{ .compatible = "atmel,at91sam9260-wdt" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, at91_wdt_dt_ids);
#endif

static struct platform_driver at91wdt_driver = {
	.probe		= at91wdt_probe,
	.remove		= __exit_p(at91wdt_remove),
	.driver		= {
		.name	= "at91_wdt",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(at91_wdt_dt_ids),
	},
};

module_platform_driver(at91wdt_driver);

MODULE_AUTHOR("Renaud CERRATO <r.cerrato@til-technologies.fr>");
MODULE_DESCRIPTION("Watchdog driver for Atmel AT91SAM9x processors");
MODULE_LICENSE("GPL");
