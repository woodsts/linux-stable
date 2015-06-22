
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define AT91_PIO4_MSKR	0x00
#define AT91_PIO4_CFGR	0x04
#define		AT91_PIO4_CFGR_DIR_MASK	BIT(8)
#define AT91_PIO4_PDSR	0x08
#define AT91_PIO4_SODR	0x10
#define AT91_PIO4_CODR	0x14
#define AT91_PIO4_ODSR	0x18
#define AT91_PIO4_IER	0x20
#define AT91_PIO4_IDR	0x24
#define AT91_PIO4_IMR	0x28
#define AT91_PIO4_ISR	0x2c

#define AT91_PIO4_GROUP_OFFSET		0x40
#define AT91_PIO4_NPINS_PER_GROUP	32
#define AT91_PIO4_GROUP(pin)		(pin / AT91_PIO4_NPINS_PER_GROUP)
#define AT91_PIO4_LINE(pin)		(pin % AT91_PIO4_NPINS_PER_GROUP)

struct at91_pio4_gpio_desc {
	unsigned int group;
	unsigned int line;
};

struct at91_gpio_soc_config {
	unsigned int ngroups;
};

/* There is only one PIO controller per SoC */
struct irq_domain		*at91_gpio_irq_domain;
struct at91_pio4_gpio_desc	*gpio_descs;
struct regmap			*regmap_base;
struct clk			*clk;
struct device			*dev;

static unsigned int at91_gpio_read(unsigned int group, unsigned int reg)
{
	unsigned int val;

	if (regmap_read(regmap_base, group * AT91_PIO4_GROUP_OFFSET + reg, &val))
		dev_err(dev, "regmap_read error\n");

	return val;
}

static void at91_gpio_write(unsigned int group, unsigned int reg, unsigned int val)
{
	regmap_write(regmap_base, group * AT91_PIO4_GROUP_OFFSET + reg, val);
}

static void at91_gpio_irq_ack(struct irq_data *d)
{

}

static int at91_gpio_irq_set_type(struct irq_data *d, unsigned type)
{
	return 0;
}

static void at91_gpio_irq_mask(struct irq_data *d)
{
	struct at91_pio4_gpio_desc *gpio_desc = irq_data_get_irq_chip_data(d);

	at91_gpio_write(gpio_desc->group, AT91_PIO4_IDR, BIT(gpio_desc->line));
}

static void at91_gpio_irq_unmask(struct irq_data *d)
{
	struct at91_pio4_gpio_desc *gpio_desc = irq_data_get_irq_chip_data(d);

	at91_gpio_write(gpio_desc->group, AT91_PIO4_IER, BIT(gpio_desc->line));
}

static struct irq_chip pio4_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= at91_gpio_irq_ack,
	.irq_mask	= at91_gpio_irq_mask,
	.irq_unmask	= at91_gpio_irq_unmask,
	.irq_set_type	= at91_gpio_irq_set_type,
	.irq_shutdown	= NULL,
	.irq_set_wake	= NULL,
};

static int at91_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	unsigned gpio = chip->base + offset;

	return pinctrl_request_gpio(gpio);
}

static void at91_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	unsigned gpio = chip->base + offset;

	pinctrl_free_gpio(gpio);
}

static int at91_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct at91_pio4_gpio_desc *gpio_desc = &gpio_descs[offset];
	unsigned int reg;

	at91_gpio_write(gpio_desc->group, AT91_PIO4_MSKR, BIT(gpio_desc->line));
	reg = at91_gpio_read(gpio_desc->group, AT91_PIO4_CFGR);
	reg &= (~AT91_PIO4_CFGR_DIR_MASK);
	at91_gpio_write(gpio_desc->group, AT91_PIO4_CFGR, reg);

	return 0;
}

static int at91_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct at91_pio4_gpio_desc *gpio_desc = &gpio_descs[offset];
	unsigned int reg;

	reg = at91_gpio_read(gpio_desc->group, AT91_PIO4_PDSR);

	return !!(reg & BIT(gpio_desc->line));
}

static int at91_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int val)
{
	struct at91_pio4_gpio_desc *gpio_desc = &gpio_descs[offset];
	unsigned int reg;

	at91_gpio_write(gpio_desc->group, AT91_PIO4_MSKR, BIT(gpio_desc->line));
	reg = at91_gpio_read(gpio_desc->group, AT91_PIO4_CFGR);
	reg |= AT91_PIO4_CFGR_DIR_MASK;
	at91_gpio_write(gpio_desc->group, AT91_PIO4_CFGR, reg);
	at91_gpio_write(gpio_desc->group, val ? AT91_PIO4_SODR : AT91_PIO4_CODR,
			BIT(gpio_desc->line));

	return 0;
}

static void at91_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct at91_pio4_gpio_desc *gpio_desc = &gpio_descs[offset];

	at91_gpio_write(gpio_desc->group,
			val ? AT91_PIO4_SODR : AT91_PIO4_CODR,
			BIT(gpio_desc->line));
}

static int at91_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_find_mapping(at91_gpio_irq_domain, offset);
}

static struct gpio_chip pio4_gpio_chip = {
	.label			= "at91-gpio",
	//.request		= at91_gpio_request,
	//.free			= at91_gpio_free,
	.direction_input	= at91_gpio_direction_input,
	.get			= at91_gpio_get,
	.direction_output	= at91_gpio_direction_output,
	.set			= at91_gpio_set,
	.to_irq			= at91_gpio_to_irq,
	.base			= 0,
};

static void at91_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long isr;
	int n;

	chained_irq_enter(chip, desc);

	for (;;) {
		isr = (unsigned long) at91_gpio_read(AT91_PIO4_GROUP(irq), AT91_PIO4_ISR);
		isr &= (unsigned long) at91_gpio_read(AT91_PIO4_GROUP(irq), AT91_PIO4_IMR);
		if (!isr)
			break;

		for_each_set_bit(n, &isr, BITS_PER_LONG) {
			generic_handle_irq(gpio_to_irq(irq));
		}
	}

	chained_irq_exit(chip, desc);
}

static struct at91_gpio_soc_config at91_gpio_sama5d2_config = {
	.ngroups = 4,
};

static const struct of_device_id at91_gpio_of_match[] = {
	{ .compatible = "atmel,sama5d2-gpio", .data = &at91_gpio_sama5d2_config },
	{ /* sentinel */ },
};

static int at91_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id	*match;
	struct device_node		*regmap_np;
	struct resource			*res;
	struct at91_gpio_soc_config	*config;
	char				**names;
	int				i, ret;

	dev = &pdev->dev;

	match = of_match_device(at91_gpio_of_match, &pdev->dev);
	if (!match) {
		dev_err(dev, "no matching device found\n");
		return -ENODEV;
	}
	config = (struct at91_gpio_soc_config *)match->data;

	pio4_gpio_chip.of_node = pdev->dev.of_node;
	pio4_gpio_chip.dev = &pdev->dev;
	pio4_gpio_chip.ngpio = config->ngroups * AT91_PIO4_NPINS_PER_GROUP;

	regmap_np = of_parse_phandle(pdev->dev.of_node, "atmel,pio_reg", 0);
	if (regmap_np) {
		regmap_base = syscon_node_to_regmap(regmap_np);
		if (IS_ERR(regmap_base)) {
			dev_err(dev, "can't get regmap\n");
			return PTR_ERR(regmap_base);
		}
	} else {
		dev_err(dev, "atmel,pio_reg property is missing\n");
		return -EINVAL;
	}

	at91_gpio_irq_domain = irq_domain_add_linear(pdev->dev.of_node,
						     pio4_gpio_chip.ngpio,
						     &irq_domain_simple_ops,
						     NULL);
	if (!at91_gpio_irq_domain) {
		dev_err(dev, "can't add the irq domain\n");
		return -ENODEV;
	}

	gpio_descs = devm_kzalloc(&pdev->dev,
				  pio4_gpio_chip.ngpio * sizeof(*gpio_descs),
				  GFP_KERNEL);
	if (!gpio_descs) {
		dev_err(dev, "can't allocate gpio_descs\n");
		return -ENOMEM;
	}

	/* There is one controller but each group has its own irq line. */
	for (i = 0; i < config->ngroups; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res) {
			dev_err(dev, "missing irq resource for group %c\n",
				'A' + i);
			return -EINVAL;
		}
		irq_set_chained_handler(res->start, at91_gpio_irq_handler);
		dev_dbg(dev, "group %i: irq %u\n", i, res->start);
	}

	for (i = 0; i < pio4_gpio_chip.ngpio; i++) {
		int irq = irq_create_mapping(at91_gpio_irq_domain, i);
		struct at91_pio4_gpio_desc *pio4_gpio_desc = &gpio_descs[i];

		pio4_gpio_desc->group = AT91_PIO4_GROUP(i);
		pio4_gpio_desc->line = AT91_PIO4_LINE(i);
		irq_set_chip_and_handler(irq, &pio4_gpio_irq_chip,
					 handle_simple_irq);
		irq_set_chip_data(irq, pio4_gpio_desc);
		dev_dbg(dev,
			"at91_gpio_domain: hwirq: %d, linux irq: %d, group: %c, line: %u)\n",
			i, irq, 'A' + pio4_gpio_desc->group, pio4_gpio_desc->line);
	}

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}

	names = devm_kzalloc(&pdev->dev, sizeof(char *) *  pio4_gpio_chip.ngpio,
			     GFP_KERNEL);
	if (!names) {
		dev_err(dev, "failed to allocate names\n");
		return -ENOMEM;
	}
	for (i = 0; i < pio4_gpio_chip.ngpio; i++)
		names[i] = kasprintf(GFP_KERNEL, "P%c%d\n",
				     AT91_PIO4_GROUP(i), AT91_PIO4_LINE(i));
	pio4_gpio_chip.names = (const char *const *)names;

	/* clk enable prepare */
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "failed to prepare and enable clock\n");
		return ret;
	}

	ret = gpiochip_add(&pio4_gpio_chip);

	dev_info(dev, "at91 pio4 gpio driver: %u groups, %d pins\n",
		 config->ngroups, pio4_gpio_chip.ngpio);
	return 0;
}

static struct platform_driver at91_gpio_driver = {
	.driver	= {
		.name		= "at91-gpio",
		.owner		= THIS_MODULE,
		.of_match_table	= at91_gpio_of_match,
	},
	.probe	= at91_gpio_probe,
};

static int __init at91_gpio_init(void)
{
	return platform_driver_register(&at91_gpio_driver);
}
postcore_initcall(at91_gpio_init);
