/*
 * TL138 EVM board
 *
 * Copyright (C) 2015 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-ad7606-par.c
 * Original Copyrights follow:
 *
 * 2007, 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/da8xx.h>
#include <mach/mux.h>

static char *ad7606_par_mux_name[] = {
	"NEMA_CS_2",
	"EMA_CLK",
	"EMA_D_0",
	"EMA_D_1",
	"EMA_D_2",
	"EMA_D_3",
	"EMA_D_4",
	"EMA_D_5",
	"EMA_D_6",
	"EMA_D_7",
	"EMA_D_8",
	"EMA_D_9",
	"EMA_D_10",
	"EMA_D_11",
	"EMA_D_12",
	"EMA_D_13",
	"EMA_D_14",
	"EMA_D_15",
	"GPIO5_6",
	"GPIO5_10",
	"GPIO5_11",
	"GPIO5_12",
	"GPIO5_13",
};

#define PINMUX_SET_NUM		ARRAY_SIZE(ad7606_par_mux_name)

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define AD7606_PAR_BUSY		GPIO_TO_PIN(5, 11)
#define AD7606_PAR_CONVST		GPIO_TO_PIN(5, 13)
#define AD7606_PAR_RESET		GPIO_TO_PIN(5, 12)
#define AD7606_PAR_FRSTDATA		GPIO_TO_PIN(5, 10)
#define AD7606_PAR_RANGE		GPIO_TO_PIN(5, 6)
#define AD7606_PAR_OS0			GPIO_TO_PIN(5, 4)
#define AD7606_PAR_OS1			GPIO_TO_PIN(5, 2)
#define AD7606_PAR_OS2			GPIO_TO_PIN(5, 0)

/**
 * struct ad7606_platform_data - platform/board specifc information
 * @default_os:		default oversampling value {0, 2, 4, 8, 16, 32, 64}
 * @default_range:	default range +/-{5000, 10000} mVolt
 * @gpio_convst:	number of gpio connected to the CONVST pin
 * @gpio_reset:		gpio connected to the RESET pin, if not used set to -1
 * @gpio_range:		gpio connected to the RANGE pin, if not used set to -1
 * @gpio_os0:		gpio connected to the OS0 pin, if not used set to -1
 * @gpio_os1:		gpio connected to the OS1 pin, if not used set to -1
 * @gpio_os2:		gpio connected to the OS2 pin, if not used set to -1
 * @gpio_frstdata:	gpio connected to the FRSTDAT pin, if not used set to -1
 * @gpio_stby:		gpio connected to the STBY pin, if not used set to -1
 */

struct ad7606_platform_data {
	unsigned			default_os;
	unsigned			default_range;
	unsigned			gpio_convst;
	unsigned			gpio_reset;
	unsigned			gpio_range;
	unsigned			gpio_os0;
	unsigned			gpio_os1;
	unsigned			gpio_os2;
	unsigned			gpio_frstdata;
	unsigned			gpio_stby;
};

static struct ad7606_platform_data ad7606_par_pdata = {
	.default_os		= 0,
	.default_range		= 5000,
	.gpio_convst		= AD7606_PAR_CONVST,
	.gpio_reset		= AD7606_PAR_RESET,
	.gpio_range		= AD7606_PAR_RANGE,
	.gpio_os0		= AD7606_PAR_OS0,
	.gpio_os1		= AD7606_PAR_OS1,
	.gpio_os2		= AD7606_PAR_OS2,
	.gpio_frstdata		= -1,
	.gpio_stby		= -1,
};

#if defined(CONFIG_IIO_DAVINCI_TMR_TRIGGER) || \
	defined(CONFIG_IIO_DAVINCI_TMR_TRIGGER_MODULE)

#define DA850_TIMER64P2_BASE		0x01f0c000 /* DA8XX_TIMER64P2_BASE */
#define IIO_DAVINCI_TIMER_BASE	DA850_TIMER64P2_BASE
#define IIO_DAVINCI_TIMER_IRQ		IRQ_DA850_TINT12_2

static struct resource iio_davinci_trigger_resources[] = {
	{
		.start	= IIO_DAVINCI_TIMER_BASE,
		.end	= IIO_DAVINCI_TIMER_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IIO_DAVINCI_TIMER_IRQ,
		.end	= IIO_DAVINCI_TIMER_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device iio_davinci_trigger = {
	.name		= "iio_davinci_tmr_trigger",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(iio_davinci_trigger_resources),
	.resource	= iio_davinci_trigger_resources,
};
#endif

static struct resource ad7606_resources[] = {
	[0] = {
		.start	= DA8XX_AEMIF_CS2_BASE,
		.end	= DA8XX_AEMIF_CS2_BASE + 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= -1,
		.end	= -1,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device ad7606_device = {
	.name		= "ad7606-8",
	.dev = {
		.platform_data = &ad7606_par_pdata,
	},
	.num_resources	= ARRAY_SIZE(ad7606_resources),
	.resource	= ad7606_resources,
};

/* Timing value configuration */
#define TA(x)		((x) << 2)
#define RHOLD(x)	((x) << 4)
#define RSTROBE(x)	((x) << 7)
#define RSETUP(x)	((x) << 13)
#define WHOLD(x)	((x) << 17)
#define WSTROBE(x)	((x) << 20)
#define WSETUP(x)	((x) << 26)

#define TA_MAX		0x3
#define RHOLD_MAX	0x7
#define RSTROBE_MAX	0x3f
#define RSETUP_MAX	0xf
#define WHOLD_MAX	0x7
#define WSTROBE_MAX	0x3f
#define WSETUP_MAX	0xf

#define TIMING_MASK	(TA(TA_MAX) | \
				RHOLD(RHOLD_MAX) | \
				RSTROBE(RSTROBE_MAX) |	\
				RSETUP(RSETUP_MAX) | \
				WHOLD(WHOLD_MAX) | \
				WSTROBE(WSTROBE_MAX) | \
				WSETUP(WSETUP_MAX))

#define DA8XX_AEMIF_CE2CFG_OFFSET		0x10
#define DA8XX_AEMIF_ASIZE_MASK		0x3
#define DA8XX_AEMIF_ASIZE_16BIT		0x1
#define DA8XX_AEMIF_ASIZE_8BIT		0x0

static int __init tl138_evm_ad7606_par_init(void)
{
	void __iomem *aemif_addr;
	unsigned set, val;
	int i;
	int ret;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(ad7606_par_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}

	aemif_addr = ioremap(DA8XX_AEMIF_CTL_BASE, SZ_32K);

	/* Configure data bus width of CS2 to 16 bit */
	writel(readl(aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET) |
		DA8XX_AEMIF_ASIZE_16BIT,
		aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET);

	/* setup timing values for a given AEMIF interface */
	set = TA(1) | RHOLD(1) | RSTROBE(3) | RSETUP(1) |
		WHOLD(3) | WSTROBE(3) | WSETUP(3);

	val = readl(aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET);
	val &= ~TIMING_MASK;
	val |= set;
	writel(val, aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET);

	iounmap(aemif_addr);

	ad7606_resources[1].start = gpio_to_irq(AD7606_PAR_BUSY);
	ad7606_resources[1].end= ad7606_resources[1].start;

	platform_device_register(&ad7606_device);

#if defined(CONFIG_IIO_DAVINCI_TMR_TRIGGER) || \
		defined(CONFIG_IIO_DAVINCI_TMR_TRIGGER_MODULE)
	platform_device_register(&iio_davinci_trigger);
#endif

	return 0;
}

static void __exit tl138_evm_ad7606_par_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(ad7606_par_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	platform_device_unregister(&ad7606_device);
}

module_init(tl138_evm_ad7606_par_init);
module_exit(tl138_evm_ad7606_par_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL7606 ADC module init driver");
MODULE_LICENSE("GPL v2");
