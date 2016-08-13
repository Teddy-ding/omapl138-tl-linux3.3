/*
 * TL138EVM board
 *
 * Copyright (C) 2015 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-tl16754-serials.c
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
#include <linux/serial_8250.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/da8xx.h>
#include <mach/mux.h>

#define TL16754_CLK		14745600
#define TL16754_PORT_N		(CONFIG_TL16754_NR_CHIPS * 4)

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

#define DA8XX_AEMIF_CE2CFG_OFFSET	0x10
#define DA8XX_AEMIF_CE4CFG_OFFSET	0x18
#define DA8XX_AEMIF_CE5CFG_OFFSET	0x1c
#define DA8XX_AEMIF_ASIZE_MASK		0x3
#define DA8XX_AEMIF_ASIZE_16BIT		0x1
#define DA8XX_AEMIF_ASIZE_8BIT		0x0


static char *tl16574_mux_name[] = {
	"NEMA_CS_4",
	"EMA_CLK",
	"NEMA_WE",
	"NEMA_OE",
	"EMA_D_0",
	"EMA_D_1",
	"EMA_D_2",
	"EMA_D_3",
	"EMA_D_4",
	"EMA_D_5",
	"EMA_D_6",
	"EMA_D_7",
	"EMA_BA_0",
	"EMA_BA_1",
	"EMA_A_0",
	"EMA_A_1",
	"EMA_A_2",
	"EMA_A_3",
	"GPIO5_7",
	"GPIO2_2",
	"GPIO2_3",
	"GPIO2_5",
	"GPIO2_6",
#if TL16754_PORT_N > 4
	"GPIO3_9",
	"GPIO5_9",
	"GPIO5_14",
	"GPIO5_15",
#endif
};

#define PINMUX_SET_NUM		ARRAY_SIZE(tl16574_mux_name)

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define TL16754_RESET			GPIO_TO_PIN(5, 7)

static const char tl16754_gpio_irq[][2] = {
	{2, 2}, {2, 3}, {2, 5}, {2, 6},
#if TL16754_PORT_N > 4
	{3, 9}, {5, 9}, {5, 14}, {5, 15},
#endif
};

static struct plat_serial8250_port tl16754_serial_pdata[] = {
	[0 ... TL16754_PORT_N - 1] = {
		.mapbase	= DA8XX_AEMIF_CS4_BASE,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
					UPF_IOREMAP | UPF_FIXED_TYPE,
		.type		= PORT_16654,
		.iotype 	= UPIO_MEM,
		.regshift	= 0,
		.uartclk	= TL16754_CLK,
	},
	{
		.flags	= 0,
	},
};

void tl138evm_tl16574_release(struct device *dev)
{
}

struct platform_device tl16754_serial_device = {
	.name	= "serial8250",
	.id	= PLAT8250_DEV_PLATFORM1,
	.dev	= {
		.platform_data	= tl16754_serial_pdata,
		.release = tl138evm_tl16574_release,
	},
};

static int __init tl138evm_tl16574_init(void)
{
	void __iomem *aemif_addr;
	unsigned int aemif_csn_offset;
	unsigned set, val;
	int i;
	int ret = 0;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(tl16574_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}

	aemif_addr = ioremap(DA8XX_AEMIF_CTL_BASE, SZ_32K);

	/* Configure data bus width of CS4 to 8 bit */
	aemif_csn_offset = DA8XX_AEMIF_CE4CFG_OFFSET;
	writel(readl(aemif_addr + aemif_csn_offset) &
		(~DA8XX_AEMIF_ASIZE_MASK), aemif_addr + aemif_csn_offset);

	/* setup timing values for a given AEMIF interface */
	set = TA(10) | RHOLD(3) | RSTROBE(10) | RSETUP(4) |
		WHOLD(3) | WSTROBE(10) | WSETUP(4);

	val = readl(aemif_addr + aemif_csn_offset);
	val &= ~TIMING_MASK;
	val |= set;
	writel(val, aemif_addr + aemif_csn_offset);

	iounmap(aemif_addr);

	for (i = 0; i < TL16754_PORT_N; i++) {
		tl16754_serial_pdata[i].mapbase = DA8XX_AEMIF_CS4_BASE + 8*i;
		tl16754_serial_pdata[i].irq =
			gpio_to_irq(GPIO_TO_PIN(tl16754_gpio_irq[i][0],
					tl16754_gpio_irq[i][1]));
	}

	ret = gpio_request(TL16754_RESET, "tl16754-reset");
	if (ret)
		pr_warning("Fail to request tl16754-reset gpio PIN %d.\n",
				TL16754_RESET);

	gpio_direction_output(TL16754_RESET, 1);
	ndelay(200);
	gpio_set_value(TL16754_RESET, 0);

	platform_device_register(&tl16754_serial_device);

	return 0;
}

static void __exit tl138evm_tl16574_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(tl16574_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	platform_device_unregister(&tl16754_serial_device);
}

module_init(tl138evm_tl16574_init);
module_exit(tl138evm_tl16574_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL138EVM TL16574 module init driver");
MODULE_LICENSE("GPL v2");
