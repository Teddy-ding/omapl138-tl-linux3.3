/*
 * TL138 EVM board
 *
 * Copyright (C) 2015 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-emifa-sram.c
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
#include <linux/mfd/davinci_aemif.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/da8xx.h>
#include <mach/mux.h>

static char *emifa_sram_mux_name[] = {
	"NEMA_CS_2",
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
	"EMA_D_8",
	"EMA_D_9",
	"EMA_D_10",
	"EMA_D_11",
	"EMA_D_12",
	"EMA_D_13",
	"EMA_D_14",
	"EMA_D_15",
	"EMA_A_0",
	"EMA_A_1",
	"EMA_A_2",
	"EMA_A_3",
	"EMA_A_4",
	"EMA_A_5",
	"EMA_A_6",
	"EMA_A_7",
	"EMA_A_8",
	"EMA_A_9",
	"EMA_A_10",
	"EMA_A_11",
	"EMA_A_12",
	"EMA_A_13",
};

#define PINMUX_SET_NUM		ARRAY_SIZE(emifa_sram_mux_name)

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

static void emifa_sram_release(struct device *dev)
{
}

static struct platform_device davinci_emif_device = {
	.name	= "davinci_aemif",
	.id	= 1,
	.dev	= {
		.release = emifa_sram_release,
	},
};

static int __init tl138_evm_emifa_sram_init(void)
{
	void __iomem *aemif_addr;
	unsigned set, val;
	int i;
	int ret;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(emifa_sram_mux_name[i],
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

	platform_device_register(&davinci_emif_device);

	return 0;
}

static void __exit tl138_evm_emifa_sram_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(emifa_sram_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	platform_device_unregister(&davinci_emif_device);
}

module_init(tl138_evm_emifa_sram_init);
module_exit(tl138_evm_emifa_sram_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL138EVM EMIFA SRAM module init driver");
MODULE_LICENSE("GPL v2");
