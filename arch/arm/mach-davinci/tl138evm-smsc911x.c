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
#include <linux/smsc911x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/da8xx.h>
#include <mach/mux.h>

static char *smsc911x_mux_name[] = {
	"NEMA_CS_5",
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
	"EMA_BA_1",
	"EMA_A_0",
	"EMA_A_1",
	"EMA_A_2",
	"EMA_A_3",
	"EMA_A_4",
	"EMA_A_5",
	"NEMA_WE",
	"NEMA_OE",
	"GPIO2_1",
	"GPIO5_8",
};

#define PINMUX_SET_NUM		ARRAY_SIZE(smsc911x_mux_name)

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define SMSC911X_GPIO_IRQ		GPIO_TO_PIN(2, 1)
#define SMSC911X_FIFO_SEL		GPIO_TO_PIN(5, 8)

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= DA8XX_AEMIF_CS5_BASE,
		.end	= DA8XX_AEMIF_CS5_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct smsc911x_platform_config da850_evm_smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_16BIT,
};

struct platform_device smsc911x_device = {
	.name	= "smsc911x",
	.id	= -1,
	.dev	= {
		.platform_data	= &da850_evm_smsc911x_config,
	},
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
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

#define DA8XX_AEMIF_CE5CFG_OFFSET		0x1c
#define DA8XX_AEMIF_ASIZE_MASK		0x3
#define DA8XX_AEMIF_ASIZE_16BIT		0x1
#define DA8XX_AEMIF_ASIZE_8BIT		0x0


/* Initialize smsc911x device connected to the EMIFA. */
static int __init tl138_evm_smsc911x_init(void)
{
	void __iomem *aemif_addr;
	unsigned set, val;
	int i;
	int ret = 0;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(smsc911x_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}

	aemif_addr = ioremap(DA8XX_AEMIF_CTL_BASE, SZ_32K);

	/* Configure data bus width of CS5 to 16 bit */
	writel(readl(aemif_addr + DA8XX_AEMIF_CE5CFG_OFFSET) |
		DA8XX_AEMIF_ASIZE_16BIT,
		aemif_addr + DA8XX_AEMIF_CE5CFG_OFFSET);


	/* setup timing values for a given AEMIF interface */
	set = TA(2) | RHOLD(2) | RSTROBE(5) | RSETUP(1) |
		WHOLD(2) | WSTROBE(5) | WSETUP(1);

	val = readl(aemif_addr + DA8XX_AEMIF_CE5CFG_OFFSET);
	val &= ~TIMING_MASK;
	val |= set;
	writel(val, aemif_addr + DA8XX_AEMIF_CE5CFG_OFFSET);

	iounmap(aemif_addr);

	smsc911x_resources[1].start = gpio_to_irq(SMSC911X_GPIO_IRQ);

	ret = gpio_request(SMSC911X_FIFO_SEL, "smsc911x-fifo-sel");
	if (ret)
		pr_warning("Fail to request smsc911x-fifo-sel gpio PIN %d.\n",
				SMSC911X_FIFO_SEL);

	/*
	When driven high all accesses to the
	LAN9221/LAN9221i are to the RX or TX Data FIFOs.
	In this mode, the A[7:3] upper address inputs are
	ignored. The chip need to setup and set FIFO_SEL low.
	*/
	gpio_direction_output(SMSC911X_FIFO_SEL, 0);

	return platform_device_register(&smsc911x_device);
}

static void __exit tl138_evm_smsc911x_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(smsc911x_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	gpio_free(SMSC911X_FIFO_SEL);

	platform_device_unregister(&smsc911x_device);
}

module_init(tl138_evm_smsc911x_init);
module_exit(tl138_evm_smsc911x_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("SMSC911X EMIFA module init driver");
MODULE_LICENSE("GPL v2");
