/*
 * TL138 EVM board
 *
 * Copyright (C) 2009 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-mcp251x.c
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
#include <linux/can/platform/mcp251x.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/da8xx.h>
#include <mach/mux.h>


static char *mcp251x_mux_name[] = {
	"SPI1_CS_2",
	"SPI1_CS_3",
	"GPIO0_8",
	"GPIO0_9",
};

#define PINMUX_SET_NUM		4

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define MCP251X_CAN0_IRQ		GPIO_TO_PIN(0, 8)
#define MCP251X_CAN1_IRQ		GPIO_TO_PIN(0, 9)

static struct davinci_spi_config mcp251x_spits_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
};

struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 16000000,
};

static struct spi_board_info mcp251x_spi_info[] = {
	[0] = {
		.modalias		= "mcp2515",
		.platform_data		= &mcp251x_info,
		.controller_data	= &mcp251x_spits_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 10000000, /* max sample rate at 3V */
		.bus_num		= 1,
		.chip_select		= 2,
	},
	[1] = {
		.modalias		= "mcp2515",
		.platform_data		= &mcp251x_info,
		.controller_data	= &mcp251x_spits_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 10000000, /* max sample rate at 3V */
		.bus_num		= 1,
		.chip_select		= 3,
	},
};

static int __init tl138_evm_mcp251x_init(void)
{
	int i;
	int ret;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(mcp251x_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}
	ret = gpio_request(MCP251X_CAN0_IRQ, "mcp251x_can0_irq");
	if (ret)
		pr_warning("Fail to request mcp251x_can0_irq PIN.\n");
	gpio_direction_input(MCP251X_CAN0_IRQ);
	gpio_free(MCP251X_CAN0_IRQ);

	ret = gpio_request(MCP251X_CAN1_IRQ, "mcp251x_can1_irq");
	if (ret)
		pr_warning("Fail to request mcp251x_can1_irq PIN.\n");
	gpio_direction_input(MCP251X_CAN1_IRQ);
	gpio_free(MCP251X_CAN1_IRQ);


	mcp251x_spi_info[0].irq = gpio_to_irq(MCP251X_CAN0_IRQ);
	mcp251x_spi_info[1].irq = gpio_to_irq(MCP251X_CAN1_IRQ);

	ret = spi_register_board_info(mcp251x_spi_info,
				 ARRAY_SIZE(mcp251x_spi_info));
	if (ret)
		pr_warning("%s: failed to register board info for spi 1 :"
			   " %d\n", __func__, ret);

	return 0;
}

static void __exit tl138_evm_mcp251x_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(mcp251x_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	pr_warning("WARNING: mcp251x not unregister board for spi 1\n");

}

module_init(tl138_evm_mcp251x_init);
module_exit(tl138_evm_mcp251x_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL2515 CAN module init driver");
MODULE_LICENSE("GPL v2");
