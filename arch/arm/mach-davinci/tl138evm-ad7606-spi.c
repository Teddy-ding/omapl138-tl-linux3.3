/*
 * TL138 EVM board
 *
 * Copyright (C) 2009 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-ad7606-spi.c
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

static char *ad7606_spi_mux_name[] = {
	"SPI1_CS_3",
	"GPIO0_13",
	"GPIO6_8",
	"GPIO6_18",
};

#define PINMUX_SET_NUM		ARRAY_SIZE(ad7606_spi_mux_name)

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define AD7606_SPI_BUSY		GPIO_TO_PIN(6, 8)
#define AD7606_SPI_CONVST		GPIO_TO_PIN(6, 10)
#define AD7606_SPI_RESET		GPIO_TO_PIN(0, 13)

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

static struct ad7606_platform_data ad7606_spi_pdata = {
	.default_os		= 0,
	.default_range		= 5000,
	.gpio_convst		= AD7606_SPI_CONVST,
	.gpio_reset		= AD7606_SPI_RESET,
	.gpio_range		= -1,
	.gpio_os0		= -1,
	.gpio_os1		= -1,
	.gpio_os2		= -1,
	.gpio_frstdata		= -1,
	.gpio_stby		= -1,
};

static struct davinci_spi_config da850evm_spiad_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
};

static struct spi_board_info ad7606_spi_info[] = {
	[0] = {
		.modalias		= "ad7606-8",
		.platform_data		= &ad7606_spi_pdata,
		.controller_data	= &da850evm_spiad_cfg,
		.mode			= SPI_MODE_3,
		.max_speed_hz		= 15000000, /* max sample rate at 3.3V */
		.bus_num		= 1,
		.chip_select		= 3,
	},
};

static int __init tl138_evm_ad7606_spi_init(void)
{
	int i;
	int ret;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(ad7606_spi_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}

	ad7606_spi_info[0].irq = gpio_to_irq(AD7606_SPI_BUSY);

	ret = spi_register_board_info(ad7606_spi_info,
				 ARRAY_SIZE(ad7606_spi_info));
	if (ret)
		pr_warning("%s: failed to register board info for spi 1 :"
			   " %d\n", __func__, ret);

	return 0;
}

static void __exit tl138_evm_ad7606_spi_exit(void)
{
	int i;

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(ad7606_spi_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	pr_warning("WARNING: ad7606 not unregister board for spi 1\n");
}

module_init(tl138_evm_ad7606_spi_init);
module_exit(tl138_evm_ad7606_spi_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL7606 ADC module init driver");
MODULE_LICENSE("GPL v2");
