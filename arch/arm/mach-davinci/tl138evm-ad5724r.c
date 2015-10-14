/*
 * TL138 EVM board
 *
 * Copyright (C) 2009 GuangZhou Tronlong co., LTD - http://www.tronlong.com
 *
 * Derived from: arch/arm/mach-davinci/tl138evm-ad5724r.c
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

static char *ad5724r_mux_name[] = {
	"SPI1_CS_2",
	"GPIO6_14",
};

#define PINMUX_SET_NUM		2

static unsigned char mode_old_list[PINMUX_SET_NUM];
static unsigned char flag_list[PINMUX_SET_NUM];

#define AD5724_LDAC		GPIO_TO_PIN(6, 14)

static struct spi_board_info ad5724r_spi_info[] = {
	[0] = {
		/* the modalias must be the same as spi device driver name */
		.modalias = "ad5724r", /* Name of spi_driver for this device */
		.max_speed_hz = 1000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 1, /* Framework bus number */
		.chip_select = 2, /* Framework chip select */
		.mode = SPI_MODE_1,
	},
};

static int __init tl138_evm_ad5724r_init(void)
{
	int i;
	int ret;

	memset(flag_list, 0, PINMUX_SET_NUM);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		ret = davinci_cfg_reg_name(ad5724r_mux_name[i],
			&mode_old_list[i], &flag_list[i]);
	}

	ret = gpio_request(AD5724_LDAC, "AD5724 LDAC");
	if (ret)
		pr_warning("Fail to request AD5724 LDAC PIN.\n");

	gpio_direction_output(AD5724_LDAC, 0);

	ret = spi_register_board_info(ad5724r_spi_info,
				 ARRAY_SIZE(ad5724r_spi_info));
	if (ret)
		pr_warning("%s: failed to register board info for spi 1 :"
			   " %d\n", __func__, ret);

	return 0;
}

static void __exit tl138_evm_ad5724r_exit(void)
{
	int i;

	gpio_free(AD5724_LDAC);

	for (i = 0; i < PINMUX_SET_NUM; i++) {
		if (flag_list[i] == 0)
			continue;

		davinci_cfg_reg_name(ad5724r_mux_name[i],
					&mode_old_list[i], &flag_list[i]);
	}

	pr_warning("WARNING: ad5724r not unregister board for spi 1\n");
}

module_init(tl138_evm_ad5724r_init);
module_exit(tl138_evm_ad5724r_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("TL5724 DAC module init driver");
MODULE_LICENSE("GPL v2");
