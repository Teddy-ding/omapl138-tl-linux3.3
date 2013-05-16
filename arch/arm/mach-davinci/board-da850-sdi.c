/*
 * SDI DA850 Development Board
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Derived from: arch/arm/mach-davinci/board-da830-evm.c
 * Original Copyrights follow:
 *
 * 2007, 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/mfd/tps6507x.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6507x.h>
#include <linux/input/tps6507x-ts.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/usb/musb.h>
#include <linux/i2c-gpio.h>
#include <linux/module.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wifi_tiwlan.h>
#endif

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/nand.h>
#include <mach/mux.h>
#include <linux/mtd/physmap.h>
#include <media/davinci/vpif_types.h>
#include <linux/mfd/davinci_aemif.h>

#include <media/tvp514x.h>

#define DA850_EVM_PHY_ID		"davinci_mdio-0:00"
#define DA850_LCD_PWR_PIN		GPIO_TO_PIN(2, 8)
#define DA850_LCD_BL_PIN		GPIO_TO_PIN(2, 15)

#define DA850_MMCSD_CD_PIN		GPIO_TO_PIN(4, 0)
#define DA850_MMCSD_WP_PIN		GPIO_TO_PIN(4, 1)

#define DA850_MII_MDIO_CLKEN_PIN	GPIO_TO_PIN(2, 6)
#ifdef CONFIG_WIFI_CONTROL_FUNC
/*WLAN GPIO PIN*/
#define DA850_WLAN_EN			GPIO_TO_PIN(6, 9)
#define DA850_WLAN_IRQ			GPIO_TO_PIN(6, 10)
#define DA850_BT_EN			GPIO_TO_PIN(0, 15)
#endif

#define TVP5147_CH0		"tvp514x-0"
#define TVP5147_CH1		"tvp514x-1"

#define VPIF_STATUS	(0x002C)
#define VPIF_STATUS_CLR	(0x0030)
#define DA850_USB1_VBUS_PIN		GPIO_TO_PIN(2, 4)
#define DA850_USB1_OC_PIN		GPIO_TO_PIN(6, 13)

//HACK
#define CONFIG_DA850_UI_EXPANDER

#define DAVINCI_BACKLIGHT_MAX_BRIGHTNESS	250
#define DAVINVI_BACKLIGHT_DEFAULT_BRIGHTNESS	250
#define DAVINCI_PWM_PERIOD_NANO_SECONDS		10000000

static struct platform_pwm_backlight_data da850evm_backlight_data = {
	.max_brightness	= DAVINCI_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness	= DAVINVI_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns	= DAVINCI_PWM_PERIOD_NANO_SECONDS,
};

static struct platform_device da850evm_backlight = {
	.name		= "pwm-backlight",
	.id		= -1,
};

static struct davinci_pm_config da850_pm_pdata = {
	.sleepcount = 128,
};

static struct platform_device da850_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data	= &da850_pm_pdata,
	},
	.id             = -1,
};

#ifdef CONFIG_DA850_UI_EXPANDER
static struct mtd_partition da850_evm_norflash_partition[] = {
	{
		.name           = "bootloaders + env",
		.offset         = 0,
		.size           = SZ_512K,
		.mask_flags     = MTD_WRITEABLE,
	},
	{
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_2M,
		.mask_flags     = 0,
	},
	{
		.name           = "filesystem",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
		.mask_flags     = 0,
	},
};

static struct davinci_aemif_timing da850_evm_norflash_timing = {
	.wsetup		= 10,
	.wstrobe	= 60,
	.whold		= 10,
	.rsetup		= 10,
	.rstrobe	= 110,
	.rhold		= 10,
	.ta		= 30,
};

static struct physmap_flash_data da850_evm_norflash_data = {
	.width		= 2,
	.parts		= da850_evm_norflash_partition,
	.nr_parts	= ARRAY_SIZE(da850_evm_norflash_partition),
	.timing		= &da850_evm_norflash_timing,
};

static struct resource da850_evm_norflash_resource[] = {
	{
		.start	= DA8XX_AEMIF_CS2_BASE,
		.end	= DA8XX_AEMIF_CS2_BASE + SZ_32M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device da850_evm_norflash_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data  = &da850_evm_norflash_data,
	},
	.num_resources	= ARRAY_SIZE(da850_evm_norflash_resource),
	.resource	= da850_evm_norflash_resource,
};

/* DA850/OMAP-L138 EVM includes a 512 MByte large-page NAND flash
 * (128K blocks). It may be used instead of the (default) SPI flash
 * to boot, using TI's tools to install the secondary boot loader
 * (UBL) and U-Boot.
 */
struct mtd_partition da850_evm_nandflash_partition[] = {
	{
		.name		= "u-boot env",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "UBL",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "u-boot",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 4 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= 0x200000,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	},
};

static struct davinci_aemif_timing da850_evm_nandflash_timing = {
	.wsetup		= 24,
	.wstrobe	= 21,
	.whold		= 14,
	.rsetup		= 19,
	.rstrobe	= 50,
	.rhold		= 0,
	.ta		= 20,
};

static struct davinci_nand_pdata da850_evm_nandflash_data = {
	.parts		= da850_evm_nandflash_partition,
	.nr_parts	= ARRAY_SIZE(da850_evm_nandflash_partition),
	.ecc_mode	= NAND_ECC_HW,
	.ecc_bits	= 4,
	.bbt_options	= NAND_BBT_USE_FLASH,
	.timing		= &da850_evm_nandflash_timing,
};

static struct resource da850_evm_nandflash_resource[] = {
	{
		.start	= DA8XX_AEMIF_CS3_BASE,
		.end	= DA8XX_AEMIF_CS3_BASE + SZ_512K + 2 * SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= DA8XX_AEMIF_CTL_BASE,
		.end	= DA8XX_AEMIF_CTL_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device da850_evm_nandflash_device = {
	.name		= "davinci_nand",
	.id		= 1,
	.dev		= {
		.platform_data	= &da850_evm_nandflash_data,
	},
	.num_resources	= ARRAY_SIZE(da850_evm_nandflash_resource),
	.resource	= da850_evm_nandflash_resource,
};

static struct platform_device *da850_evm_devices[] __initdata = {
	&da850_evm_nandflash_device,
	&da850_evm_norflash_device,
};
#endif

static struct mtd_partition spi_flash_partitions[] = {
	[0] = {
		.name = "U-Boot",
		.offset = 0,
		.size = SZ_256K,
		.mask_flags = MTD_WRITEABLE,
	},
	[1] = {
		.name = "U-Boot Environment",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
	[2] = {
		.name = "Linux",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_8M - (SZ_256K + SZ_64K + SZ_64K),
		.mask_flags = 0,
	},
	[3] = {
		.name = "MAC Address",
		.offset = MTDPART_OFS_NXTBLK,
		.size = SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
};

static struct flash_platform_data spi_flash_data = {
	.name = "m25p80",
	.parts = spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(spi_flash_partitions),
	.type = "m25p64",
};

static struct davinci_spi_config da850evm_spiflash_cfg = {
	.io_type	= SPI_IO_TYPE_DMA,
	.c2tdelay	= 8,
	.t2cdelay	= 8,
};

static struct spi_board_info da850evm_spi_info[] = {
	{
		.modalias		= "m25p80",
		.platform_data		= &spi_flash_data,
		.controller_data	= &da850evm_spiflash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 0,
	},
};

/* ---- WIFI ---- */
#ifdef CONFIG_WIFI_CONTROL_FUNC
static void (*wifi_status_cb)(void *dev_id, int card_present);
static void *wifi_status_cb_devid;
/* WIFI virtual 'card detect' status */
static int am1808_wifi_cd;
static int am1808_wifi_status_register(
	void (*callback)(void *dev_id, int card_present), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int am1808_wifi_status(int index)
{
	return am1808_wifi_cd;
}

static int am1808_wifi_set_carddetect(int val)
{
	am1808_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(wifi_status_cb_devid, val);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);

	return 0;
}

static int am1808_wifi_power(int on)
{
	gpio_set_value_cansleep(DA850_WLAN_EN, on);
	return 0;
}

struct wifi_platform_data am1808_wifi_control = {
	.set_power		= am1808_wifi_power,
	.set_carddetect		= am1808_wifi_set_carddetect,
};

static struct resource am1808_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.start		= -1,
		.end		= -1,
		.flags		= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device am1808_wifi = {
	.name		= "device_wifi",
	.id		= 1,
	.num_resources	= 1,
	.resource	= am1808_wifi_resources,
	.dev		= {
		.platform_data = &am1808_wifi_control,
	},
};

static void __init wifi_power_init(void)
{
	if (gpio_request(DA850_WLAN_EN, "wlan_en")) {
		printk(KERN_ERR "Failed to request gpio DA850_WLAN_EN\n");
		return;
	}

	gpio_direction_output(DA850_WLAN_EN, 1);
	if (gpio_request(DA850_WLAN_IRQ, "wlan_irq")) {
		printk(KERN_ERR "Failed to request gpio DA850_WLAN_IRQ_GPIO\n");
		gpio_free(DA850_WLAN_EN);
		return;
	}
	am1808_wifi_resources[0].start = am1808_wifi_resources[0].end =
					gpio_to_irq(DA850_WLAN_IRQ);
	/* In order to ensure order of PM functions */
	am1808_wifi.dev.parent = NULL;
	platform_device_register(&am1808_wifi);
}
#endif

#if defined(CONFIG_MMC_DAVINCI) || \
    defined(CONFIG_MMC_DAVINCI_MODULE)
#define HAS_MMC 1
#else
#define HAS_MMC 0
#endif

#ifdef CONFIG_DA850_UI_EXPANDER
static const short da850_evm_nand_pins[] = {
	DA850_EMA_D_0, DA850_EMA_D_1, DA850_EMA_D_2, DA850_EMA_D_3,
	DA850_EMA_D_4, DA850_EMA_D_5, DA850_EMA_D_6, DA850_EMA_D_7,
	DA850_EMA_A_1, DA850_EMA_A_2, DA850_NEMA_CS_3, DA850_NEMA_CS_4,
	DA850_NEMA_WE, DA850_NEMA_OE,
	-1
};

#define DA8XX_AEMIF_CE2CFG_OFFSET	0x10
#define DA8XX_AEMIF_ASIZE_16BIT		0x1

static void __init da850_evm_init_nor(void)
{
	void __iomem *aemif_addr;

	aemif_addr = ioremap(DA8XX_AEMIF_CTL_BASE, SZ_32K);

	/* Configure data bus width of CS2 to 16 bit */
	writel(readl(aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET) |
		DA8XX_AEMIF_ASIZE_16BIT,
		aemif_addr + DA8XX_AEMIF_CE2CFG_OFFSET);

	iounmap(aemif_addr);
}

static const short da850_evm_nor_pins[] = {
	DA850_EMA_BA_1, DA850_EMA_CLK, DA850_EMA_WAIT_1, DA850_NEMA_CS_2,
	DA850_NEMA_WE, DA850_NEMA_OE, DA850_EMA_D_0, DA850_EMA_D_1,
	DA850_EMA_D_2, DA850_EMA_D_3, DA850_EMA_D_4, DA850_EMA_D_5,
	DA850_EMA_D_6, DA850_EMA_D_7, DA850_EMA_D_8, DA850_EMA_D_9,
	DA850_EMA_D_10, DA850_EMA_D_11, DA850_EMA_D_12, DA850_EMA_D_13,
	DA850_EMA_D_14, DA850_EMA_D_15, DA850_EMA_A_0, DA850_EMA_A_1,
	DA850_EMA_A_2, DA850_EMA_A_3, DA850_EMA_A_4, DA850_EMA_A_5,
	DA850_EMA_A_6, DA850_EMA_A_7, DA850_EMA_A_8, DA850_EMA_A_9,
	DA850_EMA_A_10, DA850_EMA_A_11, DA850_EMA_A_12, DA850_EMA_A_13,
	DA850_EMA_A_14, DA850_EMA_A_15, DA850_EMA_A_16, DA850_EMA_A_17,
	DA850_EMA_A_18, DA850_EMA_A_19, DA850_EMA_A_20, DA850_EMA_A_21,
	DA850_EMA_A_22, DA850_EMA_A_23,
	-1
};

#if defined(CONFIG_MMC_DAVINCI) || \
	defined(CONFIG_MMC_DAVINCI_MODULE)
#define HAS_MMC 1
#else
#define HAS_MMC 0
#endif

#if defined(CONFIG_SPI_DAVINCI)
#define HAS_SPI 1
#else
#define HAS_SPI 0
#endif

#if defined(CONFIG_FB_DA8XX)
#define HAS_LCD	1
#else
#define HAS_LCD	0
#endif

#if defined(CONFIG_SND_DA850_SOC_EVM) || \
	defined(CONFIG_SND_DA850_SOC_EVM_MODULE)
#define HAS_MCASP 1
#else
#define HAS_MCASP 0
#endif

#if defined(CONFIG_DAVINCI_EHRPWM) || defined(CONFIG_DAVINCI_EHRPWM_MODULE)
#define HAS_EHRPWM 1
#else
#define HAS_EHRPWM 0
#endif

#if defined(CONFIG_ECAP_PWM) || \
	defined(CONFIG_ECAP_PWM_MODULE)
#define HAS_ECAP_PWM 1
#else
#define HAS_ECAP_PWM 0
#endif

#if defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)
#define HAS_BACKLIGHT 1
#else
#define HAS_BACKLIGHT 0
#endif

#if defined(CONFIG_ECAP_CAP) || defined(CONFIG_ECAP_CAP_MODULE)
#define HAS_ECAP_CAP 1
#else
#define HAS_ECAP_CAP 0
#endif

static u32 ui_card_detected;

static __init void da850_evm_setup_nor_nand(void)
{
	int ret = 0;

	if (ui_card_detected & !HAS_MMC) {
		ret = davinci_cfg_reg_list(da850_evm_nand_pins);
		if (ret)
			pr_warning("da850_evm_init: nand mux setup failed: "
					"%d\n", ret);

		ret = davinci_cfg_reg_list(da850_evm_nor_pins);
		if (ret)
			pr_warning("da850_evm_init: nor mux setup failed: %d\n",
				ret);
		da850_evm_init_nor();

		platform_add_devices(da850_evm_devices,
					ARRAY_SIZE(da850_evm_devices));
	}
}
#endif
static const short da850_mcbsp1_pins[] = {
	DA850_MCBSP1_CLKR, DA850_MCBSP1_CLKX, DA850_MCBSP1_FSR,
	DA850_MCBSP1_FSX, DA850_MCBSP1_DR, DA850_MCBSP1_DX, DA850_MCBSP1_CLKS,
	-1
};


#ifdef CONFIG_DA850_UI_RMII
static inline void da850_evm_setup_emac_rmii(int rmii_sel)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->rmii_en = 1;
	gpio_set_value_cansleep(rmii_sel, 0);
}
#else
static inline void da850_evm_setup_emac_rmii(int rmii_sel) { }
#endif

#ifdef CONFIG_DA850_UI_CLCD
static inline void da850_evm_setup_char_lcd(int a, int b, int c)
{
	gpio_set_value_cansleep(a, 0);
	gpio_set_value_cansleep(b, 0);
	gpio_set_value_cansleep(c, 0);
}
#else
static inline void da850_evm_setup_char_lcd(int a, int b, int c) { }
#endif

#ifdef CONFIG_DA850_UI_VIDEO_PORT
static inline void da850_evm_setup_video_port(int video_sel)
{
	gpio_set_value_cansleep(video_sel, 0);
}
#else
static inline void da850_evm_setup_video_port(int video_sel) { }
#endif

#ifdef CONFIG_DA850_UI_EXPANDER
static int da850_evm_ui_expander_setup(struct i2c_client *client, unsigned gpio,
						unsigned ngpio, void *c)
{
	int sel_a, sel_b, sel_c, ret;

	sel_a = gpio + 7;
	sel_b = gpio + 6;
	sel_c = gpio + 5;

	ret = gpio_request(sel_a, "sel_a");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", sel_a);
		goto exp_setup_sela_fail;
	}

	ret = gpio_request(sel_b, "sel_b");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", sel_b);
		goto exp_setup_selb_fail;
	}

	ret = gpio_request(sel_c, "sel_c");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", sel_c);
		goto exp_setup_selc_fail;
	}

	/* deselect all functionalities */
	gpio_direction_output(sel_a, 1);
	gpio_direction_output(sel_b, 1);
	gpio_direction_output(sel_c, 1);

	ui_card_detected = 1;
	pr_info("DA850/OMAP-L138 EVM UI card detected\n");

	da850_evm_setup_nor_nand();

	da850_evm_setup_emac_rmii(sel_a);

	da850_evm_setup_char_lcd(sel_a, sel_b, sel_c);

	da850_evm_setup_video_port(sel_c);

	return 0;

exp_setup_selc_fail:
	gpio_free(sel_b);
exp_setup_selb_fail:
	gpio_free(sel_a);
exp_setup_sela_fail:
	return ret;
}

static int da850_evm_ui_expander_teardown(struct i2c_client *client,
					unsigned gpio, unsigned ngpio, void *c)
{
	/* deselect all functionalities */
	gpio_set_value_cansleep(gpio + 5, 1);
	gpio_set_value_cansleep(gpio + 6, 1);
	gpio_set_value_cansleep(gpio + 7, 1);

	gpio_free(gpio + 5);
	gpio_free(gpio + 6);
	gpio_free(gpio + 7);

	return 0;
}

static struct pca953x_platform_data da850_evm_ui_expander_info = {
	.gpio_base	= DAVINCI_N_GPIO,
	.setup		= da850_evm_ui_expander_setup,
	.teardown	= da850_evm_ui_expander_teardown,
};
#endif
#ifdef CONFIG_DA850_SDI_MII
static int da850_evm_expander_setup(struct i2c_client *client, unsigned gpio,
						unsigned ngpio, void *c)
{
	int sel_a, ret;

	sel_a = gpio + 4;
	ret = gpio_request(sel_a, "sel_a");

	if (ret) {
		pr_warning("Cannot open base evm expander pin %d\n", sel_a);
		return ret;
	}
	gpio_direction_output(sel_a, 0);

	return 0;
}

static int da850_evm_expander_teardown(struct i2c_client *client,
					unsigned gpio, unsigned ngpio, void *c)
{
	/* deselect functionalities */
	gpio_set_value_cansleep(gpio + 4, 0);
	gpio_free(gpio + 4);

	return 0;
}

/* assign the baseboard expander's GPIOs after the UI board's */
#define DA850_UI_EXPANDER_N_GPIOS 16
#define DA850_BB_EXPANDER_GPIO_BASE (DAVINCI_N_GPIO + DA850_UI_EXPANDER_N_GPIOS)

static struct pca953x_platform_data da850_evm_expander_info = {
	.gpio_base	= DA850_BB_EXPANDER_GPIO_BASE,
	.setup		= da850_evm_expander_setup,
	.teardown	= da850_evm_expander_teardown,
};
#endif
/* TPS65070 voltage regulator support */

/* 3.3V */
struct regulator_consumer_supply tps65070_dcdc1_consumers[] = {
	{
		.supply = "usb0_vdda33",
	},
	{
		.supply = "usb1_vdda33",
	},
};

/* 3.3V or 1.8V */
struct regulator_consumer_supply tps65070_dcdc2_consumers[] = {
	{
		.supply = "dvdd3318_a",
	},
	{
		.supply = "dvdd3318_b",
	},
	{
		.supply = "dvdd3318_c",
	},
};

/* 1.2V */
struct regulator_consumer_supply tps65070_dcdc3_consumers[] = {
	{
		.supply = "cvdd",
	},
};

/* 1.8V LDO */
struct regulator_consumer_supply tps65070_ldo1_consumers[] = {
	{
		.supply = "sata_vddr",
	},
	{
		.supply = "usb0_vdda18",
	},
	{
		.supply = "usb1_vdda18",
	},
	{
		.supply = "ddr_dvdd18",
	},
};

/* 1.2V LDO */
struct regulator_consumer_supply tps65070_ldo2_consumers[] = {
	{
		.supply = "sata_vdd",
	},
	{
		.supply = "pll0_vdda",
	},
	{
		.supply = "pll1_vdda",
	},
	{
		.supply = "usbs_cvdd",
	},
	{
		.supply = "vddarnwa1",
	},
};

static struct tps6507x_reg_platform_data tps6507x_platform_data = {
	.defdcdc_default = true,
};

struct regulator_init_data tps65070_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 3150000,
			.max_uV = 3450000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65070_dcdc1_consumers),
		.consumer_supplies = tps65070_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 1710000,
			.max_uV = 3450000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65070_dcdc2_consumers),
		.consumer_supplies = tps65070_dcdc2_consumers,
		.driver_data = &tps6507x_platform_data,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 950000,
			.max_uV = 1350000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65070_dcdc3_consumers),
		.consumer_supplies = tps65070_dcdc3_consumers,
		.driver_data = &tps6507x_platform_data,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1710000,
			.max_uV = 1890000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65070_ldo1_consumers),
		.consumer_supplies = tps65070_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 1140000,
			.max_uV = 1320000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65070_ldo2_consumers),
		.consumer_supplies = tps65070_ldo2_consumers,
	},
};

static struct touchscreen_init_data tps6507x_touchscreen_data = {
	.poll_period =  30,	/* ms between touch samples */
	.min_pressure = 0x30,	/* minimum pressure to trigger touch */
	.vref = 0,		/* turn off vref when not using A/D */
	.vendor = 0,		/* /sys/class/input/input?/id/vendor */
	.product = 65070,	/* /sys/class/input/input?/id/product */
	.version = 0x100,	/* /sys/class/input/input?/id/version */
};

static struct tps6507x_board tps_board = {
	.tps6507x_pmic_init_data = &tps65070_regulator_data[0],
	.tps6507x_ts_init_data = &tps6507x_touchscreen_data,
};

static struct i2c_board_info __initdata da850_evm_i2c_devices[] = {
	{
		I2C_BOARD_INFO("da850sdi_keys", 0x25),
	},
	{
		I2C_BOARD_INFO("tps6507x", 0x48),
		.platform_data = &tps_board,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
#ifdef CONFIG_DA850_UI_EXPANDER
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &da850_evm_ui_expander_info,
	},
#endif
#ifdef CONFIG_DA850_SDI_MII
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &da850_evm_expander_info,
	},
#endif
	{
		I2C_BOARD_INFO("cdce913", 0x64),
	},
#ifndef CONFIG_DA850_SDI_MII
	{
		I2C_BOARD_INFO("tfp410", 0x21),
	},
#endif
};

typedef struct tfp_reg {
	u8	addr;
	u8	val;
} tfp_reg;

static struct i2c_client *muteClient = NULL;
/*
 * The CODEC_MUTE signal enables/disables the HW mute circuit.
 *     high = muted
 *     low = unmuted
 * It is pulled high by a resistor so is muted at power on.
 * The signal is controlled by P11 of the TCA6116 I2C GPIO expander.
 * 8bit Register 3 of the TCA controls P10-P17.
 * Therefore CODEC_MUTE is controlled by bit 1 of register 3
 */
#define CODEC_MUTE (1 << 1)
int da850_sdi_mute(int state)
{
    u8 addr = 0x03;
    u8 val;
    int ret;

    if (muteClient == NULL)
	return 0;

    /* read modify write */
    ret = i2c_smbus_read_byte_data(muteClient, addr);
    if (ret < 0) {
	printk (KERN_WARNING "Failed to read TCA6416 (0x21) register 0x%02x.", addr);
	return ret;
    }

    val = ret;
    if (state) {
#ifdef CONFIG_SND_DEBUG
	printk(KERN_WARNING "da850-sdi: Mute\n");
#endif
	val |= CODEC_MUTE;
    }
    else {
#ifdef CONFIG_SND_DEBUG
	printk(KERN_WARNING "da850-sdi: Umute\n");
#endif
	val &= ~CODEC_MUTE;
    }

    ret = i2c_smbus_write_byte_data(muteClient, addr, val);
    if (ret) {
	printk (KERN_WARNING "Failed to write to TCA6416 (0x21) register 0x%02x.", addr);
	return ret;
    }

    return 0;
}
EXPORT_SYMBOL(da850_sdi_mute);

static int tfp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	int i;
#ifdef CONFIG_DA850_SDI_LCDC
	struct tfp_reg regData[] = {
	    { 0x02, 0xff },
	    { 0x06, 0xff },
	    { 0x07, 0xfc },
	    { 0x03, 0xfe },
	    { 0x03, 0xff },
	};
#else
	struct tfp_reg regData[] = {
	    { 0x02, 0xfb },
	    { 0x06, 0xfb },
	    { 0x07, 0xfc },
	    { 0x03, 0xfe },
	    { 0x03, 0xff },
	};
#endif

	for (i=0; i < sizeof(regData) / (sizeof(tfp_reg)); i++) {
	    ret = i2c_smbus_write_byte_data(client,
	    				    regData[i].addr, regData[i].val);
	    if (ret) { 
	    	printk (KERN_WARNING "Failed to write to TFP410 register 0x%02x.", regData[i].addr);
		return ret;
	    }
	}

	muteClient = client;
	da850_sdi_mute (1);
	return ret;
}

static int __devexit tfp_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tfp_id[] = {
	{ "tfp410", 0 },
	{ }
};

static struct i2c_driver tfp_driver = {
	.driver = {
		.name	= "tfp410",
	},
	.probe		= tfp_probe,
	.remove		= tfp_remove,
	.id_table	= tfp_id,
};

static struct davinci_uart_config da850_evm_uart_config __initdata = {
	.enabled_uarts = 0x7,
};

typedef struct cdce_reg {
	u8	addr;
	u8	val;
} cdce_reg;

static int cdce_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	int i;
	struct cdce_reg regData[] = {
	    { 0x03, 0x09 },
	    { 0x14, 0x6d },
	    { 0x18, 0xc0 },
	    { 0x19, 0x04 },
	    { 0x1A, 0x82 },
	    { 0x1B, 0x07 },
	};

	for (i=0; i < sizeof(regData) / (sizeof(cdce_reg)); i++) {
	    ret = i2c_smbus_write_byte_data(client,
	    				    regData[i].addr | 0x80, regData[i].val);
	    if (ret) { 
	    	printk (KERN_WARNING "Failed to write to CDCE13 register 0x%02x.", regData[i].addr);
		return ret;
	    }
	}

	return ret;
}

static int __devexit cdce_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id cdce_id[] = {
	{ "cdce913", 0 },
	{ }
};

static struct i2c_driver cdce_driver = {
	.driver = {
		.name	= "cdce913",
	},
	.probe		= cdce_probe,
	.remove		= cdce_remove,
	.id_table	= cdce_id,
};

static const short da850_evm_mcasp_pins[] __initconst = {
	DA850_AHCLKX, DA850_ACLKX, DA850_AFSX,
	DA850_ACLKR, DA850_AFSR, DA850_AMUTE,
	DA850_AXR_11, DA850_AXR_12,
	-1
};

//HACK:a see hardcoded values for McBSP1 in devices-da8xx.c
static struct resource da850_asp_resources[] = {
    /* Memory region containing control registers */
    {
	.start	= 0x01D11000,
	.end	= 0x01D11FFF,
	.flags	= IORESOURCE_MEM,
    },
    /* McBSP1 Tx EDMA event */
    {
	.start	= 5,
	.end	= 5,
	.flags	= IORESOURCE_DMA,
    },
    /* McBSP1 Rx EDMA event */
    {
	.start	= 4,
	.end	= 4,
	.flags	= IORESOURCE_DMA,
    },
};

static struct platform_device da850_asp_device = {
	.name		= "davinci-mcbsp",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(da850_asp_resources),
	.resource	= da850_asp_resources,
};

static struct snd_platform_data da850_sdi_snd_data;

static const short da850_evm_mmcsd0_pins[] __initconst = {
	DA850_MMCSD0_DAT_0, DA850_MMCSD0_DAT_1, DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3, DA850_MMCSD0_CLK, DA850_MMCSD0_CMD,
	DA850_GPIO4_0, DA850_GPIO4_1,
	-1
};

static int da850_evm_mmc_get_ro(int index)
{
	return gpio_get_value(DA850_MMCSD_WP_PIN);
}

static int da850_evm_mmc_get_cd(int index)
{
	return !gpio_get_value(DA850_MMCSD_CD_PIN);
}

static struct davinci_mmc_config da850_mmc_config[] = {
	{
		.get_ro		= da850_evm_mmc_get_ro,
		.get_cd		= da850_evm_mmc_get_cd,
		.wires		= 4,
		.max_freq	= 50000000,
		.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version	= MMC_CTLR_VERSION_2,
	},
#ifdef CONFIG_DA850_USE_MMC1
	{
		.get_ro         = da850_evm_mmc_get_ro,
#ifdef CONFIG_WIFI_CONTROL_FUNC
		/* using the virtual carddetect function -
		* so ifconfig up/down will reset the board
		*/
		.get_cd         = am1808_wifi_status,
#else
		.get_cd         = NULL,
#endif
		.wires          = 4,
		.max_freq       = 50000000,
		.caps           = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version        = MMC_CTLR_VERSION_2,
#ifdef CONFIG_WIFI_CONTROL_FUNC
		.status         = am1808_wifi_status,
		.register_status_notify = am1808_wifi_status_register,
#endif

	},
#endif
	{}
};

#ifdef CONFIG_DA850_SDI_LCDC
static void da850_panel_power_ctrl(int val)
{
	/* lcd power */
	gpio_set_value_cansleep(DA850_LCD_PWR_PIN, val);

	mdelay(200);

	/* lcd backlight */
	gpio_set_value_cansleep(DA850_LCD_BL_PIN, val);
}
#endif

static int da850_lcd_hw_init(void)
{
	void __iomem *cfg_mstpri1_base;
	void __iomem *cfg_mstpri2_base;
	void __iomem *emifb;
	void __iomem *myptr;
#ifdef CONFIG_DA850_SDI_LCDC
	int status;
#endif
	u32 val;

	/*
	 * Default master priorities in reg 0 are all lower by default than LCD
	 * which is set below to 0. Hence don't need to change here.
	 */

	/* set EDMA30TC0 and TC1 to lower than LCDC (4 < 0) */
	cfg_mstpri1_base = DA8XX_SYSCFG0_VIRT(DA8XX_MSTPRI1_REG);
	val = __raw_readl(cfg_mstpri1_base);
	val &= 0xFFFF00FF;
	val |= 4 << 8;             /* 0-high, 7-low priority*/
	val |= 4 << 12;            /* 0-high, 7-low priority*/
	__raw_writel(val, cfg_mstpri1_base);

	/*
	 * Reconfigure the LCDC priority to the highest to ensure that
	 * the throughput/latency requirements for the LCDC are met.
	 */
	cfg_mstpri2_base = DA8XX_SYSCFG0_VIRT(DA8XX_MSTPRI2_REG);

	val = __raw_readl(cfg_mstpri2_base);
	val &= 0x0fffffff;
	__raw_writel(val, cfg_mstpri2_base);

	/* set BPRIO */
#define DA8XX_EMIF30_CONTROL_BASE		0xB0000000
#define DA8XX_EMIF30_BPRIO_OFFSET  		0x20
#define DA8XX_EMIFB_VIRT(x)	(emifb + (x))
	emifb = ioremap(DA8XX_EMIF30_CONTROL_BASE, SZ_4K);
	myptr = DA8XX_EMIFB_VIRT(0x20);
	__raw_writel(0x20, myptr);

#ifdef CONFIG_DA850_SDI_LCDC
	status = gpio_request(DA850_LCD_BL_PIN, "lcd bl\n");
	if (status < 0)
		return status;

	status = gpio_request(DA850_LCD_PWR_PIN, "lcd pwr\n");
	if (status < 0) {
		gpio_free(DA850_LCD_BL_PIN);
		return status;
	}

	gpio_direction_output(DA850_LCD_BL_PIN, 0);
	gpio_direction_output(DA850_LCD_PWR_PIN, 0);
#endif

	return 0;
}

static const short da850_evm_lcdc_pins[] = {
	DA850_GPIO2_8, DA850_GPIO2_15,
	-1
};

static const short da850_evm_mii_pins[] = {
	DA850_MII_TXEN, DA850_MII_TXCLK, DA850_MII_COL, DA850_MII_TXD_3,
	DA850_MII_TXD_2, DA850_MII_TXD_1, DA850_MII_TXD_0, DA850_MII_RXER,
	DA850_MII_CRS, DA850_MII_RXCLK, DA850_MII_RXDV, DA850_MII_RXD_3,
	DA850_MII_RXD_2, DA850_MII_RXD_1, DA850_MII_RXD_0, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static const short da850_evm_rmii_pins[] = {
	DA850_RMII_TXD_0, DA850_RMII_TXD_1, DA850_RMII_TXEN,
	DA850_RMII_CRS_DV, DA850_RMII_RXD_0, DA850_RMII_RXD_1,
	DA850_RMII_RXER, DA850_RMII_MHZ_50_CLK, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static int __init da850_evm_config_emac(void)
{
	void __iomem *cfg_chip3_base;
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;
	u8 rmii_en = soc_info->emac_pdata->rmii_en;

	if (!machine_is_davinci_da850_sdi())
		return 0;

	cfg_chip3_base = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);

	val = __raw_readl(cfg_chip3_base);

#ifndef CONFIG_DA850_SDI_MII
	soc_info->emac_pdata->rmii_en = 1;
	rmii_en = 1;
#endif

	if (rmii_en) {
		val |= BIT(8);
		ret = davinci_cfg_reg_list(da850_evm_rmii_pins);
		pr_info("EMAC: RMII PHY configured, MII PHY will not be"
							" functional\n");
	} else {
		val &= ~BIT(8);
		ret = davinci_cfg_reg_list(da850_evm_mii_pins);
		pr_info("EMAC: MII PHY configured, RMII PHY will not be"
							" functional\n");
	}

	if (ret)
		pr_warning("da850_evm_init: cpgmac/rmii mux setup failed: %d\n",
				ret);

	/* configure the CFGCHIP3 register for RMII or MII */
	__raw_writel(val, cfg_chip3_base);

	ret = davinci_cfg_reg(DA850_GPIO2_6);
	if (ret)
		pr_warning("da850_evm_init:GPIO(2,6) mux setup "
							"failed\n");

	ret = gpio_request(DA850_MII_MDIO_CLKEN_PIN, "mdio_clk_en");
	if (ret) {
		pr_warning("Cannot open GPIO %d\n",
					DA850_MII_MDIO_CLKEN_PIN);
		return ret;
	}

	/* Enable/Disable MII MDIO clock */
	gpio_direction_output(DA850_MII_MDIO_CLKEN_PIN, rmii_en);

	soc_info->emac_pdata->phy_id = DA850_EVM_PHY_ID;
	ret = da8xx_register_emac();
	if (ret)
		pr_warning("da850_evm_init: emac registration failed: %d\n",
				ret);

	return 0;
}
device_initcall(da850_evm_config_emac);

/* Retaining these APIs, since the VPIF drivers do not check NULL handlers */
static int da850_set_vpif_clock(int mux_mode, int hd)
{
	return 0;
}

static int da850_setup_vpif_input_channel_mode(int mux_mode)
{
	return 0;
}

static int da850_vpif_intr_status(void __iomem *vpif_base, int channel)
{
	int status = 0;
	int mask;

	if (channel < 0 || channel > 3)
		return 0;

	mask = 1 << channel;
	status = __raw_readl((vpif_base + VPIF_STATUS)) & mask;
	__raw_writel(status, (vpif_base + VPIF_STATUS_CLR));

	return status;
}

/* VPIF capture configuration */
static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity = 0,
	.hs_polarity = 1,
	.vs_polarity = 1
};

#define TVP514X_STD_ALL (V4L2_STD_NTSC | V4L2_STD_PAL)

static struct vpif_subdev_info da850_vpif_capture_sdev_info[] = {
	{
		.name	= TVP5147_CH0,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5d),
			.platform_data = &tvp5146_pdata,
		},
		.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol = 1,
			.vd_pol = 1,
			.fid_pol = 0,
		},
	},
	{
		.name	= TVP5147_CH1,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5c),
			.platform_data = &tvp5146_pdata,
		},
		.input = INPUT_SVIDEO_VI2C_VI1C,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol = 1,
			.vd_pol = 1,
			.fid_pol = 0,
		},
	},
};

static const struct vpif_input da850_ch0_inputs[] = {
	{
		.input = {
			.index = 0,
			.name = "Composite",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = TVP5147_CH0,
	},
};

static const struct vpif_input da850_ch1_inputs[] = {
       {
		.input = {
			.index = 0,
			.name = "S-Video",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = TVP5147_CH1,
	},
};

static struct vpif_capture_config da850_vpif_capture_config = {
	.setup_input_channel_mode = da850_setup_vpif_input_channel_mode,
	.intr_status = da850_vpif_intr_status,
	.subdev_info = da850_vpif_capture_sdev_info,
	.subdev_count = ARRAY_SIZE(da850_vpif_capture_sdev_info),
	.chan_config[0] = {
		.inputs = da850_ch0_inputs,
		.input_count = ARRAY_SIZE(da850_ch0_inputs),
	},
	.chan_config[1] = {
		.inputs = da850_ch1_inputs,
		.input_count = ARRAY_SIZE(da850_ch1_inputs),
	},
	.card_name      = "DA850/OMAP-L138 Video Capture",
};

/* VPIF display configuration */
static struct vpif_subdev_info da850_vpif_subdev[] = {
	{
		.name	= "adv7343",
		.board_info = {
			I2C_BOARD_INFO("adv7343", 0x2a),
		},
	},
};

static const char *vpif_output[] = {
	"Composite",
	"Component",
	"S-Video",
};

static struct vpif_display_config da850_vpif_display_config = {
	.set_clock	= da850_set_vpif_clock,
	.intr_status	= da850_vpif_intr_status,
	.subdevinfo	= da850_vpif_subdev,
	.subdev_count	= ARRAY_SIZE(da850_vpif_subdev),
	.output		= vpif_output,
	.output_count	= ARRAY_SIZE(vpif_output),
	.card_name	= "DA850/OMAP-L138 Video Display",
};

#if defined(CONFIG_DAVINCI_MCBSP0)
#define HAS_MCBSP0 1
#else
#define HAS_MCBSP0 0
#endif

#if defined(CONFIG_DAVINCI_MCBSP1)
#define HAS_MCBSP1 1
#else
#define HAS_MCBSP1 0
#endif

#if defined(CONFIG_TI_DAVINCI_EMAC) || \
	defined(CONFIG_TI_DAVINCI_EMAC_MODULE)
#define HAS_EMAC 1
#else
#define HAS_EMAC 0
#endif

#if defined(CONFIG_DA850_UI_RMII) && (HAS_EMAC)
#define HAS_RMII 1
#else
#define HAS_RMII 0
#endif

#if defined(CONFIG_DA850_UI_LCD) && defined(CONFIG_FB_DA8XX) ||\
		defined(CONFIG_FB_DA8XX_MODULE)
#define HAS_GLCD 1
#else
#define HAS_GLCD 0
#endif

#if defined(CONFIG_VIDEO_DAVINCI_VPIF_DISPLAY) ||\
		defined(CONFIG_VIDEO_DAVINCI_VPIF_DISPLAY_MODULE)
#define HAS_VPIF_DISPLAY 1
#else
#define HAS_VPIF_DISPLAY 0
#endif

#if defined(CONFIG_VIDEO_DAVINCI_VPIF_CAPTURE) ||\
		defined(CONFIG_VIDEO_DAVINCI_VPIF_CAPTURE_MODULE)
#define HAS_VPIF_CAPTURE 1
#else
#define HAS_VPIF_CAPTURE 0
#endif

/*
 * USB1 VBUS is controlled by GPIO2[4], over-current is reported on GPIO6[13].
 */
#define ON_BD_USB_DRV	GPIO_TO_PIN(2, 4)
#define ON_BD_USB_OVC	GPIO_TO_PIN(6, 13)

static const short da850_evm_usb11_pins[] = {
	DA850_GPIO2_4, DA850_GPIO6_13,
	-1
};

static irqreturn_t da850_evm_usb_ocic_irq(int, void *);

static struct da8xx_ohci_root_hub da850_evm_usb11_pdata = {
	.type			= GPIO_BASED,
	.method	= {
		.gpio_method = {
			.power_control_pin	= ON_BD_USB_DRV,
			.over_current_indicator = ON_BD_USB_OVC,
		},
	},
	.board_ocic_handler	= da850_evm_usb_ocic_irq,
};

static irqreturn_t da850_evm_usb_ocic_irq(int irq, void *handler)
{
	if (handler != NULL)
		((da8xx_ocic_handler_t)handler)(&da850_evm_usb11_pdata, 1);
	return IRQ_HANDLED;
}

static __init void da850_evm_usb_init(void)
{
	u32 cfgchip2;
	int ret;

	/*
	 * Set up USB clock/mode in the CFGCHIP2 register.
	 * FYI:  CFGCHIP2 is 0x0000ef00 initially.
	 */
	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/* USB2.0 PHY reference clock is 24 MHz */
	cfgchip2 &= ~CFGCHIP2_REFFREQ;
	cfgchip2 |=  CFGCHIP2_REFFREQ_24MHZ;

	/*
	 * Select internal reference clock for USB 2.0 PHY
	 * and use it as a clock source for USB 1.1 PHY
	 * (this is the default setting anyway).
	 */
	cfgchip2 &= ~CFGCHIP2_USB1PHYCLKMUX;
	cfgchip2 |=  CFGCHIP2_USB2PHYCLKMUX;
	/*
	 * We have to override VBUS/ID signals when MUSB is configured into the
	 * host-only mode -- ID pin will float if no cable is connected, so the
	 * controller won't be able to drive VBUS thinking that it's a B-device.
	 * Otherwise, we want to use the OTG mode and enable VBUS comparators.
	 */
	cfgchip2 &= ~CFGCHIP2_OTGMODE;
	cfgchip2 |=  CFGCHIP2_SESENDEN | CFGCHIP2_VBDTCTEN;

	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	/*
	 * TPS2065 switch @ 5V supplies 1 A (sustains 1.5 A),
	 * with the power on to power good time of 3 ms.
	 */
	ret = da8xx_register_usb20(1000, 3);
	if (ret)
		pr_warning("%s: USB 2.0 registration failed: %d\n",
			   __func__, ret);

	/* initilaize usb module */
	da8xx_board_usb_init(da850_evm_usb11_pins, &da850_evm_usb11_pdata);
}

static struct i2c_gpio_platform_data da850_gpio_i2c_pdata = {
	.sda_pin	= GPIO_TO_PIN(1, 4),
	.scl_pin	= GPIO_TO_PIN(1, 5),
	.udelay		= 2,			/* 250 KHz */
};

static struct platform_device da850_gpio_i2c = {
	.name		= "i2c-gpio",
	.id		= 1,
	.dev		= {
		.platform_data	= &da850_gpio_i2c_pdata,
	},
};

#ifdef CONFIG_CPU_FREQ
static __init int da850_evm_init_cpufreq(void)
{
	/*
	 * Always set da850-sdi maximum speed to
	 * CONFIG_DA850_SDI_MAX_SPEED and removed "system_rev" check
	 * logic which was used previously to set "da850_max_speed".
	 */
	da850_max_speed = CONFIG_DA850_SDI_MAX_SPEED;
	return da850_register_cpufreq("pll0_sysclk3");
}
#else
static __init int da850_evm_init_cpufreq(void) { return 0; }
#endif

/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM, hence
 * they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans	= da850_dma0_rsv_chans,
	.rsv_slots	= da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans	= da850_dma1_rsv_chans,
	.rsv_slots	= da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};

static struct platform_device davinci_pcm_device = {
	        .name   = "davinci-pcm-audio",
		        .id     = -1,
};

#define DA850EVM_SATA_REFCLKPN_RATE	(100 * 1000 * 1000)

static __init void da850_evm_init(void)
{
	int ret;
	char mask = 0;
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	u8 rmii_en = soc_info->emac_pdata->rmii_en;
	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warning("da850_evm_init: edma registration failed: %d\n",
				ret);

	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret)
		pr_warning("da850_evm_init: i2c0 mux setup failed: %d\n",
				ret);

	platform_device_register(&da850_gpio_i2c);

	ret = da8xx_register_watchdog();
	if (ret)
		pr_warning("da830_evm_init: watchdog registration failed: %d\n",
				ret);

	if (HAS_MMC) {
		ret = davinci_cfg_reg_list(da850_evm_mmcsd0_pins);
		if (ret)
			pr_warning("da850_evm_init: mmcsd0 mux setup failed:"
					" %d\n", ret);

		ret = gpio_request(DA850_MMCSD_CD_PIN, "MMC CD\n");
		if (ret)
			pr_warning("da850_evm_init: can not open GPIO %d\n",
					DA850_MMCSD_CD_PIN);
		gpio_direction_input(DA850_MMCSD_CD_PIN);

		ret = gpio_request(DA850_MMCSD_WP_PIN, "MMC WP\n");
		if (ret)
			pr_warning("da850_evm_init: can not open GPIO %d\n",
					DA850_MMCSD_WP_PIN);
		gpio_direction_input(DA850_MMCSD_WP_PIN);

#ifdef CONFIG_DA850_USE_MMC1
		ret = davinci_cfg_reg_list(da850_mmcsd1_pins);
		if (ret)
			pr_warning("da850_evm_init: mmcsd1 mux setup failed:"
					"%d\n", ret);
		ret = da850_register_mmcsd1(da850_mmc_config);
		if (ret)
			pr_warning("da850_evm_init: mmcsd1"
				" registration failed: %d",  ret);
#endif
		ret = da8xx_register_mmcsd0(da850_mmc_config);
		if (ret)
			pr_warning("da850_evm_init: mmcsd0"
				" registration failed: %d",  ret);
	}

#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = gpio_request(DA850_BT_EN, "WL1271_BT_EN");
	if (ret)
		pr_warning("da850_evm_init: can not open BT GPIO %d\n",
					DA850_BT_EN);
	gpio_direction_output(DA850_BT_EN, 1);
	udelay(1000);
	gpio_direction_output(DA850_BT_EN, 0);
	wifi_power_init();
#endif

	davinci_serial_init(&da850_evm_uart_config);

	i2c_add_driver(&cdce_driver);

	i2c_add_driver(&tfp_driver);

	i2c_register_board_info(1, da850_evm_i2c_devices,
			ARRAY_SIZE(da850_evm_i2c_devices));

	/*
	 * shut down uart 0 and 1; they are not used on the board and
	 * accessing them causes endless "too much work in irq53" messages
	 * with arago fs
	 */
	__raw_writel(0, IO_ADDRESS(DA8XX_UART1_BASE) + 0x30);
	__raw_writel(0, IO_ADDRESS(DA8XX_UART0_BASE) + 0x30);

	// HACK
	ret = davinci_cfg_reg_list(da850_mcbsp1_pins);
	if (ret)
	    pr_warning("da850_evm_init: mcbsp1 mux setup failed:"
		       " %d\n", ret);
	
	platform_device_register(&davinci_pcm_device);

	da850_asp_device.dev.platform_data = &da850_sdi_snd_data;

	platform_device_register(&da850_asp_device);

	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warning("da850_evm_init: lcdcntl mux setup failed: %d\n",
				ret);

	/* Handle board specific muxing for LCD here */
	ret = davinci_cfg_reg_list(da850_evm_lcdc_pins);
	if (ret)
		pr_warning("da850_evm_init: evm specific lcd mux setup "
				"failed: %d\n",	ret);

	ret = da850_lcd_hw_init();
	if (ret)
		pr_warning("da850_evm_init: lcd initialization failed: %d\n",
				ret);

#ifdef CONFIG_DA850_SDI_LCDC
	sharp_lk043t1dg01_pdata.panel_power_ctrl = da850_panel_power_ctrl,
	ret = da8xx_register_lcdc(&sharp_lk043t1dg01_pdata);
	if (ret)
		pr_warning("da850_evm_init: lcdc registration failed: %d\n",
				ret);
#endif
#ifdef CONFIG_DA850_SDI_DVI_VGA
	ret = da8xx_register_lcdc(&ti_dvi_vga_pdata);
	if (ret)
		pr_warning("da850_evm_init: lcdc registration failed: %d\n",
				ret);
#endif
#ifdef CONFIG_DA850_SDI_DVI_480P
	ret = da8xx_register_lcdc(&ti_dvi_480p_pdata);
	if (ret)
		pr_warning("da850_evm_init: lcdc registration failed: %d\n",
				ret);
#endif
#ifdef CONFIG_DA850_SDI_DVI_WVGA
	ret = da8xx_register_lcdc(&ti_dvi_wvga_pdata);
	if (ret)
		pr_warning("da850_evm_init: lcdc registration failed: %d\n",
				ret);
#endif

	ret = da8xx_register_rtc();
	if (ret)
		pr_warning("da850_evm_init: rtc setup failed: %d\n", ret);

	ret = da850_evm_init_cpufreq();
	if (ret)
		pr_warning("da850_evm_init: cpufreq registration failed: %d\n",
				ret);

	ret = da8xx_register_cpuidle();
	if (ret)
		pr_warning("da850_evm_init: cpuidle registration failed: %d\n",
				ret);

	ret = da850_register_pm(&da850_pm_device);
	if (ret)
		pr_warning("da850_evm_init: suspend registration failed: %d\n",
				ret);

	ret = da8xx_register_spi(1, da850evm_spi_info,
				 ARRAY_SIZE(da850evm_spi_info));
	if (ret)
		pr_warning("da850_evm_init: spi 1 registration failed: %d\n",
				ret);

	da850_evm_usb_init();

	ret = da850_register_sata(DA850EVM_SATA_REFCLKPN_RATE);
	if (ret)
		pr_warning("da850_evm_init: sata registration failed: %d\n",
				ret);

	if (HAS_VPIF_DISPLAY || HAS_VPIF_CAPTURE) {
		ret = da850_register_vpif();
		if (ret)
			pr_warning("da850_evm_init: VPIF registration failed: "
					"%d\n",	ret);
	}

	if (!HAS_RMII && HAS_VPIF_CAPTURE) {
		ret = davinci_cfg_reg_list(da850_vpif_capture_pins);
		if (ret)
			pr_warning("da850_evm_init: vpif capture mux failed: "
					"%d\n",	ret);

		ret = da850_register_vpif_capture(&da850_vpif_capture_config);
		if (ret)
			pr_warning("da850_evm_init: VPIF registration failed: "
					"%d\n",	ret);

	}

	if (!HAS_GLCD && HAS_VPIF_DISPLAY) {
		ret = davinci_cfg_reg_list(da850_vpif_display_pins);
		if (ret)
			pr_warning("da850_evm_init: vpif capture mux failed: "
					"%d\n",	ret);

		ret = da850_register_vpif_display(&da850_vpif_display_config);
		if (ret)
			pr_warning("da850_evm_init: VPIF registration failed: "
					"%d\n",	ret);

	}

	if (HAS_EHRPWM) {
		if (rmii_en) {
			ret = davinci_cfg_reg_list(da850_ehrpwm0_pins);
			if (ret)
				pr_warning("da850_evm_init:"
				" ehrpwm0 mux setup failed: %d\n", ret);
			else
				mask = BIT(0) | BIT(1);
		} else {
			pr_warning("da850_evm_init:"
			" eHRPWM module 0 cannot be used"
			" since it is being used by MII interface\n");
			mask = 0;
		}

		if (!HAS_LCD) {
			ret = davinci_cfg_reg_list(da850_ehrpwm1_pins);
			if (ret)
				pr_warning("da850_evm_init:"
				" eHRPWM module1 output A mux"
				" setup failed %d\n", ret);
			else
				mask = mask | BIT(2);
		} else {
			pr_warning("da850_evm_init:"
				" eHRPWM module1 outputA cannot be"
				" used since it is being used by LCD\n");
		}

		if (!HAS_SPI) {
			ret = davinci_cfg_reg(DA850_EHRPWM1_B);
			if (ret)
				pr_warning("da850_evm_init:"
					" eHRPWM module1 outputB mux"
					" setup failed %d\n", ret);
		else
			mask =  mask  | BIT(3);
		} else {
			pr_warning("da850_evm_init:"
				" eHRPWM module1 outputB cannot be"
				" used since it is being used by spi1\n");
		}

		da850_register_ehrpwm(mask);
	}

	if (HAS_ECAP_PWM) {
		ret = davinci_cfg_reg(DA850_ECAP2_APWM2);
		if (ret)
			pr_warning("da850_evm_init:ecap mux failed:"
					" %d\n", ret);
		ret = da850_register_ecap(2);
		if (ret)
			pr_warning("da850_evm_init:"
				" eCAP registration failed: %d\n", ret);
	}

	if (HAS_BACKLIGHT) {
		ret = da850_register_backlight(&da850evm_backlight,
				&da850evm_backlight_data);
		if (ret)
			pr_warning("da850_evm_init:"
				" backlight device registration"
				" failed: %d\n", ret);
	}

	if (HAS_ECAP_CAP) {
		if (HAS_MCASP)
			pr_warning("da850_evm_init:"
				"ecap module 1 cannot be used "
				"since it shares pins with McASP\n");
		else {
			ret = davinci_cfg_reg(DA850_ECAP1_APWM1);
			if (ret)
				pr_warning("da850_evm_init:ecap mux failed:%d\n"
						, ret);
			else {
				ret = da850_register_ecap_cap(1);
				if (ret)
					pr_warning("da850_evm_init"
					"eCAP registration failed: %d\n", ret);
			}
		}
	}
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init da850_evm_console_init(void)
{
	return add_preferred_console("ttyS", 2, "115200");
}
console_initcall(da850_evm_console_init);
#endif

static void __init da850_evm_map_io(void)
{
	da850_init();
}

MACHINE_START(DAVINCI_DA850_SDI, "DA850 SDI Development Board")
	.atag_offset	= 0x100,
	.map_io		= da850_evm_map_io,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= da850_evm_init,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
MACHINE_END
