/*
 * TI's OMAP-L138 LCDK Development Board
 *
 * Initial code: Syed Mohammed Khasim
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/leds.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mfd/davinci_aemif.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>
#include <mach/nand.h>

#define LCDKBOARD_PHY_ID		"davinci_mdio-0:07"
#define DA850_LCDK_MMCSD_CD_PIN		GPIO_TO_PIN(4, 0)

#define DA850_USB1_VBUS_PIN		GPIO_TO_PIN(2, 4)
#define DA850_USB1_OC_PIN		GPIO_TO_PIN(6, 13)

#define LCDKBOARD_SATA_REFCLKPN_RATE   (100 * 1000 * 1000)

#define DA850_LCDK_USER_LED0		GPIO_TO_PIN(6, 12)
#define DA850_LCDK_USER_LED1		GPIO_TO_PIN(6, 13)
#define DA850_LCDK_USER_LED2		GPIO_TO_PIN(2, 12)
#define DA850_LCDK_USER_LED3		GPIO_TO_PIN(0, 9)

#define	DA850_LCDK_USER_KEY0            GPIO_TO_PIN(2, 4)
#define	DA850_LCDK_USER_KEY1            GPIO_TO_PIN(2, 5)

#if defined(CONFIG_USB_OHCI_HCD)
#define HAS_OHCI               1
#else
#define HAS_OHCI               0
#endif

#define        DA850_LCDK_KEYS_DEBOUNCE_MS     10
#define        DA850_LCDK_GPIO_KEYS_POLL_MS    200

#define DA850_LCD_PWR_PIN              GPIO_TO_PIN(2, 8)
#define DA850_LCD_BL_PIN               GPIO_TO_PIN(2, 15)

#if defined(CONFIG_MTD_NAND_DAVINCI) || \
	defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
struct mtd_partition omapl138_lcdk_nandflash_partition[] = {
	{
		.name           = "u-boot env",
		.offset         = 0,
		.size           = SZ_128K,
		.mask_flags     = MTD_WRITEABLE,
	},
	{
		.name           = "u-boot",
		.offset         = MTDPART_OFS_APPEND,
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

static struct davinci_nand_pdata omapl138_lcdk_nandflash_data = {
	.parts          = omapl138_lcdk_nandflash_partition,
	.nr_parts       = ARRAY_SIZE(omapl138_lcdk_nandflash_partition),
	.ecc_mode       = NAND_ECC_HW,
	.options        = NAND_BUSWIDTH_16,
	.ecc_bits       = 1, /* 4 bit mode is not
			      * supported with 16 bit NAND */
	.bbt_options    = NAND_BBT_USE_FLASH,
};

static const short omapl138_lcdk_nand_pins[] = {
	DA850_EMA_D_0, DA850_EMA_D_1, DA850_EMA_D_2, DA850_EMA_D_3,
	DA850_EMA_D_4, DA850_EMA_D_5, DA850_EMA_D_6, DA850_EMA_D_7,
	DA850_EMA_D_8, DA850_EMA_D_9, DA850_EMA_D_10, DA850_EMA_D_11,
	DA850_EMA_D_12, DA850_EMA_D_13, DA850_EMA_D_14, DA850_EMA_D_15,
	DA850_EMA_A_1, DA850_EMA_A_2, DA850_NEMA_CS_3, DA850_NEMA_CS_4,
	DA850_NEMA_WE, DA850_NEMA_OE,
	-1
};

static struct resource omapl138_lcdk_nandflash_resource[] = {
	{
		.start  = DA8XX_AEMIF_CS3_BASE,
		.end    = DA8XX_AEMIF_CS3_BASE + SZ_512K + 2 * SZ_1K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = DA8XX_AEMIF_CTL_BASE,
		.end    = DA8XX_AEMIF_CTL_BASE + SZ_32K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device omapl138_lcdk_nandflash_device = {
	.name           = "davinci_nand",
	.id             = 1,
	.dev            = {
		.platform_data  = &omapl138_lcdk_nandflash_data,
	},
	.resource       = omapl138_lcdk_nandflash_resource,
	.num_resources  = ARRAY_SIZE(omapl138_lcdk_nandflash_resource),
};

static __init void omapl138_lcdk_nand_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(omapl138_lcdk_nand_pins);
	if (ret) {
		pr_warn("da850_lcdk_init:"
				"nand mux setup failed: %d\n", ret);
		return;
	}
	platform_device_register(&omapl138_lcdk_nandflash_device);
}
#else
static void omapl138_lcdk_nand_init(void) { }
#endif


static short omapl138_lcdk_mii_pins[] __initdata = {
	DA850_MII_TXEN, DA850_MII_TXCLK, DA850_MII_COL, DA850_MII_TXD_3,
	DA850_MII_TXD_2, DA850_MII_TXD_1, DA850_MII_TXD_0, DA850_MII_RXER,
	DA850_MII_CRS, DA850_MII_RXCLK, DA850_MII_RXDV, DA850_MII_RXD_3,
	DA850_MII_RXD_2, DA850_MII_RXD_1, DA850_MII_RXD_0, DA850_MDIO_CLK,
	DA850_MDIO_D,
	-1
};

static __init void omapl138_lcdk_config_emac(void)
{
	void __iomem *cfgchip3 = DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP3_REG);
	int ret;
	u32 val;
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	val = __raw_readl(cfgchip3);
	val &= ~BIT(8);
	ret = davinci_cfg_reg_list(omapl138_lcdk_mii_pins);
	if (ret) {
		pr_warning("%s: cpgmac/mii mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	/* configure the CFGCHIP3 register for MII */
	__raw_writel(val, cfgchip3);
	pr_info("EMAC: MII PHY configured\n");

	soc_info->emac_pdata->phy_id = LCDKBOARD_PHY_ID;

	ret = da8xx_register_emac();
	if (ret)
		pr_warning("%s: emac registration failed: %d\n",
			__func__, ret);
}

/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 LCDK EVM,
 * hence they are being reserved for codecs on the DSP side.
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

static const short lcdk_mmcsd0_pins[] = {
	DA850_MMCSD0_DAT_0, DA850_MMCSD0_DAT_1, DA850_MMCSD0_DAT_2,
	DA850_MMCSD0_DAT_3, DA850_MMCSD0_CLK, DA850_MMCSD0_CMD,
	DA850_GPIO3_12, DA850_GPIO3_13,
	-1
};

static int da850_lcdk_mmc_get_ro(int index)
{
	return 0;
}

static int da850_lcdk_mmc_get_cd(int index)
{
	return !gpio_get_value(DA850_LCDK_MMCSD_CD_PIN);
}

static struct davinci_mmc_config da850_mmc_config = {
	.get_ro		= da850_lcdk_mmc_get_ro,
	.get_cd		= da850_lcdk_mmc_get_cd,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static __init void omapl138_lcdk_mmc_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(lcdk_mmcsd0_pins);
	if (ret) {
		pr_warning("%s: MMC/SD0 mux setup failed: %d\n",
			__func__, ret);
		return;
	}

	ret = gpio_request_one(DA850_LCDK_MMCSD_CD_PIN,
			GPIOF_DIR_IN, "MMC CD");
	if (ret < 0) {
		pr_warning("%s: can not open GPIO %d\n",
			__func__, DA850_LCDK_MMCSD_CD_PIN);
		return;
	}

	ret = da8xx_register_mmcsd0(&da850_mmc_config);
	if (ret) {
		pr_warning("%s: MMC/SD0 registration failed: %d\n",
			__func__, ret);
		goto mmc_setup_mmcsd_fail;
	}

	return;

mmc_setup_mmcsd_fail:
	gpio_free(DA850_LCDK_MMCSD_CD_PIN);
}

static irqreturn_t omapl138_lcdk_usb_ocic_irq(int irq, void *dev_id);

static const short da850_lcdk_usb11_pins[] = {
	DA850_GPIO2_4, DA850_GPIO6_13,
	-1
};

static struct da8xx_ohci_root_hub omapl138_lcdk_usb11_pdata = {
	.type	= GPIO_BASED,
	.method	= {
		.gpio_method = {
			.power_control_pin	= DA850_USB1_VBUS_PIN,
			.over_current_indicator = DA850_USB1_OC_PIN,
		},
	},
	.board_ocic_handler	= omapl138_lcdk_usb_ocic_irq,
};

static irqreturn_t omapl138_lcdk_usb_ocic_irq(int irq, void *handler)
{
	if (handler != NULL)
		((da8xx_ocic_handler_t)handler)(&omapl138_lcdk_usb11_pdata, 1);
	return IRQ_HANDLED;
}

/* VGA */
static const short omapl138_lcdk_lcdc_pins[] = {
	DA850_GPIO2_8, DA850_GPIO2_15,
	-1
};

/* Backlight and power is for use with LCD expansion header only */
static void da850_panel_power_ctrl(int val)
{
	/* lcd backlight */
	gpio_set_value(DA850_LCD_BL_PIN, val);
	/* lcd power */
	gpio_set_value(DA850_LCD_PWR_PIN, val);
}

static int da850_lcd_hw_init(void)
{
	void __iomem *cfg_mstpri1_base;
	void __iomem *cfg_mstpri2_base;
	void __iomem *emifb;
	void __iomem *myptr;
	int status;
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
#define DA8XX_EMIF30_CONTROL_BASE               0xB0000000
#define DA8XX_EMIF30_BPRIO_OFFSET               0x20
#define DA8XX_EMIFB_VIRT(x)     (emifb + (x))
	emifb = ioremap(DA8XX_EMIF30_CONTROL_BASE, SZ_4K);
	myptr = DA8XX_EMIFB_VIRT(0x20);
	__raw_writel(0x20, myptr);

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

	/* Switch off panel power and backlight */
	da850_panel_power_ctrl(0);

	/* Switch on panel power and backlight */
	da850_panel_power_ctrl(1);

	return 0;
}

static void omapl138_lcdk_display_init(void)
{
	int ret;

	ret = davinci_cfg_reg_list(da850_lcdcntl_pins);
	if (ret)
		pr_warn("omapl138_lcdk_init: lcdcntl mux setup failed:%d\n",
				ret);

	ret = davinci_cfg_reg_list(omapl138_lcdk_lcdc_pins);
	if (ret)
		pr_warn("omapl138_lcdk_init: evm specific lcd mux setup "
				"failed: %d\n", ret);

	da850_lcd_hw_init();

	ret = da8xx_register_lcdc(&vga_monitor_pdata);
	if (ret)
		pr_warn("omapl138_lcdk_init: lcdc registration failed: %d\n",
				ret);
}

static __init void omapl138_lcdk_usb_init(void)
{
	u32 cfgchip2;
	int ret;

	/* Setup the Ref. clock frequency for the LCDK at 24 MHz. */

	cfgchip2 = __raw_readl(DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));
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

	/*
	 * TPS2065 switch @ 5V supplies 1 A (sustains 1.5 A),
	 * with the power on to power good time of 3 ms.
	 */
	ret = da8xx_register_usb20(1000, 3);
	if (ret)
		pr_warn("%s: USB 2.0 registration failed: %d\n",
				__func__, ret);

	__raw_writel(cfgchip2, DA8XX_SYSCFG0_VIRT(DA8XX_CFGCHIP2_REG));

	if (HAS_OHCI)
		da8xx_board_usb_init(da850_lcdk_usb11_pins,
				     &omapl138_lcdk_usb11_pdata);

	return;
}

static struct davinci_uart_config omapl138_lcdk_uart_config __initdata = {
	.enabled_uarts = 0x7,
};

/* I2C */
static struct i2c_board_info __initdata omapl138_lcdk_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
};

static struct i2c_gpio_platform_data da850_gpio_i2c_pdata = {
	.sda_pin        = GPIO_TO_PIN(1, 4),
	.scl_pin        = GPIO_TO_PIN(1, 5),
	.udelay         = 2,                    /* 250 KHz */
};

static struct platform_device da850_gpio_i2c = {
	.name           = "i2c-gpio",
	.id             = 1,
	.dev            = {
		.platform_data  = &da850_gpio_i2c_pdata,
	},
};

static void omapl138_lcdk_i2c_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(da850_i2c0_pins);
	if (ret)
		pr_warn("omapl138_lcdk_init: i2c0 mux setup failed: %d\n",
				ret);

	platform_device_register(&da850_gpio_i2c);

	if (ret)
		pr_warn("omapl138_lcdk_init: i2c0 registration failed: %d\n",
				ret);
	i2c_register_board_info(1, omapl138_lcdk_i2c_devices,
			ARRAY_SIZE(omapl138_lcdk_i2c_devices));
}

/* Set up OMAP-L138 LCDK low-level McASP driver */
static u8 da850_iis_serializer_direction[] = {
	INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
	INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
	INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,  INACTIVE_MODE,
	INACTIVE_MODE,  TX_MODE,        RX_MODE,        INACTIVE_MODE,
};

static struct snd_platform_data omapl138_lcdk_snd_data = {
	.tx_dma_offset  = 0x2000,
	.rx_dma_offset  = 0x2000,
	.op_mode        = DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(da850_iis_serializer_direction),
	.tdm_slots      = 2,
	.serial_dir     = da850_iis_serializer_direction,
	.asp_chan_q     = EVENTQ_0,
	.version        = MCASP_VERSION_2,
	.txnumevt       = 1,
	.rxnumevt       = 1,
};

static const short omapl138_lcdk_mcasp_pins[] __initconst = {
	DA850_AHCLKX,   DA850_ACLKX,    DA850_AFSX,
	DA850_AHCLKR,   DA850_ACLKR,    DA850_AFSR,     DA850_AMUTE,
	DA850_AXR_13,   DA850_AXR_14,
	-1
};

static void omapl138_lcdk_sound_init(void)
{
	int ret;
	ret = davinci_cfg_reg_list(omapl138_lcdk_mcasp_pins);
	if (ret)
		pr_warn("omapl138_lcdk_init: mcasp mux setup failed: %d\n",
				ret);

	da8xx_register_mcasp(0, &omapl138_lcdk_snd_data);
}

static const short omapl138_lcdk_led_pin_mux[] __initconst = {
	DA850_GPIO6_12,
#if !HAS_OHCI
	DA850_GPIO6_13,
#endif
	DA850_GPIO2_12,
	DA850_GPIO0_9,
	-1
};

static struct gpio_led gpio_leds[] = {
	{
		.active_low = 1,
		.gpio = DA850_LCDK_USER_LED0,
		.name = "user_led0",
	},
#if !HAS_OHCI
	{
		.active_low = 1,
		.gpio   = DA850_LCDK_USER_LED1,
		.name   = "user_led1",
	},
#endif
	{
		.active_low = 1,
		.gpio = DA850_LCDK_USER_LED2,
		.name = "user_led2",
	},
	{
		.active_low = 1,
		.gpio   = DA850_LCDK_USER_LED3,
		.name   = "user_led3",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds           = gpio_leds,
	.num_leds       = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name   = "leds-gpio",
	.id     = -1,
	.dev    = {
		.platform_data  = &gpio_led_info,
	},
};

static  __init void omapl138_lcdk_led_init(void)
{
	int err;

	davinci_cfg_reg_list(omapl138_lcdk_led_pin_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register leds_gpio device\n");
	return;
};

static const short omapl138_lcdk_keys_pin_mux[] __initconst = {
#if !HAS_OHCI
	DA850_GPIO2_4,
#endif
	DA850_GPIO2_5,
	-1
};

static struct gpio_keys_button omapl138_lcdk_evm_keys[] = {
#if !HAS_OHCI
	{
		.type                   = EV_KEY,
		.active_low             = 1,
		.wakeup                 = 0,
		.debounce_interval      = DA850_LCDK_KEYS_DEBOUNCE_MS,
		.code                   = KEY_F8,
		.gpio                   = DA850_LCDK_USER_KEY0,
		.desc                   = "pb1",
	},
#endif
	{
		.type                   = EV_KEY,
		.active_low             = 1,
		.wakeup                 = 0,
		.debounce_interval      = DA850_LCDK_KEYS_DEBOUNCE_MS,
		.code                   = KEY_F7,
		.gpio                   = DA850_LCDK_USER_KEY1,
		.desc                   = "pb2",
	},
};

static struct gpio_keys_platform_data omapl138_lcdk_evm_keys_pdata = {
	.buttons = omapl138_lcdk_evm_keys,
	.nbuttons = ARRAY_SIZE(omapl138_lcdk_evm_keys),
	.poll_interval = DA850_LCDK_GPIO_KEYS_POLL_MS,
};

static struct platform_device omapl138_lcdk_evm_keys_device = {
	.name = "gpio-keys-polled",
	.id = 0,
	.dev = {
		.platform_data = &omapl138_lcdk_evm_keys_pdata
	},
};

static  __init void omapl138_lcdk_keys_init(void)
{
	int err;

	davinci_cfg_reg_list(omapl138_lcdk_keys_pin_mux);
	err = platform_device_register(&omapl138_lcdk_evm_keys_device);
	if (err)
		pr_err("failed to register omapl138_lcdk_evm_keys_device\n");
	return;
};

static struct davinci_pm_config da850_pm_pdata = {
	.sleepcount = 128,
};

static struct platform_device da850_pm_device = {
	.name           = "pm-davinci",
	.dev = {
		.platform_data  = &da850_pm_pdata,
	},
	.id             = -1,
};

static __init void omapl138_lcdk_init(void)
{
	int ret;

	davinci_serial_init(&omapl138_lcdk_uart_config);

	/*
	 * shut down uart 0 and 1; they are not used on this board and
	 * accessing them causes endless "too much work in irq53" messages
	 * with arago fs
	 */
	__raw_writel(0, IO_ADDRESS(DA8XX_UART1_BASE) + 0x30);
	__raw_writel(0, IO_ADDRESS(DA8XX_UART0_BASE) + 0x30);

	omapl138_lcdk_config_emac();

	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warning("%s: EDMA registration failed: %d\n",
			__func__, ret);

	omapl138_lcdk_mmc_init();

	omapl138_lcdk_usb_init();

	ret = da8xx_register_watchdog();
	if (ret)
		pr_warning("omapl138_lcdk_init: "
			"watchdog registration failed: %d\n",
			ret);

	ret = da8xx_register_rtc();
	if (ret)
		pr_warning("omapl138_lcdk_init: rtc setup failed: %d\n", ret);

	omapl138_lcdk_i2c_init();
	omapl138_lcdk_sound_init();

	ret = da850_register_sata(LCDKBOARD_SATA_REFCLKPN_RATE);
	if (ret)
		pr_warning("omapl138_lcdk_init: sata registration failed: %d\n",
				ret);

	ret = da850_register_pm(&da850_pm_device);
	if (ret)
		pr_warning("da850_evm_init: suspend registration failed: %d\n",
				ret);

	omapl138_lcdk_led_init();
	omapl138_lcdk_keys_init();
	omapl138_lcdk_nand_init();
	omapl138_lcdk_display_init();
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init omapl138_lcdk_console_init(void)
{
	if (!machine_is_omapl138_lcdkboard())
		return 0;

	return add_preferred_console("ttyS", 2, "115200");
}
console_initcall(omapl138_lcdk_console_init);
#endif

static void __init omapl138_lcdk_map_io(void)
{
	da850_init();
}

MACHINE_START(OMAPL138_LCDKBOARD, "AM18x/OMAP-L138 lcdk board")
	.atag_offset	= 0x100,
	.map_io		= omapl138_lcdk_map_io,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= omapl138_lcdk_init,
	.dma_zone_size	= SZ_128M,
	.restart	= da8xx_restart,
MACHINE_END
