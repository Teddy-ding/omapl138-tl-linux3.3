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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/cp_intc.h>
#include <mach/da8xx.h>
#include <mach/mux.h>

#define LCDKBOARD_PHY_ID		"davinci_mdio-0:07"
#define DA850_LCDK_MMCSD_CD_PIN		GPIO_TO_PIN(4, 0)

#define DA850_USB1_VBUS_PIN		GPIO_TO_PIN(2, 4)
#define DA850_USB1_OC_PIN		GPIO_TO_PIN(6, 13)

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

	da8xx_board_usb_init(da850_lcdk_usb11_pins, &omapl138_lcdk_usb11_pdata);

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
