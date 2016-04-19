/*
 * TI DaVinci AEMIF support
 *
 * Copyright 2010 (C) Texas Instruments, Inc. http://www.ti.com/
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef _MACH_DAVINCI_AEMIF_H
#define _MACH_DAVINCI_AEMIF_H

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>

#define DA8XX_AEMIF_CE2CFG_OFFSET	0x10
#define DA8XX_AEMIF_CE3CFG_OFFSET	0x14
#define DA8XX_AEMIF_CE4CFG_OFFSET	0x18
#define DA8XX_AEMIF_CE5CFG_OFFSET	0x1c

#define DA8XX_AEMIF_ASIZE_MASK		0x3
#define DA8XX_AEMIF_ASIZE_16BIT		0x1
#define DA8XX_AEMIF_ASIZE_8BIT		0x0

#define NRCSR_OFFSET		0x00
#define AWCCR_OFFSET		0x04
#define A1CR_OFFSET		0x10

#define ACR_ASIZE_MASK		0x3
#define ACR_EW_MASK		BIT(30)
#define ACR_SS_MASK		BIT(31)

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

enum davinci_emif_cells {
	DAVINCI_NAND_DEVICE_CELL,
	DAVINCI_NOR_FLASH_CELL,
};

struct davinci_aemif_devices {
	struct platform_device *devices;
	unsigned int num_devices;
};

/* All timings in nanoseconds */
struct davinci_aemif_timing {
	u8	wsetup;
	u8	wstrobe;
	u8	whold;

	u8	rsetup;
	u8	rstrobe;
	u8	rhold;

	u8	ta;
};

int davinci_aemif_setup_timing(struct davinci_aemif_timing *t,
					void __iomem *base, unsigned cs);
#endif
