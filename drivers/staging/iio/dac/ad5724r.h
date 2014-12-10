/*
 * AD5724R SPI DAC driver
 *
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef SPI_AD5724R_H_
#define SPI_AD5724R_H_

#define AD5724R_DAC_CHANNELS		4

/* Register Select */
#define AD5724R_REG_DAC		0x0
#define AD5724R_REG_OUTPUT_RANGE	0x1
#define AD5724R_REG_PWR_CTL		0x2
#define AD5724R_REG_CONTROL		0x3

/* Channel Select */
#define DAC_CHANNEL_A			0x0
#define DAC_CHANNEL_B			0x1
#define DAC_CHANNEL_C			0x2
#define DAC_CHANNEL_D			0x3
#define DAC_CHANNEL_ALL		0x4

/* Output Range Config*/
/*
Range1: 0~5V
Range2: 0~10V
Range3: 0~10.8V
Range4: -5~+5V
Range5: -10~+10V
Range6: -10.8~+10.8V
*/

#define RANGE1_SELECT_0_5V		0x0
#define RANGE2_SELECT_0_10V		0x1
#define RANGE3_SELECT_0_10V8		0x2
#define RANGE4_SELECT_D5V		0x3
#define RANGE5_SELECT_D10V		0x4
#define RANGE6_SELECT_D10V8		0x5

/* Power Control */
#define POWERUP_ALL			0x000F

#define POWERUP_CHANNEL_A		0x0001
#define POWERDOWN_CHANNEL_A		0xFFFE

#define POWERUP_CHANNEL_B		0x0002
#define POWERDOWN_CHANNEL_B		0xFFFD

#define POWERUP_CHANNEL_C		0x0004
#define POWERDOWN_CHANNEL_C		0xFFFB

#define POWERUP_CHANNEL_D		0x0008
#define POWERDOWN_CHANNEL_D		0xFFF7

#define POWERUP_REF			0x0010
#define POWERDOWN_REF			0xFFEF

/* General Config */
#define NOP_READBACK			0x180000	/* for readback */

#define DAC_VOL(x, range) ((unsigned short)(((x) * 65535) / range))

/**
 * struct ad5724r_chip_info - chip specific information
 * @channels:		channel spec for the DAC
 * @int_vref_mv:	AD5724/34/54: the internal reference voltage
 */

struct ad5724r_chip_info {
	const struct iio_chan_spec	*channels;
	u16				int_vref_mv;
};

/**
 * struct ad5724_state - driver instance specific data
 * @indio_dev:		the industrial I/O device
 * @us:			spi_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref_mv:		actual reference voltage used
 * @pwr__mode		power mode
 * @out_range_mode	current output range options
 */

struct ad5724r_state {
	struct spi_device		*us;
	const struct ad5724r_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned short			vref_mv;
	unsigned short			pwr__mode;
	unsigned short			out_range_mode;
};

/**
 * ad5724r_supported_device_ids:
 * The AD5724/44/64 parts are available in different
 * fixed internal reference voltage options.
 */

enum ad5724r_supported_device_ids {
	ID_AD5724R,
	ID_AD5734R,
	ID_AD5754R,
};

#endif /* SPI_AD5724R_H_ */
