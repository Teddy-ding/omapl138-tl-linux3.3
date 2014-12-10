/*
 * AD5724R, AD5734R, AD5754R Digital to analog convertors spi driver
 *
 * Copyright 2010-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>

#include "../iio.h"
#include "../sysfs.h"
#include "dac.h"
#include "ad5724r.h"

#define AD5724R_CHANNEL(_chan, _bits) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = (_chan), \
	.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT, \
	.address = (_chan), \
	.scan_type = IIO_ST('u', (_bits), 16, 16 - (_bits)), \
}

#define DECLARE_AD5724R_CHANNELS(_name, _bits) \
	const struct iio_chan_spec _name##_channels[] = { \
		AD5724R_CHANNEL(0, _bits), \
		AD5724R_CHANNEL(1, _bits), \
		AD5724R_CHANNEL(2, _bits), \
		AD5724R_CHANNEL(3, _bits), \
}

static DECLARE_AD5724R_CHANNELS(ad5724r, 12);
static DECLARE_AD5724R_CHANNELS(ad5734r, 14);
static DECLARE_AD5724R_CHANNELS(ad5754r, 16);

static const struct ad5724r_chip_info ad5724r_chip_info_tbl[] = {
	[ID_AD5724R] = {
		.channels = ad5724r_channels,
		.int_vref_mv = 2500,
	},
	[ID_AD5734R] = {
		.channels = ad5734r_channels,
		.int_vref_mv = 2500,
	},
	[ID_AD5754R] = {
		.channels = ad5754r_channels,
		.int_vref_mv = 2500,
	},
};

static int ad5724r_spi_write(struct spi_device *spi,
			     u8 cmd, u8 addr, u16 val, u8 len)
{
	u32 data;
	u8 msg[3];

	/*
	 * The input shift register is 24 bits wide. The first two bits are
	 * don't care bits. The next three are the command bits, C2 to C0,
	 * followed by the 3-bit DAC address, A2 to A0, and then the
	 * 16-, 14-, 12-bit data-word. The data-word comprises the 16-,
	 * 14-, 12-bit input code followed by 0, 2, or 4 don't care bits,
	 * for the AD5724R, AD5734R, and AD5754R, respectively.
	 */
	data = (0 << 22) | (cmd << 19) | (addr << 16) | (val << (16 - len));
	msg[0] = data >> 16;
	msg[1] = data >> 8;
	msg[2] = data;

	return spi_write(spi, msg, 3);
}

static int ad5724r_spi_read(struct spi_device *spi,
				u8 cmd, u8 addr, unsigned short *rx_data)
{
	u32 data;
	u8 msg[3];
	int ret;

	/*
	 * The input shift register is 24 bits wide. The first two bits are
	 * don't care bits. The next three are the command bits, C2 to C0,
	 * followed by the 3-bit DAC address, A2 to A0, and then the
	 * 16-, 14-, 12-bit data-word. The data-word comprises the 16-,
	 * 14-, 12-bit input code followed by 0, 2, or 4 don't care bits,
	 * for the AD5724R, AD5734R, and AD5754R, respectively.
	 */
	data = (1 << 23) | (0 << 22) | (cmd << 19) | (addr << 16);
	msg[0] = data >> 16;
	msg[1] = data >> 8;
	msg[2] = data;
	ret = spi_write(spi, msg, 3);
	if (ret)
		goto out;

	ret = spi_read(spi, msg, 3);
	if (ret)
		goto out;

	*rx_data = (msg[1] << 8) | msg[2];
out:
	return ret;
}

static int ad5724r_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5724r_state *st = iio_priv(indio_dev);
	unsigned long scale_uv;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		scale_uv = (st->vref_mv * 1000) >> chan->scan_type.realbits;
		*val =  scale_uv / 1000;
		*val2 = (scale_uv % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;

	}
	return -EINVAL;
}

static int ad5724r_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ad5724r_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case 0:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		return ad5724r_spi_write(st->us, AD5724R_REG_DAC,
					chan->address, val,
					chan->scan_type.realbits);
	default:
		ret = -EINVAL;
	}

	return -EINVAL;
}

static ssize_t ad5724r_read_range_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5724r_state *st = iio_priv(indio_dev);

	char mode[][15] = {"0_5V", "0_10V", "0_10V8",
			"-5V_5V", "-10V_10V", "-10V8_10V8"};

	return sprintf(buf, "%s\n", mode[st->out_range_mode]);
}

static ssize_t ad5724r_write_range_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5724r_state *st = iio_priv(indio_dev);
	int ret;

	if (sysfs_streq(buf, "0_5V"))
		st->out_range_mode = RANGE1_SELECT_0_5V;
	else if (sysfs_streq(buf, "0_10V"))
		st->out_range_mode = RANGE2_SELECT_0_10V;
	else if (sysfs_streq(buf, "0_10V8"))
		st->out_range_mode = RANGE3_SELECT_0_10V8;
	else if (sysfs_streq(buf, "-5V_5V"))
		st->out_range_mode = RANGE4_SELECT_D5V;
	else if (sysfs_streq(buf, "-10V_10V"))
		st->out_range_mode = RANGE5_SELECT_D10V;
	else if (sysfs_streq(buf, "-10V8_10V8"))
		st->out_range_mode = RANGE6_SELECT_D10V8;
	else
		ret = -EINVAL;

	ret = ad5724r_spi_write(st->us, AD5724R_REG_OUTPUT_RANGE,
				DAC_CHANNEL_ALL, st->out_range_mode, 16);


	return ret ? ret : len;
}

static ssize_t ad5724r_read_dac_powerdown(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5724r_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	return sprintf(buf, "%d\n",
			!(st->pwr__mode & (1 << this_attr->address)));
}

static ssize_t ad5724r_write_dac_powerdown(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t len)
{
	long readin;
	int ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5724r_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	ret = strict_strtol(buf, 10, &readin);
	if (ret)
		return ret;

	if (readin == 1)
		st->pwr__mode &= ~(1 << this_attr->address);
	else if (!readin)
		st->pwr__mode |= (1 << this_attr->address);
	else
		ret = -EINVAL;

	ret = ad5724r_spi_write(st->us, AD5724R_REG_PWR_CTL, 0,
				st->pwr__mode, 16);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(out_voltage_range_mode, S_IRUGO |
			S_IWUSR, ad5724r_read_range_mode,
			ad5724r_write_range_mode, 0);

static IIO_CONST_ATTR(out_voltage_range_mode_available,
			"0_5V 0_10V 0_10V8 -5V_5V -10V_10V -10V8_10V8");

#define IIO_DEV_ATTR_DAC_POWERDOWN(_num, _show, _store, _addr)		\
	IIO_DEVICE_ATTR(out_voltage##_num##_powerdown,			\
			S_IRUGO | S_IWUSR, _show, _store, _addr)

static IIO_DEV_ATTR_DAC_POWERDOWN(0, ad5724r_read_dac_powerdown,
				   ad5724r_write_dac_powerdown, 0);
static IIO_DEV_ATTR_DAC_POWERDOWN(1, ad5724r_read_dac_powerdown,
				   ad5724r_write_dac_powerdown, 1);
static IIO_DEV_ATTR_DAC_POWERDOWN(2, ad5724r_read_dac_powerdown,
				   ad5724r_write_dac_powerdown, 2);
static IIO_DEV_ATTR_DAC_POWERDOWN(3, ad5724r_read_dac_powerdown,
				   ad5724r_write_dac_powerdown, 3);

static struct attribute *ad5724r_attributes[] = {
	&iio_dev_attr_out_voltage0_powerdown.dev_attr.attr,
	&iio_dev_attr_out_voltage1_powerdown.dev_attr.attr,
	&iio_dev_attr_out_voltage2_powerdown.dev_attr.attr,
	&iio_dev_attr_out_voltage3_powerdown.dev_attr.attr,
	&iio_dev_attr_out_voltage_range_mode.dev_attr.attr,
	&iio_const_attr_out_voltage_range_mode_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad5724r_attribute_group = {
	.attrs = ad5724r_attributes,
};

static const struct iio_info ad5724r_info = {
	.write_raw = ad5724r_write_raw,
	.read_raw = ad5724r_read_raw,
	.attrs = &ad5724r_attribute_group,
	.driver_module = THIS_MODULE,
};

static int __devinit ad5724r_probe(struct spi_device *spi)
{
	struct ad5724r_state *st;
	struct iio_dev *indio_dev;
	int ret, voltage_uv = 0;
	unsigned short rx_data;

	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	st = iio_priv(indio_dev);
	st->reg = regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_put_reg;

		voltage_uv = regulator_get_voltage(st->reg);
	}

	spi_set_drvdata(spi, indio_dev);
	st->chip_info =
		&ad5724r_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	if (voltage_uv)
		st->vref_mv = voltage_uv / 1000;
	else
		st->vref_mv = st->chip_info->int_vref_mv;

	st->us = spi;
	st->pwr__mode = POWERUP_ALL | POWERUP_REF;
	st->out_range_mode = RANGE2_SELECT_0_10V;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5724r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = AD5724R_DAC_CHANNELS;

	ret = ad5724r_spi_write(spi, AD5724R_REG_PWR_CTL, 0,
				st->pwr__mode, 16);
	if (ret)
		goto error_disable_reg;

	ret = ad5724r_spi_read(spi, AD5724R_REG_PWR_CTL, 0,
				&rx_data);
	if (ret)
		goto error_disable_reg;

	if (rx_data != st->pwr__mode)
		dev_warn(&indio_dev->dev, "power setup mode %d, expect %d\n",
					rx_data, st->pwr__mode);

	ret = ad5724r_spi_write(spi, AD5724R_REG_OUTPUT_RANGE,
				DAC_CHANNEL_ALL, st->out_range_mode, 16);
	if (ret)
		goto error_disable_reg;

	ret = ad5724r_spi_read(spi, AD5724R_REG_OUTPUT_RANGE, 0, &rx_data);
	if (ret)
		goto error_disable_reg;

	if (rx_data != st->out_range_mode)
		dev_warn(&indio_dev->dev, "out_range_mode setup %d, expect %d\n",
					rx_data, st->out_range_mode);
#if 1
	ret = ad5724r_spi_write(spi, AD5724R_REG_DAC, DAC_CHANNEL_ALL,
				DAC_VOL(3, 10), 16);
	if (ret)
		goto error_disable_reg;
#endif
	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
error_put_reg:
	if (!IS_ERR(st->reg))
		regulator_put(st->reg);
	iio_free_device(indio_dev);
error_ret:

	return ret;
}

static int __devexit ad5724r_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad5724r_state *st = iio_priv(indio_dev);

	ad5724r_spi_write(spi, AD5724R_REG_PWR_CTL, 0,
				~(POWERUP_ALL | POWERUP_REF), 16);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg)) {
		regulator_disable(st->reg);
		regulator_put(st->reg);
	}
	iio_free_device(indio_dev);

	return 0;
}

static const struct spi_device_id ad5724r_id[] = {
	{"ad5724r", ID_AD5724R},
	{"ad5734r", ID_AD5734R},
	{"ad5754r", ID_AD5754R},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5724r_id);

static struct spi_driver ad5724r_driver = {
	.driver = {
		   .name = "ad5724r",
		   .owner = THIS_MODULE,
		   },
	.probe = ad5724r_probe,
	.remove = __devexit_p(ad5724r_remove),
	.id_table = ad5724r_id,
};
module_spi_driver(ad5724r_driver);

MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD5724/34/54R DAC spi driver");
MODULE_LICENSE("GPL v2");
