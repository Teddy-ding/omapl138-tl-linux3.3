/*
 * Copyright 2014 Guangzhou Tronlong Inc.
 *
 * Licensed under the GPL-2.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#include "../iio.h"
#include "../trigger.h"

/* Timer register offsets */
#define PID12			0x0
#define TIM12			0x10
#define TIM34			0x14
#define PRD12			0x18
#define PRD34			0x1c
#define TCR			0x20
#define TGCR			0x24
#define WDTCR			0x28

/* Offsets of the 8 compare registers */
#define	CMP12_0			0x60
#define	CMP12_1			0x64
#define	CMP12_2			0x68
#define	CMP12_3			0x6c
#define	CMP12_4			0x70
#define	CMP12_5			0x74
#define	CMP12_6			0x78
#define	CMP12_7			0x7c

/* Timer register bitfields */
#define TCR_ENAMODE_DISABLE		0x0
#define TCR_ENAMODE_ONESHOT		0x1
#define TCR_ENAMODE_PERIODIC		0x2
#define TCR_ENAMODE_MASK		0x3

#define TGCR_TIMMODE_SHIFT		2
#define TGCR_TIMMODE_64BIT_GP		0x0
#define TGCR_TIMMODE_32BIT_UNCHAINED	0x1
#define TGCR_TIMMODE_64BIT_WDOG	0x2
#define TGCR_TIMMODE_32BIT_CHAINED	0x3

#define TGCR_TIM12RS_SHIFT		0
#define TGCR_TIM34RS_SHIFT		1
#define TGCR_RESET			0x0
#define TGCR_UNRESET			0x1
#define TGCR_RESET_MASK		0x3

#define WDTCR_WDEN_SHIFT             14
#define WDTCR_WDEN_DISABLE           0x0
#define WDTCR_WDEN_ENABLE            0x1
#define WDTCR_WDKEY_SHIFT            16
#define WDTCR_WDKEY_SEQ0             0xa5c6
#define WDTCR_WDKEY_SEQ1             0xda7e

struct timer_s {
	char *name;
	unsigned int id;
	unsigned long period;
	unsigned long opts;
	unsigned long flags;
	void __iomem *base;
	unsigned long tim_off;
	unsigned long prd_off;
	unsigned long enamode_shift;
	struct irqaction irqaction;
};

/* values for 'opts' field of struct timer_s */
#define TIMER_OPTS_DISABLED		0x01
#define TIMER_OPTS_ONESHOT		0x02
#define TIMER_OPTS_PERIODIC		0x04
#define TIMER_OPTS_STATE_MASK		0x07

#define TIMER_OPTS_USE_COMPARE	0x80000000
#define USING_COMPARE(t)		((t)->opts & TIMER_OPTS_USE_COMPARE)

enum {
	T2_BOT,
	T2_TOP,
	NUM_TIMERS
};

#if 0
static char *id_to_name[] = {
	[T2_BOT]	= "timer2_0",
	[T2_TOP]	= "timer2_1",
};
#endif

struct gptmr_state {
	struct iio_trigger *trig;
	struct timer_s *t;
	unsigned timer_num;
	unsigned long tick_frq;
	int irq;
};

static struct resource	*gptmr_mem;
static void __iomem	*gptmr_base;
struct clk		*gptmr_clk;

static int gptmr_enable(unsigned long gptmr_tick)
{
	u32 tgcr;
	unsigned long gptmr_freq;

	if (gptmr_tick == 0 || gptmr_tick > 200000)
		return -EINVAL;

	gptmr_freq = clk_get_rate(gptmr_clk);

	/* Disabled, Internal clock source */
	__raw_writel(0, gptmr_base + TCR);

	/* reset both timers, no pre-scaler for timer34 */
	tgcr = 0;
	__raw_writel(tgcr, gptmr_base + TGCR);

	/* Set both timers to unchained 32-bit */
	tgcr = TGCR_TIMMODE_32BIT_UNCHAINED << TGCR_TIMMODE_SHIFT;
	__raw_writel(tgcr, gptmr_base + TGCR);

	/* Unreset timers */
	tgcr |= (TGCR_UNRESET << TGCR_TIM12RS_SHIFT) |
		(TGCR_UNRESET << TGCR_TIM34RS_SHIFT);
	__raw_writel(tgcr, gptmr_base + TGCR);

	/* Init both counters to zero */
	__raw_writel(0, gptmr_base + TIM12);
	__raw_writel(0, gptmr_base + TIM34);

	/* set timeout period */
	__raw_writel(gptmr_freq / gptmr_tick, gptmr_base + PRD12);

	/* enable run continuously */
	__raw_writel(TCR_ENAMODE_PERIODIC << 6, gptmr_base + TCR);

	return 0;
}

static ssize_t iio_davinci_tmr_frequency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_trigger *trig = dev_get_drvdata(dev);
	struct gptmr_state *st = trig->private_data;
	long val;
	int ret;

	ret = strict_strtoul(buf, 10, &val);
	if (ret)
		goto error_ret;

	if (val > 200000) {
		ret = -EINVAL;
		goto error_ret;
	}

	st->tick_frq = val;

	gptmr_enable(st->tick_frq);
error_ret:
	return ret ? ret : count;
}

static ssize_t iio_davinci_tmr_frequency_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_trigger *trig = dev_get_drvdata(dev);
	struct gptmr_state *st = trig->private_data;

	return sprintf(buf, "%lu\n", st->tick_frq);
}

static DEVICE_ATTR(frequency, S_IRUGO | S_IWUSR, iio_davinci_tmr_frequency_show,
		   iio_davinci_tmr_frequency_store);

static struct attribute *iio_davinci_tmr_trigger_attrs[] = {
	&dev_attr_frequency.attr,
	NULL,
};

static const struct attribute_group iio_davinci_tmr_trigger_attr_group = {
	.attrs = iio_davinci_tmr_trigger_attrs,
};

static const struct attribute_group *iio_davinci_tmr_trigger_attr_groups[] = {
	&iio_davinci_tmr_trigger_attr_group,
	NULL
};

static irqreturn_t iio_davinci_tmr_trigger_isr(int irq, void *devid)
{
	struct gptmr_state *st = devid;

	iio_trigger_poll(st->trig, 0);

	return IRQ_HANDLED;
}

static const struct iio_trigger_ops iio_davinci_tmr_trigger_ops = {
	.owner = THIS_MODULE,
};

static int __devinit iio_davinci_tmr_trigger_probe(struct platform_device *pdev)
{
	struct gptmr_state *st;
	struct device *dev = &pdev->dev;
	int size;
	int ret = 0;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	gptmr_clk = clk_get(NULL, "timer2");
	if (IS_ERR(gptmr_clk)) {
		dev_err(dev, "failed to get timer clk\n");
		goto out;
	}

	clk_enable(gptmr_clk);

	printk("timer clk rate(%lu)", clk_get_rate(gptmr_clk));

	gptmr_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (gptmr_mem == NULL) {
		dev_err(dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	size = resource_size(gptmr_mem);
	if (!request_mem_region(gptmr_mem->start, size, pdev->name)) {
		dev_err(dev, "failed to get memory region\n");
		return -ENOENT;
	}

	gptmr_base = ioremap(gptmr_mem->start, size);
	if (!gptmr_base) {
		dev_err(dev, "failed to map memory region\n");
		release_mem_region(gptmr_mem->start, size);
		gptmr_mem = NULL;
		return -ENOMEM;
	}

	st->irq = platform_get_irq(pdev, 0);
	if (!st->irq) {
		dev_err(&pdev->dev, "No IRQs specified");
		ret = -ENODEV;
		goto out1;
	}

	st->timer_num = 2;
	st->tick_frq = 0;
	st->trig = iio_allocate_trigger("gptmr%d", st->timer_num);
	if (!st->trig) {
		ret = -ENOMEM;
		goto out1;
	}

	st->trig->private_data = st;
	st->trig->ops = &iio_davinci_tmr_trigger_ops;
	st->trig->dev.groups = iio_davinci_tmr_trigger_attr_groups;
	ret = iio_trigger_register(st->trig);
	if (ret)
		goto out2;

	ret = request_irq(st->irq, iio_davinci_tmr_trigger_isr,
			  0, st->trig->name, st);
	if (ret) {
		dev_err(&pdev->dev,
			"request IRQ-%d failed", st->irq);
		goto out4;
	}

	dev_info(&pdev->dev, "iio trigger davinci TMR%d, IRQ-%d",
		 st->timer_num, st->irq);

	platform_set_drvdata(pdev, st);

	return 0;
out4:
	iio_trigger_unregister(st->trig);
out2:
	iio_put_trigger(st->trig);
out1:
	kfree(st);
out:
	return ret;
}

static int __devexit iio_davinci_tmr_trigger_remove(struct platform_device *pdev)
{
	struct gptmr_state *st = platform_get_drvdata(pdev);

	free_irq(st->irq, st);
	iio_trigger_unregister(st->trig);
	iio_put_trigger(st->trig);
	kfree(st);

	return 0;
}

static struct platform_driver iio_davinci_tmr_trigger_driver = {
	.driver = {
		.name = "iio_davinci_tmr_trigger",
		.owner = THIS_MODULE,
	},
	.probe = iio_davinci_tmr_trigger_probe,
	.remove = __devexit_p(iio_davinci_tmr_trigger_remove),
};

static int __init iio_davinci_tmr_trig_init(void)
{
	return platform_driver_register(&iio_davinci_tmr_trigger_driver);
}
module_init(iio_davinci_tmr_trig_init);

static void __exit iio_davinci_tmr_trig_exit(void)
{
	platform_driver_unregister(&iio_davinci_tmr_trigger_driver);
}
module_exit(iio_davinci_tmr_trig_exit);

MODULE_AUTHOR("Teddy Ding <teddy@tronlong.com>");
MODULE_DESCRIPTION("Davinci system timer based trigger for the iio subsystem");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:iio-trig-davinci-timer");
