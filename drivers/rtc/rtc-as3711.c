/*
 *  Real Time Clock driver for ams AG AS3711
 *
 *  Copyright (C) 2012 ams AG.
 *
 *  Author: Florian Lobmaier <florian.lobmaier@ams.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/mfd/as3711.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#define AS3711_SET_ALM_RETRIES	5
#define AS3711_SET_TIME_RETRIES	5
#define AS3711_GET_TIME_RETRIES	5

#define to_as3711_from_rtc_dev(d) container_of(d, struct as3711, rtc.pdev.dev)

/*
 * Read current time and date in RTC
 */
static int as3711_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	struct as3711_platform_data *pdata = as3711->dev->platform_data;
	u8 as_sec;
	u8 as_min_array[3];
	int as_min;
	long time, start_time;
	struct rtc_time start_tm;
	int ret;

	as_sec = as3711_reg_read(as3711, AS3711_RTC_SECOND_REG);
	ret = as3711_block_read(as3711, AS3711_RTC_MINUTE1_REG,
				3, as_min_array);
	if (ret < 0)
		return ret;

	as_min = (as_min_array[2] << 16) 
		| (as_min_array[1] << 8) 
		| (as_min_array[0]);
	time = as_min*60 + as_sec;
	start_tm.tm_year = (pdata->rtc_start_year - 1900);
	start_tm.tm_mon = 0;
	start_tm.tm_mday = 1;
	start_tm.tm_hour = 0;
	start_tm.tm_min = 0;
	start_tm.tm_sec = 0;
	rtc_tm_to_time(&start_tm, &start_time);
	time = time + start_time;
	rtc_time_to_tm(time, tm);
	return 0;
}

/*
 * Set current time and date in RTC
 */
static int as3711_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	struct as3711_platform_data *pdata = as3711->dev->platform_data;
	long time, start_time;
	u8 as_sec;
	u8 as_min_array[3];
	int as_min;
	struct rtc_time start_tm;
	int ret;

	/* Write time to RTC */
	rtc_tm_to_time(tm, &time);
	start_tm.tm_year = (pdata->rtc_start_year - 1900);
        start_tm.tm_mon = 0;
        start_tm.tm_mday = 1;
        start_tm.tm_hour = 0;
        start_tm.tm_min = 0;
        start_tm.tm_sec = 0;
        rtc_tm_to_time(&start_tm, &start_time);
	time = time - start_time;
	as_min = time / 60;
	as_sec = time % 60;
	as_min_array[2] = (as_min & 0xFF0000) >> 16;
	as_min_array[1] = (as_min & 0xFF00) >> 8;
	as_min_array[0] = as_min & 0xFF;
	as3711_reg_write(as3711, AS3711_RTC_SECOND_REG, as_sec);
	ret = as3711_block_write(as3711, AS3711_RTC_MINUTE1_REG, 3, as_min_array);
	if (ret < 0)
		return ret;

	return ret;
}

/*
 * Read alarm time and date in RTC
 */
static int as3711_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	struct as3711_platform_data *pdata = as3711->dev->platform_data;
	u8 as_sec;
	u8 as_min_array[3];
	int as_min;
	long time, start_time;
	struct rtc_time start_tm;
	int ret;

	as_sec = as3711_reg_read(as3711, AS3711_RTC_ALARM_SECOND_REG);
	ret = as3711_block_read(as3711, AS3711_RTC_ALARM_MINUTE1_REG,
				3, as_min_array);
	if (ret < 0)
		return ret;

	as_min = (as_min_array[2] << 16) 
		| (as_min_array[1] << 8) 
		| (as_min_array[0]);
	time = as_min*60 + as_sec;
	start_tm.tm_year = (pdata->rtc_start_year - 1900);
	start_tm.tm_mon = 0;
	start_tm.tm_mday = 1;
	start_tm.tm_hour = 0;
	start_tm.tm_min = 0;
	start_tm.tm_sec = 0;
	rtc_tm_to_time(&start_tm, &start_time);
	time = time + start_time;
	rtc_time_to_tm(time, &alrm->time);
	return 0;
}

static int as3711_rtc_stop_alarm(struct as3711 *as3711)
{
	/* disable rtc alarm interrupt */
	return as3711_mask_irq(as3711, AS3711_IRQ_RTC_ALARM);
}

static int as3711_rtc_start_alarm(struct as3711 *as3711)
{
	/* enable rtc alarm interrupt */
	return as3711_unmask_irq(as3711, AS3711_IRQ_RTC_ALARM);
}

static int as3711_rtc_alarm_irq_enable(struct device *dev,
				       unsigned int enabled)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);

	if (enabled)
		return as3711_rtc_start_alarm(as3711);
	else
		return as3711_rtc_stop_alarm(as3711);
}

static int as3711_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);
	struct as3711_platform_data *pdata = as3711->dev->platform_data;
	long time, start_time;
	u8 as_sec;
	u8 as_min_array[3];
	int as_min;
	struct rtc_time start_tm;
	int ret;

	/* Write time to RTC */
	rtc_tm_to_time(&alrm->time, &time);
	start_tm.tm_year = (pdata->rtc_start_year - 1900);
        start_tm.tm_mon = 0;
        start_tm.tm_mday = 1;
        start_tm.tm_hour = 0;
        start_tm.tm_min = 0;
        start_tm.tm_sec = 0;
        rtc_tm_to_time(&start_tm, &start_time);
	time = time - start_time;
	as_min = time / 60;
	as_sec = time % 60;
	as_min_array[2] = (as_min & 0xFF0000) >> 16;
	as_min_array[1] = (as_min & 0xFF00) >> 8;
	as_min_array[0] = as_min & 0xFF;

	/* Write time to RTC */
	as3711_reg_write(as3711, AS3711_RTC_ALARM_SECOND_REG, as_sec);
	ret = as3711_block_write(as3711, AS3711_RTC_ALARM_MINUTE1_REG, 3, as_min_array);
	if (ret < 0)
		return ret;

	return ret;
}

static int as3711_rtc_update_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct as3711 *as3711 = dev_get_drvdata(dev);

	if (enabled)
		as3711_unmask_irq(as3711, AS3711_IRQ_RTC_REP);
	else
		as3711_mask_irq(as3711, AS3711_IRQ_RTC_REP);

	return 0;
}

static irqreturn_t as3711_rtc_alarm_handler(int irq, void *data)
{
	struct as3711 *as3711 = data;
	struct rtc_device *rtc = as3711->rtc.rtc;

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static irqreturn_t as3711_rtc_update_handler(int irq, void *data)
{
	struct as3711 *as3711 = data;
	struct rtc_device *rtc = as3711->rtc.rtc;

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_UF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops as3711_rtc_ops = {
	.read_time = as3711_rtc_readtime,
	.set_time = as3711_rtc_settime,
	.read_alarm = as3711_rtc_readalarm,
	.set_alarm = as3711_rtc_setalarm,
	.alarm_irq_enable = as3711_rtc_alarm_irq_enable,
	//.update_irq_enable = as3711_rtc_update_irq_enable,
};

#ifdef CONFIG_PM
static int as3711_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct as3711 *as3711 = dev_get_drvdata(&pdev->dev);
	int ret = 0;
	u16 reg;

	reg = as3711_reg_read(as3711, AS3711_INTERRUPTMASK3_REG);

	if (device_may_wakeup(&as3711->rtc.pdev->dev) &&
	    reg & AS3711_IRQ_MASK_RTC_ALARM) {
		ret = as3711_rtc_stop_alarm(as3711);
		if (ret != 0)
			dev_err(&pdev->dev, "Failed to stop RTC alarm: %d\n",
				ret);
	}

	return ret;
}

static int as3711_rtc_resume(struct platform_device *pdev)
{
	struct as3711 *as3711 = dev_get_drvdata(&pdev->dev);
	int ret;

	if (as3711->rtc.alarm_enabled) {
		ret = as3711_rtc_start_alarm(as3711);
		if (ret != 0)
			dev_err(&pdev->dev,
				"Failed to restart RTC alarm: %d\n", ret);
	}

	return 0;
}

#else
#define as3711_rtc_suspend NULL
#define as3711_rtc_resume NULL
#endif

static int as3711_rtc_probe(struct platform_device *pdev)
{
	struct as3711 *as3711 = platform_get_drvdata(pdev);
	struct as3711_rtc *rtc = &as3711->rtc;
	int ret = 0;
	u8 ctrl;

	/* enable the RTC if it's not already enabled */
	ctrl = as3711_reg_read(as3711, AS3711_RTC_CONTROL_REG);
	if (!(ctrl &  AS3711_RTC_ON_MASK)) {
		dev_info(&pdev->dev, "Starting RTC\n");

		ret = as3711_set_bits(as3711, AS3711_RTC_CONTROL_REG,
				      AS3711_RTC_ON_MASK, AS3711_RTC_ON_MASK);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to enable RTC: %d\n", ret);
			return ret;
		}
	}
	/* enable alarm wakeup */
	as3711_set_bits(as3711, AS3711_RTC_CONTROL_REG,
			AS3711_RTC_ALARM_WAKEUP_EN_MASK, 
			AS3711_RTC_ALARM_WAKEUP_EN_MASK);

	device_init_wakeup(&pdev->dev, 1);

	rtc->rtc = rtc_device_register("as3711", &pdev->dev,
					  &as3711_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err(&pdev->dev, "failed to register RTC: %d\n", ret);
		return ret;
	}

	as3711_register_irq(as3711, AS3711_IRQ_HANDLER_RTC_REP,
			    as3711_rtc_update_handler, 0,
			    "RTC Repeat", as3711);
	as3711_mask_irq(as3711, AS3711_IRQ_RTC_REP);

	as3711_register_irq(as3711, AS3711_IRQ_HANDLER_RTC_ALARM,
			    as3711_rtc_alarm_handler, 0,
			    "RTC Alarm", as3711);

	return 0;
}

static int __devexit as3711_rtc_remove(struct platform_device *pdev)
{
	struct as3711 *as3711 = platform_get_drvdata(pdev);
	struct as3711_rtc *rtc = &as3711->rtc;

	as3711_free_irq(as3711, AS3711_IRQ_RTC_REP);
	as3711_free_irq(as3711, AS3711_IRQ_RTC_ALARM);

	rtc_device_unregister(rtc->rtc);

	return 0;
}

static struct platform_driver as3711_rtc_driver = {
	.probe = as3711_rtc_probe,
	.remove = __devexit_p(as3711_rtc_remove),
	.suspend = as3711_rtc_suspend,
	.resume = as3711_rtc_resume,
	.driver = {
		.name = "as3711-rtc",
	},
};

static int __init as3711_rtc_init(void)
{
	return platform_driver_register(&as3711_rtc_driver);
}
module_init(as3711_rtc_init);

static void __exit as3711_rtc_exit(void)
{
	platform_driver_unregister(&as3711_rtc_driver);
}
module_exit(as3711_rtc_exit);

MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
MODULE_DESCRIPTION("RTC driver for the AS3711");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:as3711-rtc");
