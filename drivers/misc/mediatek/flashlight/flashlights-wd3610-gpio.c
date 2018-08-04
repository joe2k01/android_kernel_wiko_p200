/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
/* TODO: modify temp device tree name */
#ifndef WD3610_DTNAME
#define WD3610_DTNAME "mediatek,flashlights_wd3610_strobepin"
#endif

/* TODO: define driver name */
#define WD3610_NAME "flashlights-wd3610-strobepin"

/* define registers */
/* TODO: define register */

/* define mutex and work queue */
static DEFINE_MUTEX(wd3610_mutex);
static struct work_struct wd3610_work;

/* define pinctrl */
/* TODO: define pinctrl */
#define WD3610_PINCTRL_PIN_FLASHPIN 		0
#define WD3610_PINCTRL_PIN_ENPIN		1
#define WD3610_PINCTRL_PIN_SUB_FLASHPIN		2
#define WD3610_PINCTRL_PIN_SUB_ENPIN 		3

#define WD3610_PINCTRL_PINSTATE_LOW  0
#define WD3610_PINCTRL_PINSTATE_HIGH 1

#define WD3610_PINCTRL_STATE_FLASHPIN_LOW		"flashlightpin_cfg0"
#define WD3610_PINCTRL_STATE_FLASHPIN_HIGH  		"flashlightpin_cfg1"
#define WD3610_PINCTRL_STATE_ENPIN_LOW 			"flashlightpin_en0"
#define WD3610_PINCTRL_STATE_ENPIN_HIGH  		"flashlightpin_en1"
#define WD3610_PINCTRL_STATE_SUB_FLASHPIN_LOW 		"flashlightpin_cfg10"
#define WD3610_PINCTRL_STATE_SUB_FLASHPIN_HIGH  	"flashlightpin_cfg11"
#define WD3610_PINCTRL_STATE_SUB_ENPIN_LOW 		"flashlightpin_en10"
#define WD3610_PINCTRL_STATE_SUB_ENPIN_HIGH  		"flashlightpin_en11"

static struct pinctrl *wd3610_pinctrl;
static struct pinctrl_state *wd3610_flashpin_high;
static struct pinctrl_state *wd3610_flashpin_low;
static struct pinctrl_state *wd3610_enpin_high;
static struct pinctrl_state *wd3610_enpin_low;
static struct pinctrl_state *wd3610_sub_flashpin_high;
static struct pinctrl_state *wd3610_sub_flashpin_low;
static struct pinctrl_state *wd3610_sub_enpin_high;
static struct pinctrl_state *wd3610_sub_enpin_low;

/* define usage count */
static int use_count;

static int g_duty = -1;
static int sensordev = -1;

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int wd3610_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	wd3610_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(wd3610_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(wd3610_pinctrl);
	}

	/* TODO: Flashlight STROBEPIN pin initialization */
	wd3610_flashpin_high = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_FLASHPIN_HIGH);
	if (IS_ERR(wd3610_flashpin_high)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_FLASHPIN_HIGH);
		ret = PTR_ERR(wd3610_flashpin_high);
	}
	wd3610_flashpin_low = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_FLASHPIN_LOW);
	if (IS_ERR(wd3610_flashpin_low)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_FLASHPIN_LOW);
		ret = PTR_ERR(wd3610_flashpin_low);
	}

	wd3610_enpin_high = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_ENPIN_HIGH);
	if (IS_ERR(wd3610_enpin_high)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_ENPIN_HIGH);
		ret = PTR_ERR(wd3610_enpin_high);
	}
	wd3610_enpin_low = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_ENPIN_LOW);
	if (IS_ERR(wd3610_enpin_low)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_ENPIN_LOW);
		ret = PTR_ERR(wd3610_enpin_low);
	}

	wd3610_sub_flashpin_high = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_SUB_FLASHPIN_HIGH);
	if (IS_ERR(wd3610_sub_flashpin_high)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_SUB_FLASHPIN_HIGH);
		ret = PTR_ERR(wd3610_sub_flashpin_high);
	}
	wd3610_sub_flashpin_low = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_SUB_FLASHPIN_LOW);
	if (IS_ERR(wd3610_sub_flashpin_low)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_SUB_FLASHPIN_LOW);
		ret = PTR_ERR(wd3610_sub_flashpin_low);
	}

	wd3610_sub_enpin_high = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_SUB_ENPIN_HIGH);
	if (IS_ERR(wd3610_sub_enpin_high)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_SUB_ENPIN_HIGH);
		ret = PTR_ERR(wd3610_sub_enpin_high);
	}
	wd3610_sub_enpin_low = pinctrl_lookup_state(wd3610_pinctrl, WD3610_PINCTRL_STATE_SUB_ENPIN_LOW);
	if (IS_ERR(wd3610_sub_enpin_low)) {
		pr_err("Failed to init (%s)\n", WD3610_PINCTRL_STATE_SUB_ENPIN_LOW);
		ret = PTR_ERR(wd3610_sub_enpin_low);
	}
	return ret;
}

static int wd3610_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(wd3610_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
		case WD3610_PINCTRL_PIN_FLASHPIN:
			if (state == WD3610_PINCTRL_PINSTATE_LOW && !IS_ERR(wd3610_flashpin_low))
				pinctrl_select_state(wd3610_pinctrl, wd3610_flashpin_low);
			else if (state == WD3610_PINCTRL_PINSTATE_HIGH && !IS_ERR(wd3610_flashpin_high))
				pinctrl_select_state(wd3610_pinctrl, wd3610_flashpin_high);
			else
				pr_err("set err, pin(%d) state(%d)\n", pin, state);
			break;

		case WD3610_PINCTRL_PIN_ENPIN:
			if (state == WD3610_PINCTRL_PINSTATE_LOW && !IS_ERR(wd3610_enpin_low))
				pinctrl_select_state(wd3610_pinctrl, wd3610_enpin_low);
			else if (state == WD3610_PINCTRL_PINSTATE_HIGH && !IS_ERR(wd3610_enpin_high))
				pinctrl_select_state(wd3610_pinctrl, wd3610_enpin_high);
			else
				pr_err("set err, pin(%d) state(%d)\n", pin, state);
			break;

		case WD3610_PINCTRL_PIN_SUB_FLASHPIN:
			if (state == WD3610_PINCTRL_PINSTATE_LOW && !IS_ERR(wd3610_sub_flashpin_low))
				pinctrl_select_state(wd3610_pinctrl, wd3610_sub_flashpin_low);
			else if (state == WD3610_PINCTRL_PINSTATE_HIGH && !IS_ERR(wd3610_sub_flashpin_high))
				pinctrl_select_state(wd3610_pinctrl, wd3610_sub_flashpin_high);
			else
				pr_err("set err, pin(%d) state(%d)\n", pin, state);
			break;

		case WD3610_PINCTRL_PIN_SUB_ENPIN:
			if (state == WD3610_PINCTRL_PINSTATE_LOW && !IS_ERR(wd3610_sub_enpin_low))
				pinctrl_select_state(wd3610_pinctrl, wd3610_sub_enpin_low);
			else if (state == WD3610_PINCTRL_PINSTATE_HIGH && !IS_ERR(wd3610_sub_enpin_high))
				pinctrl_select_state(wd3610_pinctrl, wd3610_sub_enpin_high);
			else
				pr_err("set err, pin(%d) state(%d)\n", pin, state);
			break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_debug("pin(%d) state(%d)\n", pin, state);

	return ret;
}


/******************************************************************************
 * wd3610 operations
 *****************************************************************************/
/* flashlight enable function */
static int wd3610_enable(void)
{
	//int pin = 0, state = 0;
	 pr_debug("wd3610_enable:sensordev = %d,g_duty =  %d\n",sensordev,g_duty);
	/* TODO: wrap enable function */
	if(sensordev == 0){
		if(g_duty == 0){
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_ENPIN, WD3610_PINCTRL_PINSTATE_HIGH);
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_FLASHPIN, WD3610_PINCTRL_PINSTATE_LOW);

		}
		else {
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_ENPIN, WD3610_PINCTRL_PINSTATE_HIGH);
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_FLASHPIN, WD3610_PINCTRL_PINSTATE_HIGH);

		}
	}
	else if (sensordev == 1){
		//if(g_duty == 0){
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_ENPIN, WD3610_PINCTRL_PINSTATE_HIGH);
			wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_FLASHPIN, WD3610_PINCTRL_PINSTATE_LOW);
		//}
		//else {
		//	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_ENPIN, WD3610_PINCTRL_PINSTATE_HIGH);
		//	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_FLASHPIN, WD3610_PINCTRL_PINSTATE_HIGH);
		//}
	}
	//return wd3610_pinctrl_set(pin, state);
	return 0;
}

/* flashlight disable function */
static int wd3610_disable(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap disable function */
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	
	//return wd3610_pinctrl_set(pin, state);
	return 0;
}

/* set flashlight level */
static int wd3610_set_level(int level)
{
	//int pin = 0, state = 0;

	/* TODO: wrap set level function */
	g_duty = level;
	//return wd3610_pinctrl_set(pin, state);
	return 0;
}

/* flashlight init */
static int wd3610_init(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap init function */
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	
	//return wd3610_pinctrl_set(pin, state);
	return 0;
}

/* flashlight uninit */
static int wd3610_uninit(void)
{
	//int pin = 0, state = 0;

	/* TODO: wrap uninit function */
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	wd3610_pinctrl_set(WD3610_PINCTRL_PIN_SUB_ENPIN, WD3610_PINCTRL_PINSTATE_LOW);
	
	//return wd3610_pinctrl_set(pin, state);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer wd3610_timer;
static unsigned int wd3610_timeout_ms;

static void wd3610_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	wd3610_disable();
}

static enum hrtimer_restart wd3610_timer_func(struct hrtimer *timer)
{
	schedule_work(&wd3610_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int wd3610_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
	sensordev = channel;
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3610_timeout_ms = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		wd3610_set_level(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (wd3610_timeout_ms) {
				ktime = ktime_set(wd3610_timeout_ms / 1000,
						(wd3610_timeout_ms % 1000) * 1000000);
				hrtimer_start(&wd3610_timer, ktime, HRTIMER_MODE_REL);
			}
			wd3610_enable();
		} else {
			wd3610_disable();
			hrtimer_cancel(&wd3610_timer);
		}
		break;
	default:
		pr_debug("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int wd3610_open(void *pArg)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int wd3610_release(void *pArg)
{
	/* uninit chip and clear usage count */
	mutex_lock(&wd3610_mutex);
	use_count--;
	if (!use_count)
		wd3610_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&wd3610_mutex);

	pr_debug("Release: %d\n", use_count);

	return 0;
}

static int wd3610_set_driver(void)
{
	/* init chip and set usage count */
	mutex_lock(&wd3610_mutex);
	if (!use_count)
		wd3610_init();
	use_count++;
	mutex_unlock(&wd3610_mutex);

	pr_debug("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t wd3610_strobe_store(struct flashlight_arg arg)
{
	wd3610_set_driver();
	wd3610_set_level(arg.level);
	wd3610_enable();
	msleep(arg.dur);
	wd3610_disable();
	wd3610_release(NULL);

	return 0;
}

static struct flashlight_operations wd3610_ops = {
	wd3610_open,
	wd3610_release,
	wd3610_ioctl,
	wd3610_strobe_store,
	wd3610_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int wd3610_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * wd3610_init();
	 */

	return 0;
}

static int wd3610_probe(struct platform_device *dev)
{
	int err;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (wd3610_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		err = -EFAULT;
		goto err;
	}

	/* init work queue */
	INIT_WORK(&wd3610_work, wd3610_work_disable);

	/* init timer */
	hrtimer_init(&wd3610_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wd3610_timer.function = wd3610_timer_func;
	wd3610_timeout_ms = 100;

	/* init chip hw */
	wd3610_chip_init();

	/* register flashlight operations */
	if (flashlight_dev_register(WD3610_NAME, &wd3610_ops)) {
		err = -EFAULT;
		goto err;
	}

	/* clear usage count */
	use_count = 0;

	pr_debug("Probe done.\n");

	return 0;
err:
	return err;
}

static int wd3610_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	/* flush work queue */
	flush_work(&wd3610_work);

	/* unregister flashlight operations */
	flashlight_dev_unregister(WD3610_NAME);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wd3610_strobepin_of_match[] = {
	{.compatible = WD3610_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, wd3610_strobepin_of_match);
#else
static struct platform_device wd3610_strobepin_platform_device[] = {
	{
		.name = WD3610_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, wd3610_strobepin_platform_device);
#endif

static struct platform_driver wd3610_platform_driver = {
	.probe = wd3610_probe,
	.remove = wd3610_remove,
	.driver = {
		.name = WD3610_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = wd3610_strobepin_of_match,
#endif
	},
};

static int __init flashlight_wd3610_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&wd3610_strobepin_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&wd3610_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_wd3610_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&wd3610_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_wd3610_init);
module_exit(flashlight_wd3610_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight WD3610 STROBEPIN Driver");

