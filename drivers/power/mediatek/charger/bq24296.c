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

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
//#include <mt-plat/charging.h>
#include <mt-plat/mtk_boot_common.h>
#include "bq24296.h"


#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
#include <mt-plat/charger_class.h>
#include "mtk_charger_intf.h"
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */
/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define bq24296_SLAVE_ADDR_WRITE   0xD6
#define bq24296_SLAVE_ADDR_READ    0xD7

#define STATUS_OK	0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

static const struct bq24296_platform_data bq24296_def_platform_data = {
	.chg_name = "primary_chg",
	.ichg = 1550000, /* unit: uA */
	.aicr = 500000, /* unit: uA */
	.mivr = 4500000, /* unit: uV */
	.ieoc = 150000, /* unit: uA */
	.voreg = 4350000, /* unit : uV */
	.vmreg = 4350000, /* unit : uV */
	.intr_gpio = 76,
};

/* ============================================================ // */
/* Global variable */
/* ============================================================ // */

const unsigned int VBAT_CV_VTH[] = {
	3504000, 3520000, 3536000, 3552000,
	3568000, 3584000, 3600000, 3616000,
	3632000, 3648000, 3664000, 3680000,
	3696000, 3712000, 3728000, 3744000,
	3760000, 3776000, 3792000, 3808000,
	3824000, 3840000, 3856000, 3872000,
	3888000, 3904000, 3920000, 3936000,
	3952000, 3968000, 3984000, 4000000,
	4016000, 4032000, 4048000, 4064000,
	4080000, 4096000, 4112000, 4128000,
	4144000, 4160000, 4176000, 4192000,
	4208000, 4224000, 4240000, 4256000,
	4272000, 4288000, 4304000, 4320000,
// TINNO BEGIN
// Added by liaoye on Nov. 14, 2016 for bugid:DJJBN-406	
	4336000, 4352000, 4368000, 4384000,
	4400000,
// TINNO END
};

const unsigned int CS_VTH[] = {
	512000,  576000,  640000,  704000,
	768000,  832000,  896000,  960000,
	1024000, 1088000, 1152000, 1216000,
	1280000, 1344000, 1408000, 1472000,
	1536000, 1600000, 1664000, 1728000,
	1792000, 1856000, 1920000, 1984000,
	2048000, 2112000, 2176000, 2240000
};

const unsigned int Current_in_limit[] = {
	100000,  150000,  500000,  900000,
	1000000, 1500000, 2000000, 3000000
};
static DEFINE_MUTEX(bq24157_i2c_access);

static struct i2c_client *new_client;
static const struct i2c_device_id bq24296_i2c_id[] = { {"bq24296", 0}, {} };

bool chargin_hw_init_done = 0;
static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id bq24296_of_match[] = {
	{.compatible = "mediatek,bq24296",},		//caizhifu modified for bq24296 same with dts, 2016-10-28
	{},
};

struct bq24296_info {
	struct device *dev;
	struct i2c_client *i2c;
#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
	struct charger_device *chg_dev;
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */
	u8 chip_rev;
};

MODULE_DEVICE_TABLE(of, bq24296_of_match);
#endif

static struct i2c_driver bq24296_driver = {
	.driver = {
		   .name = "bq24296",
#ifdef CONFIG_OF
		   .of_match_table = bq24296_of_match,
#endif
		   },
	.probe = bq24296_driver_probe,
	.id_table = bq24296_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq24296_reg[bq24296_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq24296_i2c_access);

int g_bq24296_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24296]
  *
  *********************************************************/
int bq24296_read_byte(unsigned char cmd, unsigned char *returnData)
{
	int ret = 0;
	struct i2c_msg msgs[] =
  {
      {
          .addr = new_client->addr,
          .flags = 0,
          .len = 1,
          .buf = &cmd,
      },
      {
          .addr = new_client->addr,
          .flags = I2C_M_RD,
          .len = 1,
          .buf = returnData,
      },
  };
	mutex_lock(&bq24157_i2c_access);
  ret = i2c_transfer(new_client->adapter, msgs, 2);
  if(ret < 0){
  	chr_err("%s: read 0x%x register failed\n",__func__,cmd);
  }
  chr_info("%s:register = 0x%x , value = 0x%x\n",__func__,cmd,*returnData);
	mutex_unlock(&bq24157_i2c_access);
	return 1;
}

int bq24296_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
  struct i2c_msg msgs[] =
  {
      {
          .addr = new_client->addr,
          .flags = 0,
          .len = 2,
          .buf = write_data,
      },
  };

	write_data[0] = cmd;
	write_data[1] = writeData;
	chr_info("bq24157_write_byte cmd = 0x%x writeData = 0x%x\n",cmd,writeData);
	
	mutex_lock(&bq24157_i2c_access);
  ret = i2c_transfer(new_client->adapter, msgs, 1);
  if (ret < 0){
      chr_err("%s: write 0x%x to 0x%x register failed\n",__func__,writeData,cmd);
      return ret;
  }
	mutex_unlock(&bq24157_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq24296_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq24296_reg = 0;
	int ret = 0;


	ret = bq24296_read_byte(RegNum, &bq24296_reg);

	pr_err("[bq24296_read_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	bq24296_reg &= (MASK << SHIFT);
	*val = (bq24296_reg >> SHIFT);

	pr_err("[bq24296_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq24296_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq24296_reg = 0;
	int ret = 0;

	ret = bq24296_read_byte(RegNum, &bq24296_reg);
	pr_err("[bq24296_config_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	bq24296_reg &= ~(MASK << SHIFT);
	bq24296_reg |= (val << SHIFT);

	ret = bq24296_write_byte(RegNum, bq24296_reg);
	pr_err("[bq24296_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	/* Check */
	/* bq24296_read_byte(RegNum, &bq24296_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq24296_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24296_reg); */

	return ret;
}

/* write one register directly */
unsigned int bq24296_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = bq24296_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq24296_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq24296_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_VINDPM_MASK),
				       (unsigned char) (CON0_VINDPM_SHIFT)
	    );
}

void bq24296_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}

/* CON1---------------------------------------------------- */

void bq24296_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_REG_RST_MASK),
				       (unsigned char) (CON1_REG_RST_SHIFT)
	    );
}

void bq24296_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
	    );
}

void bq24296_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
	    );
}

void bq24296_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
	    );
}

void bq24296_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq24296_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BOOST_LIM_MASK),
				       (unsigned char) (CON1_BOOST_LIM_SHIFT)
	    );
}

/* CON2---------------------------------------------------- */

void bq24296_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK), (unsigned char) (CON2_ICHG_SHIFT)
	    );
}

void bq24296_set_bcold(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BCOLD_MASK), (unsigned char) (CON2_BCOLD_SHIFT)
	    );
}

void bq24296_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK),
				       (unsigned char) (CON2_FORCE_20PCT_SHIFT)
	    );
}

/* CON3---------------------------------------------------- */

void bq24296_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
	    );
}

void bq24296_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK), (unsigned char) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq24296_set_vreg(unsigned int val)
{
return;
}

void bq24296_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_BATLOWV_MASK),
				       (unsigned char) (CON4_BATLOWV_SHIFT)
	    );
}

void bq24296_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq24296_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
	    );
}

void bq24296_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq24296_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq24296_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
	    );
}

/* CON6---------------------------------------------------- */

void bq24296_set_treg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_TREG_MASK), (unsigned char) (CON6_TREG_SHIFT)
	    );
}

void bq24296_set_boostv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
	    );
}

void bq24296_set_bhot(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BHOT_MASK), (unsigned char) (CON6_BHOT_SHIFT)
	    );
}

/* CON7---------------------------------------------------- */

void bq24296_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR2X_EN_MASK),
				       (unsigned char) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq24296_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_Disable_MASK),
				       (unsigned char) (CON7_BATFET_Disable_SHIFT)
	    );
}

void bq24296_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_INT_MASK_MASK),
				       (unsigned char) (CON7_INT_MASK_SHIFT)
	    );
}

/* CON8---------------------------------------------------- */

unsigned int bq24296_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val), (unsigned char) (0xFF), (unsigned char) (0x0)
	    );
	return val;
}

unsigned int bq24296_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24296_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24296_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
bool bq24296_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface(0x0A, &val, 0xFF, 0x0);
	if (val == 0)
		g_bq24296_hw_exist = 0;
	else
		g_bq24296_hw_exist = 1;
	pr_err("[bq24296_hw_component_detect] exist=%d, Reg[0x0A]=0x%x\n", g_bq24296_hw_exist, val);

	ret = bq24296_read_interface(0x0B, &val, 0x0F, 0x03);
	if (val == 2)
		g_bq24296_hw_exist = 0;

	pr_err("[bq24296_hw_component_detect] exist=%d, Reg[0x0B]=0x%x\n", g_bq24296_hw_exist, val);
	return g_bq24296_hw_exist;
}

int is_bq24296_exist(void)
{
	pr_err("[is_bq24296_exist] g_bq24296_hw_exist=%d\n", g_bq24296_hw_exist);

	return g_bq24296_hw_exist;
}

void bq24296_dump_register(void)
{
	int i = 0;

	for (i = 0; i < bq24296_REG_NUM; i++) {
		bq24296_read_byte(i, &bq24296_reg[i]);
		pr_err("bq24296:[0x%x]=0x%x\n", i, bq24296_reg[i]);
	}
}

u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;

	chr_info("NO register value match \r\n");

	return 0;
}

static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = true;
	else
		max_value_in_last_element = false;

	if (max_value_in_last_element == true) {
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		chr_info("Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		chr_info("Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}

#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)

static unsigned int charging_hw_init(void)
{
	unsigned int status = 0;

#if defined(GPIO_SWCHARGER_EN_PIN)
	mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN, GPIO_OUT_ZERO);
#endif

	bq24296_set_en_hiz(0x0);

	bq24296_set_vindpm(0);	/* VIN DPM check 4.36V */

//caizhifu add end for charger VINDPM set, because charger current is smaller than setting value, 2016-11-04

	bq24296_set_reg_rst(0x0);
	bq24296_set_wdt_rst(0x1);	/* Kick watchdog */
	bq24296_set_sys_min(0x5);	/* Minimum system voltage 3.5V */
	bq24296_set_iprechg(0x3);	/* Precharge current 512mA */
	bq24296_set_iterm(0x0);	/* Termination current 128mA */

	bq24296_set_batlowv(0x1);	/* BATLOWV 3.0V */
	bq24296_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq24296_write_byte(0x05,0x80);

	return status;
}

static int bq24296_charger_plug_in(struct charger_device *chg_dev)
{
	//do nothing
	return 0;
}

static int bq24296_charger_plug_out(struct charger_device *chg_dev)
{
	//do nothing
	return 0;
}

static int bq24296_charging_enable(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;
	unsigned int bootmode = 0;
	
	if (1 == en) {
		bq24296_set_en_hiz(0x0);
		bq24296_set_chg_config(0x1);	/* charger enable */
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device()) {
#endif

#if defined(CONFIG_USB_MTK_HDRC_HCD)
		}
#endif

		bootmode = get_boot_mode();
		if ((bootmode == META_BOOT) || (bootmode == ADVMETA_BOOT))
			bq24296_set_en_hiz(0x1);

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		bq24296_set_chg_config(0x0);
		bq24296_set_en_hiz(0x1);	/* disable power path */
#endif
	}

	return status;
}

static int bq24296_charger_is_enabled(struct charger_device *chg_dev, bool *en)
{
	return 0;
}

static int bq24296_charger_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	unsigned int status = 0;
	/* unsigned int array_size; */
	/* unsigned char reg_value; */

	unsigned char ret_val = 0;
	unsigned char ret_force_20pct = 0;

	/* Get current level */
	bq24296_read_interface(bq24296_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

	/* Get Force 20% option */
	bq24296_read_interface(bq24296_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK,
			       CON2_FORCE_20PCT_SHIFT);

	/* Parsing */
	ret_val = (ret_val * 64) + 512;

	if (ret_force_20pct == 0) {
		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
		*(unsigned int *) uA = ret_val;
	} else {
		/* Get current level */
		/* array_size = GETARRAYNUM(CS_VTH_20PCT); */
		/* *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value); */
		/* return (int)(ret_val<<1)/10; */
		*(unsigned int *) uA = (int)(ret_val << 1) / 10;
	}

	return status;
}

static int bq24296_charger_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;
	unsigned int current_value = uA;

  uA -= 512;
	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size, set_chr_current);
	pr_err("%s:set_chr_current = %d,register_value= %d\n",__func__,set_chr_current,register_value);
	bq24296_set_ichg(register_value);

	return status;
}

static int bq24296_charger_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 550000;
	return 0;
}

static int bq24296_charger_set_cv(struct charger_device *chg_dev, u32 uV)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int set_cv_voltage;
	unsigned short register_value;
	unsigned int cv_value = uV;
	static short pre_register_value = -1;

	/* use nearest value */
	if (4200000 == cv_value)
		cv_value = 4208000;

	array_size = GETARRAYNUM(VBAT_CV_VTH);
	set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);

	bq24296_set_vreg(register_value);

	/* for jeita recharging issue */
	if (pre_register_value != register_value)
		bq24296_set_chg_config(1);

	pre_register_value = register_value;

	return status;
}

static int bq24296_charger_get_cv(struct charger_device *chg_dev, u32 *uV)
{
	//do nothing
	return 0;
}

static int bq24296_charger_get_aicr(struct charger_device *chg_dev, u32 *uA)
{
	//do nothing
	return 0;
}

static int bq24296_charger_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	int array_size = 0;
	int i = 0;

	array_size = sizeof(Current_in_limit)/sizeof(Current_in_limit[0]);
	for(i=array_size-1; i>0; i--)
	{
	    	if(Current_in_limit[i] <= uA){
	    		bq24296_set_iinlim(i);
	    		break;
	    	}
	}
	pr_debug("%s:uA = %d,Current_in_limit[%d] = %d\n",__func__,uA,i,Current_in_limit[i]);
	return 0;
}

static int bq24296_charger_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	//do nothing
	*uA = 100000;
	return 0;
}

static int bq24296_charger_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	//do nothing
	return 0;
}

static int bq24296_charger_is_timer_enabled(struct charger_device *chg_dev,
					   bool *en)
{
  return 0;
}

static int bq24296_charger_enable_timer(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;
	pr_err("charging_reset_watch_dog_timer\r\n");

	bq24296_set_wdt_rst(0x1);	/* Kick watchdog */

	return status;
}

static int bq24296_charger_enable_te(struct charger_device *chg_dev, bool en)
{
	bq24296_set_en_term(0x01);
	return 0;
}

static int bq24296_charger_enable_otg(struct charger_device *chg_dev, bool en)
{
	if (en)
	{
		bq24296_set_otg_config(0x1); /* OTG */
		bq24296_set_boostv(0x7); /* boost voltage 4.998V */
		bq24296_set_boost_lim(0x1); /* 1.5A on VBUS */
	}else{
		bq24296_set_otg_config(0);
		}

	return 0;
}

static int bq24296_charger_is_charging_done(struct charger_device *chg_dev,
					   bool *done)
{
//ÖØÐÂÐ´
	return 0;
}

static int bq24296_charger_dump_registers(struct charger_device *chg_dev)
{
	bq24296_dump_register();
	return 0;
}

static int bq24296_charger_do_event(struct charger_device *chg_dev, u32 event,
				   u32 args)
{
	struct bq24296_info *ri = charger_get_data(chg_dev);
	
	switch (event) {
	case EVENT_EOC:
		chr_info("do eoc event\n");
		charger_dev_notify(ri->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		chr_info("do recharge event\n");
		charger_dev_notify(ri->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return 0;
}


static const struct charger_ops bq24296_chg_ops = {
	/* cable plug in/out */
	.plug_in = bq24296_charger_plug_in,
	.plug_out = bq24296_charger_plug_out,
	/* enable */
	.enable = bq24296_charging_enable,
	.is_enabled = bq24296_charger_is_enabled,
	/* charging current */
	.get_charging_current = bq24296_charger_get_ichg,
	.set_charging_current = bq24296_charger_set_ichg,
	.get_min_charging_current = bq24296_charger_get_min_ichg,
	/* charging voltage */
	.set_constant_voltage = bq24296_charger_set_cv,
	.get_constant_voltage = bq24296_charger_get_cv,
	/* charging input current */
	.get_input_current = bq24296_charger_get_aicr,
	.set_input_current = bq24296_charger_set_aicr,
	.get_min_input_current = bq24296_charger_get_min_aicr,
	/* charging mivr */
	.set_mivr = bq24296_charger_set_mivr,
	/* safety timer */
	.is_safety_timer_enabled = bq24296_charger_is_timer_enabled,
	.enable_safety_timer = bq24296_charger_enable_timer,
	/* charing termination */
	.enable_termination = bq24296_charger_enable_te,
	/* OTG */
	.enable_otg = bq24296_charger_enable_otg,
	/* misc */
	.is_charging_done = bq24296_charger_is_charging_done,
	.dump_registers = bq24296_charger_dump_registers,
	/* event */
	.event = bq24296_charger_do_event,
};

static const struct charger_properties bq24296_chg_props = {
	.alias_name = "bq24296",
};
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */

static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq24296_platform_data *pdata = dev_get_platdata(&client->dev);
	struct bq24296_info *ri = NULL;
	bool use_dt = client->dev.of_node;
	int err = 0;
	pr_err("[bq24296_driver_probe]\n");

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	ri = devm_kzalloc(&client->dev, sizeof(*ri), GFP_KERNEL);
	if (!ri)
	{
		return -ENOMEM;
	}
	/* platform data */
	if (use_dt) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		memcpy(pdata, &bq24296_def_platform_data, sizeof(*pdata));
		client->dev.platform_data = pdata;
	} else {
		if (!pdata) {
			pr_err("no pdata specify\n");
			return -EINVAL;
		}
	}
	
	new_client = client;
	ri->dev = &client->dev;
	ri->i2c = client;
	ri->chip_rev = 0;
	i2c_set_clientdata(client, ri);
	/* --------------------- */
	if(!bq24296_hw_component_detect()){
		pr_err("bq24296 is not exist\n");
		err = -1;
		goto exit;
	}

  charging_hw_init();

#if defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
		/* charger class register */
		ri->chg_dev = charger_device_register(pdata->chg_name, ri->dev, ri,
							  &bq24296_chg_ops,
							  &bq24296_chg_props);
		if (IS_ERR(ri->chg_dev)) {
			pr_err("charger device register fail\n");
			return PTR_ERR(ri->chg_dev);
		}
#endif /* #if (CONFIG_MTK_GAUGE_VERSION == 30) */

	bq24296_dump_register();
	chargin_hw_init_done = 1;

	return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq24296 = 0;
static ssize_t show_bq24296_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;
	int offset = 0;
	
	bq24296_dump_register();
	for(i = 0; i<bq24296_REG_NUM; i++)
	{
	    offset += sprintf(buf+offset, "reg[0x%x] = 0x%x\n", i, bq24296_reg[i]);
	}

	return offset;
}

static ssize_t store_bq24296_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	pr_err("[store_bq24296_access]\n");

	if (buf != NULL && size != 0) {
		pr_err("[store_bq24296_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			pr_err("[store_bq24296_access] write bq24296 reg 0x%x with value 0x%x !\n",reg_address, reg_value);
			ret = bq24296_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq24296_read_interface(reg_address, &g_reg_value_bq24296, 0xFF, 0x0);
			pr_err("[store_bq24296_access] read bq24296 reg 0x%x with value 0x%x !\n",reg_address, g_reg_value_bq24296);
			pr_err("[store_bq24296_access] Please use \"cat bq24296_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq24296_access, 0664, show_bq24296_access, store_bq24296_access);	/* 664 */

static int bq24296_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	pr_err("******** bq24296_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24296_access);

	return 0;
}

struct platform_device bq24296_user_space_device = {
	.name = "bq24296-user",
	.id = -1,
};

static struct platform_driver bq24296_user_space_driver = {
	.probe = bq24296_user_space_probe,
	.driver = {
		   .name = "bq24296-user",
		   },
};

static int __init bq24296_subsys_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&bq24296_driver) != 0)
		pr_err("[bq24261_init] failed to register bq24261 i2c driver.\n");
	else
		pr_err("[bq24261_init] Success to register bq24261 i2c driver.\n");

	/* bq24296 user space access interface */
	ret = platform_device_register(&bq24296_user_space_device);
	if (ret) {
		pr_err("****[bq24296_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq24296_user_space_driver);
	if (ret) {
		pr_err("****[bq24296_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq24296_exit(void)
{
	i2c_del_driver(&bq24296_driver);
}

/* module_init(bq24296_init); */
/* module_exit(bq24296_exit); */
subsys_initcall(bq24296_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24296 Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
