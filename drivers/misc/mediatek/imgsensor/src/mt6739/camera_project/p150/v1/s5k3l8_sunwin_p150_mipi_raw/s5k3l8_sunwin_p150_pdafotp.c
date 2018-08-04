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

/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "cam_cal.h"
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k3l8_sunwin_p150_rear_mipi_raw.h"

#if 1
typedef struct stCAM_CAL_INFO_STRUCT {
        u32 u4Offset;
        u32 u4Length;
        u32 sensorID;
        u32 deviceID;/* MAIN = 0x01, SUB  = 0x02, MAIN_2 = 0x04, SUB_2 = 0x08 */
        u8 *pu1Params;
}stCAM_CAL_INFO_STRUCT;
#endif

/* #define CAM_CALGETDLT_DEBUG //test */
//#define CAM_CAL_DEBUG //test */
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB pr_debug
//#define CAM_CALDB pr_err
#else
#define CAM_CALDB(x, ...)
#endif

#define CAM_CAL_DRVNAME "S5K3L8_SUNWIN_P150_CAL_DRV"
#define CAM_CAL_DEV_MAJOR_NUMBER 226
static DEFINE_SPINLOCK(g_CAM_CALLock); /* for SMP */

/* 81 is used for V4L driver */
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER, 0);
static struct cdev *g_pCAM_CAL_CharDrv;
/* static spinlock_t g_CAM_CALLock; */
static struct class *CAM_CAL_class;
static atomic_t g_CAM_CALatomic;

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms)          mdelay(ms)

/**************  CONFIG BY SENSOR >>> ************/
#define EEPROM_WRITE_ID   0xa0
#define I2C_SPEED         300

#define lsc_size  1888//1868 
#define pdaf_size 2048

kal_uint8 S5K3L8_SUNWIN_P150_CheckID[]= {0x10,0xff,0x00,0x40,0x89};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor */
	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, EEPROM_WRITE_ID);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    kdSetI2CSpeed(I2C_SPEED); // Add this func to set i2c speed by each sensor */
    iWriteRegI2C(pusendcmd , 4, EEPROM_WRITE_ID);
}

s5k3l8_sunwin_p150_otp_info  s5k3l8_sunwin_otp_info;
bool s5k3l8_sunwin_p150_read_eeprom_data(void)
{


	s5k3l8_sunwin_otp_info.info_awb_flag = read_cmos_sensor_byte(0x0000);
	if(s5k3l8_sunwin_otp_info.info_awb_flag == 0x01){
		//module id
		s5k3l8_sunwin_otp_info.module_id = read_cmos_sensor_byte(0x0001);
		//year month day
		s5k3l8_sunwin_otp_info.year = read_cmos_sensor_byte(0x0002);
		s5k3l8_sunwin_otp_info.month = read_cmos_sensor_byte(0x0003);
		s5k3l8_sunwin_otp_info.day = read_cmos_sensor_byte(0x0004);
		//lens id
		s5k3l8_sunwin_otp_info.lens_id = read_cmos_sensor_byte(0x0005);
		//awb
		s5k3l8_sunwin_otp_info.rg_gain = ((read_cmos_sensor_byte(0x000A) << 8) & 0xff00) | (read_cmos_sensor_byte(0x000B) & 0xff);
		s5k3l8_sunwin_otp_info.bg_gain = ((read_cmos_sensor_byte(0x000C) << 8) & 0xff00) | (read_cmos_sensor_byte(0x000D) & 0xff);
		s5k3l8_sunwin_otp_info.gg_gain = ((read_cmos_sensor_byte(0x000E) << 8) & 0xff00) | (read_cmos_sensor_byte(0x000F) & 0xff);
		s5k3l8_sunwin_otp_info.golden_rg = ((read_cmos_sensor_byte(0x0010) << 8) & 0xff00) | (read_cmos_sensor_byte(0x0011) & 0xff);
		s5k3l8_sunwin_otp_info.golden_bg = ((read_cmos_sensor_byte(0x0012) << 8) & 0xff00) | (read_cmos_sensor_byte(0x0013) & 0xff);
		s5k3l8_sunwin_otp_info.golden_gg = ((read_cmos_sensor_byte(0x0014) << 8) & 0xff00) | (read_cmos_sensor_byte(0x0015) & 0xff);

		memset(s5k3l8_sunwin_otp_info.awb_unit_data,0,4);
		s5k3l8_sunwin_otp_info.awb_unit_data[0] = s5k3l8_sunwin_otp_info.rg_gain*512/1024; //CalR
		s5k3l8_sunwin_otp_info.awb_unit_data[1] = 512; //CalGr
		s5k3l8_sunwin_otp_info.awb_unit_data[2] = s5k3l8_sunwin_otp_info.gg_gain*512/1024; //CalGb
		s5k3l8_sunwin_otp_info.awb_unit_data[3] = s5k3l8_sunwin_otp_info.bg_gain*512/1024; //CalB

		memset(s5k3l8_sunwin_otp_info.awb_golden_data,0,4);
		s5k3l8_sunwin_otp_info.awb_golden_data[0] = s5k3l8_sunwin_otp_info.golden_rg*512/1024; //FacR
		s5k3l8_sunwin_otp_info.awb_golden_data[1] = 512; //FacGr
		s5k3l8_sunwin_otp_info.awb_golden_data[2] = s5k3l8_sunwin_otp_info.golden_gg*512/1024; //FacGb
		s5k3l8_sunwin_otp_info.awb_golden_data[3] = s5k3l8_sunwin_otp_info.golden_bg*512/1024; //FacB
	}
	else {
		CAM_CALDB("s5k3l8_sunwin_p150_read_eeprom_data info_awb is empty or invalid\n");
		//return false;
	}

	CAM_CALDB("s5k3l8_sunwin_otp_info module_id = 0x%x,lens_id = 0x%x\n",s5k3l8_sunwin_otp_info.module_id,s5k3l8_sunwin_otp_info.lens_id);
	CAM_CALDB("s5k3l8_sunwin_otp_info year = %d,month = %d,day = %d\n",  s5k3l8_sunwin_otp_info.year,s5k3l8_sunwin_otp_info.month,s5k3l8_sunwin_otp_info.day);
	CAM_CALDB("s5k3l8_sunwin_otp_info rg_gain = 0x%x,bg_gain = 0x%x,gg_gain = 0x%x\n",
							s5k3l8_sunwin_otp_info.rg_gain,s5k3l8_sunwin_otp_info.bg_gain,s5k3l8_sunwin_otp_info.gg_gain);
	CAM_CALDB("s5k3l8_sunwin_otp_info golden_rg = 0x%x,golden_bg = 0x%x,golden_gg = 0x%x\n",
							s5k3l8_sunwin_otp_info.golden_rg,s5k3l8_sunwin_otp_info.golden_bg,s5k3l8_sunwin_otp_info.golden_gg);
							
	//AF data		 				
	s5k3l8_sunwin_otp_info.af_flag = read_cmos_sensor_byte(0x0036);
	if(s5k3l8_sunwin_otp_info.af_flag == 0x01){
		s5k3l8_sunwin_otp_info.af_infinity = ((read_cmos_sensor_byte(0x003B) << 8) & 0xff00) | (read_cmos_sensor_byte(0x003C) & 0xff);
		s5k3l8_sunwin_otp_info.af_macro = ((read_cmos_sensor_byte(0x0037) << 8) & 0xff00) | (read_cmos_sensor_byte(0x0038) & 0xff);
		
	}else {
		CAM_CALDB("s5k3l8_sunwin_otp_info af data is empty or invalid\n");
	}
	CAM_CALDB("s5k3l8_sunwin_otp_info af_infinity = 0x%x,af_macro = 0x%x\n",s5k3l8_sunwin_otp_info.af_infinity,s5k3l8_sunwin_otp_info.af_macro);
	
	//lsc data
	uint32_t i, j = 0,check_sum = 0;
	memset(s5k3l8_sunwin_otp_info.lsc_data,0,lsc_size);
	s5k3l8_sunwin_otp_info.lsc_flag = read_cmos_sensor_byte(0x0040);
	if(s5k3l8_sunwin_otp_info.lsc_flag == 0x01){
		for(i = 0x0041; i <= 0x078C; i++){
			s5k3l8_sunwin_otp_info.lsc_data[j] =  read_cmos_sensor_byte(i);
			check_sum += s5k3l8_sunwin_otp_info.lsc_data[j];
			j++;
		}
	}
	else{
		CAM_CALDB("s5k3l8_sunwin_otp_info lsc data is empty or invalid\n");
		
	}
	#if 0
	for(i =0; i++; i <lsc_size){
		CAM_CALDB("s5k3l8_sunwin_otp_info s5k3l8_sunwin_otp_info.lsc_data[%d] = %d\n",i,s5k3l8_sunwin_otp_info.lsc_data[i]);
	}
	CAM_CALDB("s5k3l8_sunwin_otp_info 0x078D = %d\n",read_cmos_sensor_byte(0x078D));
	#endif
	if(read_cmos_sensor_byte(0x078D) == check_sum % 256){
		CAM_CALDB("s5k3l8_sunwin_otp_info lsc data is suceess\n");
	}
	//pdaf data
	check_sum = 0;
	j = 0;
	memset(s5k3l8_sunwin_otp_info.pdaf_data,0,pdaf_size );
	s5k3l8_sunwin_otp_info.pdaf_flag = read_cmos_sensor_byte(0x0790);
	if(s5k3l8_sunwin_otp_info.pdaf_flag == 0x01){
		for(i = 0x0791; i <= 0x0D0C; i++){
			s5k3l8_sunwin_otp_info.pdaf_data[j] =  read_cmos_sensor_byte(i);
			check_sum += s5k3l8_sunwin_otp_info.pdaf_data[j];
			j++;
		}
	}
	else{
		CAM_CALDB("s5k3l8_sunwin_p150_read_eeprom_data pdaf data is empty or invalid\n");
		
	}
	#if 0
	for(i = 0; i++; i < pdaf_size){
		CAM_CALDB("s5k3l8_sunwin_p150_read_eeprom_data s5k3l8_sunwin_otp_info.pdaf_data[%d] = %d\n",i,s5k3l8_sunwin_otp_info.pdaf_data[i]);
	}
	CAM_CALDB("s5k3l8_sunwin_p150_read_eeprom_data 0x0D0D = %d\n",read_cmos_sensor_byte(0x0D0D));
	#endif
	if(read_cmos_sensor_byte(0x0D0D) == check_sum % 256){
		CAM_CALDB("s5k3l8_sunwin_p150_read_eeprom_data pdaf data is suceess\n");
	}
	
	return true;
}

#if  0
#define GAIN_DEFAULT      0x0100
#define RG_Ratio_typical  0x118		
#define BG_Ratio_typical  0x100

bool s5k3l8_sunwin_p150_otp_wb_update(void)
{
	kal_uint16 R_GAIN;
	kal_uint16 B_GAIN;
	kal_uint16 G_GAIN;
	kal_uint16 G_gain_R;
	kal_uint16 G_gain_B;

	CAM_CALDB("s5k3l8_sunwin_p150_otp_wb_update start\n");
	CAM_CALDB("s5k3l8_sunwin_p150_otp_wb R.Gr_Ratio = 0x%x,s5k3l8_sunwin_otp_info.bg_gain =0x%x",s5k3l8_sunwin_otp_info.rg_gain,s5k3l8_sunwin_otp_info.bg_gain);

	if(!s5k3l8_sunwin_otp_info.rg_gain || !s5k3l8_sunwin_otp_info.bg_gain||!BG_Ratio_typical ||!RG_Ratio_typical)
	{
		CAM_CALDB("s5k3l8_sunwin_p150_otp_wb: OTP WB ratio Data Err!\n");
		return false;
	}
	if(s5k3l8_sunwin_otp_info.bg_gain < BG_Ratio_typical)
	{
		if(s5k3l8_sunwin_otp_info.rg_gain < RG_Ratio_typical)
		{
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = GAIN_DEFAULT * BG_Ratio_typical / s5k3l8_sunwin_otp_info.bg_gain;
			R_GAIN = GAIN_DEFAULT * RG_Ratio_typical / s5k3l8_sunwin_otp_info.rg_gain;
		}
		else
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = GAIN_DEFAULT * s5k3l8_sunwin_otp_info.rg_gain / RG_Ratio_typical;
			B_GAIN = G_GAIN * BG_Ratio_typical / s5k3l8_sunwin_otp_info.bg_gain;	        
		}
	}
	else
	{
		if(s5k3l8_sunwin_otp_info.rg_gain < RG_Ratio_typical)
		{
			B_GAIN = GAIN_DEFAULT;
			G_GAIN = GAIN_DEFAULT * s5k3l8_sunwin_otp_info.bg_gain / BG_Ratio_typical;
			R_GAIN = G_GAIN * RG_Ratio_typical / s5k3l8_sunwin_otp_info.rg_gain;
		}
		else
		{
			G_gain_B = GAIN_DEFAULT* s5k3l8_sunwin_otp_info.bg_gain / BG_Ratio_typical;
			G_gain_R = GAIN_DEFAULT* s5k3l8_sunwin_otp_info.rg_gain / RG_Ratio_typical;

			if(G_gain_B > G_gain_R)
			{
				B_GAIN = GAIN_DEFAULT;
				G_GAIN = G_gain_B;
				R_GAIN = G_GAIN * RG_Ratio_typical / s5k3l8_sunwin_otp_info.rg_gain;
			}
			else
			{
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = G_gain_R;
				B_GAIN = G_GAIN * BG_Ratio_typical / s5k3l8_sunwin_otp_info.bg_gain;
			}	        
		}		
	}
	CAM_CALDB("s5k3l8_sunwin_p150_otp_wb:[R_GAIN=0x%x],[G_GAIN=0x%x],[B_GAIN=0x%x] \n",R_GAIN, G_GAIN, B_GAIN);

	write_cmos_sensor(0x6028,0x4000);
	
	write_cmos_sensor(0x3058, 0x01);
	
	write_cmos_sensor(0x020e, G_GAIN);
	//write_cmos_sensor(0x020f, G_GAIN&0xff);

	write_cmos_sensor(0x0210, R_GAIN);
	//write_cmos_sensor(0x0211, R_GAIN&0xff);
	 
	write_cmos_sensor(0x0212, B_GAIN);
	//write_cmos_sensor(0x0213, B_GAIN&0xff);
	  
	write_cmos_sensor(0x0214, G_GAIN);
	//write_cmos_sensor(0x0215, G_GAIN&0xff);
	
	CAM_CALDB("s5k3l8_sunwin_p150_otp_wb_update end\n");    
	return true;;
}
#endif
/********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
		struct file *a_pstFile,
		unsigned int a_u4Command,
		unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
		struct file *file,
		unsigned int a_u4Command,
		unsigned long a_u4Param
		)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pWorkingBuff = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;

	#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
	#endif
	if (_IOC_DIR(a_u4Command) != _IOC_NONE) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (pBuff == NULL) {
			CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff, (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/* get input structure address */
				kfree(pBuff);
				CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	pWorkingBuff = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (pWorkingBuff == NULL) {
		kfree(pBuff);
		CAM_CALDB("[CAM_CAL] ioctl allocate mem failed\n");
		return -ENOMEM;
	}

	if (copy_from_user((u8 *)pWorkingBuff, (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pWorkingBuff);
		CAM_CALDB("[CAM_CAL] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
		case CAM_CALIOC_S_WRITE:
			CAM_CALDB("[CAM_CAL] Write CMD\n");
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv1);
			#endif
			//i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if (ktv2.tv_sec > ktv1.tv_sec)
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			else
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

			CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
			#endif
			break;
		case CAM_CALIOC_G_READ:
			CAM_CALDB("[CAM_CAL] Read CMD\n");
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv1);
			#endif
			CAM_CALDB("[CAM_CAL] offset %d\n", ptempbuf->u4Offset);
			CAM_CALDB("[CAM_CAL] length %d\n", ptempbuf->u4Length);

			//i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			if(ptempbuf->u4Length == 2)
			{	
				if(ptempbuf->u4Offset == 0x07){
					memcpy(pWorkingBuff,&s5k3l8_sunwin_otp_info.af_infinity,ptempbuf->u4Length);
					CAM_CALDB("[pWorkingBuff] = %d, af_infinity = %d\n", *pWorkingBuff,s5k3l8_sunwin_otp_info.af_infinity);
				}
				else if(ptempbuf->u4Offset == 0x09) {
					memcpy(pWorkingBuff,&s5k3l8_sunwin_otp_info.af_macro,ptempbuf->u4Length);
					CAM_CALDB("[pWorkingBuff] = %d, af_macro = %d\n", *pWorkingBuff,s5k3l8_sunwin_otp_info.af_macro);
				}					
			}
			else if (ptempbuf->u4Length == 4){
				 memcpy(pWorkingBuff,&S5K3L8_SUNWIN_P150_CheckID[1],ptempbuf->u4Length);
			}
			else if(ptempbuf->u4Length == 8)
			{
				if(ptempbuf->u4Offset == 0x10){
					memcpy(pWorkingBuff,s5k3l8_sunwin_otp_info.awb_unit_data,ptempbuf->u4Length);
				}
				else if(ptempbuf->u4Offset == 0x14) {
					memcpy(pWorkingBuff,s5k3l8_sunwin_otp_info.awb_golden_data,ptempbuf->u4Length);
				}
			}
			else if (ptempbuf->u4Length == 1868){
				memcpy(pWorkingBuff,s5k3l8_sunwin_otp_info.lsc_data,ptempbuf->u4Length);

			}
			else if (ptempbuf->u4Length == 1404){
				memcpy(pWorkingBuff,s5k3l8_sunwin_otp_info.pdaf_data,ptempbuf->u4Length);

			}
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if (ktv2.tv_sec > ktv1.tv_sec)
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			else
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

			CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
			#endif

			break;
		default:
			CAM_CALDB("[CAM_CAL] No CMD\n");
			i4RetValue = -EPERM;
			break;
	}
	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params, (u8 *)pWorkingBuff, ptempbuf->u4Length)) {
			
			kfree(pBuff);
			kfree(pWorkingBuff);
			CAM_CALDB("[CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pWorkingBuff);
	return i4RetValue;
}

static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	int ret = 0;

	CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		ret = -EBUSY;
	} else {
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
		ret = 0;
	}
	spin_unlock(&g_CAM_CALLock);

	/* #if defined(MT6572) */
	/* do nothing */
	/* #else */
	/* if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "S24CS64A")) */
	/* { */
	/* CAM_CALDB("[CAM_CAL] Fail to enable analog gain\n"); */
	/* return -EIO; */
	/* } */
	/* #endif */

	return ret;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/* .ioctl = CAM_CAL_Ioctl */
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

	#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
	#else
	if (register_chrdev_region(g_CAM_CALdevno, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
	#endif

	/* Allocate driver */
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (g_pCAM_CAL_CharDrv == NULL) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/* Attatch file operation. */
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[CAM_CAL] Attatch file operation failed\n");
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, "S5K3L8_SUNWIN_P150_CAL_DRV");
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);

		CAM_CALDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}
static int CAM_CAL_probe(struct platform_device *pdev)
{
	//return i2c_add_driver(&CAM_CAL_i2c_driver);
	return 0;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	//i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}

/* platform structure */
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe      = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver     = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};

static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init S5K3L8_SUNWIN_P150_CAM_CAL_init(void)
{
	//i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
	int i4RetValue = 0;
	CAM_CALDB("S5K3L8_CAM_CAL]\n");
	
	/* Register char driver */
	i4RetValue = RegisterCAM_CALCharDrv();
	if (i4RetValue) {
		CAM_CALDB("[CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}
	CAM_CALDB("[S5K3L8_CAM_CAL] Attached!! \n");

	if(platform_driver_register(&g_stCAM_CAL_Driver)){
		CAM_CALDB("failed to register CAM_CAL driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("failed to register CAM_CAL driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit S5K3L8_SUNWIN_P150_CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(S5K3L8_SUNWIN_P150_CAM_CAL_init);
module_exit(S5K3L8_SUNWIN_P150_CAM_CAL_exit);

MODULE_DESCRIPTION("S5K3L8_SUNWIN_P150_CAM_CAL driver");
MODULE_AUTHOR("tinno");
MODULE_LICENSE("GPL");

