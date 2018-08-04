/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_proximity.c
*
*    Author: luoguojin
*
*   Created: 2016-09-19
*
*  Abstract: close proximity function
*
*   Version: v1.0
*
* Revision History:
*        v1.0:
*            First release based on xiaguobin's solution. By luougojin 2016-08-19
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include <alsps.h>
#if FTS_PSENSOR_EN
#include <hwmsensor.h>
//#include <hwmsen_dev.h>
#include <sensors_io.h>

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct fts_proximity_st {
    u8      mode                : 1;    /* 1- proximity enable 0- disable */
    u8      detect              : 1;    /* 0-->close ; 1--> far away */
    u8      unused              : 4;
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_proximity_st fts_proximity_data;
struct i2c_client *ps_client;
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern int ps_report_interrupt_data(int value);
/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_enter_proximity_mode(struct i2c_client *client, int mode);
static ssize_t fts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
/*****************************************************************************
* functions body
*****************************************************************************/


/* read and write proximity mode
*   read example: cat  fts_touch_proximity_mode---read  proximity mode
*   write example:echo 01 > fts_touch_proximity_mode ---write proximity mode to 01
*/
static DEVICE_ATTR (fts_touch_proximity_mode, S_IRUGO | S_IWUSR, fts_touch_proximity_show, fts_touch_proximity_store);
static struct attribute *fts_touch_proximity_attrs[] = {
    &dev_attr_fts_touch_proximity_mode.attr,
    NULL,
};

static struct attribute_group fts_touch_proximity_group = {
    .attrs = fts_touch_proximity_attrs,
};


static ssize_t fts_touch_proximity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    mutex_lock(&fts_data->input_dev->mutex);
    return snprintf(buf, PAGE_SIZE, "Proximity: %s\n", fts_proximity_data.mode ? "On" : "Off");
    mutex_unlock(&fts_data->input_dev->mutex);
}

static ssize_t fts_touch_proximity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    int ret;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);

    mutex_lock(&fts_data->input_dev->mutex);
    val = simple_strtoul(buf, 0, 10);
    if (val == 1) {
        if (!fts_proximity_data.mode) {
            fts_proximity_data.mode = 1;
            ret = fts_enter_proximity_mode(client, 1);
        }
    } else {
        if (fts_proximity_data.mode) {
            fts_proximity_data.mode = 0;
            ret = fts_enter_proximity_mode(client, 0);
        }
    }
    mutex_unlock(&fts_data->input_dev->mutex);

    return count;
}

/************************************************************************
* Name: fts_enter_proximity_mode
* Brief:  change proximity mode
* Input:  proximity mode
* Output: no
* Return: success =0
***********************************************************************/
static int fts_enter_proximity_mode(struct i2c_client *client, int mode)
{
    int ret = 0;
    u8 buf_addr = 0;
    u8 buf_value = 0;

    buf_addr = FTS_REG_FACE_DEC_MODE_EN;
    if (mode)
        buf_value = 0x01;
    else
        buf_value = 0x00;

    ret = fts_i2c_write_reg(client, buf_addr, buf_value);
    if (ret < 0) {
        FTS_ERROR("[PROXIMITY] Write proximity register(0xB0) fail!");
        return ret;
    }

    fts_proximity_data.mode = buf_value ? 1 : 0;
    FTS_DEBUG("[PROXIMITY] proximity mode = %d", fts_proximity_data.mode);

    return ret ;
}

/*****************************************************************************
*  Name: fts_proximity_recovery
*  Brief: need call when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_recovery(struct i2c_client *client)
{
    int ret = 0;

    if (fts_proximity_data.mode)
        ret = fts_enter_proximity_mode(client, 1);

    return ret;
}

/*****************************************************************************
*  Name: fts_ps_operate
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
/*static int fts_ps_operate(void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout)
{
    int err = 0;
    int value;
    struct hwm_sensor_data *sensor_data;

    FTS_DEBUG("[PROXIMITY]COMMAND = %d", command);
    switch (command) {
    case SENSOR_DELAY:
        if ((buff_in == NULL) || (size_in < sizeof(int))) {
            FTS_ERROR("[PROXIMITY]Set delay parameter error!");
            err = -EINVAL;
        }
        break;

    case SENSOR_ENABLE:
        if ((buff_in == NULL) || (size_in < sizeof(int))) {
            FTS_ERROR("[PROXIMITY]Enable sensor parameter error!");
            err = -EINVAL;
        } else {
            value = *(int *)buff_in;
            FTS_DEBUG("[PROXIMITY]SENSOR_ENABLE value = %d", value);
            
            err = fts_enter_proximity_mode(fts_data->client, value);
        }
        break;

    case SENSOR_GET_DATA:
        if ((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
            FTS_ERROR("[PROXIMITY]get sensor data parameter error!");
            err = -EINVAL;
        } else {
            sensor_data = (struct hwm_sensor_data *)buff_out;
            sensor_data->values[0] = (int)fts_proximity_data.detect;
            FTS_DEBUG("sensor_data->values[0] = %d", sensor_data->values[0]);
            sensor_data->value_divide = 1;
            sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
        }
        break;
    default:
        FTS_ERROR("[PROXIMITY]ps has no operate function:%d!", command);
        err = -EPERM;
        break;
    }

    return err;
}
*/
/*****************************************************************************
*  Name: fts_proximity_readdata
*  Brief:
*  Input:
*  Output:
*  Return: 0 - need return in suspend
*****************************************************************************/
int fts_proximity_readdata(struct i2c_client *client)
{
    //int ret;
    int proximity_status = 1;
    u8  regvalue;
    struct hwm_sensor_data sensor_data;



    if (fts_proximity_data.mode == 0)
        return -EPERM;


    fts_i2c_read_reg(client, FTS_REG_FACE_DEC_MODE_STATUS, &regvalue);

    if (regvalue == 0xC0) {
        /* close. need lcd off */
        proximity_status = 0;
    } else if (regvalue == 0xE0) {
        /* far away */
        proximity_status = 1;
    }
printk("geroge proximity_status = %d, (int)fts_proximity_data.detect= %d \n",proximity_status,(int)fts_proximity_data.detect);
    if (proximity_status != (int)fts_proximity_data.detect) {
        FTS_DEBUG("[PROXIMITY] p-sensor state:%s", proximity_status ? "AWAY" : "NEAR");
        fts_proximity_data.detect = proximity_status ? 1 : 0;
        sensor_data.values[0] = proximity_status;
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        //ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
       // if (ret) {
         //   FTS_ERROR("[PROXIMITY] call hwmsen_get_interrupt_data failed, ret=%d", ret);
        //    return ret;
       // }
//ps_report_interrupt_data(proximity_status);

SIMULATOR_PS_REPORT_DATA(fts_proximity_data.detect); 

        return 0;
    }

    return -1;
}

/*****************************************************************************
*  Name: fts_proximity_suspend
*  Brief: Run when tp enter into suspend
*  Input:
*  Output:
*  Return: 0 - need return in suspend
*****************************************************************************/
int fts_proximity_suspend(void)
{
    if (fts_proximity_data.mode == 1)
        return 0;
    else
        return -1;
}

/*****************************************************************************
*  Name: fts_proximity_resume
*  Brief: Run when tp resume
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_resume(void)
{
    if (fts_proximity_data.mode == 1)
        return 0;
    else
        return -1;
}

int tpd_read_ps(void)
{
    //fts_proximity_data.detect;
    return 0;
}

static int tpd_get_ps_value(void)
{
    return fts_proximity_data.detect;
}


/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
 int tp_ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	printk("####ps_open_report_data \n");
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

 int tp_ps_enable_nodata(int en)
{
	
	int value = 0;
	value = en;
	 printk("####geroge ps_enable_nodata\n");
        if(value)
        {
            if(fts_enter_proximity_mode(ps_client, 1) != 0)
            {
                printk("geroge enable ps fail\n");
               // return -1;
            }
        }
        else
        {
            if(fts_enter_proximity_mode(ps_client, 0) != 0)
            {
                printk("geroge disable ps fail\n");
                //return -1;
            }
        }
	return 0;
}

 int tp_ps_set_delay(u64 ns)
{
	return 0;
}

 int tp_ps_get_data(int *value, int *status)
{
	int err = 0;
        if((err = tpd_read_ps()))
        {
            err = -1;;
        }
        else
        {
            *value = tpd_get_ps_value();
            printk("geroge huang sensor_data->values[0] 1082 = %d\n", *value);
            *status = SENSOR_STATUS_ACCURACY_MEDIUM;
        }
	return 0;
}
 int tp_ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

 int tp_ps_flush(void)
{
	return 0;
}


/*****************************************************************************
*  Name: fts_proximity_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_init(struct i2c_client *client)
{
    int err = 0;
    //struct hwmsen_object obj_ps;



    FTS_FUNC_ENTER();

    memset((u8 *)&fts_proximity_data, 0, sizeof(struct fts_proximity_st));
    fts_proximity_data.detect = 1;  /* defalut far awway */

    err = sysfs_create_group(&client->dev.kobj, &fts_touch_proximity_group);
    if (0 != err) {
        FTS_ERROR("[PROXIMITY] Create sysfs node failed,ret=%d", err);
        sysfs_remove_group(&client->dev.kobj, &fts_touch_proximity_group);
        return err;
    }
ps_client=client;

   // obj_ps.polling = 0; /* interrupt mode */
   // obj_ps.sensor_operate = fts_ps_operate;
   // err = hwmsen_attach(ID_PROXIMITY, &obj_ps);
   // if (err)
   //     FTS_ERROR("[PROXIMITY]fts proximity attach fail = %d!", err);
   // else
    //    FTS_INFO("[PROXIMITY]fts proximity attach ok = %d\n", err);

fts_enter_proximity_mode(client, 1);
SET_SIMULATOR_PS_CB(tp_ps_enable_nodata, tp_ps_get_data);





    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: fts_proximity_exit
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
int fts_proximity_exit(struct i2c_client *client)
{
    sysfs_remove_group(&client->dev.kobj, &fts_touch_proximity_group);
    return 0;
}
#endif /* FTS_PSENSOR_EN */

