#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/miscdevice.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

/*----------------------------------------------------------------------------*/
#define TN_SENSORS_STATE_IOCTL_TYPE        ('s')
#define TN_SENSORS_STATE_IOCTL_MARKET_AREA        _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 0, int)
#define TN_SENSORS_STATE_IOCTL_PSENSOR_EXISTS    _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 1, int)
#define TN_SENSORS_STATE_IOCTL_LSENSOR_EXISTS    _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 2, int)
#define TN_SENSORS_STATE_IOCTL_GYRO_EXISTS        _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 3, int)
#define TN_SENSORS_STATE_IOCTL_ASENSOR_EXISTS    _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 4, int)
#define TN_SENSORS_STATE_IOCTL_MSENSOR_EXISTS    _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 5, int)
#define TN_SENSORS_STATE_IOCTL_OTG_EXISTS        _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 6, int)
#define TN_SENSORS_STATE_IOCTL_HALL_EXISTS        _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 7, int)
#define TN_SENSORS_STATE_IOCTL_STEPCOUNTER_EXISTS        _IOR(TN_SENSORS_STATE_IOCTL_TYPE, 8, int)
/*----------------------------------------------------------------------------*/

#define MARKET_VAULE_LEN 24
extern char Market_Area[MARKET_VAULE_LEN];
extern int g_iProximity_sensor;
extern int g_iLight_sensor;
extern int g_iGyroscope_sensor;
extern int g_iAcceleration_sensor;
extern int g_iMagnetic_sensor;
extern int g_iOTG_open;
extern int g_iHall_sensor;
extern int g_iStepcounter_sensor;
/*----------------------------------------------------------------------------*/
static int tn_sensors_state_open(struct inode *inode, struct file *filp);
static int tn_sensors_state_release(struct inode *inode, struct file *filp);
static long tn_sensors_state_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

static int tn_sensors_state_probe(struct platform_device *pdev);
static int tn_sensors_state_remove(struct platform_device *pdev);
static ssize_t tn_read_sensors_state(struct device *dev, struct device_attribute *attr, char *buf);

/*static char m_sensors_state_info[8];


static ssize_t tn_read_sensors_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(strlen(m_sensors_state_info) !=0)
	{
		return sprintf(buf, "%s", m_sensors_state_info);    
	}
	return sprintf(buf, "%s", "unknow"); 
}

static DEVICE_ATTR(tn_sensors_state_info, 0444, tn_read_sensors_state, NULL);*/

static const struct file_operations tn_sensors_state_fops = {
    .owner = THIS_MODULE,
    .open = tn_sensors_state_open,
    .release = tn_sensors_state_release,
    .unlocked_ioctl = tn_sensors_state_ioctl,
};

static struct miscdevice md = {
    .minor   = MISC_DYNAMIC_MINOR,
    .name    = "tn_sensors_state",
    .fops    = &tn_sensors_state_fops,
};

/*----------------------------------------------------------------------------*/
static int tn_sensors_state_open(struct inode *inode, struct file *filp)
{
    return 0;
}

/*----------------------------------------------------------------------------*/
static int tn_sensors_state_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/*----------------------------------------------------------------------------*/
static long tn_sensors_state_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret;
    printk(KERN_INFO "##TINNO cmd = %d [%s]\n", cmd, __func__);
    
    switch (cmd) {
    case TN_SENSORS_STATE_IOCTL_MARKET_AREA:
        if (strlen(Market_Area) != 0) {
            ret = copy_to_user((char *)arg, Market_Area, strlen(Market_Area));
        }
        printk(KERN_INFO "##ioc_ret=%d, Market_Area = %s [%s]\n", ret, Market_Area, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_PSENSOR_EXISTS:
        ret = copy_to_user((int)arg, &g_iProximity_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iProximity_sensor = %d [%s]\n", ret, g_iProximity_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_LSENSOR_EXISTS:
        ret = copy_to_user((int)arg, &g_iLight_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iLight_sensor = %d [%s]\n", ret, g_iLight_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_GYRO_EXISTS:
        ret = copy_to_user((int)arg, &g_iGyroscope_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iGyroscope_sensor = %d [%s]\n", ret, g_iGyroscope_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_ASENSOR_EXISTS:
        ret = copy_to_user((int)arg, &g_iAcceleration_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iAcceleration_sensor = %d [%s]\n", ret, g_iAcceleration_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_MSENSOR_EXISTS:
        ret = copy_to_user((int)arg, &g_iMagnetic_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iMagnetic_sensor = %d [%s]\n", ret, g_iMagnetic_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_OTG_EXISTS:
        ret = copy_to_user((int)arg, &g_iOTG_open, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iOTG_open = %d [%s]\n", ret, g_iOTG_open, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_HALL_EXISTS:
        ret = copy_to_user((int)arg, &g_iHall_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iHall_sensor = %d [%s]\n", ret, g_iHall_sensor, __func__);
        break;
    case TN_SENSORS_STATE_IOCTL_STEPCOUNTER_EXISTS:
        ret = copy_to_user((int)arg, &g_iStepcounter_sensor, sizeof(int));
        printk(KERN_INFO "##ioc_ret=%d, g_iStepcounter_sensor = %d [%s]\n", ret, g_iStepcounter_sensor, __func__);
        break;
    default:
        break;
    }

    if(ret){
        printk(KERN_INFO "cmd=%d, copy_to_user err [%s]\n", cmd, __func__);
        return -1;
    }   
    return 0;
}


/*----------------------------------------------------------------------------*/
static int __init tn_sensors_state_init(void)
{
    int ret;
    printk("%s\n", __func__);    
    printk(KERN_INFO "TINNO sensor_state Market_Area = %s\n",Market_Area);
    printk(KERN_INFO "TINNO sensor_state g_iProximity_sensor = %d\n",g_iProximity_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iLight_sensor = %d\n",g_iLight_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iGyroscope_sensor = %d\n",g_iGyroscope_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iAcceleration_sensor = %d\n",g_iAcceleration_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iMagnetic_sensor = %d\n",g_iMagnetic_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iOTG_open = %d\n",g_iOTG_open);
    printk(KERN_INFO "TINNO sensor_state g_iHall_sensor = %d\n",g_iHall_sensor);
    printk(KERN_INFO "TINNO sensor_state g_iStepcounter_sensor = %d\n",g_iStepcounter_sensor);

    ret = misc_register(&md);
    if (ret) {
        printk("Failed to register sensors state device\n");
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit tn_sensors_state_exit(void)
{
    int ret;

    printk("%s\n", __func__);
    misc_deregister(&md);
    //if (ret) {
    //    printk("Failed to deregister sensors state device\n");
    //}
}

/*static int tn_sensors_state_probe(struct platform_device *pdev){

    device_create_file(&pdev->dev, &dev_attr_tn_sensors_state_info);
    return 0;
}

static int tn_sensors_state_remove(struct platform_device *pdev){
	device_remove_file(&pdev->dev, &dev_attr_tn_sensors_state_info);
	return 0;
}*/
/*----------------------------------------------------------------------------*/
module_init(tn_sensors_state_init);
module_exit(tn_sensors_state_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TN Sensors state dummy driver");
MODULE_AUTHOR("TN Mobile");

// TINNO END
