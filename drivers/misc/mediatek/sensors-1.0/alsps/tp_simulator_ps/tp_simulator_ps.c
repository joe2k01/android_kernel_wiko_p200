
#ifdef CONFIG_SIMULATOR_PS_SUPPORT

#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <alsps.h>
#include <tp_simulator_ps.h>

#define LOG_TAG  "[simulator_ps_drv]:" 
#define __FUN(f)   printk(KERN_ERR LOG_TAG "[ %s:%d ]\n", __func__,__LINE__)
#define klog(fmt, args...)    printk(KERN_ERR "[%s:%d]" LOG_TAG fmt, __func__,__LINE__,##args)

#define PRINT_SORT_LIST(a, l) \
do \
{ \
    int i; \
    for (i = 0; i < l; i++) { \
        if (a[i] != NULL) \
            klog("[%d]: %s\n", i, a[i]->name); \
    } \
} while(0)

///////////////////////////////////////////////////////////////////
static struct simulator_ps_cb  simulator_ps_cb_t;
static struct simulator_ps_cb  *st_ps_p = &simulator_ps_cb_t;
static int simulator_ps_local_init(void);
static int simulator_ps_remove(void);
static int simulator_ps_open_report_data(int open) { return 0; }
static int simulator_ps_set_delay(u64 ns) { return 0; }
static int simulator_ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs) { return 0; }
static int simulator_ps_flush(void) { return 0; }

static struct alsps_init_info simulator_ps_init_info = {
	.name = SIMULATOR_PS_DEVICE,
	.init = simulator_ps_local_init,
	.uninit = simulator_ps_remove
};

/** a: list */
/** s: key string */
/** l: list length */
/** d: debug */
int sort_load_by_drv_name(struct alsps_init_info **a, char *s, int l, int d)
{
    int i, j = 0;
    struct alsps_init_info *t = NULL;

    klog("entry sort\n");

    if (d) {
        klog("sort befor:\n");
        PRINT_SORT_LIST(a, l);
    }

    for (i = 0; i < l; i++) {
        if (a[i] && a[i]->name) {
            if (strcmp(a[i]->name, s) == 0) {
                t = a[i];
                for (j = i; j < l-1; j++) {
                    a[j] =  a[j+1];
                }
                a[j] = t;
                goto sort_done;
            }
        }
    }

sort_done:

    if (d) {
        klog("sort after:\n");
        PRINT_SORT_LIST(a, l);
    }

    return 0;
}


static int simulator_ps_enable_nodata(int en)
{
    klog("enter: en:%d\n", en);
    if (st_ps_p->enable_nodata != NULL) {
        return st_ps_p->enable_nodata(en);
    }
    return 0;
}

static int simulator_ps_get_data(int *value, int *status)
{
    if (st_ps_p && st_ps_p->init_ok == 1) {
        if (st_ps_p->get_data != NULL) {
            return st_ps_p->get_data(value, status);
        }
    }
    return 0;
}

int is_simulator_ps_drv_init_ok(int (*func_s)(int s)) 
{
    klog("enter: func_s:%pf\n", func_s);
    if (st_ps_p->init_ok == 1) {
        return 1;
    }

    if (st_ps_p->func_s == NULL && func_s != NULL) {
        st_ps_p->func_s = func_s;
    }
    else {
        klog("no cb, maybe has error.\n");
    }
    return 0;
}

int simulator_ps_report_data(int dat) 
{
    if (st_ps_p && st_ps_p->init_ok == 1) {
        return ps_report_interrupt_data(dat); // this func is defined by alsps.c.
    }
    return -1;
}

int set_simulator_ps_callback(int (*enable_nodata)(int en), int (*get_data)(int *ps_value, int *status))
{
    if (enable_nodata == NULL || get_data == NULL) {
        klog("err!\n");
        return -1;
    }

    klog("enter: enable_nodata:%pf, get_data:%pf\n", enable_nodata, get_data);
    if (st_ps_p->enable_nodata == NULL) {
        st_ps_p->enable_nodata = enable_nodata;
    }

    if (st_ps_p->get_data == NULL) {
        st_ps_p->get_data = get_data;
    }

    klog("exit\n");
    return 0;
}

static int simulator_ps_local_init(void)
{
    int err = 0;
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = {0};

    klog("enter\n");
    memset(&simulator_ps_cb_t, 0, sizeof(simulator_ps_cb_t));

	ps_ctl.is_use_common_factory = false;
	ps_ctl.open_report_data = simulator_ps_open_report_data;
	ps_ctl.enable_nodata = simulator_ps_enable_nodata;
	ps_ctl.set_delay = simulator_ps_set_delay;
	ps_ctl.batch = simulator_ps_batch;
	ps_ctl.flush = simulator_ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		klog("register control path failed = %d\n", err);
		goto probe_fail;
	}

    ps_data.vender_div = 100;
    ps_data.get_data = simulator_ps_get_data;
    err = ps_register_data_path(&ps_data);
    if (err) {
        klog("register data path failed = %d\n", err);
        goto probe_fail;
    }

    st_ps_p->init_ok = 1;
    klog( "set init ok flag.\n");
    if (st_ps_p->func_s != NULL) {
        klog( "callback to :%pf\n", st_ps_p->func_s);
        st_ps_p->func_s(1);
    }

    klog( "@@probe ok!\n");
    return 0;

probe_fail:
    return -1;
}

static int simulator_ps_remove(void)
{
    klog( " omg?\n");
    return 0;
}


static int __init simulator_ps_init(void)
{
    klog( " init\n");
	alsps_driver_add(&simulator_ps_init_info);
	return 0;
}

static void __exit simulator_ps_exit(void)
{
    klog( " exit\n");
}

///////////////////////////////////////////////////////////////////
module_init(simulator_ps_init);
module_exit(simulator_ps_exit);

//MODULE_LICENSE("GPL");
//MODULE_DESCRIPTION("tp_simulator_ps");
//MODULE_AUTHOR("<mingyi.guo@tinno.com>");

#endif

