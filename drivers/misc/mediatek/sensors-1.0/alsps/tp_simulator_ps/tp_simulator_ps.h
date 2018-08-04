#ifndef __tp_simulator_ps_h__
#define __tp_simulator_ps_h__

#define SIMULATOR_PS_DEVICE "simulator_ps"

extern int simulator_ps_report_data(int dat);
extern int is_simulator_ps_drv_init_ok(int (*func_s)(int s));
extern int set_simulator_ps_callback(int (*enable_nodata)(int en), int (*get_data)(int *ps_value, int *status));
extern int sort_load_by_drv_name(struct alsps_init_info **a, char *s, int l, int d);

struct simulator_ps_cb {
    int (*func_s)(int support);
	int (*enable_nodata)(int *ps_value);
	int (*get_data)(int *ps_value, int *status);
    int init_ok;
};


#define SIMULATOR_PS_REPORT_DATA(d) \
do \
{ \
    simulator_ps_report_data(d); \
} while(0)


#define SET_SIMULATOR_PS_CB(e, g) \
do \
{ \
    set_simulator_ps_callback(e, g); \
} while(0)


#define IS_SIMULATOR_PS_INIT_OK(r, cb) \
do \
{ \
    *r = is_simulator_ps_drv_init_ok(cb); \
} while(0)


#define SORT_LOAD_BY_NAME(i, k, l) \
do \
{ \
  sort_load_by_drv_name(i, k, l, 1); \
} while(0)






#endif
