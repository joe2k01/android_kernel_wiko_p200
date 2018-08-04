#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1440)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT pr_debug
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[hx8394f_hsd] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
#if 1
#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif

#if defined(CONFIG_PROJECT_C300)
extern void lcm_enn(int onoff);
extern void lcm_enp(int onoff);
extern  int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value);
#endif

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

extern int get_lcm_id_status(void);

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_DELAY                                           0xFC

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table  lcm_deep_sleep_mode_in_setting_v2[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};

static struct LCM_setting_table lcm_initialization_setting_v2[] = 
{
	{0xB9, 3,  {0xFF,0x83,0x94}},
	{0xB1, 10, {0x48,0x11,0x71,0x09,0x32,0x44,0x71,0x51,0x55,0x30}},
	{0xBA, 6,  {0x63,0x03,0x68,0x6B,0xB2,0xC0}},
	{0xB2, 5,  {0x00,0x80,0x78,0x0C,0x07}},
	{0xB4, 21, {0x01,0x63,0x01,0x63,0x01,0x63,0x02,0x02,0x7C,0x75,0x00,0x3F,0x01,0x63,0x01,0x63,0x01,0x63,0x01,0x0C,0x7C}},
	{0xD3,33, {0x00,0x00,0x01,0x02,0x3C,0x1C,0x00,0x00,0x32,0x10,0x09,0x00,0x09,0x32,0x15,0xAD,0x05,0xAD,0x32,0x00,0x00,0x00,0x00,0x37,0x03,0x0B,0x0B,0x37,0x00,0x00,0x00,0x0C,0x40}},
	{0xD5, 44, {0x19,0x19,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x20,0x21,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x24,0x25,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xD6, 44, {0x18,0x18,0x19,0x19,0x1B,0x1B,0x1A,0x1A,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x25,0x24,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
	{0xE0,58, {0x00,0x05,0x0D,0x14,0x17,0x1A,0x1E,0x1D,0x3A,0x4A,0x59,0x59,0x62,0x75,0x7B,0x81,0x90,0x94,0x92,0x9F,0xB4,0x58,0x58,0x5D,0x67,0x6A,0x71,0x7A,0x7F,0x00,0x05,0x0D,0x14,0x17,0x1A,0x1E,0x1D,0x3A,0x4A,0x59,0x59,0x62,0x75,0x7B,0x81,0x90,0x94,0x92,0x9F,0xB4,0x58,0x58,0x5D,0x67,0x6A,0x71,0x7A,0x7F}},
	{0xCC, 1,  {0x03}},
	{0xC0, 2,  {0x1F,0x31}},
	{0xB6, 2,  {0xBE,0xBE}},
	{0xD4, 1,  {0x02}},
	{0xBD, 1,  {0x01}},
	{0xB1, 1,  {0x00}},
	{0xBD, 1,  {0x00}},
	{0xC6, 1,  {0xED}},    
	{0x35, 1,  {0x00}},
	{0x36, 1,  {0x02}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY,20,{}},				
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->physical_width  = 64.38;
        params->physical_height = 130.71; 

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active = 2;  // 4
		params->dsi.vertical_backporch = 16;  //22
		params->dsi.vertical_frontporch	= 9; // 36
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active = 32;  // 8
		params->dsi.horizontal_backporch = 48; // 24
		params->dsi.horizontal_frontporch = 60; // 20
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205; //
		params->dsi.PLL_CLOCK=257;  // 207 LINE </EGAFM-297> <change the mipi clock to reduce disturbing the wifi> <20160413> panzaoyan
		params->dsi.ssc_disable=1;

		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 0;//0:te esd check 1:read register
if (params->dsi.customization_esd_check_enable){
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	//params->dsi.lcm_esd_check_table[1].cmd = 0x0B;
//	params->dsi.lcm_esd_check_table[1].count = 1;
//	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
}
#endif /*BUILD_LK*/

// TINNO BEGIN
// huaidong.tan , CDAAAE-652 , DATE20171113 , porting mtk round corner
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->corner_pattern_width = 32;//720;//Note:这里是屏幕的宽度，不是原始图片宽度
	params->corner_pattern_height = 32;//圆角的高度
#endif
// TINNO END
}


static void lcm_init(void)
{
	LCM_DBG();
	SET_RESET_PIN(1);
	MDELAY(20);
#if defined(CONFIG_PROJECT_C300)
	lcm_enn(1);
	MDELAY(1);
	_lcm_i2c_write_bytes(0,0x0F);
	MDELAY(1);
	_lcm_i2c_write_bytes(1,0x0F);
	MDELAY(1);
	lcm_enp(1);
#endif
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120);
	LCM_DBG("jacky debug,lcm reset end \n");
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
	LCM_DBG("jacy debug,lcm init end \n");
}

static void lcm_suspend(void)
{	
	LCM_DBG();
    push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10); // 1ms
    #if defined(CONFIG_PROJECT_C300)
	lcm_enn(0);
	MDELAY(1);
	lcm_enp(0);
    #endif
    SET_RESET_PIN(1);
    MDELAY(50);
}

static void lcm_resume(void)
{
	#if defined(CONFIG_PROJECT_C300)
	lcm_enn(1);
	MDELAY(1);
	_lcm_i2c_write_bytes(0,0x0F);
	MDELAY(1);
	_lcm_i2c_write_bytes(1,0x0F);
	MDELAY(1);
	lcm_enp(1);
	#endif
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(5); 
	SET_RESET_PIN(1);
	MDELAY(10); 
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
	LCM_DBG();
}      

static unsigned int lcm_compare_id(void)
{
#if 1
	s32 lcd_hw_id = -1;
	lcd_hw_id = get_lcm_id_status();
	LCM_DBG("lcm_compare_id lcd_hw_id=%d \n",lcd_hw_id);
	if (0==lcd_hw_id)
	{
		return 1;
	}
	else
	{
		return 0;
	}
#else
	int data[4] = {0, 0, 0, 0};
	int tmp = 0 ,rc = 0, iVoltage = 0;
	rc = IMM_GetOneChannelValue(AUXADC_LCD_ID_CHANNEL, data, &tmp);
		if(rc < 0)
		{
			LCM_DBG("read LCD_ID vol error\n");
			return 0;
		}
		else
		{
			iVoltage = (data[0]*1000) + (data[1]*10) + (data[2]);
			LCM_DBG("data[0]=%d,data[1]=%d,data[2]=%d,data[3]=%d,iVoltage=%d\n",
					data[0], data[1], data[2], data[3], iVoltage);
		if((BOE_LCM_MIN_VOL <= iVoltage) && (iVoltage < BOE_LCM_MAX_VOL))
				return 1;
			else
				return 0;
		}
		return 0;
#endif
}

LCM_DRIVER hd8394_c300_hdplus_dsi_vdo_hsd_lcm_drv = 
{
	.name		= "hd8394_hdplus_dsi_vdo_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

