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
    LCM_PRINT ("[ili9881c] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
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
	                      {0xFF, 3,  {0x98,0x81,0x03}},
						{0x01,1,{0x00}},
						{0x02,1,{0x00}},
						{0x03,1,{0x53}},
						{0x04,1,{0x13}},
						{0x05,1,{0x00}},
						{0x06,1,{0x04}},
						{0x07,1,{0x00}},
						{0x08,1,{0x00}},
						{0x09,1,{0x14}},
						{0x0a,1,{0x14}},
						{0x0b,1,{0x00}},
						{0x0c,1,{0x01}},
						{0x0d,1,{0x00}},
						{0x0e,1,{0x00}},
						{0x0f,1,{0x14}},
						{0x10,1,{0x14}},
						{0x11,1,{0x00}},
						{0x12,1,{0x00}},
						{0x13,1,{0x00}},
						{0x14,1,{0x00}},
						{0x15,1,{0x00}},
						{0x16,1,{0x00}},
						{0x17,1,{0x00}},
						{0x18,1,{0x00}},
						{0x19,1,{0x00}},
						{0x1a,1,{0x00}},
						{0x1b,1,{0x00}},
						{0x1c,1,{0x00}},
						{0x1d,1,{0x00}},
						{0x1e,1,{0x44}},
						{0x1f,1,{0x80}},
						{0x20,1,{0x02}},
						{0x21,1,{0x03}},
						{0x22,1,{0x00}},
						{0x23,1,{0x00}},
						{0x24,1,{0x00}},
						{0x25,1,{0x00}},
						{0x26,1,{0x00}},
						{0x27,1,{0x00}},
						{0x28,1,{0x33}},
						{0x29,1,{0x03}},
						{0x2a,1,{0x00}},
						{0x2b,1,{0x00}},
						{0x2c,1,{0x00}},
						{0x2d,1,{0x00}},
						{0x2e,1,{0x00}},
						{0x2f,1,{0x00}},
						{0x30,1,{0x00}},
						{0x31,1,{0x00}},
						{0x32,1,{0x00}},
						{0x33,1,{0x00}},
						{0x34,1,{0x04}},
						{0x35,1,{0x00}},
						{0x36,1,{0x00}},
						{0x37,1,{0x00}},
						{0x38,1,{0x3C}},
						{0x39,1,{0x00}},
						{0x3a,1,{0x40}},
						{0x3b,1,{0x40}},
						{0x3c,1,{0x00}},
						{0x3d,1,{0x00}},
						{0x3e,1,{0x00}},
						{0x3f,1,{0x00}},
						{0x40,1,{0x00}},
						{0x41,1,{0x00}},
						{0x42,1,{0x00}},
						{0x43,1,{0x00}},
						{0x44,1,{0x00}},
						{0x50,1,{0x01}},
						{0x51,1,{0x23}},
						{0x52,1,{0x45}},
						{0x53,1,{0x67}},
						{0x54,1,{0x89}},
						{0x55,1,{0xab}},
						{0x56,1,{0x01}},
						{0x57,1,{0x23}},
						{0x58,1,{0x45}},
						{0x59,1,{0x67}},
						{0x5a,1,{0x89}},
						{0x5b,1,{0xab}},
						{0x5c,1,{0xcd}},
						{0x5d,1,{0xef}},
						{0x5e,1,{0x11}},
						{0x5f,1,{0x01}},
						{0x60,1,{0x00}},
						{0x61,1,{0x15}},
						{0x62,1,{0x14}},
						{0x63,1,{0x0C}},
						{0x64,1,{0x0D}},
						{0x65,1,{0x0E}},
						{0x66,1,{0x0F}},
						{0x67,1,{0x06}},
						{0x68,1,{0x02}},
						{0x69,1,{0x02}},
						{0x6a,1,{0x02}},
						{0x6b,1,{0x02}},
						{0x6c,1,{0x02}},
						{0x6d,1,{0x02}},
						{0x6e,1,{0x08}},
						{0x6f,1,{0x02}},
						{0x70,1,{0x02}},
						{0x71,1,{0x02}},
						{0x72,1,{0x02}},
						{0x73,1,{0x02}},
						{0x74,1,{0x02}},
						{0x75,1,{0x01}},
						{0x76,1,{0x00}},
						{0x77,1,{0x15}},
						{0x78,1,{0x14}},
						{0x79,1,{0x0C}},
						{0x7a,1,{0x0D}},
						{0x7b,1,{0x0E}},
						{0x7c,1,{0x0F}},
						{0x7D,1,{0x08}},
						{0x7E,1,{0x02}},
						{0x7F,1,{0x02}},
						{0x80,1,{0x02}},
						{0x81,1,{0x02}},
						{0x82,1,{0x02}},
						{0x83,1,{0x02}},
						{0x84,1,{0x06}},
						{0x85,1,{0x02}},
						{0x86,1,{0x02}},
						{0x87,1,{0x02}},
						{0x88,1,{0x02}},
						{0x89,1,{0x02}},
						{0x8A,1,{0x02}},

						{0xFF, 3,  {0x98,0x81,0x04}},					
						{0x6C,1,{0x15}},
						{0x6E,1,{0x2B}},
						{0x6F,1,{0x35}},
						{0x35,1,{0x1F}},
						{0x3A,1,{0x24}},
						{0x8D,1,{0x14}},  ////////
						{0x87,1,{0xBA}},
						{0x26,1,{0x76}},
						{0xB2,1,{0xD1}},
						{0xB5,1,{0x06}},
						{0x33,1,{0x14}},

						{0xFF, 3,  {0x98,0x81,0x01}},
						{0x22,1,{0x0A}},
						{0x31,1,{0x00}},
						{0x53,1,{0x8B}},
						{0x50,1,{0xC7}},
						{0x51,1,{0xC4}},
						{0x60,1,{0x1C}},
						{0x62,1,{0x00}},
						{0x63,1,{0x00}},
						{0x2E,1,{0xF0}},
						{0xA0,1,{0x00}},
						{0xA1,1,{0x32}},
						{0xA2,1,{0x41}},
						{0xA3,1,{0x15}},
						{0xA4,1,{0x18}},
						{0xA5,1,{0x2B}},
						{0xA6,1,{0x1F}},
						{0xA7,1,{0x20}},
						{0xA8,1,{0xA8}},
						{0xA9,1,{0x1B}},
						{0xAA,1,{0x28}},
						{0xAB,1,{0x8C}},
						{0xAC,1,{0x19}},
						{0xAD,1,{0x18}},
						{0xAE,1,{0x4C}},
						{0xAF,1,{0x21}},
						{0xB0,1,{0x28}},
						{0xB1,1,{0x57}},
						{0xB2,1,{0x67}},
						{0xB3,1,{0x3F}},
						{0xC0,1,{0x00}},
						{0xC1,1,{0x32}},
						{0xC2,1,{0x41}},
						{0xC3,1,{0x15}},
						{0xC4,1,{0x18}},
						{0xC5,1,{0x2B}},
						{0xC6,1,{0x1F}},
						{0xC7,1,{0x20}},
						{0xC8,1,{0xA8}},
						{0xC9,1,{0x1B}},
						{0xCA,1,{0x28}},
						{0xCB,1,{0x8C}},
						{0xCC,1,{0x19}},
						{0xCD,1,{0x18}},
						{0xCE,1,{0x4C}},
						{0xCF,1,{0x21}},
						{0xD0,1,{0x28}},
						{0xD1,1,{0x57}},
						{0xD2,1,{0x67}},
						{0xD3,1,{0x3F}},
						{0xFF, 3,  {0x98,0x81,0x00}},
						{0x35, 1,  {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY,150,{}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY,50,{}},				
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

		//bingqian.tang date20160413 add for GGAFMA-394
		#ifdef  CONFIG_PROJECT_P6601_WIK_FR
        	params->physical_width  = 62.10; //62.10;
        	params->physical_height = 110.40; //110.40;
		#endif
		//bingqian.tang end

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

		params->dsi.horizontal_sync_active = 8;
		params->dsi.horizontal_backporch = 24;
		params->dsi.horizontal_frontporch = 20;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=257;  //LINE </EGAFM-297> <change the mipi clock to reduce disturbing the wifi> <20160413> panzaoyan
		params->dsi.ssc_disable=1;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 0; 
	params->dsi.customization_esd_check_enable = 0;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;

	params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
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
    MDELAY(20); // 1ms
    #if defined(CONFIG_PROJECT_C300)
	lcm_enn(0);
	MDELAY(1);
	lcm_enp(0);
   #endif
    SET_RESET_PIN(1);
    MDELAY(120);
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
	if (1==lcd_hw_id)
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
		{
				LCM_DBG();
				return 1;
		}
		else
		{
				LCM_DBG();
				return 0;
		}
		}
		return 0;
#endif
}

LCM_DRIVER ili9881c_c300_hdplus_dsi_vdo_hsd_lcm_drv = 
{
	.name		= "ili9881c_hdplus_dsi_vdo_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

