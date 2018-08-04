
//#include "s5k4h8mipi_Sensor.h"
//#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/module.h>

//#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/fs.h>

//#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "kd_imgsensor.h" 
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_typedef.h"

#include "s5k4h8mipi_Sensor.h"

int s5k4h8_flag_otp=0x00;
int s5k4h8_VCMID=0x00;
int s5k4h8_DriverICID=0x00;
int s5k4h8_RGr_ratio=0x00;
int s5k4h8_BGr_ratio=0x00;
int s5k4h8_GbGr_ratio=0x00;
int s5k4h8_VCM_start=0x00;
int s5k4h8_VCM_end=0x00;


//struct Darling_S5K4H8_otp_struct *S5K4H8_OTP =NULL;
u16 s5k4h8_af_data[2];
#define s5k4h8_RGr_ratio_Typical  600	//modify by v3702 
#define s5k4h8_BGr_ratio_Typical  721  //modify by v3702
#define s5k4h8_GbGr_ratio_Typical  1019

#define LOG_TAG  "S5K4H8:"
extern  void S5K4H8_wordwrite_cmos_sensor(u16 addr, kal_uint16 para);
extern  void S5K4H8_bytewrite_cmos_sensor(u16 addr, kal_uint16 para);
extern kal_uint16 S5K4H8_byteread_cmos_sensor(u16 addr);
#define SENSORDB_S5K4H8_OTP(fmt,arg...)  printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)

static bool s5k4h8_start_read_otp_daling(void)
{
    BYTE val = 0;
    int i;
    //S5K4H8_wordwrite_cmos_sensor(0xFCFC, 0xD000);
    S5K4H8_bytewrite_cmos_sensor(0x0A02,0x0f); // 0x0f Select the page to write by writing to 0xD0000A02 0x01~0x0C
    S5K4H8_bytewrite_cmos_sensor(0x0A00, 0x01); //Enter read mode by writing 01h    to 0xD0000A00
	mdelay(15);

   val = S5K4H8_byteread_cmos_sensor(0x0A04);
	SENSORDB_S5K4H8_OTP(" read 0x0A04 = 0x %x\n,",val);
	//SENSORDB_S5K4H8_OTP(" read val = 0x %x\n,",val);

	 if(val == 0x01){
	 	
		return 1;
	 }

	 else{

 		val = S5K4H8_byteread_cmos_sensor(0x0A1A);
        SENSORDB_S5K4H8_OTP(" read 0x0A1A = 0x %x\n,",val);

		//SENSORDB_S5K4H8_OTP(" read val = 0x %x\n,",val);
	 if(val == 0x01){
	 		
			return 1;
		 }
	}
		
    return 0;
}



/*************************************************************************************************
* Function : stop_read_otp_sunny
* Description : after read otp_sunny , stop and reset otp_sunny block setting
**************************************************************************************************/
 void s5k4h8_stop_read_otp_daling(void)
{
    S5K4H8_bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface bywriting 00h to 0xD0000A00
}


void Darling_S5K4H8_read_OTP(void)
{

		int i = 0;
		unsigned char val = 0;
		int a26=0,a27=0,a28=0,a29=0,a30=0,a31=0,a32=0,a33=0,a34=0,a35=0,a36=0,a37=0,a38=0,a39=0,a=0;
		SENSORDB_S5K4H8_OTP(" Darling_S5K4H8_read_OTP \n");
		S5K4H8_bytewrite_cmos_sensor(0x0A02,0x0f);
		S5K4H8_bytewrite_cmos_sensor(0x0A00,0x01);
        mdelay(15);

		for(i;i<=8;i++){
		     val=S5K4H8_byteread_cmos_sensor(0x0A04);//flag of info and awb
		     if(val == 0x01) break;
		}

		SENSORDB_S5K4H8_OTP(" read 0x0A04 = 0x%x\n",val);
		
	if(val == 0x01)
	{
		s5k4h8_flag_otp = 0x01;
		S5K4H8_byteread_cmos_sensor(0x0A05);
		S5K4H8_byteread_cmos_sensor(0x0A06);
		S5K4H8_byteread_cmos_sensor(0x0A07);
		S5K4H8_byteread_cmos_sensor(0x0A08);
		S5K4H8_byteread_cmos_sensor(0x0A09);
		S5K4H8_byteread_cmos_sensor(0x0A0A);
		S5K4H8_byteread_cmos_sensor(0x0A0B);
		s5k4h8_RGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A0D)<<8)|S5K4H8_byteread_cmos_sensor(0x0A0C);
		s5k4h8_BGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A0F)<<8)|S5K4H8_byteread_cmos_sensor(0x0A0E);
		s5k4h8_GbGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A11)<<8)|S5K4H8_byteread_cmos_sensor(0x0A10);
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A05) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A05));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A06) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A06));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A09) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A09));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A0A) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A0A));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A0C) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A0C));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A0D) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A0D));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A0E) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A0E));	
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A0F) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A0F));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A10) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A10));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A11) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A11));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL s5k4h8_RGr_ratio= 0x%x, s5k4h8_BGr_ratio = 0x%x, s5k4h8_GbGr_ratio = 0x%0x",s5k4h8_RGr_ratio,s5k4h8_BGr_ratio,s5k4h8_GbGr_ratio);
		//SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8 s5k4h8_af_data[0]= 0x%x, s5k4h8_af_data[1] = 0x%x",(S5K4H8_byteread_cmos_sensor(0x0A32)<<8)|S5K4H8_byteread_cmos_sensor(0x0A31),(S5K4H8_byteread_cmos_sensor(0x0A34)<<8)|S5K4H8_byteread_cmos_sensor(0x0A33));

	}
	else if(val != 0x00)
	{
	val=S5K4H8_byteread_cmos_sensor(0x0A1A);//flag of info and awb
	SENSORDB_S5K4H8_OTP(" read 0x0A1A = 0x %x\n,",val);
	if(val == 0x01)
		{
		s5k4h8_flag_otp += 0x01;
	
		a26	=    S5K4H8_byteread_cmos_sensor(0x0A26);
		a27	= 	 S5K4H8_byteread_cmos_sensor(0x0A27);
		a28	= 	 S5K4H8_byteread_cmos_sensor(0x0A28);
		a29	= 	 S5K4H8_byteread_cmos_sensor(0x0A29);
		a30	= 	 S5K4H8_byteread_cmos_sensor(0x0A2A);
		a31	= 	 S5K4H8_byteread_cmos_sensor(0x0A2B);
		a32	= 	 S5K4H8_byteread_cmos_sensor(0x0A2C);

		a39 =    S5K4H8_byteread_cmos_sensor(0x0A2D);

		a33 =    S5K4H8_byteread_cmos_sensor(0x0A2E);
		a34 =    S5K4H8_byteread_cmos_sensor(0x0A2F);
		a35 =    S5K4H8_byteread_cmos_sensor(0x0A30);
		a36 =    S5K4H8_byteread_cmos_sensor(0x0A31);
		a37 =    S5K4H8_byteread_cmos_sensor(0x0A32);
		a38 =    S5K4H8_byteread_cmos_sensor(0x0A33);


		 s5k4h8_RGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A23)<<8)|S5K4H8_byteread_cmos_sensor(0x0A22);
		 s5k4h8_BGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A25)<<8)|S5K4H8_byteread_cmos_sensor(0x0A24);
		 s5k4h8_GbGr_ratio =(S5K4H8_byteread_cmos_sensor(0x0A27)<<8)|S5K4H8_byteread_cmos_sensor(0x0A26);


		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A2E) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A22));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A2F) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A23));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A30) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A24));	
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A31) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A25));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A32) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A26));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL S5K4H8_byteread_cmos_sensor(0x0A33) = 0x%x",S5K4H8_byteread_cmos_sensor(0x0A27));
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL 22 s5k4h8_RGr_ratio= 0x%x, s5k4h8_BGr_ratio = 0x%x, s5k4h8_GbGr_ratio = 0x%0x\n",s5k4h8_RGr_ratio,s5k4h8_BGr_ratio,s5k4h8_GbGr_ratio);	 
		}
	else if (val!= 0X01)
		s5k4h8_flag_otp += 0x00;
	}
	
	val=S5K4H8_byteread_cmos_sensor(0x0A30);//falg of VCM
	SENSORDB_S5K4H8_OTP(" read VCM flag 0x0A30 = 0x %x\n,",val);
	if(val == 0x01)
	{
		s5k4h8_flag_otp += 0x01;
		s5k4h8_af_data[0] =(S5K4H8_byteread_cmos_sensor(0x0A34)<<8)|S5K4H8_byteread_cmos_sensor(0x0A33);
		s5k4h8_af_data[1]=(S5K4H8_byteread_cmos_sensor(0x0A32)<<8)|S5K4H8_byteread_cmos_sensor(0x0A31);

		printk("S5K4H8 S5K4H8_byteread_cmos_sensor(0x0A31)= 0x%x, S5K4H8_byteread_cmos_sensor(0x0A32) = 0x%x\n",S5K4H8_byteread_cmos_sensor(0x0A31),S5K4H8_byteread_cmos_sensor(0x0A32));
		printk("S5K4H8 S5K4H8_byteread_cmos_sensor(0x0A33)= 0x%x, S5K4H8_byteread_cmos_sensor(0x0A34) = 0x%x\n",S5K4H8_byteread_cmos_sensor(0x0A33),S5K4H8_byteread_cmos_sensor(0x0A34));
		printk("S5K4H8 s5k4h8_af_data[0]= 0x%x, s5k4h8_af_data[1] = 0x%x\n",s5k4h8_af_data[0],s5k4h8_af_data[1]);	
	}
	else if(val != 0x01)
	{
		val=S5K4H8_byteread_cmos_sensor(0x0A37);//falg of VCM
		SENSORDB_S5K4H8_OTP(" read VCM flag 0x0A37 = 0x %x\n,",val);
		if(val == 0x01)
		{
			s5k4h8_flag_otp += 0x01;
			s5k4h8_af_data[0] =(S5K4H8_byteread_cmos_sensor(0x0A3B)<<8)|S5K4H8_byteread_cmos_sensor(0x0A3A);
			s5k4h8_af_data[1] =(S5K4H8_byteread_cmos_sensor(0x0A39)<<8)|S5K4H8_byteread_cmos_sensor(0x0A38);
		
			printk("S5K4H8 S5K4H8_byteread_cmos_sensor(0x0A38)= 0x%x, S5K4H8_byteread_cmos_sensor(0x0A39) = 0x%x\n",S5K4H8_byteread_cmos_sensor(0x0A38),S5K4H8_byteread_cmos_sensor(0x0A39));
			printk("S5K4H8 S5K4H8_byteread_cmos_sensor(0x0A3B)= 0x%x, S5K4H8_byteread_cmos_sensor(0x0A37) = 0x%x\n",S5K4H8_byteread_cmos_sensor(0x0A3A),S5K4H8_byteread_cmos_sensor(0x0A3B));
			printk("S5K4H8 s5k4h8_af_data[0]= 0x%x, s5k4h8_af_data[1] = 0x%x\n",s5k4h8_af_data[0],s5k4h8_af_data[1]);
		}
		else if(val != 0x01)
			s5k4h8_flag_otp += 0x00;
	}

	s5k4h8_stop_read_otp_daling();

}

void Darling_S5K4H8_apply_OTP(void){

		SENSORDB_S5K4H8_OTP(" Darling_S5K4H8_apply_OTP s5k4h8_flag_otp = %d \n",s5k4h8_flag_otp);
	
	if(((s5k4h8_flag_otp)&0x03) == 0x00){ 

		SENSORDB_S5K4H8_OTP(" Darling_S5K4H8_apply_OTP  err:  \n");
		return;

	}

	
		int R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;
		R_gain = (s5k4h8_RGr_ratio_Typical*1000) /s5k4h8_RGr_ratio;
		B_gain = (s5k4h8_BGr_ratio_Typical*1000) /s5k4h8_BGr_ratio;
		Gb_gain = (s5k4h8_GbGr_ratio_Typical*1000) /s5k4h8_GbGr_ratio;
		Gr_gain = 1000;
		Base_gain = R_gain;
		if(Base_gain>B_gain) Base_gain=B_gain;
		if(Base_gain>Gb_gain) Base_gain=Gb_gain;
		if(Base_gain>Gr_gain) Base_gain=Gr_gain;
		R_gain = 0x100 * R_gain / Base_gain;
		B_gain = 0x100 * B_gain / Base_gain;
		Gb_gain = 0x100 * Gb_gain / Base_gain;
		Gr_gain = 0x100 * Gr_gain / Base_gain;
		SENSORDB_S5K4H8_OTP("HYNIX_CAM_CAL 33 R_gain= 0x%x, B_gain = 0x%x, Gb_gain = 0x%x, Gr_gain = 0x%x",R_gain,B_gain,Gb_gain,Gr_gain);	

	if(Gr_gain>0x100)
	{
		S5K4H8_bytewrite_cmos_sensor(0x020E,Gr_gain>>8);
		S5K4H8_bytewrite_cmos_sensor(0x020F,Gr_gain&0xff);
	}
	if(R_gain>0x100)
	{
		S5K4H8_bytewrite_cmos_sensor(0x0210,R_gain>>8);
		S5K4H8_bytewrite_cmos_sensor(0x0211,R_gain&0xff);
	}
	if(B_gain>0x100)
	{
		S5K4H8_bytewrite_cmos_sensor(0x0212,B_gain>>8);
		S5K4H8_bytewrite_cmos_sensor(0x0213,B_gain&0xff);
	}
	if(Gb_gain>0x100)
	{
		S5K4H8_bytewrite_cmos_sensor(0x0214,Gb_gain>>8);
		S5K4H8_bytewrite_cmos_sensor(0x0215,Gb_gain&0xff);
		
	}

	SENSORDB_S5K4H8_OTP(" update otp ok \n,");
	
}



	/*************************************************************************************************
* Function : otp_daling_update_wb()
* Description : update otp_sunny data from otp_sunny , it otp_sunny data is valid,
it include get ID and WB update function
* Return : [bool] 0 : update fail
1 : update success
**************************************************************************************************/
bool s5k4h8_otp_daling_update_wb(void)
{
    BYTE zone = 0x01;
    BYTE FLG = 0x00;
    BYTE MID = 0x00;
	//BYTE LENS_ID= 0x00,VCM_ID= 0x00;
    int i;
    
      if(!s5k4h8_start_read_otp_daling()){

		  s5k4h8_stop_read_otp_daling();
	  
           SENSORDB_S5K4H8_OTP(" No otp_daling Data or otp_daling data is invalid!!");
		   return 0;
      	}

	s5k4h8_stop_read_otp_daling();
    SENSORDB_S5K4H8_OTP(" statrt read otp  \n");

    Darling_S5K4H8_read_OTP();
	SENSORDB_S5K4H8_OTP(" statrt write otp  \n");
	Darling_S5K4H8_apply_OTP();
	SENSORDB_S5K4H8_OTP("  write otp end \n");
    return 1;
}

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 2
#define S5K4H8_DEVICE_ID 0x20
#define CAM_CAL_DEV_MAJOR_NUMBER 226

/*******************************************************************************
 *
 ********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "S5K4H8_CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
 *
 ********************************************************************************/
/* fix warning MSG 
static unsigned short g_pu2Normal_i2c[] = {S5K4H8_DEVICE_ID , I2C_CLIENT_END};
static unsigned short g_u2Ignore = I2C_CLIENT_END;
static struct i2c_client_address_data g_stCAM_CAL_Addr_data = {
    .normal_i2c = g_pu2Normal_i2c,
    .probe = &g_u2Ignore,
    .ignore = &g_u2Ignore
}; */

static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x20>>1)}; //make dummy_eeprom co-exist

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
/*******************************************************************************
 *
 ********************************************************************************/

u8 S5K4H8_CheckID[]= {0x10,0xbc,0x00,0x40,0x88};



// maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes
int s5k4h8_iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] iReadCAM_CAL!! \n");

    if(ui4_length > 8)
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation\n");
        return -1;
    }
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = g_pstI2Cclient->addr & (0x00FF | 0X1000);	
    spin_unlock(&g_CAM_CALLock); // for SMP
    
    //SENSORDB_S5K4H8_OTP("[EERPOM] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
    if (i4RetValue != 2)
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    //SENSORDB_S5K4H8_OTP("[EERPOM] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
    if (i4RetValue != ui4_length)
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] I2C read data failed!! \n");
        return -1;
    }
    spin_lock(&g_CAM_CALLock); //for SMP
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & 0x00FF;
    spin_unlock(&g_CAM_CALLock); // for SMP    
	
    SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] iReadCAM_CAL done!! \n");
    return 0;
}

//int iReadData(stCAM_CAL_INFO_STRUCT * st_pOutputBuffer)
static int s5k4h8_iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;
   SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] iReadData \n" );
   
   if (ui4_offset + ui4_length >= 0x2000)
   {
      SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Read Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
      return -1;
   }

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;
   do 
   {
       if(i4ResidueDataLength >= 8)
       {
           i4RetValue = s5k4h8_iReadCAM_CAL((u16)u4CurrentOffset, 8, pBuff);
           if (i4RetValue != 0)
           {
                SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }           
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
       }
       else
       {
           i4RetValue = s5k4h8_iReadCAM_CAL((u16)u4CurrentOffset, i4ResidueDataLength, pBuff);
           if (i4RetValue != 0)
           {
                SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] I2C iReadData failed!! \n");
                return -1;
           }  
           u4IncOffset += 8;
           i4ResidueDataLength -= 8;
           u4CurrentOffset = ui4_offset + u4IncOffset;
           pBuff = pinputdata + u4IncOffset;
           //break;
       }
   }while (i4ResidueDataLength > 0);
//fix warning MSG   SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
   SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] iReadData done\n" );
   return 0;
}

static int s5k4h8_selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    
    printk("[s5k4h8_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);

	if(size ==2 ){
		if(offset == 7){
	  	
		memcpy((void *)data,(void *)&s5k4h8_af_data[0],size);
		printk("s5k4h8_af_data[0] = 0x%x\n", s5k4h8_af_data[0]);
		}
	
	else if(offset ==9){
		memcpy((void *)data,(void *)&s5k4h8_af_data[1],size);
		printk("s5k4h8_af_data[1] = 0x%x\n",s5k4h8_af_data[1]);

		}
	}

	if(size == 4){
		memcpy((void *)data,(void *)&S5K4H8_CheckID[1],size);

	}
	printk("+ls.test[S5K4H8_CAM_CAL]  data1 = %d\n",*(UINT32 *)data);
}

#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
		struct file * a_pstFile,
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
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    struct stCAM_CAL_INFO_STRUCT *ptempbuf = NULL;

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(struct stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(struct stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (struct stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
//fix warning MSG     SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] init Working buffer address 0x%x  command is 0x%08x\n", pWorkingBuff, a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {


		 case CAM_CALIOC_S_WRITE:
    		SENSORDB_S5K4H8_OTP("[SENSORDB_S5K4H8_OTP] CAM_CALIOC_S_WRITE \n");        
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv1);
			#endif
			i4RetValue = 0;
			// i4RetValue=iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if(ktv2.tv_sec > ktv1.tv_sec)
			{
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			}
			else
			{
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
			}
			#endif
			break;

	
      
        case CAM_CALIOC_G_READ:
            SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Read CMD\n");
			#ifdef CAM_CALGETDLT_DEBUG            
			do_gettimeofday(&ktv1);
			#endif 
            SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] length %d \n", ptempbuf->u4Length);
          // SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Before read Working buffer address 0x%x \n", pWorkingBuff);


			if(ptempbuf->u4Length == 2){

				i4RetValue = s5k4h8_selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);

			}else if(ptempbuf->u4Length == 4){

				i4RetValue = s5k4h8_selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);
			}        

			// LOG_INF("[GC8024_CXT_K300__CAM_CAL] After read Working buffer address 0x%x \n", pWorkingBuff);


			#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if(ktv2.tv_sec > ktv1.tv_sec)
			{
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			}
			else
			{
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
			}
			printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
			#endif            

            break;
        default :
      	     SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        	break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] to user length %d \n", ptempbuf->u4Length);
//fix warning MSG        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] to user  Working buffer address 0x%x \n", pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

	kfree(pBuff);
	kfree(pWorkingBuff);
	return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int s5k4h8_CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

	//#if defined(MT6572)
	// do nothing
	//#else
	//if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "S24CS64A"))
	//{
	//    LOG_INF("[GC8024_CXT_K300__CAM_CAL] Fail to enable analog gain\n");
	//    return -EIO;
	//}
	//#endif	

	return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int s5k4h8_CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic,0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
	.owner = THIS_MODULE,
	.open = s5k4h8_CAM_CAL_Open,
	.release = s5k4h8_CAM_CAL_Release,
	//.ioctl = CAM_CAL_Ioctl
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
	struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	//Allocate driver
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if(NULL == g_pCAM_CAL_CharDrv)
	{
		unregister_chrdev_region(g_CAM_CALdevno, 1);

        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	//Attatch file operation.
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }
 
    CAM_CAL_class = class_create(THIS_MODULE, "S5K4H8_CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        SENSORDB_S5K4H8_OTP("Unable to create class, err = %d\n", ret);
        return ret;
    }
	
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

	return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
	//Release char driver
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

	return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	//i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe		= CAM_CAL_probe,
	.remove	= CAM_CAL_remove,
	.driver		= {
		.name	= CAM_CAL_DRVNAME,
		.owner	= THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_i2C_init(void)
{

	// i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);


	int i4RetValue = 0;
	SENSORDB_S5K4H8_OTP("[S5K4H8_CAM_CAL]\n");
	//Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
	if(i4RetValue){
		SENSORDB_S5K4H8_OTP(" [S5K4H8_CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}
	SENSORDB_S5K4H8_OTP(" [S5K4H8_CAM_CAL] Attached!! \n");


	if(platform_driver_register(&g_stCAM_CAL_Driver)){
		printk("failed to register CAM_CAL driver\n");
		return -ENODEV;
	}
	if (platform_device_register(&g_stCAM_CAL_Device))
	{
		printk("failed to register CAM_CAL driver\n");
		return -ENODEV;
	}	
	SENSORDB_S5K4H8_OTP(" S5K4H8_CAM_CAL  Attached Pass !! \n");
	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);
