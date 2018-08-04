#include <linux/module.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>

#define Market_AREA "androidboot.Market_Area="
#define SENSOR_INFO "androidboot.Sensor_Info="
#define PA_MODE_INFO "androidboot.Audio_Pa_Mode="

#define MARKET_VAULE_LEN 16
#define SENSOR_VALUE_LEN 24
#define PA_MODE_VALUE_LEN 1
#define MARKET_AREA_BUFFER_SIZE 128
#define SENSOR_INFO_BUFFER_SIZE 128
#define PA_MODE_INFO_BUFFER_SIZE 128
#ifndef MIN
    #define MIN(x, y)   (((x) <= (y)) ? (x) : (y))
#endif /* MIN */

extern char* saved_command_line;
char Market_Area[MARKET_VAULE_LEN] = {0};
int g_iProximity_sensor = 0;
int g_iLight_sensor = 0;
int g_iGyroscope_sensor = 0;
int g_iAcceleration_sensor = 0;
int g_iMagnetic_sensor = 0;
int g_iOTG_open = 0;
int g_iHall_sensor = 0;
int g_PaMode = 0;
int g_iStepcounter_sensor = 0;

static int __init tinno_sensors_config(void)
{
   char *p = NULL;
   int i;
   int j = 0;
   char market_area_buffer[MARKET_VAULE_LEN] = {0};
   char sensor_info_buffer[SENSOR_VALUE_LEN] = {0};
   char pa_mode_buffer= '0';
   int market_area_len = 0;
   int sensor_info_len = 0;
   int pa_mode_len = 0;

   if (saved_command_line == NULL)
   {
      pr_err("saved_command_line is NULL exist\n");
      return -1;
   }

   printk(KERN_INFO "kernel start tinno_board_config saved_command_line = %s !\n", saved_command_line);

   p = strstr(saved_command_line, Market_AREA);
   if (p == NULL)
   {
      pr_err("can not find androidboot.Market_Area in saved_command_line");
      return -1;
   }

   market_area_len = strlen(Market_AREA);

   pr_err("market_area_len = %d", market_area_len); 
     
   for (i=0, j=0; i<MARKET_AREA_BUFFER_SIZE && j<MARKET_VAULE_LEN; i++)
   {
      if((*(p + market_area_len + i) == ' ') || (*(p + market_area_len + i) == '\0'))
      {
          break;
      }
      market_area_buffer[j++] = *(p + market_area_len + i);
   }

   p = strstr(saved_command_line, SENSOR_INFO);
   if (p == NULL)
   {
      pr_err("can not find androidboot.Sensor_Info in saved_command_line");
      return -1;
   }
   
   sensor_info_len = strlen(SENSOR_INFO);
   for(i=0, j=0; i<SENSOR_INFO_BUFFER_SIZE && j<SENSOR_VALUE_LEN; i++)
   {
      if((*(p + sensor_info_len + i) == ' ') || (*(p + sensor_info_len + i) == '\0'))
      {
          break;
      }
      sensor_info_buffer[j++] = *(p + sensor_info_len + i);  
   }

   p = strstr(saved_command_line, PA_MODE_INFO);
   if (p != NULL)
   {
       pa_mode_len = strlen(PA_MODE_INFO);
       for(i=0, j=0; i<PA_MODE_INFO_BUFFER_SIZE && j<PA_MODE_VALUE_LEN; i++)
       {
           if((*(p + pa_mode_len + i) == ' ') || (*(p + pa_mode_len + i) == '\0'))
           {
           break;
           }
           pa_mode_buffer = *(p + pa_mode_len + i); 
           j++;
           g_PaMode = (int)(pa_mode_buffer - '0');
       }
   } else {
       pr_err("can not find androidboot.Audio_Pa_Mode in saved_command_line");
   }
   
   if((strlen(market_area_buffer) > 0) && (strlen(market_area_buffer) < MARKET_VAULE_LEN))
   {
      strncpy(Market_Area, market_area_buffer, MIN(strlen(market_area_buffer), sizeof(Market_Area)));
   }
   else
   {
      pr_err("failed to get market area value");
   }

   pr_err("HJDDbgSensor, Market_Area=%s, sensor_info_buffer=%s \n", Market_Area, sensor_info_buffer);

#if 0
   // For sensor custom testing
   memset(sensor_info_buffer, 0, sizeof(sensor_info_buffer));
   strcpy(sensor_info_buffer, ""); // OAPLMHG
   pr_err("HJDDbgSensor2, Market_Area=%s, sensor_info_buffer=%s \n", Market_Area, sensor_info_buffer);
#endif

   if (strstr(sensor_info_buffer, "A"))
   {
      g_iAcceleration_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "G"))
   {
      g_iGyroscope_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "M"))
   {
      g_iMagnetic_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "P"))
   {
      g_iProximity_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "L"))
   {
      g_iLight_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "O"))
   {
      g_iOTG_open = 1;
   }
   if (strstr(sensor_info_buffer, "H"))
   {
      g_iHall_sensor = 1;
   }
   if (strstr(sensor_info_buffer, "S"))
   {
      g_iStepcounter_sensor = 1;
   }
   
   printk(KERN_INFO "Market_Area=%s\n",Market_Area);
   printk(KERN_INFO "g_iAcceleration_sensor=%d\n",g_iAcceleration_sensor);
   printk(KERN_INFO "g_iMagnetic_sensor=%d\n",g_iMagnetic_sensor);
   printk(KERN_INFO "g_iProximity_sensor=%d\n",g_iProximity_sensor);
   printk(KERN_INFO "g_iLight_sensor=%d\n",g_iLight_sensor);
   printk(KERN_INFO "g_iGyroscope_sensor=%d\n",g_iGyroscope_sensor);
   printk(KERN_INFO "g_iOTG_open=%d\n",g_iOTG_open);
   printk(KERN_INFO "g_iHall_sensor=%d\n",g_iHall_sensor);
   printk(KERN_INFO "g_iStepcounter_sensor=%d\n",g_iStepcounter_sensor);

   return 0;
}

early_initcall(tinno_sensors_config);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("tinno board config driver");
MODULE_AUTHOR("Tinno Mobile");
