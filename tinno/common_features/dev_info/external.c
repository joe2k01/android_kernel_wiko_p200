

#ifdef CONFIG_TINNO_PRODUCT_INFO

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>             

#include "dev_info.h"
#include "external.h"
//#include "flash_info.h"
#include <linux/flash_info.h>
/*******************************************************************************************/
// camera info.
G_CAM_INFO_S g_cam_info_struct[MAX_DRIVERS] = {0};

// flash info.
extern FLASH_INFO_SETTINGS flash_info_settings[];
extern int num_of_emi_records; 

//For MTK secureboot.add by yinglong.tang
extern int sec_schip_enabled(void);
extern int sec_usbdl_enabled(void);
extern int sec_boot_enabled(void);
/*******************************************************************************************/

static int get_lockmode_from_uboot(char *str)
{
    klog(" %s: %s\n",__func__,str);
    FULL_PRODUCT_DEVICE_INFO(ID_BL_LOCK_STATUS, (str != NULL ? str : "unknow"));
    return 1;
}

static int get_secboot_from_uboot(char *str)
{
    klog(" %s: %s\n",__func__,str);
    FULL_PRODUCT_DEVICE_INFO(ID_SECBOOT, (str != NULL ? str : "unknow"));
    return 1;
}

//For MTK secureboot.add by yinglong.tang
int get_mtk_secboot_cb(char *buf, void *args) 
{
    int sec_boot = sec_boot_enabled();
    int sec_usbdl = sec_usbdl_enabled();
    int sec_schip = sec_schip_enabled();

    if (!sec_boot && !sec_usbdl && !sec_schip) {
        return sprintf(buf, "NOT SUPPORT!");
    }

    return sprintf(buf, "[secboot:(%d) sec_usbdl:(%d) efuse:(%d)]", 
           sec_boot, sec_usbdl, sec_schip);
}

// uboot64/common/loader/sprd_cpcmdline.c : void cp_cmdline_fixup(void)
__setup("bootloader.lock=", get_lockmode_from_uboot);
__setup("secboot.enable=", get_secboot_from_uboot);

/*******************************************************************************************/

// for camera info.
int get_camera_info(char *buf, void *arg0) {
    char *tmp = (char *)arg0;
    int i = 0;
    long resolv = 0;
    int pi = 0;

    if (strcmp(tmp, "main") == 0) {
        i = 0;
    }
    else {
        i = 1;
    }

    resolv = g_cam_info_struct[i].w*g_cam_info_struct[i].h;
    pi = resolv/1000/1000 + (resolv/1000/100%10 > 5 ? 1 : 0);

    klog("%s:%d - (%s) %s:(%d*%d)\n",__func__, __LINE__, tmp,
        g_cam_info_struct[i].name, g_cam_info_struct[i].w, g_cam_info_struct[i].h);

    return sprintf(buf, "%s [%d*%d] %dM",
        g_cam_info_struct[i].name, g_cam_info_struct[i].w, g_cam_info_struct[i].h, pi);
}

/*******************************************************************************************/
// flash info.(mtk,sprd platform)
int get_mmc_chip_info(char *buf, void *arg0)
{
    int i = 0;
    char flashId[sizeof(FLASH_INFO_SETTINGS)];
    struct mmc_card *card = (struct mmc_card *)arg0;
    FLASH_INFO_SETTINGS *fis = flash_info_settings;

    memset(flashId, 0, sizeof(flashId));
    sprintf(flashId, "%4x%4x%4x%4x", 
        card->raw_cid[0], card->raw_cid[1], card->raw_cid[2], card->raw_cid[3]);

    klog("%s:%d current FlashID is: %s\n", __func__, __LINE__, flashId);
    for(i = 0; i < num_of_emi_records; i++) {
        if(strncasecmp(flashId,fis[i].id,strlen(fis[i].id) -2) == 0) {
            return sprintf(buf,"%s_%s+%s", 
                fis[i].vendorName, fis[i].romSize, fis[i].ramSize);
        }
    }
    return 0;
}										

#endif
