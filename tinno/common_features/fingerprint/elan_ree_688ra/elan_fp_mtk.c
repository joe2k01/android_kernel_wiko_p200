#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/of_reserved_mem.h>

#define MTK_PLAFROM             (1)
#define USE_SPI1_4GB            (0) // Use for MTK DMA (spi1)
#define GPIO_PINCTRL            (1)
#define HV_IC                   (0)

#if MTK_PLAFROM
//#include <mt_spi.h>
//#include <mt_spi_hal.h>
//#include <mt-plat/mt_gpio.h>
#include "mt_spi.h"
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

#include <linux/completion.h>
#include <linux/version.h>
#include "elan_fp_mtk.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "../fp_drv/fp_drv.h"

#define VERSION_LOG	"2.2.0"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
#define reinit_completion(x) INIT_COMPLETION(*(x))
#endif

static int elan_debug = 1;
#define ELAN_DEBUG(format, args ...) \
    do { \
        if (elan_debug) \
            printk(KERN_ERR"[ELAN] " format, ##args); \
    } while (0)

#define SPI_MAX_SPEED               3*1000*1000
#define KEY_FP_INT                  KEY_POWER
#define ELAN_CALI_TIMEOUT_MSEC      1000
#define SPI_ALINE_BYTE              4
#define WRITE_REG_HEAD              0x80
#define READ_REG_HEAD               0x40
#define READ_SERIER_REG_HEAD        0xC0
#define START_SCAN                  0x01
#define START_READ_IMAGE            0x10

static int read_all = 2; // 0:one frame(int), 1:one row(int), 2:one frame(status cmd), 3:one row(status cmd)
struct completion cmd_done;

static int key_status = 0;
static struct fasync_struct *fasync_queue = NULL;
static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
static int elan_work_flag = 0;
static int int_status = 0;

static unsigned int IMG_WIDTH = 0;
static unsigned int IMG_HEIGHT = 0;
static unsigned int IMG_SIZE = 0;

static unsigned char * imagebuffer = NULL; // for SPI Transfer and orginal data
static unsigned char * imagebuffer_TX = NULL; // for SPI Transfer and orginal data
static unsigned char * imagebuffer_RX = NULL; // for SPI Transfer and orginal data

static unsigned char bCMD_REG = 0; // CMD = 0, REG= 1
static int image_index = 0;

static unsigned char IOIRQ_STATUS = 0;
static unsigned int spi_speed = SPI_MAX_SPEED; // 3MHz

static struct workqueue_struct *elan_wq;
static int factory_status = 0;
static int raw_byts = 2; //1:8bits, 2:16bits
static volatile int display_status = 0; // Screen On:0 Off:1
static int display_notify_option = 0;
static int EFSA80S_688R_712R_IC = 0;
static int DUMMY_BYTE = 1;
static int register_fb_flag = 0;

// guomingyi add.
static int get_finger_state(char *buf, void *args); 
static int read_chip_id(char *buf, void *args);

#if defined(CONFIG_FB)
static struct notifier_block fb_notif;
#endif

struct elan_data {
    struct spi_device       *spi;
    struct input_dev        *input_dev;
    int                     int_gpio;
    int                     irq;
    int                     rst_gpio;
    struct work_struct      work;	
    bool                    irq_status; // 1:enable, 0:disable
    wait_queue_head_t       elan_wait; 
    struct wake_lock        wake_lock;
    struct wake_lock        hal_wake_lock;
    struct miscdevice       elan_dev; /* char device for ioctl */
    struct regulator        *reg;
    struct pinctrl          *elan_pinctrl;
    struct pinctrl_state    *pins_default;
    struct pinctrl_state    *eint_as_int, *eint_in_low, *eint_in_float, *fp_rst_low, *fp_rst_high;
};

#if NET_LINK
struct sock *nl_sk = NULL;

int pid = 0;

#define NETLINK_TEST 25
#define MAX_MSGSIZE 32


void sendnlmsg(char *msg)
{
	struct sk_buff *skb_1;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSGSIZE);
	int ret = 0;
	if (!msg || !nl_sk || !pid) {
		return ;
	}
	skb_1 = alloc_skb(len, GFP_KERNEL);
	if (!skb_1) {
		pr_err("alloc_skb error\n");
		return;
	}

	nlh = nlmsg_put(skb_1, 0, 0, 0, MAX_MSGSIZE, 0);

	NETLINK_CB(skb_1).portid = 0;
	NETLINK_CB(skb_1).dst_group = 0;

	memcpy(NLMSG_DATA(nlh), msg, sizeof(char));
	pr_debug("send message: %d\n", *(char *)NLMSG_DATA(nlh));

	ret = netlink_unicast(nl_sk, skb_1, pid, MSG_DONTWAIT);
	if (!ret) {
		//kfree_skb(skb_1);
		pr_err("send msg from kernel to usespace failed ret 0x%x\n", ret);
	}
}

void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];
	skb = skb_get (__skb);
	
	if(skb->len >= NLMSG_SPACE(0))
	{
		nlh = nlmsg_hdr(skb);

		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;
		
		kfree_skb(skb);
		ELAN_DEBUG("got it %d\n",nlh->nlmsg_pid);
		
	}

}

int netlink_unicast_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;
	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));

	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST,
			&netlink_cfg);

	if(!nl_sk){
		ELAN_DEBUG("create netlink socket error\n");
		return 1;
	}

	return 0;
}
#endif

static irqreturn_t elan_irq_handler(int irq, void *dev_id);

#if USE_SPI1_4GB
static dma_addr_t SpiDmaBufTx_pa;
static dma_addr_t SpiDmaBufRx_pa;
static char *spi_tx_local_buf;
static char *spi_rx_local_buf;

static int reserve_memory_spi_fn(struct reserved_mem *rmem)
{
    pr_err(" name: %s, base: 0x%llx, size: 0x%llx\n", rmem->name,
            (unsigned long long)rmem->base, (unsigned long long)rmem->size);
    BUG_ON(rmem->size < 0x16000);
    SpiDmaBufTx_pa = rmem->base;
    SpiDmaBufRx_pa = rmem->base+0xb000;
    return 0;
}
RESERVEDMEM_OF_DECLARE(reserve_memory_spi1, "mediatek,spi-reserve-memory", reserve_memory_spi_fn);
#endif

static int elan_spi_transfer(struct spi_device *spi, const char *txbuf, char *rxbuf, int len)
{
    struct spi_transfer t = {0};
    struct spi_message m;
    int ret = 0;

#if USE_SPI1_4GB
    /* map physical addr to virtual addr */
    if (NULL == spi_tx_local_buf) {
        spi_tx_local_buf = (char *)ioremap_nocache(SpiDmaBufTx_pa, 0xb000);
        if (!spi_tx_local_buf) {
            pr_err("SPI Failed to dma_alloc_coherent()\n");
            return -ENOMEM;
        }
    }
    if (NULL == spi_rx_local_buf) {
        spi_rx_local_buf = (char *)ioremap_nocache(SpiDmaBufRx_pa, 0xb000);
        if (!spi_rx_local_buf) {
            pr_err("SPI Failed to dma_alloc_coherent()\n");
            return -ENOMEM;
        }
    }
    if(txbuf != NULL){
        memcpy(spi_tx_local_buf, txbuf, len);
        t.tx_buf = spi_tx_local_buf;
        t.tx_dma = SpiDmaBufTx_pa;
    }
    if(rxbuf != NULL){
        t.rx_buf = spi_rx_local_buf;
        t.rx_dma = SpiDmaBufRx_pa;
    }
#else
    t.tx_buf = txbuf;
    t.rx_buf = rxbuf;
#endif        
    //memset(&t, 0, sizeof(t));
    spi_message_init(&m);
    t.bits_per_word = 8;
    t.len = len;
    t.speed_hz = spi_speed;
    spi_message_add_tail(&t, &m);
    ret = spi_sync(spi, &m);
    if (ret == 0) {
#if USE_SPI1_4GB
        if (rxbuf != NULL)
            memcpy(rxbuf, spi_rx_local_buf, len);
#endif
    } 
    else {
        pr_err("%s : spi_sync failed, ret = %d\n", __func__, ret);
    }
    return ret;
}

static int elan_kmalloc_image(void)
{
    int alloc_len = 0;

    IMG_SIZE = (IMG_WIDTH*IMG_HEIGHT);

    //(line len + dummy(1))*line counts + cmd(1) + 4Byte alignment(MTK DMA mode)
    alloc_len = ((IMG_WIDTH*raw_byts + DUMMY_BYTE)*IMG_HEIGHT + 1 + (1024-1))/1024*1024;
    imagebuffer = kmalloc(sizeof(unsigned char)*alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
    if (imagebuffer == NULL) {
        ELAN_DEBUG("imagebuffer kmalloc error\n");
        kfree(imagebuffer);
        return -ENOMEM;
    }

    //max len + cmd(1) + dummy(1) + 4Byte alignment(MTK DMA mode)
    alloc_len = (IMG_WIDTH*raw_byts + 2 + (SPI_ALINE_BYTE-1))/SPI_ALINE_BYTE*SPI_ALINE_BYTE;
    imagebuffer_TX = kmalloc(sizeof(unsigned char)*alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
    if (imagebuffer_TX == NULL) {
        ELAN_DEBUG("imagebuffer_TX kmalloc error\n");
        kfree(imagebuffer_TX);
        return -ENOMEM;
    }

    imagebuffer_RX = kmalloc(sizeof(unsigned char)*alloc_len, GFP_KERNEL); // add 2 is cmd & dummy
    if (imagebuffer_RX == NULL) {
        ELAN_DEBUG("imagebuffer_RX kmalloc error\n");
        kfree(imagebuffer_RX);
        return -ENOMEM;
    }

    return 0;
}

static int elan_read_register(struct spi_device *spi, unsigned char *RegInfo)
{
    // [0] = length, [1] = register address, [2~length+1] = data
    char *txbuf = imagebuffer_TX;
    char *rxbuf = imagebuffer_RX;

    int i;
    //int len = RegInfo[0] + 2;
	
    if (RegInfo[0] < 2 || bCMD_REG == 0) { // read with dummy
        if (bCMD_REG == 1) // 0 == CMD, 1 == REG
            txbuf[0] = READ_REG_HEAD + RegInfo[1]; // one byte data read (+1 = cmd)
        else if (bCMD_REG == 0) // 0 == CMD, 1 == REG
            txbuf[0] = RegInfo[1]; // one byte data read (+1 = cmd)

        elan_spi_transfer(spi, txbuf, rxbuf, RegInfo[0]+1);

        if (RegInfo[0] < 2) { // read reg
            RegInfo[2] = rxbuf[1];
            ELAN_DEBUG("%s() Read = 0x%02x\n", __func__, rxbuf[1]);
        }
        else { // read cmd over one byte
            for (i=0; i<RegInfo[0]-1; i++) {
                RegInfo[i+2] = rxbuf[i+2];
                ELAN_DEBUG("%s() Read CMD = 0x%02x\n", __func__, rxbuf[i+2]);
            }
        }	
    }
	else {
        txbuf[0] = READ_SERIER_REG_HEAD + RegInfo[1]; // mutli-byte read (+2 = cmd & dummy)
        elan_spi_transfer(spi, txbuf, rxbuf, RegInfo[0]+2);

        for(i=0; i<RegInfo[0]; i++)
            RegInfo[i + 2] = rxbuf[i+2];

        ELAN_DEBUG("%s() Read = ", __func__);

        for(i=0; i<RegInfo[0]; i++)
            ELAN_DEBUG("0x%02x ", rxbuf[i+2]);

        ELAN_DEBUG("\n");
    }

    return 0;
}

static int elan_write_register(struct spi_device *spi, unsigned char *RegInfo)
{
    // [0] = length, [1] = register address, [2~length+1] = data
    char *txbuf = imagebuffer_TX;
    //char *rxbuf = imagebuffer_RX;
    int i;
    //int len = RegInfo[0] + 1;

    if (bCMD_REG == 1) // 0 == CMD, 1 == REG
        txbuf[0] = WRITE_REG_HEAD + RegInfo[1];
    else if (bCMD_REG == 0)
        txbuf[0] = RegInfo[1];

    for (i=0; i<RegInfo[0]; i++)
        txbuf[i + 1] = RegInfo[i + 2];

    elan_spi_transfer(spi, txbuf, NULL, RegInfo[0]+1);

    ELAN_DEBUG("%s() ", __func__);

    for (i=0; i<RegInfo[0]; i++)
        ELAN_DEBUG("0x%02x ", txbuf[i + 1]);

    ELAN_DEBUG("\n");

    return 0;
}

static int elan_check_buf(struct elan_data *fp)
{
    char txbuf[4] = {0};
    char rxbuf[4] = {0};
    int ret = 0;
    txbuf[0] = 0x03;
    ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, 3);
    if (ret != 0) {
        ELAN_DEBUG("spi failed, ret = %d\n", ret);
        return -2;
    }

    if (rxbuf[2] & 0x04) {
        ELAN_DEBUG("Check Scan Cmd Success!! %x\n",rxbuf[2]);
    }
    else {
        ELAN_DEBUG("Check Scan Cmd Fail!! Scan Cmd : 0x%02x\n", rxbuf[2]);
        return -1;
    }

    return 0;
}

static int elan_recv_image(struct elan_data *fp)
{
    int read_all_count = 0;
    char *txbuf = imagebuffer_TX;
    char *rxbuf = imagebuffer_RX;
    int len = 0;
    int ret = 0;

    if(IOIRQ_STATUS & 0xA0) { // WOE Interrupt Enable
/*
        txbuf[0] = 0x03;
        ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, 3);
        if (ret != 0) {
            ELAN_DEBUG("spi failed, ret =  %d\n", ret);
            return -2;
        }

        if (rxbuf[2] & 0x10) {
            ELAN_DEBUG("check woe success\n");
        }
        else {
            ELAN_DEBUG("check woe failed, status_cmd = 0x%02x\n", rxbuf[2]);
            return -1;
        }
*/
        elan_work_flag = 1;
        wake_up(&elan_poll_wq);
        return 0;
    }

    if(IOIRQ_STATUS & 0x08) { // BUFFER Interrupt Enable
/*
        ret = elan_check_buf(fp);
        if(ret < 0)
            return ret;

        txbuf[0] = 0x03;
        ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, 3);
        if (ret != 0) {
            ELAN_DEBUG("spi failed, ret =  %d\n", ret);
            return -2;
        }

        if (rxbuf[2] & 0x84) {
            ELAN_DEBUG("check scan success, %x\n",rxbuf[2]);
        }
        else {
            ELAN_DEBUG("check scan failed, status_cmd = 0x%02x\n", rxbuf[2]);
            return -1;
        }
*/
        txbuf[0] = START_READ_IMAGE;
        if (read_all == 0) {
            imagebuffer_TX[0] = START_READ_IMAGE;
            image_index = 0;

            if (EFSA80S_688R_712R_IC == 1)
                len = ((IMG_WIDTH*raw_byts+DUMMY_BYTE)*IMG_HEIGHT+(1024-1))/1024*1024;
            else if (EFSA80S_688R_712R_IC == 0)
                len = ((IMG_WIDTH*raw_byts+DUMMY_BYTE)*IMG_HEIGHT+1+(1024-1))/1024*1024;

            ret = elan_spi_transfer(fp->spi, imagebuffer_TX, imagebuffer, len); // +2 is cmd & dummy
            if (ret != 0) {
                ELAN_DEBUG("spi failed, ret = %d\n", ret);
                return -2;
            }
            ELAN_DEBUG("imagebuffer=%x %x %x %x\n", imagebuffer[0], imagebuffer[1], imagebuffer[2], imagebuffer[3]);

            for (image_index=0; image_index<IMG_HEIGHT; image_index++) {
                if (image_index == 0){
                    memcpy(imagebuffer, &imagebuffer[2], IMG_WIDTH*raw_byts);
                }
                else {						
                    memcpy(&imagebuffer[IMG_WIDTH*raw_byts*image_index], \
                        &imagebuffer[(IMG_WIDTH*raw_byts+DUMMY_BYTE)*image_index+2], IMG_WIDTH*raw_byts);
                }
            }
            image_index = IMG_HEIGHT;
        }
		else if (read_all == 1) {
            len = IMG_WIDTH*raw_byts+2;
            if (image_index <= IMG_HEIGHT) {
                ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, len);
                if (ret != 0) {
                    ELAN_DEBUG("spi failed, ret = %d\n", ret);
                    return -2;
                }
                memcpy(&imagebuffer[(len-2)*image_index], &rxbuf[2], len-2);
            }
            image_index++;
        }
        else if (read_all == 2) {
            image_index = 0;
            ret = elan_check_buf(fp);
            if (ret == 0) {
                imagebuffer_TX[0] = START_READ_IMAGE;
                if (EFSA80S_688R_712R_IC == 1)
                    len = ((IMG_WIDTH*raw_byts+DUMMY_BYTE)*IMG_HEIGHT+(1024-1))/1024*1024;
                else if (EFSA80S_688R_712R_IC == 0)
                    len = ((IMG_WIDTH*raw_byts+DUMMY_BYTE)*IMG_HEIGHT+1+(1024-1))/1024*1024;

                ret = elan_spi_transfer(fp->spi, imagebuffer_TX, imagebuffer, len); // +2 is cmd & dummy
                if (ret != 0) {
                    ELAN_DEBUG("SPI Fail %d!!\n", ret);
                    return -2;
                }
                ELAN_DEBUG("imagebuffer=%x %x %x %x\n", imagebuffer[0], imagebuffer[1], imagebuffer[2], imagebuffer[3]);

                for (image_index=0; image_index<IMG_HEIGHT; image_index++) {
                    if (image_index == 0) {         		
                        memcpy(imagebuffer, &imagebuffer[2], IMG_WIDTH*raw_byts);
                    }
                    else {						
                        memcpy(&imagebuffer[IMG_WIDTH*raw_byts*image_index], \
                            &imagebuffer[(IMG_WIDTH*raw_byts+DUMMY_BYTE)*image_index+2], IMG_WIDTH*raw_byts);
                    }
                }
                image_index = IMG_HEIGHT;
            }
            else
                return ret;
        }
        else if (read_all == 3) {
            image_index = 0;
            read_all_count = 0;
            do {
                if (image_index <= IMG_HEIGHT) {
                    ret = elan_check_buf(fp);
                    if (ret == 0) {
                        len = IMG_WIDTH*raw_byts+2;
                        ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, len);
                        if (ret != 0) {
                            ELAN_DEBUG("SPI Fail %d!!\n", ret);
                            return -2;
                        }
                        memcpy(&imagebuffer[(len-2)*image_index], &rxbuf[2], len-2);
                        image_index++;
                    }
                }
                read_all_count++;
            } while (read_all_count < 1000 && image_index < IMG_HEIGHT);
            image_index = IMG_HEIGHT;
        }
        else
        {
            image_index = 0;
            do {				
                if (image_index <= IMG_HEIGHT) {
                    ret = elan_spi_transfer(fp->spi, txbuf, rxbuf, len);
                    if (ret != 0) {
                        ELAN_DEBUG("spi failed, ret = %d\n", ret);
                        return -2;
                    }
                    memcpy(&imagebuffer[(len-2)*image_index], &rxbuf[2], len-2);
                }
                image_index++;
                ELAN_DEBUG("image_index = %d, IMG_HEIGHT = %d\n", image_index, IMG_HEIGHT);
            } while (read_all_count < read_all && image_index < IMG_HEIGHT);
        }

        if (image_index == IMG_HEIGHT) {
            ELAN_DEBUG("%s() finish\n", __func__);
            complete(&cmd_done);
            image_index = 0;
        }
    }
    return 0;
}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (elan_debug) {
        elan_debug=0;
    }
    else {
        elan_debug=1;
    }

    return sprintf(buf, "[ELAN] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);

static struct attribute *elan_attributes[] = {
    &dev_attr_drv_version.attr,
    &dev_attr_elan_debug.attr,
    NULL
};

static struct attribute_group elan_attr_group = {
    .attrs = elan_attributes,
};

static DEFINE_MUTEX(elan_set_gpio_mutex);
static DEFINE_MUTEX(elan_factory_mutex);

static int elan_free_image(void)
{
    kfree(imagebuffer);
    kfree(imagebuffer_TX);
    kfree(imagebuffer_RX);
    return 0;
}

static void elan_reset(struct elan_data *fp)
{

#if GPIO_PINCTRL
    pinctrl_select_state(fp->elan_pinctrl, fp->fp_rst_low);
    mdelay(5);
    pinctrl_select_state(fp->elan_pinctrl, fp->fp_rst_high);
    mdelay(50);
#else
    gpio_set_value(fp->rst_gpio, 0);
    mdelay(5);
    gpio_set_value(fp->rst_gpio, 1);
    mdelay(50);
#endif

}

static void elan_irq_disable(struct elan_data *fp)
{
    if (fp->irq_status) {
        fp->irq_status = false;
        disable_irq(fp->irq);
    }
}

static void elan_irq_enable(struct elan_data *fp)
{
    if (!fp->irq_status) {
        fp->irq_status = true;
        enable_irq(fp->irq);
    }
}

static ssize_t elan_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
    int ret = 0;

    if (user_buf[0] == 10) {
        IOIRQ_STATUS = user_buf[1];
        elan_work_flag = 0;
        ELAN_DEBUG("set IOIRQ_STATUS = 0x%x, elan_work_flag = %d\n", IOIRQ_STATUS, elan_work_flag);
    }
    else if (user_buf[0] == 0X10) {
        read_all = user_buf[1];
        ELAN_DEBUG("read_all = %d\n", read_all);
    }

  	return ret;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = evdata->data;

    ELAN_DEBUG("%s fb notifier callback event = %lu, evdata->data = %d\n",__func__, event, *blank);

    if (evdata && evdata->data) {
        if (event == FB_EVENT_BLANK) {
            if (*blank == FB_BLANK_UNBLANK) {
                display_status = 0;
                elan_work_flag = 1;
                wake_up(&elan_poll_wq);
                ELAN_DEBUG("Display On\n");
            }
            else if (*blank == FB_BLANK_POWERDOWN) {
                display_status = 1;
                elan_work_flag = 1;
                wake_up(&elan_poll_wq);
                ELAN_DEBUG("Display Off\n");
            }
        }
    }
    return 0;
}
#endif

static long elan_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct elan_data *fp = filp->private_data;
    //unsigned char buf[16];
    //unsigned char *pUserBuf;
    unsigned char pUserBuf[100] = {0};
    int keycode;
    int err = 0;
    int i = 0;
    int status = 0;
    int ret = 0;
    int wake_lock_arg;

#if MTK_PLAFROM
    struct mt_chip_conf *spi_conf_mt65xx;
    int int_buf[8] = {0};
#endif

    ELAN_DEBUG("%s()\n", __func__);

    switch (cmd) {
        case ID_IOCTL_READ_REGISTER: //2
            ELAN_DEBUG("[IOCTL] READ REGISTER\n");
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            bCMD_REG = 1; // CMD = 0, REG= 1

            elan_read_register(fp->spi, pUserBuf);
            if (copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
                return -1;
            break;

        case ID_IOCTL_WRITE_REGISTER: //3
            ELAN_DEBUG("[IOCTL] WRITE REGISTER\n");
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            bCMD_REG = 1; // CMD = 0, REG= 1

            //if (copy_from_user(buf, pUserBuf, pUserBuf[0]+2))
                //return -1;	
            elan_write_register(fp->spi, pUserBuf);
            break;

        case ID_IOCTL_RESET: //6
            ELAN_DEBUG("[IOCTL] RESET\n");
            elan_reset(fp);
            break;

        case ID_IOCTL_GET_RAW_IMAGE: //10
            ELAN_DEBUG("[IOCTL] GET RAW IMAGE\n");
            //pUserBuf = (unsigned char *)arg;

            // Wait for INT IRQ Read complete.
            status = wait_for_completion_interruptible_timeout(&cmd_done,
                        msecs_to_jiffies(ELAN_CALI_TIMEOUT_MSEC));
            if (status <= 0) {
                ELAN_DEBUG("%s() retry image %x.\n", __func__, status);
                msleep(50);
                err = elan_recv_image(fp);
                if (err !=0) {
                    ELAN_DEBUG("%s() retry image error err=%x.\n", __func__, err);
                    ELAN_DEBUG("wait image fail\n");
                    if (err == -2) {
                        ELAN_DEBUG("SPI FAIL\n");
                        return -3;
                    }
                    if (status == 0) {
                        if(int_status == 1) {
                            ELAN_DEBUG("%s() get image error,  int abnormal high %x.\n", __func__, int_status);
                            return -2;
                        }
                        else {
                            ELAN_DEBUG("%s() get image error, wait timeout, int low. %x\n", __func__, int_status);
                            return -1;
                        }
                    }
                    else {
                        ELAN_DEBUG("%s() get image error, system interrupt wait %x.\n", __func__, status);
                        return -4;
                    }
                    return -1;
                }
                else {
                    if (status == 0) {
                        if (int_status == 1) {
                            ELAN_DEBUG("%s() got image success,  int abnormal high %x.\n", __func__, int_status);
                            return 2;
                        }
                        else {
                            ELAN_DEBUG("%s() got image success, wait timeout, but retry OK. %x\n", __func__, int_status);
                            return 1;
                        }
                    }
                    else {
                        ELAN_DEBUG("%s() got image success, system interrupt wait %x.\n", __func__, status);
                        return 3;
                    }
                }
            }
            else {
                err = 0;
                ELAN_DEBUG("wait image success\n");
            }

            image_index = 0;		

            err = copy_to_user((unsigned char *)arg, imagebuffer, IMG_SIZE*raw_byts);

            return err;
            break;

		case ID_IOCTL_READ_CMD: //15
            ELAN_DEBUG("[IOCTL] READ COMMAND\n");
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            bCMD_REG = 0; // CMD = 0, REG= 1

            elan_read_register(fp->spi, pUserBuf);
            if (copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0]+2))
                return -1;
            break;

        case ID_IOCTL_WRITE_CMD: //16
            ELAN_DEBUG("[IOCTL] WRITE COMMAND\n");
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            bCMD_REG = 0; // CMD = 0, REG= 1

            //if (copy_from_user(buf, pUserBuf, pUserBuf[0]+2))
                //return -1;
            if (pUserBuf[1] == START_SCAN)
                image_index = 0;
            reinit_completion(&cmd_done);
            elan_write_register(fp->spi, pUserBuf);
            int_status = 0;
            break;

        case ID_IOCTL_IOIRQ_STATUS: //17
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(&IOIRQ_STATUS, (unsigned char *)arg, 1);

            //if (copy_from_user(&IOIRQ_STATUS, pUserBuf, 1))
                //return -1;
            ELAN_DEBUG("[IOCTL] IOIRQ_STATUS = %x\n", IOIRQ_STATUS);
            break;

        case ID_IOCTL_POLL_INIT: //20
            ELAN_DEBUG("[IOCTL] POLL INIT\n");
            elan_work_flag = 0;
            int_status = 0;
            break;

        case ID_IOCTL_READ_ALL: //21
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            read_all = (signed char) pUserBuf[0];
            ELAN_DEBUG("[IOCTL] READ ALL, read_all = %d\n", read_all);
            break;

        case ID_IOCTL_INPUT_KEYCODE: //22
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE DOWN & UP, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }

            input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
            input_sync(fp->input_dev);
            input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_POLL_EXIT:  //23
            ELAN_DEBUG("[IOCTL] POLL EXIT\n");
            elan_work_flag = 1;
            wake_up(&elan_poll_wq);
            break;
			
        case ID_IOCTL_SET_KEYCODE: //24
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] SET KEYCODE, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_set_capability(fp->input_dev, EV_KEY, keycode);
            break;

        case ID_IOCTL_XY_SETTING: //25
            ELAN_DEBUG("[IOCTL] X, Y SETTING\n");
            //pUserBuf = (unsigned char *)arg;
            err = copy_from_user(pUserBuf, (unsigned char *)arg, 100);
            IMG_WIDTH = (pUserBuf[0]<<8)|pUserBuf[1];
			IMG_HEIGHT = (pUserBuf[2]<<8)|pUserBuf[3];
            ELAN_DEBUG("IMG_WIDTH=%d, IMG_HEIGHT=%d\n", IMG_WIDTH, IMG_HEIGHT);
            if (imagebuffer) {
                ELAN_DEBUG("Free buffer\n");
                elan_free_image();
            } 
            ELAN_DEBUG("Rekmalloc buffer\n");
            elan_kmalloc_image();				
            break;

        case ID_IOCTL_READ_FACTORY_STATUS: //26
            mutex_lock(&elan_factory_mutex);
            ELAN_DEBUG("[IOCTL] READ factory_status = %d\n", factory_status);
            mutex_unlock(&elan_factory_mutex);
            return factory_status;
            break;

        case ID_IOCTL_WRITE_FACTORY_STATUS: //27
            mutex_lock(&elan_factory_mutex);
            factory_status = (int __user)arg;
            ELAN_DEBUG("[IOCTL] WRITE factory_status = %d\n", factory_status);
            mutex_unlock(&elan_factory_mutex);
            break;

        case ID_IOCTL_INPUT_KEYCODE_DOWN: //28
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE DOWN, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_INPUT_KEYCODE_UP: //29
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE UP, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_GET_RAW_IMAGE_POLL: //30
            ELAN_DEBUG("[IOCTL] GET RAW IMAGE BY POLLING\n");
            //pUserBuf = (unsigned char *)arg;

            i = 0;
            do {
                err = elan_recv_image(fp);
                if (err == 0)
                    break;
                mdelay(1);
                i++;
            } while (err != 0 && i < 1000);

            if (err != 0) {
                ELAN_DEBUG("[IOCTL] GET RAW IMAGE BY POLLING Fail, err = %d\n", err);
                return err;
            }
            image_index = 0;
            err = copy_to_user((unsigned char *)arg, imagebuffer, IMG_SIZE*raw_byts);

            return err;
            break;

        case ID_IOCTL_RAW_BYTES: //31
            raw_byts = (int __user)arg;
            ELAN_DEBUG("[IOCTL] RAW BYTES, %d\n", raw_byts);
            if (imagebuffer) {
                ELAN_DEBUG("Free buffer\n");
                elan_free_image();
            } 
            ELAN_DEBUG("Rekmalloc buffer\n");
            elan_kmalloc_image();
            break;

        case ID_IOCTL_INT_STATUS: //40
#if GPIO_PINCTRL

           // fp->int_gpio = irq_to_gpio(fp->irq);
            fp->int_gpio =5;//build error
#endif
            return gpio_get_value(fp->int_gpio);

        case ID_IOCTL_WAKE_LOCK_UNLOCK: //41
            wake_lock_arg = (int __user)arg;
            if(!wake_lock_arg) {
                wake_unlock(&fp->hal_wake_lock);
                /** ELAN_DEBUG("[IOCTL] HAL WAKE UNLOCK = %d\n", wake_lock_arg); */
            }
            else if(wake_lock_arg) {
                wake_lock(&fp->hal_wake_lock);
                /** ELAN_DEBUG("[IOCTL] HAL WAKE LOCK = %d\n", wake_lock_arg); */
            }
            else
                ELAN_DEBUG("[IOCTL] ERROR WAKE LOCK ARGUMENT\n");
            break;

        case ID_IOCTL_EN_IRQ: //55
            elan_irq_enable(fp);
            ELAN_DEBUG("[IOCTL] ENABLE IRQ\n");
            break;

        case ID_IOCTL_DIS_IRQ: //66
            elan_irq_disable(fp);
            ELAN_DEBUG("[IOCTL] DISABLE IRQ\n");
            break;

        case ID_IOCTL_SET_IRQ_TYPE: //91
            ELAN_DEBUG("[IOCTL] SET IRQ TYPE\n");
            irq_set_irq_type(fp->irq, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
            break;

        case ID_IOCTL_SET_IC: //92
            ELAN_DEBUG("[IOCTL] SET IC\n");
            EFSA80S_688R_712R_IC = (int __user)arg;
            if (EFSA80S_688R_712R_IC == 0)
                DUMMY_BYTE = 1;
            else if (EFSA80S_688R_712R_IC == 1)
                DUMMY_BYTE = 2;
            else
                ELAN_DEBUG("[IOCTL] WRONG IC ?\n");

            ELAN_DEBUG("[IOCTL] EFSA80S_688R_712R_IC = %d\n", EFSA80S_688R_712R_IC);
            break;

        case ID_IOCTL_DISPLAY_STATUS: //93
            ELAN_DEBUG("[IOCTL] DISPLAY_STATUS = %d\n", display_status);
            return display_status;
            break;

        case ID_IOCTL_DISPLAY_NOTIFY: //94
            display_notify_option = (int __user)arg;
            ELAN_DEBUG("[IOCTL] DISPLAY_NOTIFY = %d\n", display_notify_option);
#if defined(CONFIG_FB)
            if (display_notify_option && !register_fb_flag) {
                ELAN_DEBUG("[IOCTL] Register fb_notify\n");
                fb_notif.notifier_call = fb_notifier_callback;
                fb_register_client(&fb_notif);
                register_fb_flag = 1;
            }
#endif
            break;

        case ID_IOCTL_REQUEST_IRQ: //95
            ret = request_irq(fp->irq, elan_irq_handler,
                    IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
                    fp->spi->dev.driver->name, fp);
            if (ret < 0) {
                ELAN_DEBUG("[IOCTL] REGISTER IRQ FAILED, ret = %d\n", ret);
                if (ret == -EBUSY)
                    ret = 99;
                else
                    ret = 100;
                
                return ret;
            }
            ELAN_DEBUG("[IOCTL] REGISTER IRQ SUCCESS, ret = %d\n", ret);
            break;

        case ID_IOCTL_FREE_IRQ: //96
            ELAN_DEBUG("[IOCTL] FREE IRQ\n");
            free_irq(fp->irq, fp);
            break;

        case IOCTL_SPI_CONFIG:
#if MTK_PLAFROM
            err = copy_from_user(int_buf, (int *)arg, 6*sizeof(int));
            if (err)
                break;
            ELAN_DEBUG("[elan]: ");
            for (i=0; i<6; i++)
                ELAN_DEBUG("%x ", int_buf[i]);
            ELAN_DEBUG("\n");
            spi_conf_mt65xx = (struct mt_chip_conf *)fp->spi->controller_data;
            spi_conf_mt65xx->setuptime      = int_buf[0];
            spi_conf_mt65xx->holdtime       = int_buf[1];
            spi_conf_mt65xx->high_time      = int_buf[2];
            spi_conf_mt65xx->low_time       = int_buf[3];
            spi_conf_mt65xx->cs_idletime    = int_buf[4];
            spi_conf_mt65xx->ulthgh_thrsh   = int_buf[5];
#else
            spi_speed = (int __user)arg;
            fp->spi->max_speed_hz = spi_speed;
            ret = spi_setup(fp->spi);
            if (ret < 0) {
                ELAN_DEBUG("spi_setup failed, ret = %d\n", ret);
                break;
            }
            ELAN_DEBUG("[IOCTL] SPI_SPEED = %d\n", spi_speed);
#endif
            break;

        case IOCTL_READ_KEY_STATUS:
            return key_status;
            break;

        case IOCTL_WRITE_KEY_STATUS:
            key_status = arg;
            if (fasync_queue) {
                kill_fasync(&fasync_queue, SIGIO, POLL_IN);
            }
            break;	
        default:
            ELAN_DEBUG("INVALID COMMAND\n");
            break;
    }
    return 0;
}

static unsigned int elan_poll(struct file *file, poll_table *wait)
{
    int mask=0;
    //ELAN_DEBUG("%s()\n",__func__);

    //wait_event_interruptible(elan_poll_wq, elan_work_flag > 0);
    poll_wait(file, &elan_poll_wq, wait);

    if (elan_work_flag > 0)
        mask = elan_work_flag;

    elan_work_flag = 0;
    return mask;
}

static int elan_fasync(int fd, struct file * filp, int on)
{
    ELAN_DEBUG("%s enter \n",__func__);
    return fasync_helper(fd, filp, on, &fasync_queue);
}

ssize_t elan_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
    ssize_t ret = 0;
    return ret;
}

static int elan_open(struct inode *inode, struct file *filp)
{
    struct elan_data *fp = container_of(filp->private_data, struct elan_data, elan_dev);	
    filp->private_data = fp;
    ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

static int elan_close(struct inode *inode, struct file *filp)
{
    ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

static const struct file_operations elan_fops = {
    .owner          = THIS_MODULE,
    .open           = elan_open,
    .read           = elan_read,
    .write          = elan_write,
    .unlocked_ioctl = elan_ioctl,
    .poll           = elan_poll,
    .release        = elan_close,
    .fasync         = elan_fasync,
};

static void elan_work_func(struct work_struct *work)
{
    struct elan_data *fp;
    fp = container_of(work, struct elan_data, work);

    ELAN_DEBUG("%s() IOIRQ=%x.\n", __func__, IOIRQ_STATUS);

    elan_recv_image(fp);
    return;
}

static irqreturn_t elan_irq_handler(int irq, void *dev_id)
{
    struct elan_data *fp = (struct elan_data *)dev_id;
    ELAN_DEBUG("%s()\n", __func__);

	if (fp == NULL)
        return IRQ_NONE;

    wake_lock_timeout(&fp->wake_lock, msecs_to_jiffies(1000));

#if NET_LINK
    sendnlmsg("7");
#endif

    queue_work(elan_wq, &fp->work);
    return IRQ_HANDLED;
}

static int elan_setup_cdev(struct elan_data *fp)
{
    fp->elan_dev.minor = MISC_DYNAMIC_MINOR;
    fp->elan_dev.name = "elan_fp";
    fp->elan_dev.fops = &elan_fops;
    fp->elan_dev.mode = S_IFREG|S_IRWXUGO; 
    if (misc_register(&fp->elan_dev) < 0) {
        ELAN_DEBUG("misc_register failed!!");
        return -1;		
    }
    else {
        ELAN_DEBUG("misc_register finished!!");		
    }
    return 0;
}

static int elan_sysfs_create(struct elan_data *sysfs)
{
    struct elan_data *fp = spi_get_drvdata(sysfs->spi);
    int error = 0;

    /* Register sysfs */
    error = sysfs_create_group(&fp->spi->dev.kobj, &elan_attr_group);
    if (error) {
        ELAN_DEBUG("Failed to create sysfs attributes, err: %d\n", error);
        goto fail_un;
    }
    return 0;
fail_un:
    /* Remove sysfs */
    sysfs_remove_group(&fp->spi->dev.kobj, &elan_attr_group);

    return error;
}

static int elan_gpio_config(struct elan_data *fp, struct device_node *np)
{	
    int ret = 0;

#if GPIO_PINCTRL
    pinctrl_select_state(fp->elan_pinctrl, fp->eint_as_int);
    fp->irq = irq_of_parse_and_map(np, 0);
    ELAN_DEBUG("gpio to irq success, irq = %d\n",fp->irq);
    if (!fp->irq) {
        ELAN_DEBUG("irq_of_parse_and_map failed");
        ret = -1;
    }
#else
    // Configure INT GPIO (Input)
    ret = gpio_request(fp->int_gpio, "elan-irq");
    if (ret < 0)
        ELAN_DEBUG("interrupt pin request gpio failed, ret = %d\n", ret);
    else {
        gpio_direction_input(fp->int_gpio);
        fp->irq = gpio_to_irq(fp->int_gpio);
        if (fp->irq < 0) {
            ELAN_DEBUG("gpio to irq failed, irq = %d", fp->irq);
            ret = -1;
        }
        else
            ELAN_DEBUG("gpio to irq success, irq = %d\n",fp->irq);
    }
	
    // Configure RST GPIO (Output)
    ret = gpio_request(fp->rst_gpio, "elan-rst");
    if (ret < 0) {
        gpio_free(fp->int_gpio);
        free_irq(fp->irq, fp);
        ELAN_DEBUG("reset pin request gpio failed, ret = %d\n", ret);
    }
    else
        gpio_direction_output(fp->rst_gpio, 1);
#endif
    return ret;
}

static int elan_dts_init(struct elan_data *fp, struct device_node *np)
{
    int ret = 0;
    ELAN_DEBUG("%s()\n", __func__);
#if GPIO_PINCTRL
    fp->pins_default = pinctrl_lookup_state(fp->elan_pinctrl, "fp_default");
    if (IS_ERR(fp->pins_default)) {
     //   ret = PTR_ERR(fp->pins_default);
     //   dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl default %d!\n", ret);
    //    return ret;
    }
    fp->fp_rst_high = pinctrl_lookup_state(fp->elan_pinctrl, "fp_rst_high");
    if (IS_ERR(fp->fp_rst_high)) {
        ret = PTR_ERR(fp->fp_rst_high);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
        return ret;
    }
    fp->fp_rst_low = pinctrl_lookup_state(fp->elan_pinctrl, "fp_rst_low");
    if (IS_ERR(fp->fp_rst_low)) {
        ret = PTR_ERR(fp->fp_rst_low);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
        return ret;
    }
    fp->eint_as_int = pinctrl_lookup_state(fp->elan_pinctrl, "eint_as_int");
    if (IS_ERR(fp->eint_as_int)) {
        ret = PTR_ERR(fp->eint_as_int);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl eint_as_int!\n");
        return ret;
    }
    fp->eint_in_low = pinctrl_lookup_state(fp->elan_pinctrl, "eint_in_low");  //pins_eint_output0
    if (IS_ERR(fp->eint_in_low)) {
        ret = PTR_ERR(fp->eint_in_low);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl eint_in_low!\n");
        return ret;
    }
    fp->eint_in_float = pinctrl_lookup_state(fp->elan_pinctrl, "eint_in_float"); //pins_eint_output1
    if (IS_ERR(fp->eint_in_float)) {
        ret = PTR_ERR(fp->eint_in_float);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl eint_in_float!\n");
        return ret;
    }
#else
    fp->rst_gpio = of_get_named_gpio(np, "elan,rst-gpio", 0);
    ELAN_DEBUG("rst_gpio = %d\n", fp->rst_gpio);
    if (fp->rst_gpio < 0)
        return fp->rst_gpio;

    fp->int_gpio = of_get_named_gpio(np, "elan,irq-gpio", 0);
    ELAN_DEBUG("int_gpio = %d\n", fp->int_gpio);
    if (fp->int_gpio < 0)
        return fp->int_gpio;
#endif
    return ret;
}

static int elan_probe(struct spi_device *spi)
{	
    struct elan_data *fp = NULL;
    struct input_dev *input_dev = NULL;
    int ret = 0;
    unsigned char rxbuf[8] = {0};
    unsigned char txbuf[8] = {0};

    ELAN_DEBUG("%s(), version = %s\n", __func__, VERSION_LOG);

#if NET_LINK
    netlink_unicast_init(); //initial netlink
#endif

    init_completion(&cmd_done);

    /* Setup SPI */
    spi->mode = SPI_MODE_0; 		// set at spi_board_info
    spi->max_speed_hz = SPI_MAX_SPEED; 	// set at spi_board_info
    spi->chip_select = 0; 		// set at spi_board_info
    spi->bits_per_word = 8;			// do not change

    ret = spi_setup(spi);
    if (ret < 0)
        ELAN_DEBUG("spi_setup failed, ret = %d\n", ret);
	
    /* Allocate Device Data */
    fp = kzalloc(sizeof(struct elan_data), GFP_KERNEL);
    if (!fp) {
        ELAN_DEBUG("kzmalloc elan data failed\n");
        return -ENOMEM;
    }

    init_waitqueue_head(&fp->elan_wait);

    /* Init Input Device */
    input_dev = input_allocate_device();
    if (!input_dev)
        ELAN_DEBUG("alloc input_dev failed\n");

    fp->spi = spi;

    spi_set_drvdata(spi, fp);

    input_dev->name = "elan";
    input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = &spi->dev;
    input_set_drvdata(input_dev, fp);

    input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
    set_bit(KEY_F11, input_dev->keybit); 
    set_bit(KEY_F12, input_dev->keybit);

    input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.

    fp->input_dev = input_dev;

    /* Init Sysfs */
    ret = elan_sysfs_create(fp);
    if (ret < 0)
        ELAN_DEBUG("sysfs create failed, ret = %d\n", ret);

    wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");
    wake_lock_init(&fp->hal_wake_lock, WAKE_LOCK_SUSPEND, "hal_fp_wake_lock");

    /* Init Char Device */
    ret = elan_setup_cdev(fp);
    if (ret < 0)
        ELAN_DEBUG("setup device failed, ret = %d\n", ret);
	
    /* Register Input Device */
    ret = input_register_device(input_dev);
    if (ret)
        ELAN_DEBUG("register input device failed, ret = %d\n", ret);

    spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
   ELAN_DEBUG("xxxxxxxxxspi->dev.of_node = %d\n", spi->dev.of_node);

#if 0
    fp->reg = regulator_get(&spi->dev, "vddwifipa");
    ret = regulator_set_voltage(fp->reg, 2800000, 2800000);	/*set 2.8v*/
    ret = regulator_enable(fp->reg);
    if (ret) {
        ELAN_DEBUG("regulator_set_voltage(%d) failed!\n", ret);
        return -1;
    }
#endif

#if GPIO_PINCTRL
    fp->elan_pinctrl = devm_pinctrl_get(&spi->dev);
    if (IS_ERR(fp->elan_pinctrl)) {
        ret = PTR_ERR(fp->elan_pinctrl);
        dev_err(&spi->dev, "fwq Cannot find fp elan_pinctrl!\n");
        return ret;
    }
#endif

    ret = elan_dts_init(fp, spi->dev.of_node);
    if (ret < 0)
        ELAN_DEBUG("device tree initial failed, ret = %d\n", ret);

    ret = elan_gpio_config(fp, spi->dev.of_node);
    if (ret < 0)
        ELAN_DEBUG("gpio config failed, ret = %d\n", ret);

    elan_reset(fp);

#if HV_IC
    //Command to eFHV1303
    txbuf[0] = 0x04;
    ret = elan_spi_transfer(spi, txbuf, NULL, 1);
    //Local Mode -> Bypass Mode
    txbuf[0] = 0x0B;
    txbuf[1] = 0x02;
    ret = elan_spi_transfer(spi, txbuf, NULL, 2);
#endif

    txbuf[0] = 0xc1;
    ret = elan_spi_transfer(spi, txbuf, rxbuf, 6);
    if (ret != 0)
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
    else {
        ELAN_DEBUG("read device %02x %02x %02x %02x.\n", rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5]);
        IMG_WIDTH = (unsigned int)(rxbuf[5] - rxbuf[4] + 1);
        IMG_HEIGHT = (unsigned int)(rxbuf[3] - rxbuf[2] + 1);
        ELAN_DEBUG("WIDTH(Y) = %d, HEIGHT(X) = %d\n", IMG_WIDTH, IMG_HEIGHT);
    }

    /* Allocate image buffer */
    ret = elan_kmalloc_image();
    if (ret)
        ELAN_DEBUG("kmalloc image buffer failed, ret = %d\n", ret);

    elan_wq = create_singlethread_workqueue("elan_wq");

    if (!elan_wq)
        ELAN_DEBUG("create workqueue failed\n");

    INIT_WORK(&fp->work, elan_work_func);

    ret = request_irq(fp->irq, elan_irq_handler,
                    IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
                    spi->dev.driver->name, fp);
    if (ret)
        ELAN_DEBUG("request irq failed, ret = %d\n", ret);

    irq_set_irq_wake(fp->irq, 1);

    ELAN_DEBUG("%s() End\n", __func__);

    // guomingyi add.
    REGISTER_FP_DEV_INFO(spi->dev.driver->name, NULL, spi, get_finger_state, read_chip_id);
    return 0;
}

/** guomingyi add for test */
static int read_chip_id(char *buf, void *args)
{
    int ret = 0;
    unsigned char rxbuf[8] = {0};
    unsigned char txbuf[8] = {0};
    struct spi_device *spi = (struct spi_device *)args;

    if (spi == NULL) {
        ELAN_DEBUG("err: spi=null!\n");
        return 0;
    }

    txbuf[0] = 0xc1;
    ret = elan_spi_transfer(spi, txbuf, rxbuf, 6);
    if (ret != 0) {
        ELAN_DEBUG("spi_sync failed, ret = %d\n", ret);
        return 0;
    }
    else {
        ELAN_DEBUG("#read device [%02x %02x %02x %02x]\n", rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5]);
    }

    return sprintf(buf, "%02x-%02x-%02x-%02x", rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5]);
}

static int get_finger_state(char *buf, void *args) 
{
    return sprintf(buf, "%d", 0);
}
/** guomingyi add for test */


#ifdef CONFIG_OF
static struct of_device_id elan_of_match[] = {
    { .compatible = "mediatek,fingerprint",},
    {},
};

MODULE_DEVICE_TABLE(of, elan_of_match);
#endif

static struct spi_driver elan_driver = {
    .driver = {
        .name   = "elan_fp",
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = elan_of_match,
#endif
    },
    .probe = elan_probe,
    //.remove = __exit_p(elan_remove),
    //.id_table = efp_id,
};

#if 1
#if MTK_PLAFROM
static struct mt_chip_conf spi_xxxx_conf = {  
    //SPI speed
    .setuptime = 160,
    .holdtime = 3,
    .high_time = 10,
    .low_time = 10,
    .cs_idletime = 2,
    .ulthgh_thrsh = 0,

    //SPI mode
    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,

    .com_mod = 1,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif
static struct spi_board_info elan_spi_board_info[] = {
    [0] = {
        .modalias               = "elan_fp",
        .bus_num                = 1, // change by customer
        .chip_select            = 0, // change by customer, usually = 0.
        .max_speed_hz           = SPI_MAX_SPEED,
        .mode                   = SPI_MODE_0,
        .controller_data        = (void*)&spi_xxxx_conf,
    },
};
#endif

static int __init elan_init(void)
{
    ELAN_DEBUG("%s() Start\n", __func__);
    spi_register_board_info(elan_spi_board_info, ARRAY_SIZE(elan_spi_board_info));
    printk(KERN_ERR" ==elan_init====%s \n",elan_driver.driver.name);
	
    if (spi_register_driver(&elan_driver))
        return -EINVAL;

    ELAN_DEBUG("%s() End\n", __func__);
    return 0;
}

static void __exit elan_exist(void)
{	
    spi_unregister_driver(&elan_driver);

    if (elan_wq)
        destroy_workqueue(elan_wq);
}

module_init(elan_init);
module_exit(elan_exist);

MODULE_AUTHOR("ELAN");
MODULE_DESCRIPTION("spi fingerprint driver for ree");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
