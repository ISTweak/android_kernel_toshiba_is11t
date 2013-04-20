/*
  cam8mp Driver

  Copyright (C) 2011 FUJITSU LIMITED

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/time.h>
#include <linux/fs.h>
#include "../../../arch/arm/mach-msm/smd_private.h"
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include "camsensor_gpio.h"

#if 0
#define LOGI(fmt, args...)      printk(KERN_DEBUG "cam8mp: " fmt, ##args)
#else
#define LOGI(fmt, args...)      do{}while(0)
#endif
#define LOGE(fmt, args...)      printk(KERN_ERR "cam8mp: " fmt, ##args)

#define LED_2C_SLAVE_ADDR       0x76
struct i2c_adapter *i2c_led;

/* GPIO */
#define VSYNC                   14
#define MCLK                    15
#define BOOT                    22
#define RST                     25
#define VDD                     31
#define SPI_CS1                 44
#define SPI_CLK                 45
#define SPI_MOSI                47
#define SPI_MISO                48

#define DL_BUFFER_SIZE          (36*1024)
#define I2C_SLV_ADDR            0xC0
#define SPI_BIT_PER_WORD        8
#define SPI_SPEED_HZ            9596343
#define SPI_BLOCK_SZ            (3600)
#define BACKLIGHT_FILE          "/sys/class/backlight/msmfb_bl0/brightness"

/*===================================================================*
    LOCAL DECLARATIONS
 *===================================================================*/
struct cam_ctrl {
    const struct msm_camera_sensor_info *data;
    wait_queue_head_t           vsync_wait;
    spinlock_t                  vsync_lock;
    int vsync_irqno;
    int state;
 #define IRQ_INIT       -1
 #define IRQ_DISABLE    0
 #define IRQ_ENABLE     1
    int vsnyc_cnt;
    int vsnyc_max;
    long ms;
};

DEFINE_MUTEX(cam8mp_mtx);
static struct cam_ctrl *cam_ctrl = NULL;
static struct spi_device *cam8mp_spi_dev = NULL;
static uint8_t *pWorkMem = NULL;
static struct spi_transfer xfer[(DL_BUFFER_SIZE+SPI_BLOCK_SZ-1) / SPI_BLOCK_SZ];

/*===================================================================*
    EXTERNAL DECLARATIONS
 *===================================================================*/
extern int _I2C_LOG_;

/*-------------------------------*
   VSYNC INT
 *-------------------------------*/
static irqreturn_t isr_vsync(int irq, void *data)
{
    unsigned long flags;
    struct timespec ts;

    spin_lock_irqsave(&cam_ctrl->vsync_lock, flags);
    LOGI(">>>>> isr_vsync sts:[%d] cnt:[%d] <<<<<\n",cam_ctrl->state, cam_ctrl->vsnyc_cnt);
    switch (cam_ctrl->state) {
    case IRQ_INIT:
        cam_ctrl->state = IRQ_DISABLE;
        break;
    case IRQ_ENABLE:
        ++cam_ctrl->vsnyc_cnt;
        if (cam_ctrl->vsnyc_cnt == 1 && cam_ctrl->ms) {
            ktime_get_ts(&ts);
            if((((ts.tv_sec * 1000) + (ts.tv_nsec/(1000*1000))) - cam_ctrl->ms) < 5){
                --cam_ctrl->vsnyc_cnt;
                cam_ctrl->ms = 0;
                LOGI(">>>>> isr_vsync DUMMY VSYNC !!!! <<<<<\n");
            }
        }
        if (cam_ctrl->vsnyc_cnt >= cam_ctrl->vsnyc_max) {
            cam_ctrl->state = IRQ_DISABLE;
            wake_up(&cam_ctrl->vsync_wait);
        }
        break;
    default:
        cam_ctrl->state = IRQ_DISABLE;
        wake_up(&cam_ctrl->vsync_wait);
        break;
    }
    spin_unlock_irqrestore(&cam_ctrl->vsync_lock, flags);
    return IRQ_HANDLED;
}

/*-------------------------------*
   VSYNC
 *-------------------------------*/
static int cam8mp_sensor_vsync(int32_t count, int32_t ms)
{
    struct timespec ts;
    unsigned long flags;
    int rc = 0;

    LOGI("==> START wait VSYNC cnt:(%d) <==\n", count);

    if (count < 1)  count = 1;
    spin_lock_irqsave(&cam_ctrl->vsync_lock, flags);
    cam_ctrl->vsnyc_max = count;
    cam_ctrl->vsnyc_cnt = 0;
    ktime_get_ts(&ts);
    cam_ctrl->ms = (ts.tv_sec * 1000) + (ts.tv_nsec/(1000*1000));
    cam_ctrl->state = IRQ_ENABLE;
    spin_unlock_irqrestore(&cam_ctrl->vsync_lock, flags);
    disable_irq(INT_VFE);
    if (request_irq(cam_ctrl->vsync_irqno, isr_vsync,
                     IRQF_TRIGGER_RISING, "vsync", 0) < 0) {
        LOGE("request_irq (VSYNC) failed !\n");
    }
    rc = wait_event_timeout(cam_ctrl->vsync_wait,
                            cam_ctrl->state != IRQ_ENABLE,
                            msecs_to_jiffies(ms));
    spin_lock_irqsave(&cam_ctrl->vsync_lock, flags);
    cam_ctrl->state = IRQ_DISABLE;
    disable_irq(cam_ctrl->vsync_irqno);
    free_irq(cam_ctrl->vsync_irqno, 0);
    spin_unlock_irqrestore(&cam_ctrl->vsync_lock, flags);
    if (!rc) {
        LOGI("==> End wait VSYNC Time Out ! <==\n");
    }
    enable_irq(INT_VFE);

    LOGI("==> End wait VSYNC <==\n");
    return 0;
}

/*-------------------------------*
   I2C Control
 *-------------------------------*/
static int cam8mp_sensor_i2c(struct cfg_i2c_cmd* cmd, int log)
{
    struct CameraSensorI2CCmdType   i2c_cmd;
    int   rc = 0;

    if (!cmd->txlen || cmd->txlen >= 1024 || cmd->rxlen >= 1024) {
        LOGE("+%s :I2C Data length error !\n", __func__);
        return -EINVAL;
    }
    if (copy_from_user(pWorkMem, cmd->tx, cmd->txlen)) {
        LOGE(" * %s TX data copy_from_user Error !\n", __func__);
        return -EFAULT;
    }
    if (cmd->rxlen) {
        if (copy_from_user(pWorkMem+cmd->txlen, cmd->rx, cmd->rxlen)) {
            LOGE(" * %s RX data copy_from_user Error !\n", __func__);
            return -EFAULT;
        }
    }

    i2c_cmd.slave_addr = I2C_SLV_ADDR;
    i2c_cmd.pwdata     = pWorkMem;
    i2c_cmd.wlen       = cmd->txlen;
    i2c_cmd.prdata     = pWorkMem+cmd->txlen;
    i2c_cmd.rlen       = cmd->rxlen;
    _I2C_LOG_ = log;
    if (!cmd->rxlen)
        rc = camsensor_gpioi2c_write(&i2c_cmd);
    else {
        if (!(rc = camsensor_gpioi2c_read(&i2c_cmd))) {
            if (copy_to_user(cmd->rx, pWorkMem+cmd->txlen, cmd->rxlen)) {
                LOGE(" * %s RX data copy_to_user Error !\n", __func__);
                rc = -EFAULT;
            }
        }
    }
    _I2C_LOG_ = 0;
    return rc;
}

/*--------------------------------*
   SPI Control
 *--------------------------------*/
static int cam8mp_sensor_spi(struct cfg_spi_cmd *cmd, int rw)
{
    struct spi_message  msg;
    int         i, xcnt, rc = -EFAULT;
    uint8_t     *p = pWorkMem;
    uint32_t    ttl, len;

    if ((rw == 0 || rw == 1) && cmd->len > 0) {
        if (copy_from_user(pWorkMem, cmd->dt, cmd->len)) {
            LOGE("%s * copy_from_user Error !\n", __func__);
            return -EFAULT;
        }
        memset(xfer, 0, sizeof(xfer));
        xcnt = (cmd->len+SPI_BLOCK_SZ-1) / SPI_BLOCK_SZ;

        spi_message_init(&msg);
        for(i=0,ttl=cmd->len;i < xcnt;++i) {
            len = ttl > SPI_BLOCK_SZ ? SPI_BLOCK_SZ : ttl;
            ttl -= len;
            xfer[i].tx_buf = !rw ? p : NULL;
            xfer[i].rx_buf = rw ? p : NULL;
            xfer[i].len = len;
            xfer[i].bits_per_word = SPI_BIT_PER_WORD;
            xfer[i].speed_hz = SPI_SPEED_HZ;
            spi_message_add_tail(&xfer[i], &msg);
            p += len;
        }
        rc = spi_sync(cam8mp_spi_dev, &msg);
        if (!rc) {
            LOGI("%s << SPI TX/RX Complete. <<\n",__func__);
            if (rw) {
                if (copy_to_user(cmd->dt, pWorkMem, cmd->len)) {
                    LOGE("%s * copy_to_user Error !\n", __func__);
                    rc = -EFAULT;
                }
            }
        } else {
            LOGE("%s * spi_sync(): Error (%d)\n", __func__, rc);
        }
    } else {
        LOGE("+%s : Spi tx/rx len error !\n", __func__);
        rc = -EINVAL;
    }
    return rc;
}

/*--------------------------------*
   LED Control
 *--------------------------------*/
static void cam8mp_led_control(int ctrl)
{
    int     rc = 0;
    struct i2c_msg msg;
    uint8_t cmd[2], dt[3] = {0x01,0x31,0x31};

    msg.addr    = LED_2C_SLAVE_ADDR;
    msg.buf     = cmd;
    msg.len     = 2;
    msg.flags   = 0;

    LOGI("+%s (%d)\n", __func__, ctrl);
    switch (ctrl) {
    case LED_OFF:
        {
            mm_segment_t    orgfs;
            struct file     *srcf;
            char   brightness[16] = {0};
            orgfs = get_fs();
            set_fs(KERNEL_DS);
            srcf = filp_open(BACKLIGHT_FILE, O_RDONLY, 0);
            dt[0] = 0x01;
            if (IS_ERR(srcf)) {
                LOGE(" *%s %s Open Error!\n", __func__, BACKLIGHT_FILE);
            }else{
                if ((srcf->f_op == NULL) || (srcf->f_op->read == NULL)) {
                    LOGE(" *%s %s does not have a read method!\n", __func__, BACKLIGHT_FILE);
                }else{
                    memset(brightness,0,sizeof(brightness));
                    if (srcf->f_op->read(srcf, brightness, sizeof(brightness), &srcf->f_pos) < 0 ) {
                        LOGE(" *%s %s read error!\n",__func__, BACKLIGHT_FILE);
                    }else{
                        if (brightness[0] == '0' && brightness[1] == 0x0a) {
                            dt[0] = 0x00;
                        }
                    }
                }
                filp_close(srcf, NULL);
            }
            set_fs(orgfs);
        }
    case LED_LOW:
    case LED_HIGH:
        cmd[0] = 0x00;
        cmd[1] = 0x00;
        if (i2c_transfer(i2c_led, &msg, 1) < 0) {
            pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
        }
        cmd[0] = 0x01;
        cmd[1] = 0x0E;
        if (i2c_transfer(i2c_led, &msg, 1) < 0) {
            pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
        }
        cmd[0] = 0x02;
        cmd[1] = dt[ctrl];
        if (i2c_transfer(i2c_led, &msg, 1) < 0) {
            pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
        }
        LOGI(" cmd[0]=0x%x cmd[1]=0x%x\n",cmd[0],cmd[1]);
        break;
    default:
        break;
    }
    LOGI("-%s Done.\n", __func__);
}

/*-------------------------------*
   Power ON
 *-------------------------------*/
#define VREG_TABLE  5
static struct vreg_info {
    char    *name;
    unsigned int    lvl;
    struct vreg *vreg;
} vreg_info[VREG_TABLE] = {
    {"gp11",  1800, NULL},
    {"gp10",  2800, NULL},
    {"gp5",   1200, NULL},
    {"gp2",   2800, NULL},
    {"lvsw0", 1800, NULL}
};

static int cam8mp_sensor_poweron(void)
{
    int i;
    LOGI("+%s\n", __func__);

    for(i=0;i<VREG_TABLE;++i) {
        if (IS_ERR(vreg_info[i].vreg)) {
            LOGE("%s: * vreg_get(%s) failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
        if (vreg_set_level(vreg_info[i].vreg, vreg_info[i].lvl)) {
            LOGE("%s: * %s set level failed !\n", __func__, vreg_info[i].name);
            return -1;
        }
    }

    if (vreg_enable(vreg_info[0].vreg)) {
        LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[0].name);
        goto _sensor_poweron_fail;
    }
    mdelay(2);

    gpio_tlmm_config(GPIO_CFG(SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(SPI_CS1, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(SPI_MOSI, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_8MA), GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(SPI_MISO, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);

    if (vreg_enable(vreg_info[1].vreg)) {
        LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[1].name);
        goto _sensor_poweron_fail;
    }
    mdelay(1);

    if (vreg_enable(vreg_info[2].vreg)) {
        LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[2].name);
        goto _sensor_poweron_fail;
    }
    mdelay(2);

    gpio_set_value(VDD, 1);
    mdelay(2);

    if (vreg_enable(vreg_info[3].vreg)) {
        LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[3].name);
        goto _sensor_poweron_fail;
    }
    mdelay(1);

    if (vreg_enable(vreg_info[4].vreg)) {
        LOGE("%s: * vreg %s enable failed !\n", __func__, vreg_info[4].name);
        goto _sensor_poweron_fail;
    }
    mdelay(1);

    msm_camio_clk_rate_set(9600000);
    msm_camio_camif_pad_reg_reset();
    gpio_tlmm_config(GPIO_CFG(MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_4MA), GPIO_CFG_ENABLE);
    mdelay(2);

    gpio_set_value(BOOT, 1);
    mdelay(2);

    gpio_set_value(RST, 1);
    mdelay(2);

    LOGI("-%s Done.\n", __func__);

    return 0;

_sensor_poweron_fail:
    for(i=0;i<VREG_TABLE;++i) {
        vreg_disable(vreg_info[i].vreg);
    }

    return -1;
}

/*-------------------------------*
   Power OFF
 *-------------------------------*/
static void cam8mp_sensor_poweroff(void)
{
    LOGI("+%s\n", __func__);

    gpio_tlmm_config(GPIO_CFG(SPI_MISO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(SPI_MISO, 0);
    gpio_tlmm_config(GPIO_CFG(SPI_MOSI, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_8MA), GPIO_CFG_ENABLE);
    gpio_set_value(SPI_MOSI, 0);
    gpio_tlmm_config(GPIO_CFG(SPI_CS1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(SPI_CS1, 0);
    gpio_tlmm_config(GPIO_CFG(SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(SPI_CLK, 0);
    mdelay(1);

    gpio_set_value(RST, 0);
    mdelay(1);

    gpio_tlmm_config(GPIO_CFG(MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
                     GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(MCLK, 0);
    mdelay(1);

    gpio_set_value(BOOT, 0);

    vreg_disable(vreg_info[4].vreg);
    mdelay(1);

    vreg_disable(vreg_info[3].vreg);
    mdelay(10);

    gpio_set_value(VDD, 0);
    mdelay(2);

    vreg_disable(vreg_info[2].vreg);
    mdelay(20);

    vreg_disable(vreg_info[1].vreg);
    mdelay(10);

    vreg_disable(vreg_info[0].vreg);
    mdelay(10);

    LOGI("-%s Done.\n", __func__);
}

/*=====================================================================*
   Driver Function
 *=====================================================================*/
/*---------------------------------------------------------------------*
   msm_open_control
 *---------------------------------------------------------------------*/
int cam8mp_sensor_init(const struct msm_camera_sensor_info *data)
{
    cam_ctrl = kzalloc(sizeof(struct cam_ctrl), GFP_KERNEL);
    if (!cam_ctrl) {
        LOGE(" -%s kzalloc() Failed!\n",__func__);
        return -ENOMEM;
    }

    if (data)
        cam_ctrl->data = data;

    cam_ctrl->vsync_irqno = gpio_to_irq(VSYNC);
    cam_ctrl->state = IRQ_INIT;
    cam_ctrl->vsnyc_cnt = 0;
    init_waitqueue_head(&cam_ctrl->vsync_wait);
    spin_lock_init(&cam_ctrl->vsync_lock);

    if (cam8mp_sensor_poweron() < 0) {
        kfree(cam_ctrl);
        LOGI("-%s Failed.\n", __func__);
        return -1;
    }
    return 0;
}

/*---------------------------------------------------------------------*
   msm_ioctl_control()
 *---------------------------------------------------------------------*/
int cam8mp_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg;
    uint32_t *smem_ptr = NULL;
    int   rc = 0;

    if (copy_from_user(&cfg, (void *)argp, sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    mutex_lock(&cam8mp_mtx);
    switch (cfg.cfgtype) {
    case CFG_I2C_CMD:
        rc = cam8mp_sensor_i2c(&cfg.cfg.i2c_cmd, cfg.rs);
        if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
        break;

    case CFG_SPI_CMD:
        rc = cam8mp_sensor_spi(&cfg.cfg.spi_cmd, cfg.rs);
        if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
        break;

    case CFG_VSYNC:
        rc = cam8mp_sensor_vsync(cfg.cfg.vsync.count, cfg.cfg.vsync.ms);
        break;

    case CFG_SET_LED:
        cam8mp_led_control(cfg.cfg.led);
        break;

    case CFG_GET_TEMP:
        smem_ptr = (uint32_t *)smem_alloc_vendor1(SMEM_OEM_004); 
        if(smem_ptr == NULL){
            LOGE("+%s (CFG_GET_TEMP)\n", __func__);
            rc = -EINVAL;
        } else {
            cfg.cfg.temp = *smem_ptr;
            if (copy_to_user((void *)argp, &cfg, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        }
        break;

    default:
        LOGE("+%s (%d)\n", __func__,cfg.cfgtype);
        rc = -EINVAL;
        break;
    }
    mutex_unlock(&cam8mp_mtx);

    if (rc) LOGE("-%s Done.(%d)\n", __func__, rc);
    return rc;
}

/*---------------------------------------------------------------------*
   msm_release_control()
 *---------------------------------------------------------------------*/
int cam8mp_sensor_release(void)
{
    LOGI("+%s\n", __func__);

    mutex_lock(&cam8mp_mtx);

    cam8mp_led_control(LED_OFF);
    cam8mp_sensor_poweroff();
    kfree(cam_ctrl);

    mutex_unlock(&cam8mp_mtx);

    LOGI("-%s Done.\n", __func__);
    return 0;
}

/*-------------------------------*
   Sensor Driver Setup (Kernel Init)
 *-------------------------------*/
static int __devinit cam8mp_spi_probe(struct spi_device *spi)
{
    int rc = 0;

    cam8mp_spi_dev = spi;
    spi->bits_per_word = SPI_BIT_PER_WORD;
    rc = spi_setup(spi);
    if (rc) LOGE(" - spi_setup Error !\n");
    return rc;
}

static int __devexit cam8mp_spi_remove(struct spi_device *spi)
{
    LOGI("+%s\n", __func__);
    return 0;
}

static struct spi_driver cam8mp_spi_driver = {
    .driver     = {
        .name   = "cam8mp",
        .owner  = THIS_MODULE,
    },
    .probe      = cam8mp_spi_probe,
    .remove     = __devexit_p(cam8mp_spi_remove),
};

static int cam8mp_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
    int i, rc = 0;
    struct i2c_msg msg;
    uint8_t cmd[2];

    s->s_init = cam8mp_sensor_init;
    s->s_release = cam8mp_sensor_release;
    s->s_config  = cam8mp_sensor_config;
    s->s_camera_type = FRONT_CAMERA_2D;
    s->s_mount_angle  = 0;

    pWorkMem = kzalloc((DL_BUFFER_SIZE), GFP_KERNEL);
    LOGI("+%s Dlmem:0x%x\n", __func__, (uint32_t)pWorkMem);
    if(!pWorkMem)
        LOGE(" -%s DlMem kzalloc() Failed!\n",__func__);

    for(i=0;i<VREG_TABLE;++i) {
        vreg_info[i].vreg = vreg_get(NULL, vreg_info[i].name);
        if (IS_ERR(vreg_info[i].vreg)) {
            LOGE("%s: * vreg_get(%s) failed (%ld) !\n",
                __func__, vreg_info[i].name, PTR_ERR(vreg_info[i].vreg));
            break;
        }
    }

    rc = spi_register_driver(&cam8mp_spi_driver);

    i2c_led = i2c_get_adapter(0);
    msg.addr    = LED_2C_SLAVE_ADDR;
    msg.buf     = cmd;
    msg.len     = 2;
    msg.flags   = 0;

    cmd[0] = 0x00;
    cmd[1] = 0x00;
    if (i2c_transfer(i2c_led, &msg, 1) < 0) {
        pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
    }
    cmd[0] = 0x01;
    cmd[1] = 0x0E;
    if (i2c_transfer(i2c_led, &msg, 1) < 0) {
        pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
    }
    cmd[0] = 0x07;
    cmd[1] = 0x5C;
    if (i2c_transfer(i2c_led, &msg, 1) < 0) {
        pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
    }
    cmd[0] = 0x08;
    cmd[1] = 0x5C;
    if (i2c_transfer(i2c_led, &msg, 1) < 0) {
        pr_err("+%s:LED Ctrl I2C ERROR rc=%d\n",__func__, rc);
    }
    return rc;
}

static int __cam8mp_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, cam8mp_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __cam8mp_probe,
    .driver = {
        .name = "msm_camera_cam8mp",
        .owner = THIS_MODULE,
    },
};

static int __init cam8mp_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(cam8mp_init);
