/*
Capacitive touch panel Driver

Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <linux/i2c.h>

#include "k06_captouch.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"

/*#define _debug_msg_*/
#ifdef _debug_msg_
#define debug_log(fmt, args...) printk( KERN_INFO fmt, ## args)
#else
#define debug_log(fmt, args...) do {} while (0)
#endif


#define TS_DRIVER_NAME          "i2c_captouch"

#define I2C_ERR_RETRY           3

#define MAX_HW_POINTNUM         10
#define MAX_INIT_WAIT           500
#define MULTI_READ_INT_WAIT     80

#define TCHINT_STS_NONE         0x00
#define TCHINT_STS_TCHEVT       0x01
#define TCHINT_STS_FORCE_PENUP  0x02
#define TCHINT_STS_FORCE_CALIB  0x04

#define DELTA_CHK_CALIB_CNT         10
#define DELTA_CHK_CALIB_RETRY_MSEC  500
#define DELTA_CHK_TIME_MSEC         10000

#define DELTA_CHK_STS_NONE      0x00
#define DELTA_CHK_STS_CHK_ON    0x01
#define DELTA_CHK_STS_TIMER_ON  0x02
#define DELTA_CHK_STS_CAL_ON    0x04

#define CHK_CRC_RET_SAME_CRC    0x00
#define CHK_CRC_RET_DIFF_CRC    0x01
#define CHK_CRC_RET_CFG_ERR     0x02

#define SEND_CTRL_CONT_MSEC     16
#define SEND_CTRL_INTER_MSEC    26


/* Device information structure */
typedef struct  {
    struct input_dev *input;
    struct timer_list timer;
    int irq;
    int gpio_int;
    int pressure_max;
    int max_point;
    hw_config_data  *default_config;
    struct cdev i2ccap_cdev;
    struct early_suspend i2ccap_esus;
    struct workqueue_struct *i2ccap_wq;
    struct work_struct i2ccap_wq_func;
    struct work_struct i2ccap_wq_calibration_func;
    point_data point[MAX_HW_POINTNUM];
    int tap_now;
    int touch_report_id;
    int dev_id;
    unsigned int delta_chk_flg;
    unsigned long delta_chk_to_jiffies;
    int delta_chk_calib_cnt;
    struct i2c_client *client;
    void ( *reset_control )( int value );
    unsigned long cont_msec;
    unsigned long next_msec;
} i2c_ts_data;

/* Object table register structure */
typedef struct {
    uint8_t type;
    uint8_t start_lsb;
    uint8_t start_msb;
    uint8_t size;
    uint8_t instances;
    uint8_t num_of_report_id;
}element;

/* Information block register structure */
struct i2ccap_information_block{
    uint8_t family_id;
    uint8_t variant_id;
    uint8_t version;
    uint8_t build;
    uint8_t matrix_x_size;
    uint8_t matrix_y_size;
    uint8_t num_of_elements;
    element *elements;
    uint32_t checksum;
};

static struct i2ccap_information_block i2ccap_info;
static int ioctl_irq_sts = 0;
static int cdev_major = 0;
static struct class* udev_class;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void i2ccap_early_suspend(struct early_suspend *h);
static void i2ccap_late_resume(struct early_suspend *h);
#endif

static int i2ccap_hw_reset( i2c_ts_data *ts )
{
    if ( ts == NULL){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }
    
    ts->reset_control(0);
    mdelay(1);
    ts->reset_control(1);
    mdelay(40);

    /* Reset Point status */
    memset( ts->point, 0xFFFFFFFF, sizeof(point_data)*ts->max_point );
    ts->tap_now = 0x00;
    
    return 0;
}


static int i2ccap_i2c_read( i2c_ts_data *ts, uint8_t *buf, uint16_t addr, uint16_t length )
{
    struct i2c_msg    msg[2];
    struct i2c_client *client;
    uint8_t           addr_buff[2];
    int               retval;
    int               retry;
    
    if ( (ts == NULL) || (buf == NULL) || (length == 0) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    client = ts->client;

    addr_buff[0] = addr & 0xff;
    addr_buff[1] = (addr >> 8) & 0xff;

    msg[0].addr     = client->addr;
    msg[0].flags    = 0;
    msg[0].len      = 2;
    msg[0].buf      = addr_buff;

    msg[1].addr     = client->addr;
    msg[1].flags    = I2C_M_RD;
    msg[1].len      = length;
    msg[1].buf      = buf;

    for ( retry = 0 ; retry < I2C_ERR_RETRY ; retry ++ ){
        retval = i2c_transfer(client->adapter, msg, 2);
        if ( retval == 2 ) {
            break;
        }
        printk(KERN_ERR "%s : i2c read error reg:%d -> retry\n", __func__, addr );
    }
    if ( retry >= I2C_ERR_RETRY ){
        printk(KERN_ERR "%s : i2c read error reg:%d retry out\n", __func__, addr );
        return -EIO;
    }
        
    return 0;
}

static int i2ccap_i2c_write( i2c_ts_data *ts, uint8_t *buf, uint16_t addr, uint16_t length )
{
    struct i2c_msg    msg;
    struct i2c_client *client;
    uint8_t           write_buff[257];
    int               retval;
    int               retry;

    if ( (ts == NULL) || (buf == NULL) || (length == 0) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    client = ts->client;

    write_buff[0] = addr & 0xff;
    write_buff[1] = (addr >> 8) & 0xff;
    memcpy( &write_buff[2], buf, length );

    msg.addr     = client->addr;
    msg.flags    = 0;
    msg.len      = length + 2;
    msg.buf      = write_buff;

    for ( retry = 0 ; retry < I2C_ERR_RETRY ; retry ++ ){
        retval = i2c_transfer(client->adapter, &msg, 1);
        if ( retval == 1 ) {
            break;
        }
        printk(KERN_ERR "%s : i2c write error reg:%d -> retry\n", __func__, addr );
    }
    if ( retry >= I2C_ERR_RETRY ){
        printk(KERN_ERR "%s : i2c write error reg:%d retry out\n", __func__, addr );
        return -EIO;
    }

	return 0;
}

static uint16_t get_object_address ( uint8_t id ) 
{
    int cnt;
    
    if  ( i2ccap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < i2ccap_info.num_of_elements ; cnt ++ ){
        if ( i2ccap_info.elements[cnt].type == id ) {
            return (uint16_t)( ((i2ccap_info.elements[cnt].start_msb<<8) & 0xFF00) | (i2ccap_info.elements[cnt].start_lsb & 0xFF) );
        }
    }
    
    return 0;
}

static uint8_t get_object_size ( uint8_t id ) 
{
    int cnt;
    
    if  ( i2ccap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < i2ccap_info.num_of_elements ; cnt ++ ){
        if ( i2ccap_info.elements[cnt].type == id ) {
            return i2ccap_info.elements[cnt].size + 1;
        }
    }

    return 0;
}


static uint8_t report_to_object_id ( uint8_t report_id )
{
    int     cnt;
    uint8_t search_id = 0;
    
    if  ( i2ccap_info.num_of_elements == 0 ) {
        return 0;
    }
    
    for ( cnt = 0 ; cnt < i2ccap_info.num_of_elements ; cnt ++ ){
        search_id += (i2ccap_info.elements[cnt].instances + 1) * (i2ccap_info.elements[cnt].num_of_report_id);
        if ( report_id <= search_id ){
            return i2ccap_info.elements[cnt].type;
        }
    }
    
    return 0;
}

static int get_tch_and_atch ( i2c_ts_data *ts, uint8_t *data )
{
    uint8_t     val ;
    uint8_t     read_buff[128];
    int         retval = 0;
    int         cnt;
    
    if ( (ts==NULL) || (data==NULL) ){
        return -EINVAL;
    }
    
    val = 0xF3;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+5, 1 );

    mdelay(10);

    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 2 ){
        i2ccap_i2c_read( ts, read_buff, get_object_address(DEBUG_DIAGNOSTIC_T37), 2 );
        if ( (read_buff[0]==0xF3) && (read_buff[1]==0x00) ){
            break;
        }
        mdelay(2);
    }
    if ( cnt >= MAX_INIT_WAIT ){
        retval = -1;
    }
    
    i2ccap_i2c_read( ts, read_buff, get_object_address(DEBUG_DIAGNOSTIC_T37), 82 );
    memcpy ( (uint8_t *)data, read_buff, 82 );
    
#ifdef _debug_msg_
    {
        int j;
        int offset=2; 
        printk(KERN_INFO "ObjectNo %02d Register values(tch) buf[0]=0x%x, bur[1]=0x%x\n", DEBUG_DIAGNOSTIC_T37, data[0], data[1] );
        printk(KERN_ERR "      :tch_data |atch_data\n");
        for ( j = 0 ; j < 19 ; j ++ ) {
            printk(KERN_ERR "  %03d :  0x%01x%02x, |  0x%01x%02x \n",
                j, 
                data[offset+1],  data[offset],
                data[offset+41], data[offset+40]
            );
            offset += 2;
        }
    }
#endif

    val = 0x01;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+5, 1 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt ++ ){
        i2ccap_i2c_read( ts, read_buff, get_object_address(DEBUG_DIAGNOSTIC_T37), 2 );
        if ( (read_buff[0]==0xF3) && (read_buff[1]==0x01) ){
            break;
        }
        mdelay(1);
    }
    if ( cnt >= MAX_INIT_WAIT ){
        retval = -1;
    }

    return retval;
}

static int check_tch_and_atch( i2c_ts_data *ts )
{
    int         retval = 0;
    uint8_t     tmp_tch[128];
    char        tch_cnt, atch_cnt;
    uint8_t     check_mask;
    char        cnt, shift;

    if ( ts == NULL ){
        return -EINVAL;
    }

    retval = get_tch_and_atch ( ts, tmp_tch );
    if ( retval != 0x00 ){
        return -EAGAIN;
    }
    
    tch_cnt = 0;
    atch_cnt = 0;

    for ( cnt = 0 ; cnt < i2ccap_info.matrix_x_size*2 ; cnt += 2 ){
        for ( shift = 0 ; shift < 8 ; shift ++ ){
            check_mask = 1 << shift;
            if ( tmp_tch[2+cnt]&check_mask ){
                tch_cnt ++;
            }
            if ( tmp_tch[3+cnt]&check_mask ){
                tch_cnt ++;
            }
            if ( tmp_tch[42+cnt] & check_mask ){
                atch_cnt ++;
            }
            if ( tmp_tch[43+cnt] & check_mask ){
                atch_cnt ++;
            }
        }
        
    }

#ifdef _debug_msg_
    printk(KERN_ERR "%s : tch_cnt = %d, atch_cnt  = %d\n", __func__, tch_cnt, atch_cnt );
#endif
    
    if ( (tch_cnt>0) && (atch_cnt==0) ){
        retval = 0;
    }else if ( (tch_cnt+DELTA_CHK_CALIB_CNT) <= atch_cnt ){
        retval = -1;
    }else{
        retval = -EAGAIN;
    }

    return retval;

}

static int check_config_crc( i2c_ts_data *ts, int check_crc )
{
    uint8_t     val ;
    uint32_t    get_crc = 0 ;
    char        read_buff[16];
    int         cnt;
    char        cfg_err = 0;

    if ( ts == NULL ) {
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }
    
    if ( ts->irq ){
        disable_irq( ts->irq );
    }
    
    /* Start calibration for Get Message include NV-Memory CRC */
    val = 1;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+2, 1 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            i2ccap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) && (read_buff[1]&0x08) ){
                cfg_err = 0x01;
                break;
            }
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) && (read_buff[1]&0x10) ){
                break;
            }
        }
        mdelay( 10 );
    }
    get_crc = ((read_buff[4] << 16)&0x00FF0000) | ((read_buff[3] << 8)&0x0000FF00) | read_buff[2];

    if ( ts->irq ){
        enable_irq( ts->irq );
    }

    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : message wait timeout\n", __func__ );
        cfg_err = 0x01;
    }

    printk(KERN_ERR "%s : HW-CRC = 0x%06x, Default-CRC = 0x%06x, CFG-ERR Flag = %d\n", __func__, get_crc, check_crc, cfg_err );
    
    if ( cfg_err != 0 ){
        return CHK_CRC_RET_CFG_ERR;
    }
    /* Check CRC and config registers */
    if ( get_crc != check_crc ) {
        return CHK_CRC_RET_DIFF_CRC;
    }
    
    return CHK_CRC_RET_SAME_CRC;
}

static int backup_config_registers( i2c_ts_data *ts )
{
    uint8_t val ;
    char    read_buff[16];
    int     retval = 0;
    int     cnt;
    int     wrk_crc = 0 ;
    
    printk(KERN_ERR "%s :[IN]\n", __func__ );
    
    if ( ts == NULL ) {
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    if ( ts->irq ){
        disable_irq( ts->irq );
    }

    val = 0x55;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+1, 1 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            i2ccap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) ){
                wrk_crc = ((read_buff[4] << 16)&0x00FF0000) | ((read_buff[3] << 8)&0x0000FF00) | read_buff[2];
#ifdef _debug_msg_
                printk(KERN_ERR "%s : Get CRC def=0x%06x, ret=0x%06x\n", __func__, ts->default_config->config_crc, wrk_crc );
#endif
                if (wrk_crc == ts->default_config->config_crc ){
                    retval = wrk_crc;
                    break;
                }
            }
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : backup command response timeout or CRC is not match(cfg=0x%06x, wrk=0x%06x)\n", __func__, ts->default_config->config_crc, wrk_crc );
        retval = -EIO;
    }
    printk( KERN_INFO "%s : send backup command is done\n", __func__ );
    
    /* Reset Device */
    val = 0x01;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6), 1 );
    mdelay( 64 );
    for ( cnt = 0 ; cnt < MAX_INIT_WAIT ; cnt += 10 ){
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            i2ccap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 5 );
            if ( (report_to_object_id(read_buff[0])==GEN_COMMANDPROCESSOR_T6) && (read_buff[1]&0x80) ){
                break;
            }
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "%s : reset command response timeout\n", __func__ );
        retval = -EIO;
    }
    printk( KERN_INFO "%s : send software reset command is done\n", __func__ );

    if ( ts->irq ){
        enable_irq( ts->irq );
    }
    return retval;

}

static int set_config_registers( i2c_ts_data *ts, hw_config_data *config_data )
{
    int retval ;
    
    printk(KERN_ERR "%s :[IN]\n", __func__ );
    
    if ( (ts==NULL) || (config_data == NULL) ) {
        printk(KERN_ERR "%s :parameter is NULL \n", __func__ );
        return -EINVAL;
    }

    /* write registers */
    retval = i2ccap_i2c_write( ts, config_data->config_t7, get_object_address(GEN_POWERCONFIG_T7),
                       get_object_size(GEN_POWERCONFIG_T7) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t8, get_object_address(GEN_ACQUIRECONFIG_T8),
                       get_object_size(GEN_ACQUIRECONFIG_T8) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t9, get_object_address(TOUCH_MULTITOUCHSCREEN_T9),
                       get_object_size(TOUCH_MULTITOUCHSCREEN_T9) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t15, get_object_address(TOUCH_KEYARRAY_T15),
                       get_object_size(TOUCH_KEYARRAY_T15) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t18, get_object_address(SPT_COMCONFIG_T18),
                       get_object_size(SPT_COMCONFIG_T18) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t19, get_object_address(SPT_GPIOPWM_T19),
                       get_object_size(SPT_GPIOPWM_T19) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t20, get_object_address(PROCI_GRIPFACESUPPRESSION_T20),
                       get_object_size(PROCI_GRIPFACESUPPRESSION_T20) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t22, get_object_address(PROCG_NOISESUPPRESSION_T22),
                       get_object_size(PROCG_NOISESUPPRESSION_T22) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t23, get_object_address(TOUCH_PROXIMITY_T23),
                       get_object_size(TOUCH_PROXIMITY_T23) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t24, get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24),
                       get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t25, get_object_address(SPT_SELFTEST_T25),
                       get_object_size(SPT_SELFTEST_T25) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t27, get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27),
                       get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27) );
    if ( retval != 0)   goto err_out;

    retval = i2ccap_i2c_write( ts, config_data->config_t28, get_object_address(SPT_CTECONFIG_T28),
                       get_object_size(SPT_CTECONFIG_T28) );
    if ( retval != 0)   goto err_out;

#ifdef _debug_msg_
    {
        char read_buff[256];
        int i, j;
        int id;
        
        for ( i = 0 ; i <  i2ccap_info.num_of_elements ; i ++ ) {
            id = i2ccap_info.elements[i].type;
            i2ccap_i2c_read( ts, read_buff, get_object_address( i2ccap_info.elements[i].type ), get_object_size( i2ccap_info.elements[i].type ) );
            printk(KERN_INFO "ObjectNo %02d Register values \n", i2ccap_info.elements[i].type );
            for ( j = 0 ; j < get_object_size(i2ccap_info.elements[i].type)  ; j += 10 ) {
                printk(KERN_ERR "  %03d :  %03d, %03d, %03d, %03d, %03d - %03d, %03d, %03d, %03d, %03d \n",
                    j, 
                    read_buff[j],   read_buff[j+1], read_buff[j+2], read_buff[j+3], read_buff[j+4],
                    read_buff[j+5], read_buff[j+6], read_buff[j+7], read_buff[j+8], read_buff[j+9]
                );
            }
        }
    }
#endif

    return 0;

err_out:
    printk(KERN_ERR "%s : i2c_write_err ret = %d \n", __func__, retval );
    return retval;
    
}

static int reset_configration( i2c_ts_data *ts )
{
    int     retval;
    
    printk(KERN_ERR "%s :[IN]\n", __func__ );

    /* Set config registers ( Excluding the UserData ) */
    retval = set_config_registers( ts, ts->default_config );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : Write Config Error ret = %d \n", __func__, retval );
        return retval;
    }

    /* backup config registers */
    retval = backup_config_registers( ts );
    if (retval < 0){
        return -EIO;
    }
    printk(KERN_ERR "%s : reset Configuration is success. New CRC is 0x%06x. \n", __func__, retval );

    return retval;

}

static int i2ccap_hardware_init( i2c_ts_data *ts, i2c_captouch_platform_data *pdata )
{
    uint8_t read_buff[512]  = {0};
    int     cnt             = 0;
    int     report_id       = 1;
    int     retval;
    hw_config_data set_zero_config = {0};
    
    if ( (ts == NULL) || (pdata == NULL) ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return -EINVAL;
    }

    /*----------------------------- Wait power on ----------------------------*/
    pdata->vreg_config(1);
    for ( cnt = 0 ; cnt <= MAX_INIT_WAIT ; cnt += 10 ){
        /* wait for enable interrupt pin */
        if ( gpio_get_value( ts->gpio_int ) == 0x00 ){
            /* do not read a Message register                           */
            /* Because it does not understand the offset at this timing.*/
            break;
        }
        mdelay( 10 );
    }
    if ( cnt >= MAX_INIT_WAIT ){
        printk(KERN_ERR "[captouch] Warning : INT is not enable, power on after %d msec\n", MAX_INIT_WAIT );
    }


    /*----------------------- area regstry init --------------------------*/
    retval = i2ccap_i2c_read( ts, (uint8_t *)&i2ccap_info, 0, 7 );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : Information block read error retval = %d\n", __func__, retval );
        return -EIO;
    }
    i2ccap_info.elements = kzalloc( i2ccap_info.num_of_elements*sizeof(element), GFP_KERNEL );
    if ( i2ccap_info.elements == NULL ) {
        printk(KERN_ERR "%s : element structure allocation error \n", __func__ );
        return -ENOMEM;
    }

#ifdef _debug_msg_
    printk(KERN_INFO "family_id       %d\n", i2ccap_info.family_id );
    printk(KERN_INFO "variant_id      %d\n", i2ccap_info.variant_id );
    printk(KERN_INFO "version         %d\n", i2ccap_info.version );
    printk(KERN_INFO "build           %d\n", i2ccap_info.build );
    printk(KERN_INFO "matrix_x_size   %d\n", i2ccap_info.matrix_x_size );
    printk(KERN_INFO "matrix_y_size   %d\n", i2ccap_info.matrix_y_size );
    printk(KERN_INFO "num_of_elements %d\n\n", i2ccap_info.num_of_elements );
#endif

    retval = i2ccap_i2c_read( ts, (uint8_t *)i2ccap_info.elements, 7, i2ccap_info.num_of_elements*sizeof(element) );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : element block read error retval = %d\n", __func__, retval );
        return -EIO;
    }

#ifdef _debug_msg_
    {
        int i = 0;
        for ( i=0; i<i2ccap_info.num_of_elements; i++ ) {
            printk(KERN_INFO "type             %d\n", i2ccap_info.elements[i].type);
            printk(KERN_INFO "start_lsb        %d\n", i2ccap_info.elements[i].start_lsb);
            printk(KERN_INFO "start_msb        %d\n", i2ccap_info.elements[i].start_msb);
            printk(KERN_INFO "  -->StartOffsets 0x%x(%d)\n",
                ( ((i2ccap_info.elements[i].start_msb<<8) & 0xFF00) | (i2ccap_info.elements[i].start_lsb & 0xFF) ),
                ( ((i2ccap_info.elements[i].start_msb<<8) & 0xFF00) | (i2ccap_info.elements[i].start_lsb & 0xFF) )
            );
            printk(KERN_INFO "size-1           %d\n", i2ccap_info.elements[i].size);
            printk(KERN_INFO "instances-1      %d\n", i2ccap_info.elements[i].instances);
            printk(KERN_INFO "num_of_report_id %d\n\n", i2ccap_info.elements[i].num_of_report_id);
        }
    }
#endif

    for( cnt=0; cnt<i2ccap_info.num_of_elements; cnt++ ) {
        if( i2ccap_info.elements[cnt].type == TOUCH_MULTITOUCHSCREEN_T9 ) {
            printk(KERN_ERR "touch_report_id = %d\n", report_id);
            ts->touch_report_id = report_id;
            break;
        }
        report_id += (i2ccap_info.elements[cnt].instances + 1) * (i2ccap_info.elements[cnt].num_of_report_id);
    }
    
    ts->dev_id = 0;
    ts->default_config = &pdata[0].hw_config ;

    retval = i2ccap_i2c_read( ts, read_buff, get_object_address(SPT_USERDATA_T38), get_object_size(SPT_USERDATA_T38) );
    if ( retval != 0 ){
        printk(KERN_ERR "%s : user data read error retval = %d\n", __func__, retval );
        return -EIO;
    }

    if ( read_buff[1] != 0x01 ){
        retval = check_config_crc( ts, ts->default_config->config_crc );
        
        if ( retval != CHK_CRC_RET_SAME_CRC ){
            if ( retval == CHK_CRC_RET_CFG_ERR ){
                printk(KERN_ERR "%s : CFG_ERR bit is ON. write All 0x00 Configrations. \n", __func__ );
                ts->default_config = &set_zero_config;
                reset_configration( ts );
                ts->default_config = &pdata[0].hw_config ;
            }
            printk(KERN_ERR "%s : CRC is not match. reset Configrations \n", __func__ );
            reset_configration( ts );
        }
    }

    return 0;
}

static irqreturn_t i2ccap_interrupt( int irq, void *dev_id )
{
    i2c_ts_data   *ts = dev_id;

    disable_irq_nosync( ts->irq );
    queue_work( ts->i2ccap_wq, &ts->i2ccap_wq_func );

    return IRQ_HANDLED;

}

static void interrupt_work_func( struct work_struct *work )
{

    uint32_t    x, y;
    uint8_t     read_buff[64];
    uint8_t     val;
    int         tap_pos;
    int         tch_flg = 0;
    int         cnt;
    int         retval;
    i2c_ts_data *ts = container_of( work, i2c_ts_data, i2ccap_wq_func );
    struct timeval tv;
    unsigned long now_msec;

    /* Message Check */
    for ( cnt = 0 ; cnt < ts->max_point ; cnt ++ ){

        /* Check INT */
        if ( gpio_get_value( ts->gpio_int ) != 0x00 ){
            break;
        }

        /* read Message register */
        retval = i2ccap_i2c_read( ts, read_buff, get_object_address(GEN_MESSAGEPROCESSOR_T5), 8 );
        if ( retval != 0 ){
            printk( KERN_ERR "%s : I2C read error. retval = %d\n", __func__, retval );
            tch_flg = TCHINT_STS_NONE;
            input_report_abs( ts->input, ABS_MT_TOUCH_MAJOR, 0 );
            input_sync( ts->input );
            break;
        }

        switch ( report_to_object_id(read_buff[0]) ){

            case TOUCH_MULTITOUCHSCREEN_T9:
                /* check validate point data */
                tap_pos = read_buff[0] - ts->touch_report_id;
                if ( tap_pos >= ts->max_point ){
                    printk(KERN_INFO "invalid TapID ID=%d, x=%d, y=%d, area=%d\n", tap_pos, ts->point[tap_pos].x, ts->point[tap_pos].y, ts->point[tap_pos].area );
                    break;
                }

                /* set new point data */
                if( read_buff[1] & 0x20 ) {
                    ts->point[tap_pos].x = 0xFFFFFFFF;
                    ts->point[tap_pos].y = 0xFFFFFFFF;
                    ts->point[tap_pos].area = 0xFFFFFFFF;
                    ts->point[tap_pos].delta = 0xFFFFFFFF;
                    ts->tap_now &= ~((int)(0x0001 << tap_pos));
#ifdef _debug_msg_
                    printk( KERN_ERR "%s : Pen Up!(%d)\n", __func__, tap_pos );
#endif
                }else if( read_buff[1] & 0x80 ) {
                    x = (uint32_t)( ((read_buff[2] << 8) | (read_buff[4] & 0xC0)) >> 6 );
                    y = (uint32_t)( ((read_buff[3] << 4) | (read_buff[4] & 0x0C)) >> 2 );

                    ts->point[tap_pos].x = y;
                    ts->point[tap_pos].y = x;
                    ts->point[tap_pos].area = read_buff[5];
                    ts->point[tap_pos].delta = read_buff[6];

                    if ( read_buff[1] & 0x40 ){
#ifdef _debug_msg_
                        printk( KERN_ERR "%s : Pen Pressed!(%d)\n", __func__, tap_pos );
#endif
                        if ( ts->tap_now & (int)(0x0001 << tap_pos) ){
                            printk( KERN_ERR "************* %s : Pen Up Error tap_now = 0x%x, tap_pos = %d****************\n", __func__, ts->tap_now, tap_pos );
                            tch_flg = TCHINT_STS_NONE;
                        }

                        if ( ts->delta_chk_flg & DELTA_CHK_STS_CHK_ON ){
                            retval = check_tch_and_atch( ts );
                            if ( retval == 0 ){
                                if ( !(ts->delta_chk_flg&DELTA_CHK_STS_TIMER_ON) ){
                                    ts->delta_chk_to_jiffies = jiffies + msecs_to_jiffies( DELTA_CHK_TIME_MSEC );
                                    ts->delta_chk_flg |= DELTA_CHK_STS_TIMER_ON;
#ifdef _debug_msg_
                                    printk(KERN_INFO "%s : delta OK . Timer Start jiffies = %ld, flg=0x%x\n", __func__, ts->delta_chk_to_jiffies, ts->delta_chk_flg );
#endif

                                }else if ( (long)jiffies - (long)ts->delta_chk_to_jiffies >= 0 ){
                                    val = ts->default_config->config_t8[7];
                                    i2ccap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );
                                    ts->delta_chk_flg = DELTA_CHK_STS_NONE;
#ifdef _debug_msg_
                                    printk(KERN_INFO "%s : delta OK flg = 0x%x. Timer End. Clear Flag \n", __func__, ts->delta_chk_flg );
#endif
                                }
                                if ( ts->delta_chk_flg & DELTA_CHK_STS_CAL_ON  ){
                                    del_timer_sync( &ts->timer );
                                    ts->delta_chk_flg &= ~DELTA_CHK_STS_CAL_ON ;
                                }
                                
                            }else if ( retval == -EAGAIN ){
#ifdef _debug_msg_
                                printk(KERN_INFO "%s : delta check again. .\n", __func__ );
#endif
                                if ( !(ts->delta_chk_flg&DELTA_CHK_STS_TIMER_ON) ){
                                    ts->delta_chk_to_jiffies = jiffies + msecs_to_jiffies( DELTA_CHK_TIME_MSEC );
                                }
                                
                            }else{
                                ts->delta_chk_flg &= ~DELTA_CHK_STS_TIMER_ON;
                                tch_flg |= TCHINT_STS_FORCE_CALIB | TCHINT_STS_FORCE_PENUP;
#ifdef _debug_msg_
                                printk(KERN_INFO "%s : delta check err . force PenUp.\n", __func__ );
#endif
                            }
                        }
                        if ( ts->tap_now == 0x00 ){
                            do_gettimeofday(&tv);
                            now_msec = ( ((unsigned long)tv.tv_sec*1000000) + (unsigned long)tv.tv_usec ) / 1000;
                            ts->next_msec = now_msec;
                            ts->cont_msec = now_msec + SEND_CTRL_CONT_MSEC; 
                        }
                    }

                    ts->tap_now |= (int)(0x0001 << tap_pos);
                }
                tch_flg |= TCHINT_STS_TCHEVT;
                break;

            case PROCI_GRIPFACESUPPRESSION_T20:
#ifdef _debug_msg_
                printk(KERN_INFO "%s : I got PROCI_GRIPFACESUPPRESSION_T20 Messages. 0x%02x\n", __func__, read_buff[1] );
#endif
                tch_flg |= TCHINT_STS_FORCE_PENUP;
                if ( (read_buff[1]&0x01) && (ts->delta_chk_flg&DELTA_CHK_STS_CHK_ON) ){
                    ts->delta_chk_flg &= ~DELTA_CHK_STS_TIMER_ON;
                    tch_flg |= TCHINT_STS_FORCE_CALIB;
                }
                break;

            case PROCG_NOISESUPPRESSION_T22:
#ifdef _debug_msg_
                printk(KERN_INFO "%s : I got PROCG_NOISESUPPRESSION_T22 Messages. 0x%02x, 0x%02x, 0x%02x\n", __func__, read_buff[1], read_buff[2], read_buff[3] );
#endif
                if ( (read_buff[1]&0x08) && (ts->delta_chk_flg&DELTA_CHK_STS_CHK_ON) ){
                    ts->delta_chk_flg &= ~DELTA_CHK_STS_TIMER_ON;
                    tch_flg |= TCHINT_STS_FORCE_CALIB | TCHINT_STS_FORCE_PENUP;
                }
                break;
            
            case GEN_COMMANDPROCESSOR_T6:
#ifdef _debug_msg_
                printk(KERN_INFO "%s : I got GEN_COMMANDPROCESSOR_T6 Messages. Status = 0x%02x\n", __func__, read_buff[1] );
#endif
                if ( ts->delta_chk_flg & DELTA_CHK_STS_CAL_ON  ){
                    if ( read_buff[1] & 0x10 ){
#ifdef _debug_msg_
                        printk(KERN_INFO "%s : calibration complete \n", __func__ );
#endif
                        del_timer_sync( &ts->timer );
                        ts->delta_chk_flg &= ~DELTA_CHK_STS_CAL_ON ;
                    }
                }
                break;
            default:
#ifdef _debug_msg_
                printk(KERN_INFO "Type = %d, ReadData= 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n",
                    report_to_object_id(read_buff[0]),
                    read_buff[0], read_buff[1], read_buff[2], read_buff[3],
                    read_buff[4], read_buff[5], read_buff[6], read_buff[7], read_buff[8]
                );
#endif
                break;
        }
        
        udelay( MULTI_READ_INT_WAIT );
    }

    if ( tch_flg & TCHINT_STS_FORCE_CALIB ){
#ifdef _debug_msg_
        printk(KERN_INFO "%s : TCHINT_STS_FORCE_CALIB bit is ON. 0x%x\n", __func__, tch_flg );
#endif
        if ( !(ts->delta_chk_flg&DELTA_CHK_STS_CAL_ON ) ){
#ifdef _debug_msg_
            printk(KERN_INFO "%s : calibration start.\n", __func__ );
#endif
            val = 1;
            i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+2, 1 );
            ts->delta_chk_flg |= DELTA_CHK_STS_CAL_ON ;
            ts->timer.expires = jiffies + msecs_to_jiffies( DELTA_CHK_CALIB_RETRY_MSEC );
            ts->delta_chk_calib_cnt = 0;
            add_timer( &ts->timer );
        }
    }
    
    if ( tch_flg & TCHINT_STS_FORCE_PENUP ){
#ifdef _debug_msg_
        printk(KERN_INFO "%s : TCHINT_STS_FORCE_PENUP bit is ON. 0x%x\n", __func__, tch_flg );
#endif
        memset ( ts->point, 0xFF, sizeof(point_data)*ts->max_point );
        ts->tap_now = 0;
        tch_flg |= TCHINT_STS_TCHEVT;
    }

    if ( tch_flg & TCHINT_STS_TCHEVT ){
        if ( ts->tap_now == 0x00 ) {
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
            input_sync(ts->input);
        }else{
            do_gettimeofday(&tv);
            now_msec = ( ((unsigned long)tv.tv_sec*1000000) + (unsigned long)tv.tv_usec ) / 1000;
            if ( (long)(now_msec - ts->next_msec) >= 0 ){
                for ( cnt = 0 ; cnt < ts->max_point ; cnt ++ ) {
                    if ( ts->tap_now & (int)(0x0001<<cnt) ) {
#ifdef _debug_msg_
                        printk(KERN_INFO "pen down ! ID=%d, x=%d, y=%d, area=%d\n",
                               cnt, ts->point[cnt].x, ts->point[cnt].y,
                               ts->point[cnt].area
                        );
#endif
                        input_report_abs(ts->input, ABS_MT_TRACKING_ID, cnt);
                        input_report_abs(ts->input, ABS_MT_POSITION_X, ts->point[cnt].x);
                        input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->point[cnt].y);
                        input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ts->pressure_max);
                        input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, ts->point[cnt].area);
                        input_mt_sync(ts->input);

                    }
                }
                if ( (long)(now_msec - ts->cont_msec) >= 0 ){
                    ts->next_msec = now_msec + SEND_CTRL_INTER_MSEC;
                } 
            }
        }
        input_sync(ts->input);
    }

    enable_irq(ts->irq);

    return;
}


static void calibration_timeout_func( unsigned long ptr )
{
    i2c_ts_data *ts = (i2c_ts_data *)ptr;

    queue_work( ts->i2ccap_wq, &ts->i2ccap_wq_calibration_func );

}

static void calibration_work_func( struct work_struct *work )
{
    i2c_ts_data *ts = container_of( work, i2c_ts_data, i2ccap_wq_calibration_func );
    char    val = 0x00;

    disable_irq( ts->irq );
    
    if ( ts->delta_chk_calib_cnt >= DELTA_CHK_CALIB_CNT ){
        printk(KERN_INFO "calibration retry out!\n" );
        ts->delta_chk_flg &= ~DELTA_CHK_STS_CAL_ON ;
    }else{
        val = 1;
        i2ccap_i2c_write( ts, &val, get_object_address(GEN_COMMANDPROCESSOR_T6)+2, 1 );
        ts->delta_chk_calib_cnt ++ ;

        ts->timer.expires = jiffies + msecs_to_jiffies( DELTA_CHK_CALIB_RETRY_MSEC );
        add_timer( &ts->timer );
    }
    

    enable_irq(ts->irq);

}

static int i2ccap_cdev_open( struct inode *inode, struct file *file )
{
    i2c_ts_data *ts = container_of( inode->i_cdev, i2c_ts_data, i2ccap_cdev );
    
    printk( KERN_INFO "%s :[IN]\n", __func__ );
    file->private_data = ts;
    return 0;
}

static int i2ccap_cdev_release(struct inode *inode, struct file *file)
{
    printk( KERN_INFO "%s :[IN]\n", __func__ );

    return 0;
}

static int i2ccap_cdev_ioctl( struct inode    *inode,
                               struct file     *file,
                               unsigned int    cmd,
                               unsigned long   arg    )
{
    int             retval = 0;
    i2c_ts_data    *ts = (i2c_ts_data *)file->private_data;
    
    printk( KERN_INFO "%s(cmd=0x%08x) :[IN]\n", __func__, cmd );
    
    switch(cmd){
        case IOCTL_SET_DBGMODE:
            if ( ioctl_irq_sts == 0 ){
                free_irq(ts->irq, ts);
                ioctl_irq_sts = 1; 
            }else{
                retval = request_irq( ts->irq, i2ccap_interrupt, IRQF_TRIGGER_LOW, "touchscreen", ts );
                ioctl_irq_sts = 0; 
            }
            break;

        default:
            printk( KERN_INFO "%s DBG : show All IOCTL Command code\n", __func__ );
            printk( KERN_INFO "  IOCTL_SET_DBGMODE = 0x%x\n", IOCTL_SET_DBGMODE );
            break;
    }
    printk( KERN_INFO "%s(cmd=0x%08x) :[OUT] retval = %d\n", __func__, cmd, retval );
    return retval;
}

static struct file_operations i2ccap_fops = {
    .owner   = THIS_MODULE,
	.release = i2ccap_cdev_release,
    .open    = i2ccap_cdev_open,
    .ioctl   = i2ccap_cdev_ioctl,
};

static int i2ccap_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int result;
    struct input_dev *input_dev = 0;
    i2c_ts_data *ts;
    i2c_captouch_platform_data *pdata = client->dev.platform_data;
    int devno;
    uint8_t val;
    dev_t dev = MKDEV(cdev_major, 0);

    printk( KERN_INFO "%s :[IN] use HW-I2C\n", __func__ );

    ts = kzalloc(sizeof(i2c_ts_data), GFP_KERNEL);
    if ( ts == NULL ) {
        printk( KERN_ERR"%s: alloc ts failed\n", __func__ );
        result = -ENOMEM;
        goto fail_alloc_ts;
    }

    /* Make charactor device    */
    udev_class = class_create(THIS_MODULE, "i2c_cap");
    if(IS_ERR(udev_class)) {
        result = PTR_ERR(udev_class);
        printk( KERN_ERR"%s: class_create failed result = %d\n", __func__, result );
        goto fail_class_create;
    }

    /* Figure out our device number. */
    result = alloc_chrdev_region(&dev, 0, 1, "i2c_cap");
    cdev_major = MAJOR(dev);
    if (result < 0) {
        printk( KERN_ERR"%s: alloc_chrdev_region failed result = %d\n", __func__, result );
        goto fail_cdev_region;
    }
    if (cdev_major == 0)
        cdev_major = result;
  
    /* Now set up two cdevs. */
    devno = MKDEV( cdev_major, 0 ); 
    cdev_init( &(ts->i2ccap_cdev), &i2ccap_fops );
    ts->i2ccap_cdev.owner = THIS_MODULE;
    ts->i2ccap_cdev.ops = &i2ccap_fops;
    result = cdev_add ( &(ts->i2ccap_cdev), devno, 1 );
    if(result){
        printk( KERN_ERR"%s: cdev_add failed result = %d\n", __func__, result );
        goto fail_cdev_add;
    }
    
    if ( IS_ERR(device_create(udev_class, NULL, devno, NULL, "i2c_cap")) ){
        printk(KERN_ERR "can't create device\n");
        goto fail_dev_create;
    }

    input_dev = input_allocate_device();
    if ( input_dev == NULL ) {
        result = -ENOMEM;
        printk( KERN_ERR"%s: input_allocate_device failed result = %d\n", __func__, result );
        goto fail_alloc_input;
    }
    input_dev->name = TS_DRIVER_NAME;
    input_dev->phys = "i2ccap_info/input0";
    input_dev->dev.parent = &client->dev;

    ts->max_point = pdata->max_point;
    ts->gpio_int = pdata->gpio_int;
    ts->pressure_max = pdata->pressure_max;

    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

    /* finger position */
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, pdata->max_point-1, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, pdata->pressure_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);

    result = input_register_device(input_dev);
    if ( result ){
        printk( KERN_ERR"%s: input_register_device failed result = %d\n", __func__, result );
        goto fail_input_register;
    }
    
    ts->input = input_dev;
    ts->client = client;
    ts->reset_control = pdata->reset_control;

    result = i2ccap_hardware_init( ts, pdata );
    if ( result ){
        printk( KERN_ERR"%s: hardware init failed result = %d\n", __func__, result );
        goto fail_hw_init;
    }

    /* workqueue init   */
    ts->i2ccap_wq = create_singlethread_workqueue( "i2ccap_wq" );
    if ( ts->i2ccap_wq == NULL ) {
        printk( KERN_ERR"%s: create workqueue failed\n", __func__ );
        result = -ENOMEM;
        goto fail_create_wq;
    }
    INIT_WORK( &ts->i2ccap_wq_func, interrupt_work_func );
    INIT_WORK( &ts->i2ccap_wq_calibration_func, calibration_work_func );

    /* irq init */
    ts->irq = MSM_GPIO_TO_INT( ts->gpio_int );
    result = request_irq(ts->irq, i2ccap_interrupt, IRQF_TRIGGER_LOW, "touchscreen", ts);
    if (result){
        printk( KERN_ERR"%s: request_irq failed result = %d\n", __func__, result );
        goto fail_req_irq;
    }
    
    i2c_set_clientdata(client, ts);

    setup_timer( &ts->timer, calibration_timeout_func, (unsigned long)ts );

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->i2ccap_esus.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
    ts->i2ccap_esus.suspend = i2ccap_early_suspend;
    ts->i2ccap_esus.resume = i2ccap_late_resume;
    register_early_suspend( &ts->i2ccap_esus );
#endif
    
    /* setup check delta items */
    val = ts->default_config->resume_atchcalsthr;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );
    ts->delta_chk_flg = DELTA_CHK_STS_CHK_ON;

    return 0;

fail_req_irq:
    destroy_workqueue( ts->i2ccap_wq );
fail_create_wq:
fail_hw_init:
    input_unregister_device( input_dev );
    input_dev = NULL;
fail_input_register:
    input_free_device( input_dev );
fail_alloc_input:
    device_destroy( udev_class, MKDEV(cdev_major, 0) );
fail_dev_create:
    cdev_del( &(ts->i2ccap_cdev) );
fail_cdev_add:
fail_cdev_region:
    class_destroy( udev_class );
fail_class_create:
    kfree(ts);
fail_alloc_ts:
    printk(KERN_INFO "***** TOUCH_DEBUG : ERROR EXIT %s\n", __func__);
    return result;

}


static int i2ccap_i2c_remove(struct i2c_client *client)
{
    i2c_ts_data *ts = i2c_get_clientdata( client );
    i2c_captouch_platform_data *pdata = client->dev.platform_data;
    dev_t dev = MKDEV(cdev_major, 0);

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    free_irq(ts->irq, ts);
    pdata->vreg_config(0);

    destroy_workqueue( ts->i2ccap_wq );
    input_unregister_device( ts->input );
    input_free_device( ts->input );

    device_destroy( udev_class, MKDEV(cdev_major, 0) );
    class_destroy( udev_class );
    cdev_del( &(ts->i2ccap_cdev) );
    unregister_chrdev_region( dev, 1);
    
    i2c_set_clientdata(client, NULL);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &ts->i2ccap_esus );
#endif
    kfree(ts);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void i2ccap_early_suspend( struct early_suspend *esus )
{

    i2c_ts_data *ts;
    char    val[2] = { 0x00, 0x00 };
    char    com_val = 0x02;
    int     ret;

    if ( esus == NULL ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return;
    }

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    ts = container_of( esus, i2c_ts_data, i2ccap_esus );

    disable_irq( ts->irq );
    ret = cancel_work_sync( &ts->i2ccap_wq_func );
    if ( ret == 1 ){
        enable_irq( ts->irq );
    }

    del_timer_sync( &ts->timer );

    i2ccap_i2c_write( ts, &com_val, get_object_address(SPT_COMCONFIG_T18)+1, 1 );
    mdelay(1);
    i2ccap_i2c_write( ts, val, get_object_address(GEN_POWERCONFIG_T7), 2 );

}

static void i2ccap_late_resume( struct early_suspend *esus )
{
    i2c_ts_data *ts;
    char val = 0x00;
    
    if ( esus == NULL ){
        printk(KERN_ERR "%s :parameter error \n", __func__ );
        return;
    }

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    ts = container_of( esus, i2c_ts_data, i2ccap_esus );

    i2ccap_hw_reset( ts );

    val = ts->default_config->resume_atchcalsthr;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );

    ts->delta_chk_flg = DELTA_CHK_STS_CHK_ON;

    enable_irq( ts->irq );

}

#else   /* CONFIG_HAS_EARLYSUSPEND */

static int i2ccap_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
    i2c_ts_data *ts = i2c_get_clientdata( client );
    char    val[2] = { 0x00, 0x00 };
    char    com_val = 0x02;
    int     ret;
    
    printk( KERN_INFO "%s :[IN]\n", __func__ );

    disable_irq( ts->irq );
    ret = cancel_work_sync( &ts->i2ccap_wq_func );
    if ( ret == 1 ){
        enable_irq( ts->irq );
    }

    del_timer_sync( &ts->timer );

    i2ccap_i2c_write( ts, &com_val, get_object_address(SPT_COMCONFIG_T18)+1, 1 );
    mdelay(1);
    i2ccap_i2c_write( ts, val, get_object_address(GEN_POWERCONFIG_T7), 2 );

    return 0;
}

static int i2ccap_i2c_resume(struct i2c_client *client)
{
    
    i2c_ts_data *ts = i2c_get_clientdata( client );
    char val = 0x00;

    printk( KERN_INFO "%s :[IN]\n", __func__ );

    i2ccap_hw_reset( ts );

    val = ts->default_config->resume_atchcalsthr;
    i2ccap_i2c_write( ts, &val, get_object_address(GEN_ACQUIRECONFIG_T8)+7, 1 );

    ts->delta_chk_flg = DELTA_CHK_STS_CHK_ON;

    enable_irq( ts->irq );

    return 0;
}

#endif

static const struct i2c_device_id i2ccap_i2c_id[] = {
	{ TS_DRIVER_NAME, 4 },  { }
};

static struct i2c_driver i2ccap_i2c_driver = {
    .id_table   = i2ccap_i2c_id,
    .probe      = i2ccap_i2c_probe,
    .remove     = __devexit_p(i2ccap_i2c_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = i2ccap_i2c_suspend,
    .resume     = i2ccap_i2c_resume,
#endif
    .driver     = {
        .name = TS_DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init i2ccap_init(void)
{
    int retval;
    printk( KERN_INFO "%s :[IN]\n", __func__ );

    retval = i2c_add_driver( &i2ccap_i2c_driver );
    return retval;
}
module_init(i2ccap_init);

static void __exit i2ccap_exit(void)
{
    printk( KERN_INFO "%s :[IN]\n", __func__ );
    i2c_del_driver( &i2ccap_i2c_driver );
}
module_exit(i2ccap_exit);

MODULE_DESCRIPTION("i2c Touch Screen driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c_captouch");

