/*
 * Copyright(C) 2011 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*============================================================================
INCLUDE FILES FOR MODULE
============================================================================*/
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

/* ----------------------------------------------------------------------- *
 * Global Variables                                                        *
 * ----------------------------------------------------------------------- */
typedef enum {
    LCD_STATE_INIT,
    LCD_STATE_OFF,
    LCD_STATE_WAIT_UPDATE,
    LCD_STATE_WAIT_DISPLAY_ON,
    LCD_STATE_ON
} mddi_NT35580A_lcd_state_e;

typedef enum {
    BKL_STATE_OFF,
    BKL_STATE_WAIT_LCD,
    BKL_STATE_1ST_ON,
    BKL_STATE_ON
} mddi_NT35580A_bkl_state_e;

#define MLDC_RST_GPIO (33)

#define BD6184_I2C_SLAVE_ADDR 0x76

static mddi_NT35580A_lcd_state_e mddi_NT35580A_lcd_state = LCD_STATE_INIT;
static mddi_NT35580A_bkl_state_e mddi_NT35580A_bkl_state = BKL_STATE_OFF;

static struct i2c_adapter *i2c_bkl;

#define NT356580X_LCD_CHG 0x02

/*===========================================================================
    GLOBAL FUNCTIONS PROTOTYPES
============================================================================*/
/* PANEL CONTROL FUNCTIONS */
void mddi_panel_fullscrn_update_notify(void);
void mddi_panel_updatedone_notify(void);

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/
/* PANEL CONTROL FUNCTIONS */
static int mddi_NT35580x_panel_on(struct platform_device *pdev);
static int mddi_NT35580x_panel_off(struct platform_device *pdev);
static void mddi_NT35580x_delay_on(struct work_struct *ignored);
static DECLARE_WORK(display_on_wq, mddi_NT35580x_delay_on);

static int mddi_NT35580A_panel_on(struct platform_device *pdev);
static int mddi_NT35580A_panel_off(struct platform_device *pdev);

static int mddi_NT35580A_panel_reset_on(void);
static int mddi_NT35580A_panel_reset_off(void);
static int mddi_NT35580A_panel_initialize(void);
static int mddi_NT35580A_panel_enter_sleep(void);
static int mddi_NT35580A_panel_exit_sleep(void);
static int mddi_NT35580A_panel_display_on(void);
static int mddi_NT35580A_panel_display_off(void);

static int mddi_NT35580_panel_on(struct platform_device *pdev);
static int mddi_NT35580_panel_off(struct platform_device *pdev);

static int mddi_NT35580_panel_enter_sleep(void);
static int mddi_NT35580_panel_exit_sleep(void);
static int mddi_NT35580_panel_display_on(void);
static int mddi_NT35580_panel_display_off(void);

/* BKL CONTROL FUNCTIONS */
void mddi_NT35580A_set_backlight(struct msm_fb_data_type *mfd);
static void mddi_NT35580_backlight_on(void);

/* LCD HW Control Functions */
static int _mddi_NT35580A_LCD_RESX_init(void);
static int _mddi_NT35580A_LCD_RESX(boolean hilow);

/* External Driver Wrappers */
static int _mddi_NT35580A_LCD_mddi_cmd(uint32 addr,uint32 data);
static int _mddi_NT35580A_BD6184_i2c_write(unsigned char addr,unsigned char data);

/*****************************************************************************/
/*  PANEL CONTROL FUNCTIONS                                                  */
/*****************************************************************************/
static int mddi_NT35580x_panel_on(struct platform_device *pdev)
{
    int ret = 0;
    
    if (system_rev >= NT356580X_LCD_CHG) {
        ret = mddi_NT35580_panel_on(pdev);
    }
    else {
        ret = mddi_NT35580A_panel_on(pdev);
    }
    
    return 0;
}

static int mddi_NT35580x_panel_off(struct platform_device *pdev)
{
    int ret = 0;
    
    if (system_rev >= NT356580X_LCD_CHG) {
        ret = mddi_NT35580_panel_off(pdev);
    }
    else {
        ret = mddi_NT35580A_panel_off(pdev);
    }
    
    return 0;
}

static void mddi_NT35580x_delay_on(struct work_struct *ignored)
{
    int ret = 0;
    
    if (mddi_NT35580A_lcd_state == LCD_STATE_WAIT_DISPLAY_ON)
    {
        if (system_rev >= NT356580X_LCD_CHG) {
            ret = mddi_NT35580_panel_display_on();
        }
        else {
            ret = mddi_NT35580A_panel_display_on();
        }

        if (ret != 0) {
            printk("[LCD]%s Display ON Fail(%d)\n",__func__,ret);
        }
        else {
            printk("[LCD]%s Display ON\n",__func__);
            mddi_NT35580A_lcd_state = LCD_STATE_ON;

            mddi_wait(20);
            mddi_NT35580_backlight_on();
        }
    }
    else
    {
        printk("[LCD]%s State(%d).Skip Request\n",__func__,mddi_NT35580A_lcd_state);
    }
    
    return;
}

static int mddi_NT35580A_panel_on(struct platform_device *pdev)
{
    int ret = 0;
    
    printk(KERN_INFO "[LCD]%s ENTER state[%d]\n",__func__,mddi_NT35580A_lcd_state);
    
    switch (mddi_NT35580A_lcd_state)
    {
        case LCD_STATE_INIT:
            //panel power already on at boot
            mddi_NT35580A_lcd_state = LCD_STATE_ON;
        break;
        
        case LCD_STATE_OFF:
            //panel reset on
            ret |= mddi_NT35580A_panel_reset_on();
            
            //panel initialize
            ret |= mddi_NT35580A_panel_initialize();
            
            //panel exit sleep mode
            ret |= mddi_NT35580A_panel_exit_sleep();
            
            if (ret == 0) {
                mddi_NT35580A_lcd_state = LCD_STATE_WAIT_UPDATE;
            }
            break;
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_ON:
            break;
        default:
            break;
    }
    
    printk(KERN_INFO "[LCD]%s LEAVE state[%d] ret(%d)\n",__func__,mddi_NT35580A_lcd_state,ret);
    
    return 0;
}

static int mddi_NT35580A_panel_off(struct platform_device *pdev)
{
    int ret = 0;
    
    printk(KERN_INFO "[LCD]%s ENTER state[%d]\n",__func__,mddi_NT35580A_lcd_state);
    switch (mddi_NT35580A_lcd_state)
    {
        case LCD_STATE_ON:
            //panel display off
            ret |= mddi_NT35580A_panel_display_off();
            
            //ponel enter sleep mode
            ret |= mddi_NT35580A_panel_enter_sleep();
            
            //panel reset off
            ret |= mddi_NT35580A_panel_reset_off();
            
            if (ret==0) {
                mddi_NT35580A_lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
            //panel enter sleep mode
            ret |= mddi_NT35580A_panel_enter_sleep();
            
            //panel reset off
            ret |= mddi_NT35580A_panel_reset_off();
            
            if (ret==0) {
                mddi_NT35580A_lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_INIT:
        case LCD_STATE_OFF:
            break;
        default:
            break;
    }
    printk(KERN_INFO "[LCD]%s LEAVE state[%d] ret(%d)\n",__func__,mddi_NT35580A_lcd_state,ret);
    
    return 0;
}

static int mddi_NT35580_panel_on(struct platform_device *pdev)
{
    int ret = 0;
    
    printk(KERN_INFO "[LCD]%s ENTER state[%d]\n",__func__,mddi_NT35580A_lcd_state);
    
    switch (mddi_NT35580A_lcd_state)
    {
        case LCD_STATE_INIT:
            //panel power already on at boot
            mddi_NT35580A_lcd_state = LCD_STATE_ON;
            break;
        case LCD_STATE_OFF:
            
/* FUJITSU:2011-10-14 resume problem start */
            mddi_host_client_cnt_reset();
/* FUJITSU:2011-10-14 resume problem end */
            //panel exit sleep mode
            ret |= mddi_NT35580_panel_exit_sleep();
            
            if (ret == 0) {
                mddi_NT35580A_lcd_state = LCD_STATE_WAIT_UPDATE;
            }
            break;
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_ON:
            break;
        default:
            break;
    }
    
    printk(KERN_INFO "[LCD]%s LEAVE state[%d] ret(%d)\n",__func__,mddi_NT35580A_lcd_state,ret);
    
    return 0;
}

static int mddi_NT35580_panel_off(struct platform_device *pdev)
{
    int ret = 0;
    
    printk(KERN_INFO "[LCD]%s ENTER state[%d]\n",__func__,mddi_NT35580A_lcd_state);
    switch (mddi_NT35580A_lcd_state)
    {
        case LCD_STATE_ON:
/* FUJITSU:2011-10-14 resume problem start */
            mddi_host_client_cnt_reset();
/* FUJITSU:2011-10-14 resume problem end */
            //panel display off
            ret |= mddi_NT35580_panel_display_off();
            
            //ponel enter sleep mode
            ret |= mddi_NT35580_panel_enter_sleep();
            
            if (ret==0) {
                mddi_NT35580A_lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
            //panel enter sleep mode
            ret |= mddi_NT35580_panel_enter_sleep();
            
            if (ret==0) {
                mddi_NT35580A_lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_INIT:
        case LCD_STATE_OFF:
            break;
        default:
            break;
    }
    printk(KERN_INFO "[LCD]%s LEAVE state[%d] ret(%d)\n",__func__,mddi_NT35580A_lcd_state,ret);
    
    return 0;
}

/*****************************************************************************/
/*  PANEL Other I/F FUNCTIONS                                                */
/*****************************************************************************/
// Notify that there was fullscreen update
void mddi_panel_fullscrn_update_notify(void)
{
    if (mddi_NT35580A_lcd_state == LCD_STATE_WAIT_UPDATE)
    {
        mddi_NT35580A_lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
    }
    return;
}

// Notify that DMA_UPDATE was done
void mddi_panel_updatedone_notify(void)
{
    // if fullscreen update arrived
    if(mddi_NT35580A_lcd_state == LCD_STATE_WAIT_DISPLAY_ON)
    {
        schedule_work(&display_on_wq);
    }
    return;
}

/*****************************************************************************/
/*  BACKLIGHT CONTROL FUNCTIONS                                              */
/*****************************************************************************/
void mddi_NT35580A_set_backlight(struct msm_fb_data_type *mfd)
{
    int ret = 0;
    int32 level = 0;
    
    level = mfd->bl_level;
    
    if(level) {
        /* BKL ON */
        if(mddi_NT35580A_bkl_state == BKL_STATE_OFF) {
            //initial configure
            ret |= _mddi_NT35580A_BD6184_i2c_write(0x01,0x0E);
            
            /* NOTICE : light turn on sharply at 1st time */
            ret |= _mddi_NT35580A_BD6184_i2c_write(0x09,0x30); //TLH set sharply
            
            /* Intensity */
            ret |= _mddi_NT35580A_BD6184_i2c_write(0x03,level);

            if (mddi_NT35580A_lcd_state == LCD_STATE_ON)
            {
                /* MLED Power On */
                ret |= _mddi_NT35580A_BD6184_i2c_write(0x02,0x01);
                if (ret == 0)
                {
                    mddi_NT35580A_bkl_state = BKL_STATE_1ST_ON;
                    printk(KERN_INFO "[BKL]%s(%d) Backlight turn ON.\n",__func__,level);
                }
            }
            else
            {
                printk(KERN_ERR "[BKL]%s: Resume, Backlight is still OFF\n",__func__);
                mddi_NT35580A_bkl_state = BKL_STATE_WAIT_LCD;
            }

            if(ret != 0)
            {
                printk(KERN_ERR "[BKL]:I2C set failed. state change skipped.\n");
            }
        }
        else {
            if(mddi_NT35580A_bkl_state == BKL_STATE_1ST_ON){
                
                /* 2nd time. slope reset */
                _mddi_NT35580A_BD6184_i2c_write(0x09,0x66);
                
                mddi_NT35580A_bkl_state = BKL_STATE_ON;
            }
            /* Intensity */
            _mddi_NT35580A_BD6184_i2c_write(0x03,level);
        }
    }
    else {
        /* BKL OFF */
        
        /* MLED PowerOff */
        _mddi_NT35580A_BD6184_i2c_write(0x02,0x00);
        
        mddi_NT35580A_bkl_state = BKL_STATE_OFF;
        printk(KERN_INFO "[BKL]%s(%d) Backlight turn OFF.\n",__func__,level);
    }
    
    return;
}

static void mddi_NT35580_backlight_on(void)
{
    int ret = 0;
    
    if (mddi_NT35580A_bkl_state == BKL_STATE_WAIT_LCD)
    {
        /* MLED Power On */
        ret = _mddi_NT35580A_BD6184_i2c_write(0x02,0x01);
        if (ret != 0)
        {
            printk(KERN_ERR "[BKL]%s: Backlight turn ON Failed. ret = %d\n", __func__, ret);
            return;
        }
        mddi_NT35580A_bkl_state = BKL_STATE_1ST_ON;
        printk(KERN_ERR "[BKL]%s: Backlight turn ON. ret = %d\n", __func__, ret);
    }
    else {
        printk("[BKL]%s: Request Not Arrived (set_backlight).\n", __func__);
    }
    
    return;
}

/*---------------------------------------------------------------------------*/
/*  LCD Power Control Functions                                              */
/*---------------------------------------------------------------------------*/
static int mddi_NT35580A_panel_reset_on(void)
{
    int ret = 0;
    
    //wait 10ms
    mddi_wait(10);
    
    ret = _mddi_NT35580A_LCD_RESX(TRUE);
    if (ret != 0){
        printk("[LCD]%s ERROR LCD RESX set fail(%d)\n",__func__,ret);
        return ret;
    }
    //wait 200ms to initialize
    mddi_wait(200);
        
    return ret;
}

static int mddi_NT35580A_panel_reset_off(void)
{
    int ret = 0;
    
    ret = _mddi_NT35580A_LCD_RESX(FALSE);
    if (ret != 0){
        printk("[LCD]%s ERROR LCD RESX set fail(%d)\n",__func__,ret);
        return ret;
    }
    
    //wait 10ms
    mddi_wait(10);
    
    return ret;
}

/*---------------------------------------------------------------------------*/
/* LCD Initialize Function                                                   */
/*---------------------------------------------------------------------------*/
static int mddi_NT35580A_panel_initialize(void)
{
    int ret = 0;
    
    //command for initializing lcd panel
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A02, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A03, 0x00DF);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B02, 0x0003);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B03, 0x0055);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D02, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D03, 0x0000);
    
    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

/*---------------------------------------------------------------------------*/
/* LCD Mode Control Functions                                                */
/*---------------------------------------------------------------------------*/
static int mddi_NT35580A_panel_exit_sleep(void)
{
    int ret = 0;

    //command for exiting sleep mode - 1
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x1100, 0x0000);

    //wait 120ms
    mddi_wait(120);

    //command for exiting sleep mode - 2
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xF280, 0x0055);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xF281, 0x00AA);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xF282, 0x0066);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xF38E, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0180, 0x0002);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0380, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0480, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0580, 0x002C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0680, 0x0023);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2080, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2280, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2480, 0x0050);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2580, 0x006B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2780, 0x0064);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A80, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B80, 0x00B1);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2C80, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D80, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD080, 0x0008);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD180, 0x0016);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD280, 0x0005);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD380, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD480, 0x0062);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD580, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD680, 0x005B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD780, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD880, 0x00DE);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xD980, 0x000E);

    //wait 64ms
    mddi_wait(64);

    //command for exiting sleep mode - 3
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4080, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4180, 0x0027);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4280, 0x0042);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4380, 0x0070);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4480, 0x0010);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4580, 0x0037);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4680, 0x0062);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4780, 0x008C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4880, 0x001E);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4980, 0x0025);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4A80, 0x00D4);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4B80, 0x001E);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4C80, 0x0041);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4D80, 0x006C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4E80, 0x00BC);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4F80, 0x00DB);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5080, 0x0077);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5180, 0x0073);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5880, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5980, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5A80, 0x0038);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5B80, 0x0057);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5C80, 0x0014);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5D80, 0x003E);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5E80, 0x0061);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x5F80, 0x0040);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6080, 0x001A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6180, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6280, 0x0089);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6380, 0x001D);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6480, 0x0048);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6580, 0x006A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6680, 0x00A6);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6780, 0x00D9);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6880, 0x0078);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x6980, 0x007F);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7080, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7180, 0x003A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7280, 0x0055);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7380, 0x007C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7480, 0x001A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7580, 0x0041);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7680, 0x0063);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7780, 0x009B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7880, 0x0019);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7980, 0x0024);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7A80, 0x00DB);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7B80, 0x001C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7C80, 0x003D);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7D80, 0x006A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7E80, 0x00BB);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x7F80, 0x00DB);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8080, 0x0077);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8180, 0x0073);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8880, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8980, 0x0022);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8A80, 0x0038);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8B80, 0x0058);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8C80, 0x0016);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8D80, 0x0041);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8E80, 0x0061);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x8F80, 0x003A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9080, 0x0019);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9180, 0x0022);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9280, 0x007B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9380, 0x001B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9480, 0x003E);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9580, 0x0063);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9680, 0x009A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9780, 0x00CA);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9880, 0x0064);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x9980, 0x007F);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA080, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA180, 0x003A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA280, 0x0060);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA380, 0x008C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA480, 0x0018);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA580, 0x0045);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA680, 0x0066);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA780, 0x00B1);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA880, 0x001B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xA980, 0x0026);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAA80, 0x00E5);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAB80, 0x001C);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAC80, 0x0047);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAD80, 0x0069);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAE80, 0x00BC);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xAF80, 0x00DB);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xB080, 0x0077);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xB180, 0x0073);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xB880, 0x0020);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xB980, 0x0021);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBA80, 0x0038);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBB80, 0x0057);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBC80, 0x0017);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBD80, 0x003B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBE80, 0x0063);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xBF80, 0x002F);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC080, 0x001B);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC180, 0x0026);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC280, 0x0063);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC380, 0x001A);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC480, 0x0036);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC580, 0x0067);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC680, 0x0087);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC780, 0x00B4);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC880, 0x0064);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xC980, 0x007F);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x3500, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4400, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4401, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x3600, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xDA80, 0x0040);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2100, 0x0000);
    
    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580A_panel_enter_sleep(void)
{
    int ret = 0;

    //command for setting sleep mode
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0xDA80, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x1000, 0x0000);

    //wait 68ms
    mddi_wait(68);

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580A_panel_display_on(void)
{
    int ret = 0;

    //command for setting display on - 1
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2900, 0x0000);

    //wait 17ms
    mddi_wait(17);

    //command for setting display on - 2
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2C80, 0x0022);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2080, 0x0040);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B80, 0x00BA);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2280, 0x000C);

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580A_panel_display_off(void)
{
    int ret = 0;

    //command for setting display off
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2800, 0x0000);

    //wait 68ms
    mddi_wait(68);

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;

}

static int mddi_NT35580_panel_exit_sleep(void)
{
    int ret = 0;

    //command for exiting sleep mode
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x1100, 0x0000);
/* FUJITSU:2011-09-30 state_change start */
    if (ret != 0){
        printk("[LCD]%s return %d (immediately)\n",__func__,ret);
        return ret;
    }
/* FUJITSU:2011-09-30 state_change end */

    //wait 100ms
    mddi_wait(100);

    //command for LCD config
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A02, 0x0001);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2A03, 0x00DF);
    
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B02, 0x0003);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B03, 0x0055);
    
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D00, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D01, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D02, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2D03, 0x0000);

    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x3A00, 0x0077);

    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x3600, 0x0000);

    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x0180, 0x0002);

    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2080, 0x0033);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2C80, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2B80, 0x0068);
    
    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580_panel_enter_sleep(void)
{
    int ret = 0;

    //command for setting sleep in
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x1000, 0x0000);

/* FUJITSU:2011-09-30 state_change start */
    //wait 100ms
    mddi_wait(100);
/* FUJITSU:2011-09-30 state_change start */

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580_panel_display_on(void)
{
    int ret = 0;

    //command for setting display on
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2900, 0x0000);

    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x3500, 0x0042);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4400, 0x0000);
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x4401, 0x0000);

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

static int mddi_NT35580_panel_display_off(void)
{
    int ret = 0;

    //command for setting display off
    ret |= _mddi_NT35580A_LCD_mddi_cmd(0x2800, 0x0000);

    if (ret != 0){
        printk("[LCD]%s return %d\n",__func__,ret);
    }
    return ret;
}

/*---------------------------------------------------------------------------*/
/*   LCD HW CONTROL FUNCTIONS                                                */
/*---------------------------------------------------------------------------*/
static int _mddi_NT35580A_LCD_RESX_init(void)
{
    int ret = 0;
    
    //gpio init
    gpio_tlmm_config(GPIO_CFG(MLDC_RST_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    return ret;
}

static int _mddi_NT35580A_LCD_RESX(boolean hilow)
{
    int ret = 0;
    
    if (hilow){
        //RESX set Hi
        gpio_set_value(MLDC_RST_GPIO, 1);
    }
    else{
        //RESX set Low
        gpio_set_value(MLDC_RST_GPIO, 0);
    }
    
    return ret;
}

/*===========================================================================*/
/*    External Driver WRAPPER FUNCTIONS                                      */
/*===========================================================================*/

/* --- MDDI Driver Wrapper ------------------------------------------------- */

static int _mddi_NT35580A_LCD_mddi_cmd(uint32 addr,uint32 data)
{
    int ret = 0;
    
    ret = mddi_queue_register_write(addr, data, TRUE, 0);
    
    if (ret!= 0){
        printk("[LCD] mddi command set fail (addr:0x%8x data:0x%8x) ret:%d \n",addr,data,ret);
    }
    
    return ret;
}

/* --- I2C Driver Wrapper -------------------------------------------------- */

static int _mddi_NT35580A_BD6184_i2c_write(unsigned char addr,unsigned char data)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret = 0;

    msg.addr  = BD6184_I2C_SLAVE_ADDR;
    msg.buf   = buf;
    msg.len   = 2;
    msg.flags = 0;
    
    buf[0] = addr;
    buf[1] = data;
    
    ret = i2c_transfer(i2c_bkl, &msg, 1);
    if (ret < 0) {
        printk("[LED]%s I2C(addr:%x,data:%x) ERROR ret = %d\n",__func__,addr,data,ret);
    }
    else {
        /* I2C transfer successful. return success(0) */
        ret = 0;
    }
    
    return ret;
}

/*---------------------------------------------------------------------------*/
/* LCD CONFIGURES                                                            */
/*---------------------------------------------------------------------------*/
static int __devinit mddi_NT35580A_probe(struct platform_device *pdev)
{
    msm_fb_add_device(pdev);

    return 0;
}

static struct platform_driver this_driver = {
    .probe      = mddi_NT35580A_probe,
    .shutdown   = NULL,
    .driver = {
        .name   = "mddi_NT35580A_4fwvga",
    },
};

static struct msm_fb_panel_data mddi_NT35580A_panel_data = {
    .on     = mddi_NT35580x_panel_on,
    .off    = mddi_NT35580x_panel_off,
    .set_backlight  = mddi_NT35580A_set_backlight
};

static struct platform_device this_device = {
    .name   = "mddi_NT35580A_4fwvga",
    .id     = 0,
    .dev    = {
        .platform_data = &mddi_NT35580A_panel_data,
    }
};

 
static int __init mddi_NT35580A_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

    mddi_NT35580A_lcd_state = LCD_STATE_INIT;

    i2c_bkl = i2c_get_adapter(0);

    if (!i2c_bkl) {
        printk("[LCD]%s i2c_get_adapter(0) failure.\n",__func__);
    }
    
    mddi_NT35580A_bkl_state = BKL_STATE_OFF;


    printk(KERN_DEBUG "[LCD]%s:[0x%x]\n",__func__,system_rev);


    ret = platform_driver_register(&this_driver);

    if (!ret) {
        pinfo = &mddi_NT35580A_panel_data.panel_info;
        pinfo->xres                       = 480;
        pinfo->yres                       = 854;
        pinfo->type                       = MDDI_PANEL;
        pinfo->pdest                      = DISPLAY_1;
        pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
        pinfo->wait_cycle                 = 0;
        pinfo->bpp                        = 16;
        pinfo->fb_num                     = 2;
        pinfo->clk_rate                   = 192000000;
        pinfo->clk_min                    = 190000000; //PMDH min
        pinfo->clk_max                    = 200000000; //PMDH max
        pinfo->lcd.vsync_enable           = FALSE;

        if (system_rev >= NT356580X_LCD_CHG) {
            pinfo->lcd.refx100                = 5850; //58.5Hz
            pinfo->lcd.v_back_porch           = 0;
            pinfo->lcd.v_front_porch          = 12;
            pinfo->lcd.v_pulse_width          = 5;
        }
        else {
            pinfo->lcd.refx100                = 72 * 100;
            pinfo->lcd.v_back_porch           = 12;
            pinfo->lcd.v_front_porch          = 2;
            pinfo->lcd.v_pulse_width          = 0;
        }
        pinfo->lcd.hw_vsync_mode          = FALSE;
        pinfo->lcd.vsync_notifier_period  = 0;
        pinfo->bl_min                     = 1;
        pinfo->bl_max                     = 127;

        ret = platform_device_register(&this_device);
        if (ret){
            printk("[LCD]%s platform_device_register fail.(%d)\n",__func__,ret);
            platform_driver_unregister(&this_driver);
        }
    }
    else {
        printk("[LCD]%s platform_driver_register fail.(%d)\n",__func__,ret);
    }
    
    /* Initialize GPIO */
    _mddi_NT35580A_LCD_RESX_init();

    printk("[LCD]%s done.(%d)\n",__func__,ret);

    return ret;
}

module_init(mddi_NT35580A_init);

