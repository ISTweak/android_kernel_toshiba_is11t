/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

/* FUJITSU:2011/05/12  begin */
#ifdef CONFIG_MACH_F11K06
#include <linux/wifi_tiwlan.h>
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/i2c/isa1200.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>

/* FUJITSU:2011/05/12   begin */
#if 0
#ifdef CONFIG_MACH_F11K06
#include <linux/cyttsp.h>
#endif // CONFIG_MACH_F11K06
#endif
/* FUJITSU:2011/05/12   end */

#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
/* FUJITSU:2011_06_04   start */
#include <linux/walkmotion.h>
#include <linux/i2cuart.h>
/* FUJITSU:2011_06_04   end */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <linux/cyttsp.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"

/* FUJITSU:2011-03-15  Add for  HW start */
#ifdef CONFIG_MACH_F11K06
#include "msm-keypad-devices.h"
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-03-15  Add for  HW end */

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
/* FUJITSU:2011-04-22  USB setteing start */
#include "proc_comm.h"
/* FUJITSU:2011-04-22  USB setteing end */
#endif

#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_reqs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"

/* FUJITSU:2011/05/12   begin */
#include <linux/mmc/wlan_power.h>
/* FUJITSU:2011/05/12   end */

/* FUJITSU:2011-05-06    add BT HS start */
#define MSM_UARTDM2_USE
#define MSM_SPI_CRCI_DISABLE
/* FUJITSU:2011-05-06   add BT HS end */

/* FUJITSU:2011-04-25  start  */
#ifdef CONFIG_MACH_F11K06
#include "../../../drivers/fujitsu/touchscreen/k06_captouch.h"
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-25  end */

/* FUJITSU:2011-06-06  DEL@proximity start  */
/* FUJITSU:2011-04-25  ADD@proximity start  */
#if 0
#ifdef CONFIG_MACH_F11K06
int msm_prox_read_nvitem(unsigned int id);
int msm_prox_write_nvitem(unsigned int id, unsigned short *data);
#endif // CONFIG_MACH_F11K06
#endif
/* FUJITSU:2011-04-25  ADD@proximity end */
/* FUJITSU:2011-06-06  DEL@proximity end */

/* FUJITSU:2011-08-12 change pmem size up start */
#define MSM_PMEM_SF_SIZE	0x2000000
//#define MSM_PMEM_SF_SIZE	0x1700000
/* FUJITSU:2011-08-12 change pmem size up end */
#define MSM_FB_SIZE		0x500000
/* #define MSM_PMEM_ADSP_SIZE      0x1800000 */
#define MSM_PMEM_ADSP_SIZE      0x2700000
#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI1_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900

/* FUJITSU:2011-02-21  modify SD_DET GPIO port start */
#ifdef CONFIG_MACH_F11K06
#define PMIC_GPIO_SD_DET    19
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-02-21  modify SD_DET GPIO port end */

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
#define PMIC_GPIO_SDC4_EN_N 23  /* PMIC GPIO Number 24() */
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

#define FPGA_SDCC_STATUS       0x8E0001A8

/* FUJITSU:2011/02/23    add BD6184GUL SLAVE ADDR start */
#define LED_2C_SLAVE_ADDR   0x76    
/* FUJITSU:2011/02/23    add BD6184GUL SLAVE ADDR end */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define PMIC_GPIO_FLASH_BOOST_ENABLE	15	/* PMIC GPIO Number 16 */

/* FUJITSU:2011-05-24 F11EIF only VIB del start */
#if 0
#define PMIC_GPIO_HAP_ENABLE   14  /* PMIC GPIO Number 15 */
#endif
/* FUJITSU:2011-05-24 F11EIF only VIB del end */

#define HAP_LVL_SHFT_MSM_GPIO 24
#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */
#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

/* FUJITSU:2011_06_04   start */
/* FUJITSU:2011_06_06 mod start */
#if 1
#define KO_UART4_NIRQ 115
#else
#define KO_UART4_NIRQ 7
#endif
/* FUJITSU:2011_06_06 mod end */
/* FUJITSU:2011_06_04   end */

/* FUJITSU:2011-04-19  VIB start */
#ifdef CONFIG_MACH_F11K06
void msm_init_pmic_vibrator(void);
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-19  VIB end   */



static int pm8058_gpios_init(void)
{
	int rc;

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	struct pm8058_gpio sdcc_det = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_1P5,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
#endif
	struct pm8058_gpio sdc4_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L5,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.output_value   = 0,
	};

/* FUJITSU:2011-05-24 F11EIF only VIB del start */
#if 0
	struct pm8058_gpio haptics_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
	};
#endif
/* FUJITSU:2011-05-24 F11EIF only VIB del end */

	struct pm8058_gpio flash_boost_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

/* FUJITSU:2011_06_04   start */
/* FUJITSU:2011_06_06 del start */
#if 0
	struct pm8058_gpio i2cuart_ko_uart4_nirq = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_NO,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.vin_sel        = PM_GPIO_VIN_S3,
	};
#endif
/* FUJITSU:2011_06_06 del end */
/* FUJITSU:2011_06_04   end */

	if (machine_is_msm7x30_fluid()) {

/* FUJITSU:2011-05-24 F11EIF only VIB del start */
#if 0
		rc = pm8058_gpio_config(PMIC_GPIO_HAP_ENABLE, &haptics_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_HAP_ENABLE + 1));
			return rc;
		}
#endif
/* FUJITSU:2011-05-24 F11EIF only VIB del end */

		rc = pm8058_gpio_config(PMIC_GPIO_FLASH_BOOST_ENABLE,
			&flash_boost_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_FLASH_BOOST_ENABLE + 1));
			return rc;
		}
	}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	if (machine_is_msm7x30_fluid())
		sdcc_det.inv_int_pol = 1;

	rc = pm8058_gpio_config(PMIC_GPIO_SD_DET - 1, &sdcc_det);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}
#endif

	if (machine_is_msm7x30_fluid()) {
		rc = pm8058_gpio_config(PMIC_GPIO_SDC4_EN_N, &sdc4_en);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
				  "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N), 0);
	}

/* FUJITSU:2011_06_04   start */
/* FUJITSU:2011_06_06 del start */
#if 0
	rc = pm8058_gpio_config((KO_UART4_NIRQ - 1), &i2cuart_ko_uart4_nirq);
	if (rc) {
		pr_err("%s KO_UART4_NIRQ config failed\n", __func__);
		return rc;
	}
#endif
/* FUJITSU:2011_06_06 del end */
/* FUJITSU:2011_06_04   end */

	return 0;
}


/* FUJITSU:2011/04/25  CAP begin */
#ifdef CONFIG_MACH_F11K06

#define PMIC_GPIO_TS_RESET          24
#define GPIO_TCH_INT                18
#define CAP_DEVICE_ID               "i2c_captouch"

#define PRODUCT_NO_EVM              0x05
#define PRODUCT_NO_EVM_FSL          0x15
#define PRODUCT_NO_1_0              0x00
#define PRODUCT_NO_1_2              0x01

static struct vreg *cap_vreg_l13;

static void cap_reset_control( int value )
{
    int retval;
    struct pm8058_gpio tch_reset = {
            .direction      = PM_GPIO_DIR_OUT,
            .pull           = PM_GPIO_PULL_NO,
            .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
            .vin_sel        = PM_GPIO_VIN_L2,
            .output_value   = 0,
            .out_strength   = PM_GPIO_STRENGTH_HIGH,
            .function       = PM_GPIO_FUNC_NORMAL,
            .inv_int_pol    = 0,
    };
    
    if ( value ){
        tch_reset.output_value   = 1;
    }else{
        tch_reset.output_value   = 0;
    }

    retval = pm8058_gpio_config( PMIC_GPIO_TS_RESET, &tch_reset);
    if ( retval != 0 ){
        printk(KERN_ERR "%s: set reset error retval = %d\n", __func__, retval );
    }
}

static void cap_vreg_config_old_model( int on )
{
    struct  i2c_msg msg;
    struct  i2c_adapter *i2c_bkl;
    int     retval;
    u_int8_t buf[8];

    /* Get adpter */
    i2c_bkl = i2c_get_adapter( 0 );

    /* Set Param    */
    msg.addr    = LED_2C_SLAVE_ADDR;
    msg.buf     = buf;
    msg.len     = 2;
    msg.flags   = 0;

    if ( on ){

        buf[0] = 0x15;
        buf[1] = 0x94;

        retval = i2c_transfer( i2c_bkl, &msg, 1 );
        if ( retval != 1 ) {
            printk(KERN_ERR "%s: set voltage error retval = %d\n", __func__, retval );
        }

        cap_reset_control( 0 );

        buf[0] = 0x13;
        buf[1] = 0x0F;  /* 13H: LDO2 LDO1 LDO4 LDO3 all ON  */

        retval = i2c_transfer( i2c_bkl, &msg, 1 );
        if ( retval != 1 ) {
            printk(KERN_ERR "%s: output enable error retval = %d\n", __func__, retval );
        }

        msleep( 40 );
        cap_reset_control( 1 );
    }else{
        cap_reset_control( 0 );

        /* Voltage ON  13H(0x0C)    */
        buf[0] = 0x13;  /* client addr     */
        buf[1] = 0x03;  /* 13H: LDO4 LDO3 OFF  */

        retval = i2c_transfer( i2c_bkl, &msg, 1 );
        if ( retval != 1 ) {
            printk(KERN_ERR "%s: output disable error retval = %d\n", __func__, retval );
            return;
        }
    }

}

static void cap_vreg_config( int on )
{
    int     retval;

    /* for Old model (use BD6184) */
    switch ( system_rev ){
        case PRODUCT_NO_EVM:
        case PRODUCT_NO_EVM_FSL:
        case PRODUCT_NO_1_0:
        case PRODUCT_NO_1_2:
            cap_vreg_config_old_model( on );
            return ;
        default:
            break;
    }
    
    /* for New model (use PM8058) */
    if ( on ){

        /* get structure */
        cap_vreg_l13 = vreg_get( NULL, "wlan" );
        if ( IS_ERR(cap_vreg_l13) ){
            printk( KERN_ERR "%s: get vreg structure error retval = %d\n", __func__, (int)cap_vreg_l13 );
            return;
        }

        /* set voltage */
        retval = vreg_set_level( cap_vreg_l13, 2700 );
        if ( retval ) {
            printk( KERN_ERR "%s: set voltage error retval = %d\n", __func__, retval );
        }

        cap_reset_control( 0 );

        /* output enable */
        retval = vreg_enable( cap_vreg_l13 );
        if ( retval ) {
            printk( KERN_ERR "%s: output enable error retval = %d\n", __func__, retval );
        }

        msleep( 40 );
        cap_reset_control( 1 );

    }else{

        cap_reset_control( 0 );

        /* check structure */
        if ( IS_ERR(cap_vreg_l13) ){
            printk( KERN_ERR "%s: invalid vreg structure \n", __func__ );
            return;
        }

        /* output disable */
        retval = vreg_disable( cap_vreg_l13 );
        if ( retval ) {
            printk( KERN_ERR "%s: output disable error retval = %d\n", __func__, retval );
            return;
        }

        vreg_put( cap_vreg_l13 );
        cap_vreg_l13 = NULL;

    }

}

static int captouch_init(void)
{
    int rc;

    /* request */
    rc = gpio_request( GPIO_TCH_INT, "gpio_ts_int" );
    if (rc) {
        pr_err("gpio_request failed on RESET(157) (rc=%d)\n", rc);
    }

    /* config */
    rc = gpio_tlmm_config(GPIO_CFG( GPIO_TCH_INT, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc) {
        printk(KERN_ERR "%s: gpio_tlmm_config(INT)=%d\n", __func__, rc );
    }
    return rc;
}

i2c_captouch_platform_data k06_cap_pdata[] =
{
    {
        .x_max        = 1023,
        .y_max        = 1023,
        .pressure_max = 255,
        .max_point    = 3,
        .gpio_int     = GPIO_TCH_INT,
        .touch_addr   = 0x4A,
        .hw_config    = {
            .config_crc   = 0x2E4AEE,
            .config_t38   = {   1,  0, 11,  6, 17,  0,  0,  0 },
            .config_t7    = {   8,255, 50 },
            .config_t8    = {   8,  5,  5,  5,  0,  0,  0,  1 },
            .config_t9    = { 131,  0,  0, 19, 11,  0, 33, 75,  2,  2,  0,  1,  3, 48,  3, 10, 20, 10,  0,  0,  0,  0,  2,  2, 13, 13,152, 50,162, 70, 20 },
            .config_t15   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t18   = {   0,  0 },
            .config_t19   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t20   = {   7,  1,  1,  1,  1,  3,  0, 15,  0,  0,  0,  0 },
            .config_t22   = {   7,  0,  0,  0,  0,  0,  0,  0, 40,  0,  0,  8,  9, 10, 20, 21,  0 },
            .config_t23   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t24   = {   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t25   = {   3,  0,248, 42,124, 21,  0,  0,  0,  0,  0,  0,  0,  0 },
            .config_t27   = {   0,  0,  0,  0,  0,  0,  0 },
            .config_t28   = {   0,  0,  3, 16, 24,  0 },
            .resume_atchcalsthr = 40,
        },
        .vreg_config  = &cap_vreg_config,
        .reset_control = &cap_reset_control,
    }
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/04/25  CAP end */


/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
static struct i2c_board_info k06_cap_info[] __initdata = {
    {
        I2C_BOARD_INFO( CAP_DEVICE_ID, 0x4A ),
        .platform_data = &k06_cap_pdata,
        .irq = MSM_GPIO_TO_INT(GPIO_TCH_INT),
    },
};
#endif
/* FUJITSU:2011/04/25  CAP end */
/* FUJITSU:2011/05/12  end */


static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}



static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};


/* FUJITSU:2011-04-12  Add for QWERTY keyboard start */
#ifdef CONFIG_MACH_F11K06
static const unsigned int f11k06_keymap[] = {
    KEY(0, 2, KEY_P),
    KEY(0, 3, KEY_N),
    KEY(0, 4, KEY_V),
    KEY(0, 5, KEY_DELETE),
    KEY(0, 6, KEY_W),
    KEY(0, 7, KEY_BACK),
    KEY(1, 2, KEY_SEARCH),
    KEY(1, 3, KEY_DOT),
    KEY(1, 4, KEY_SPACE),
    KEY(1, 5, KEY_R),
    KEY(1, 6, KEY_S),
    KEY(1, 7, KEY_HOME),
    KEY(2, 6, KEY_RIGHTALT),
    KEY(2, 7, KEY_MENU),
    KEY(3, 2, KEY_ZENKAKUHANKAKU),
    KEY(3, 3, KEY_I),
    KEY(3, 4, KEY_Y),
    KEY(3, 5, KEY_F),
    KEY(3, 6, KEY_Z),
    KEY(3, 7, KEY_LEFTALT),
    KEY(4, 3, KEY_K),
    KEY(4, 4, KEY_H),
    KEY(4, 5, KEY_C),
    KEY(4, 6, KEY_MAIL),
    KEY(4, 7, KEY_Q),
    KEY(5, 2, KEY_BACKSPACE),
    KEY(5, 3, KEY_M),
    KEY(5, 4, KEY_B),
    KEY(5, 5, KEY_COMMA),
    KEY(5, 6, KEY_E),
    KEY(5, 7, KEY_A),
    KEY(6, 2, KEY_ENTER),
    KEY(6, 3, KEY_O),
    KEY(6, 4, KEY_U),
    KEY(6, 5, KEY_T),
    KEY(6, 6, KEY_D),
    KEY(6, 7, KEY_COPY),
    KEY(7, 3, KEY_L),
    KEY(7, 4, KEY_J),
    KEY(7, 5, KEY_G),
    KEY(7, 6, KEY_X),
    KEY(7, 7, KEY_LEFTSHIFT),
    KEY(8, 0, KEY_LEFT),
    KEY(8, 1, KEY_UP),
    KEY(9, 0, KEY_DOWN),
    KEY(9, 1, KEY_RIGHT),
};

static struct matrix_keymap_data f11k06_keymap_data = {
    .keymap_size = ARRAY_SIZE(f11k06_keymap),
    .keymap = f11k06_keymap,
};

static struct pmic8058_keypad_data f11k06_keyboard_data = {
    .input_name = "f11k06_keyboard",
    .input_phys_device = "f11k06_keyboard/input0",
    .num_rows = 10,
    .num_cols = 8,
    .rows_gpio_start = 8,
    .cols_gpio_start = 0,
    .debounce_ms = { 8, 10 },
    .scan_delay_ms = 32,
    .row_hold_ns = 122000,
    .wakeup = 1,
    .keymap_data = &f11k06_keymap_data,
};
static struct msm_gpio f11k06_keyboard_gpios[] = {
    { GPIO_CFG(40, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "open_det" },
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-12  Add for QWERTY keyboard end */



static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
	.enable		= pm8058_pwm_enable,
};

/* Put sub devices with fixed location first in sub_devices array */
#define	PM8058_SUBDEV_KPD	0
#define	PM8058_SUBDEV_LED	1

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PMIC8058_IRQ_BASE, 0),
};



static struct mfd_cell pm8058_subdevs[] = {
	{	.name = "pm8058-keypad",
		.id		= -1,
		.num_resources	= ARRAY_SIZE(resources_keypad),
		.resources	= resources_keypad,
	},
	{	.name = "pm8058-led",
		.id		= -1,
	},
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwm",
		.id		= -1,
		.platform_data	= &pm8058_pwm_data,
		.data_size	= sizeof(pm8058_pwm_data),
	},
	{	.name = "pm8058-nfc",
		.id		= -1,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

/* FUJITSU:2011-04-12  Add for KeyBackLight start */
#ifdef CONFIG_MACH_F11K06
static struct platform_device msm_device_pmic_leds = {
    .name = "pmic-leds",
    .id = -1,
};
/* FUJITSU:2011-07-05 Add for LED start */
static struct platform_device notification_led = {
	.name = "notification-led",
	.id = -1,
};
/* FUJITSU:2011-07-05 Add for LED end */
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-12  Add for KeyBackLight end */


static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_base = PMIC8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};


#ifdef CONFIG_F11K06_CAMERA
static uint32_t camera_off_gpio_table[] = {
    /* parallel CAMERA interfaces */
    GPIO_CFG(2,  0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* M_CAM_SCL */
    GPIO_CFG(3,  0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* M_CAM_SDA */
    GPIO_CFG(4,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
    GPIO_CFG(5,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
    GPIO_CFG(6,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
    GPIO_CFG(7,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
    GPIO_CFG(8,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
    GPIO_CFG(9,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
    GPIO_CFG(10, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
    GPIO_CFG(11, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
    GPIO_CFG(12, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
    GPIO_CFG(13, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
    GPIO_CFG(22, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* BOOT */
    GPIO_CFG(24, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* STANDBY */
    GPIO_CFG(25, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* RESET */
    GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)  /* VDD Core */
};

static uint32_t camera_on_gpio_table[] = {
    /* parallel CAMERA interfaces */

    GPIO_CFG(2,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA), /* M_CAM_SCL */
    GPIO_CFG(3,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA), /* M_CAM_SDA */
    GPIO_CFG(4,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
    GPIO_CFG(5,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
    GPIO_CFG(6,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
    GPIO_CFG(7,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
    GPIO_CFG(8,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
    GPIO_CFG(9,  1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
    GPIO_CFG(10, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
    GPIO_CFG(11, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
    GPIO_CFG(12, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
    GPIO_CFG(13, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
    GPIO_CFG(22, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* BOOT */
    GPIO_CFG(24, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* STANDBY */
    GPIO_CFG(25, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA), /* RESET */
    GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VDD Core */
};

static void config_gpio_table(uint32_t *table, int len)
{
    int n, rc;
    for (n = 0; n < len; n++) {
        rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
        if (rc) {
            pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
                __func__, table[n], rc);
            break;
        }
    }
}

static int config_camera_on_gpios(void)
{
    if (gpio_request( 2, "m_cam_scl" ))
        pr_err("gpio_request failed on SCL(2)\n");
    if (gpio_request( 3, "m_cam_sda" ))
        pr_err("gpio_request failed on SDA(3)\n");

    config_gpio_table(camera_on_gpio_table,
        ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
    gpio_export( 2, false );
    gpio_free( 2 );
    gpio_export( 3, false );
    gpio_free( 3 );

    config_gpio_table(camera_off_gpio_table,
        ARRAY_SIZE(camera_off_gpio_table));
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
    .ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};

#ifdef CONFIG_CAM8MP
static struct msm_camera_sensor_flash_data flash_cam8mp = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_cam8mp_data = {
    .sensor_name    = "cam8mp",
    .sensor_reset   = 25,
    .sensor_pwd     = 31,
    .vcm_pwd        = 22,
    .vcm_enable     = 24,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
    .flash_data     = &flash_cam8mp,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_cam8mp = {
    .name      = "msm_camera_cam8mp",
	.dev       = {
        .platform_data = &msm_camera_sensor_cam8mp_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif
#endif /*CONFIG_F11K06_CAMERA*/

/* FUJITSU:2011_06_04   start */
static struct i2c_board_info compassinfo[] __initdata = {
	{
		I2C_BOARD_INFO("compass", 0x0c),
	},
};
/* FUJITSU:2011_06_04   end */


#ifdef CONFIG_MSM7KV2_AUDIO
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t audio_fluid_icodec_tx_config =
  GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
}

void msm_snddev_tx_route_config(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		} else
			gpio_set_value(85, 0);
	}
}

void msm_snddev_tx_route_deconfig(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		}
	}
}

void msm_snddev_poweramp_on(void)
{
/* FUJITSU:2011-04-08 speaker amp start */
#if 0
	gpio_set_value(82, 1);	/* enable spkr poweramp */
#else
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(19,1);
#endif
/* FUJITSU:2011-04-08 speaker amp end */
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
/* FUJITSU:2011-04-08 speaker amp start */
#if 0
	gpio_set_value(82, 0);	/* disable spkr poweramp */
#else
	gpio_set_value(19,0);
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
#endif
/* FUJITSU:2011-04-08 speaker amp end */
	pr_info("%s: power off amplifier\n", __func__);
}

static struct vreg *snddev_vreg_ncp, *snddev_vreg_gp4;

void msm_snddev_hsed_voltage_on(void)
{
	int rc;

	snddev_vreg_gp4 = vreg_get(NULL, "gp4");
	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_enable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_enable(gp4) failed (%d)\n", __func__, rc);

	snddev_vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_enable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable(ncp) failed (%d)\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc;

	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_disable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(snddev_vreg_ncp);

	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_disable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_disable(gp4) failed (%d)\n", __func__, rc);

	vreg_put(snddev_vreg_gp4);

}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}
#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

/* FUJITSU:2011-04-12  Add for QWERTY keyboard start */
#ifdef CONFIG_MACH_F11K06
    if (msm_gpios_request_enable(f11k06_keyboard_gpios, ARRAY_SIZE(f11k06_keyboard_gpios)))
        pr_err("%s: unable to enable gpios\n", __func__);
    pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data = &f11k06_keyboard_data;
    pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size = sizeof f11k06_keyboard_data;
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-12  Add for QWERTY keyboard end */

	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}


struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};


static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;
static struct vreg *vreg_marimba_3;

static u8 read_bahama_ver(void)
{
	int rc;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };
	u8 bahama_version;

	rc = marimba_read_bit_mask(&config, 0x00,  &bahama_version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			 "%s: version read failed: %d\n",
			__func__, rc);
			return rc;
	} else {
		printk(KERN_INFO
		"%s: version read got: 0x%x\n",
		__func__, bahama_version);
	}

	switch (bahama_version) {
	case 0x08: /* varient of bahama v1 */
	case 0x10:
	case 0x00:
		return VER_1_0;
	case 0x09: /* variant of bahama v2 */
		return VER_2_0;
	default:
		return VER_UNSUPPORTED;
	}
}


static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {

		int i;
		struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};

		if (read_bahama_ver() == VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					printk(KERN_ERR
						"%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				printk(KERN_INFO "%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	printk(KERN_INFO "core type: %d\n", type);

	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_3);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
				__func__, rc);
	}

	return rc;
};

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (value != BAHAMA_ID) {
		rc = vreg_disable(vreg_marimba_3);
		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
	}

	return rc;
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = msm_gpios_request_enable(marimba_svlte_config_clock,
				ARRAY_SIZE(marimba_svlte_config_clock));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: msm_gpios_request_enable failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = gpio_direction_output(GPIO_PIN
			(marimba_svlte_config_clock->gpio_cfg), 0);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio_direction_output failed (%d)\n",
					__func__, rc);
			return rc;
		}
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B


static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.bahama_core_config = msm_bahama_core_config,
	.codec = &mariba_codec_pdata,
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */

static void __init msm7x30_init_marimba(void)
{
	int rc;

	vreg_marimba_1 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_1, 1800);

	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_2, 1200);

	vreg_marimba_3 = vreg_get(NULL, "usb2");
	if (IS_ERR(vreg_marimba_3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_3));
		return;
	}
	rc = vreg_set_level(vreg_marimba_3, 1800);
}


#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};



/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static struct resource msm_mi2s_resources[] = {
/* FUJITSU:2011-02-21 del DTV start */
/* FUJITSU:2011-02-21 del DTV end */
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */


static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

/* FUJITSU:2011_06_04   start */
static struct fj_wm_platform_data fj_wm_device_platform_data = {
	.motion_irq = MSM_GPIO_TO_INT(42),
	.mc_init_delay = 20,
};

static struct platform_device fj_walkmotion_device = {
	.name           = "fj-walkmotion",
	.id             = 0,
	.dev = {
		.platform_data = &fj_wm_device_platform_data
	},
};

static struct i2cuart_platform_data i2cuart_device_platform_data = {
/* FUJITSU:2011_06_06 mod start */
#if 1
	.irq = MSM_GPIO_TO_INT( KO_UART4_NIRQ ),
#else
	.irq = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, (KO_UART4_NIRQ - 1)),
#endif
/* FUJITSU:2011_06_06 mod end */
};

struct platform_device i2cuart_device = {
	.name	= "i2cuart",
	.id	= -1,
	.dev = {
		.platform_data = &i2cuart_device_platform_data
	},
};
/* FUJITSU:2011_06_04   end */

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x8D000000,
		.end		= 0x8D000100,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= MSM_GPIO_TO_INT(88),
		.end		= MSM_GPIO_TO_INT(88),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr6" },
    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr5" },
    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr4" },
    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr3" },
    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr2" },
    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr1" },
    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr0" },
    { GPIO_CFG(88, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "smsc911x_irq"  },
};

static void msm7x30_cfg_smsc911x(void)
{
	int rc;

	rc = msm_gpios_request_enable(smsc911x_gpios,
			ARRAY_SIZE(smsc911x_gpios));
	if (rc)
		pr_err("%s: unable to enable gpios\n", __func__);
}

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
#if 0
	.vendor         = "GOOGLE",
#else
    .vendor         = "Fujitsu",
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
static char *usb_functions_normal[] = {
    "usb_mass_storage",
};
static char *usb_functions_normal_adb[] = {
    "usb_mass_storage",
    "adb",
};
#if 1
static char *usb_functions_lismo[] = {
    "lismo",
//  "usb_mass_storage",
};
static char *usb_functions_lismo_adb[] = {
    "lismo",
    "adb",
//  "usb_mass_storage",
};
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *fusion_usb_functions_default[] = {
	"diag",
	"nmea",
	"usb_mass_storage",
};

static char *fusion_usb_functions_default_adb[] = {
	"diag",
	"adb",
	"nmea",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

/* FUJITSU:2011-04-20 USB_GB_LISMO start */
#if 0
static char *usb_functions_rndis_diag[] = {
	"rndis",
	"diag",
};

static char *usb_functions_rndis_adb_diag[] = {
	"rndis",
	"diag",
	"adb",
};
#endif

#if 0
static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};
#else
static char *usb_functions_all[] = {
    "lismo",
#ifdef CONFIG_USB_ANDROID_RNDIS
    "rndis",
#endif
    "usb_mass_storage",
    "adb",
#ifdef CONFIG_USB_ANDROID_DIAG
    "diag",
#endif
#ifdef CONFIG_USB_F_SERIAL
    "modem",
    "nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
    "rmnet",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
    "acm",
#endif
};
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */

static struct android_usb_product usb_products[] = {
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
    /* Normal Mode                                                      */
    /* MSC */
    {
        .product_id     = 0x12A8,
        .num_functions  = ARRAY_SIZE(usb_functions_normal),
        .functions      = usb_functions_normal,
    },
    /* ADB + MSC */
    {
        .product_id     = 0x12A8,
        .num_functions  = ARRAY_SIZE(usb_functions_normal_adb),
        .functions      = usb_functions_normal_adb,
    },
#if 1
    /* Lismo Serial Mode                                                 */
    /* LISMO */
    {
        .product_id     = 0x12AA,
        .num_functions  = ARRAY_SIZE(usb_functions_lismo),
        .functions      = usb_functions_lismo,
    },
    /* ADB + LISMO */
    {
        .product_id     = 0x12AA,
        .num_functions  = ARRAY_SIZE(usb_functions_lismo_adb),
        .functions      = usb_functions_lismo_adb,
    },
#endif
    /* Tethering Mode                                                    */
    {
        .product_id     = 0x12A9,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
        .functions      = usb_functions_rndis,
    },
    {
        .product_id     = 0x12A9,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
        .functions      = usb_functions_rndis_adb,
    },
    /* Debug Mode                                                         */
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
	{
		.product_id     = 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions      = usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
#if 0
	{
		/* RNDIS + DIAG */
		.product_id	= 0x902C,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
	{
		/* RNDIS + ADB + DIAG */
		.product_id	= 0x902D,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
};

static struct android_usb_product fusion_usb_products[] = {
	{
		.product_id     = 0x9028,
		.num_functions  = ARRAY_SIZE(fusion_usb_functions_default),
		.functions      = fusion_usb_functions_default,
	},
	{
		.product_id     = 0x9029,
		.num_functions  = ARRAY_SIZE(fusion_usb_functions_default_adb),
		.functions      = fusion_usb_functions_default_adb,
	},
	{
		.product_id     = 0xf00e,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis),
		.functions      = usb_functions_rndis,
	},
	{
		.product_id     = 0x9024,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
/* FUJITSU:2011-06-07 USB_GB_LISMO start */
#if 0
	.vendor		= "Qualcomm Incorporated",
#else
	.vendor		= "Fujitsu",
#endif
/* FUJITSU:2011-06-07 USB_GB_LISMO end */
	.product        = "Mass storage",
	.release	= 0x0100,
	.can_stall	= 1,

};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
#if 0
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
#else
    .vendorID       = 0x04C5,
    .vendorDescr    = "Fujitsu",
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
/* FUJITSU:2011-04-20 USB_GB_LISMO start */
#if 0
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name		= "Qualcomm HSUSB Device",
	.manufacturer_name	= "Qualcomm Incorporated",
#else
    .vendor_id  = 0x04C5,
    .product_id = 0x12A8,
    .version    = 0x0100,
    .product_name       = "Fujitsu HSUSB Device",
    .manufacturer_name  = "Fujitsu",
#endif
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

/* FUJITSU:2011-04-26   USB setteing start */
#ifdef CONFIG_MACH_F11K06
char f11k06_serial_number[32];
static void msm_read_nv_serial(char *serialno) {
	uint8_t *dataread = NULL;
	uint8_t nv_serial_read[12] = {0};
	uint8_t nv_serial_read_temp[8] = {0};
	uint8_t byte1;
	uint8_t byte2;
	int i;

	memset(serialno, 0, SMEM_V1_OEM_011_SIZE);
	dataread = smem_alloc_vendor1(SMEM_OEM_011);
	if(dataread == NULL)
	{
		strcpy(serialno,"K0600000");   //need to get the string to add
printk(KERN_INFO "msm_read_nv_serial f11k06_serial_number = %s \n",f11k06_serial_number);
		return;
	}
	else
	{
		memcpy(nv_serial_read_temp, dataread, SMEM_V1_OEM_011_SIZE);
		memcpy(nv_serial_read, dataread, SMEM_V1_OEM_011_SIZE);
		for (i=0;i<3;i++)
		{
			byte1 = nv_serial_read_temp[5+i];
			byte1 = byte1 >> 4;
			byte1 = byte1 & 0x0F;
			byte1 = byte1 + 0x30;
			byte2 = nv_serial_read_temp[5+i] & 0x0F;
			byte2 = byte2 + 0x30;
			nv_serial_read [4 + ((2 * i) + 1)] = byte1;
			nv_serial_read [5 + ((2 * i) + 1)] = byte2;
		}
		nv_serial_read[11] = '\0';
		strcpy(serialno,nv_serial_read);
#if 0
		for (i=0;i<16;i++)
		{
printk(KERN_INFO "msm_read_nv_serial nv_serial_read[%d] = %02x = %c \n",i,serialno[i],nv_serial_read[i]);
		}
#endif
		if (strlen(serialno) == 0) {
			strcpy(serialno,"K0600000");   //need to get the string to add
printk(KERN_INFO "msm_read_nv_serial Not NULL Setf11k06_serial_number = %s \n",f11k06_serial_number);
		}
	}
	return;
}
/* FUJITSU:2011-04-20 USB_GB_LISMO end */
#else
/* FUJITSU:2011-04-26 USB setteing start */
#define SERIAL_NUMBER_SIZE	11
static void msm_read_nv_serial(char *serialno)
{
   // because SMEM ID not defined.
	uint8_t *dataread = NULL;
	uint8_t nv_serial_read[MSM_SERIAL_NUMBER_SIZE+1] = {0};
	uint8_t nv_serial_read_temp[MSM_NV_UE_IMEI_SIZE] = {0};
	uint8_t byte1;
	uint8_t byte2;
	int i;

	dataread = smem_alloc(SMEM_OEM_001,MSM_NV_UE_IMEI_SIZE);
	if(dataread == NULL)
	{
//		memset(serialno,0,SERIAL_NUMBER_SIZE);
		strcpy(serialno,"EIF0000000");   //need to get the string to add
		return;
	}
	else
	{
		memcpy(nv_serial_read_temp, dataread, MSM_NV_UE_IMEI_SIZE);
		memcpy(nv_serial_read, dataread, MSM_NV_UE_IMEI_SIZE);
		for (i=0;i<3;i++)
		{
			byte1 = nv_serial_read_temp[5+i];
			byte1 = byte1 >> 4;
			byte1 = byte1 & 0x0F;
			byte1 = byte1 + 0x30;

			byte2 = nv_serial_read_temp[5+i] & 0x0F;
			byte2 = byte2 + 0x30;
			nv_serial_read [4 + ((2 * i) + 1)] = byte1;
			nv_serial_read [5 + ((2 * i) + 1)] = byte2;
		}
		nv_serial_read[MSM_SERIAL_NUMBER_SIZE] = '\0';
//		memset(serialno,0,MSM_SERIAL_NUMBER_SIZE);
		strcpy(serialno,nv_serial_read);
	}
	return;
}
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-26   USB setteing end */

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

/* FUJITSU:2011-04-26   USB setteing start */
#ifdef CONFIG_MACH_F11K06
	strcpy(f11k06_serial_number,"K06_____");   //need to get the string to add
	msm_read_nv_serial(f11k06_serial_number);
#else
	msm_read_nv_serial(serialno);
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-26   USB setteing end */

/* FUJITSU:2011-05-16 USB setteing start */
#ifdef CONFIG_MACH_F11K06
	android_usb_pdata.serial_number = f11k06_serial_number;
#else
	android_usb_pdata.serial_number = serialno;
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-05-16 USB setteing end */
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#endif


/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static struct i2c_board_info msm_i2c_board_info[] = {

};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

};
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM,
	.vendor_id	= 0x5c6,
	.product_name	= "Qualcomm HSUSB Device",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name
			= "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions
			= ARRAY_SIZE(usb_func_composition),
	.function_map	= usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.core_clk	= 1,
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,

/* FUJITSU:2011-02-21  SUSPEND POWER COLLAPSE disable start */
#if 1
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
#endif
#if 0
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 0,
#endif
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
#if 1
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
#endif
#if 0
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 0,
#endif
/* FUJITSU:2011-02-21  SUSPEND POWER COLLAPSE disable end */

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 1,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;

/* FUJITSU:2011-05-06   add BT HS start */
#ifdef MSM_SPI_CRCI_DISABLE
	u32 gp_ctl = 0;
#endif
/* FUJITSU:2011-05-06   add BT HS end */

	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

/* FUJITSU:2011-05-06   add BT HS start */
#ifdef MSM_SPI_CRCI_DISABLE
	gp_ctl = ioread32(ADMH0_GP_CTL);

	gp_ctl = gp_ctl & 0xFFFFCFFF;
	iowrite32(gp_ctl,ADMH0_GP_CTL);

	gp_ctl = ioread32(ADMH0_GP_CTL);
#endif
/* FUJITSU:2011-05-06   add BT HS end */

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

#ifdef CONFIG_SPI_QSD
static struct spi_board_info lcdc_sharp_spi_board_info[] __initdata = {
	{
		.modalias	= "lcdc_sharp_ls038y7dx01",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 26331429,
	}
};
static struct spi_board_info lcdc_toshiba_spi_board_info[] __initdata = {
	{
		.modalias       = "lcdc_toshiba_ltm030dd40",
		.mode           = SPI_MODE_3|SPI_CS_HIGH,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 9963243,
	}
};

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
static struct spi_board_info k06_spi_board_info[] __initdata = {
    {
        .modalias      = "cam8mp",
        .mode          = SPI_MODE_3,
        .bus_num       = 0,
        .chip_select   = 1,
        .max_speed_hz  = 9596343,
    },
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */
#endif

static struct msm_gpio qsd_spi_gpio_config_data[] = {
 #ifndef CONFIG_F11K06_CAMERA
	{ GPIO_CFG(45, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
 #endif
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.clk_name = "spi_clk",
	.pclk_name = "spi_pclk",
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
/* FUJITSU:2011-06-21 USB start */
#if 0	/* USB_GB Out 2011-06-21 */
        int rc;
        static int vbus_is_on;
        struct pm8058_gpio usb_vbus = {
                .direction      = PM_GPIO_DIR_OUT,
                .pull           = PM_GPIO_PULL_NO,
                .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
                .output_value   = 1,
                .vin_sel        = 2,
                .out_strength   = PM_GPIO_STRENGTH_MED,
                .function       = PM_GPIO_FUNC_NORMAL,
                .inv_int_pol    = 0,
        };

        /* If VBUS is already on (or off), do nothing. */
        if (unlikely(on == vbus_is_on))
                return;

        if (on) {
                rc = pm8058_gpio_config(36, &usb_vbus);
                if (rc) {
                        pr_err("%s PMIC GPIO 36 write failed\n", __func__);
                        return;
                }
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

        vbus_is_on = on;
#else	/* USB_GB Out 2011-06-21 */
/* FUJITSU:2011-07-13 USB start */
	pr_info("[f11k06]%s: %s phy_info(%d)\n", __func__, (on)?"ON":"OFF", PHY_TYPE(phy_info));
	if (on) {
		msm_hsusb_vbus_powerup();
	}
	else {
		msm_hsusb_vbus_shutdown();
	}
/* FUJITSU:2011-07-13 USB end */
#endif	/* USB_GB Out 2011-06-21 */      
/* FUJITSU:2011-06-21 USB end */
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
};

#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
/* FUJITSU:2011-06-21 USB start */
#if 0	/* USB_GB Out 2011-06-21 */
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif
/* FUJITSU:2011-06-21 USB end */
#endif

#ifndef CONFIG_USB_EHCI_MSM
/* FUJITSU:2011-07-13 USB start */
#if 0  
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
/* FUJITSU:2011-07-13 USB end */
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM
/* FUJITSU:2011-07-13 USB start */
#if 0  
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#endif
/* FUJITSU:2011-07-13 USB end */
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
/* FUJITSU:2011-06-21 USB start */
#if 0	/* USB_GB Out 2011-06-21 */
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
#endif
/* FUJITSU:2011-06-21 USB end */
};

/* FUJITSU:2011/05/12  TMS start */
#ifdef CONFIG_MACH_F11K06
static struct platform_device memread_dev = {
	.name = "mread",
};
#endif
/* FUJITSU:2011/05/12  TMS end */

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM
/* FUJITSU:2011-07-13 USB start */
#if 0
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif
#endif
/* FUJITSU:2011-07-13 USB end */

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};


/* FUJITSU:2011-05-06     add BT HS start */
#ifdef MSM_UARTDM2_USE
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};
#endif
/* FUJITSU:2011-05-06     add BT HS end */


static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (machine_is_msm7x30_fluid()) {
		if (!strcmp(name, "lcdc_sharp_wvga_pt"))
			return 0;
	} else {
		if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
			return -EPERM;
		else if (!strncmp(name, "lcdc_toshiba_wvga_pt", 20))
			return 0;
		else if (!strcmp(name, "mddi_orise"))
			return -EPERM;
		else if (!strcmp(name, "mddi_quickvx"))
			return -EPERM;
	}
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
       .name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	* the only valid choice at this time. The board structure is
	* set to all zeros by the C runtime initialization and that is now
	* the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	* include/linux/android_pmem.h.
	*/
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	/* NPA Flow IDs */
	.high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
	.high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
	/* AXI rates in KHz */
	.high_axi_3d = 192000,
	.high_axi_2d = 192000,
#endif
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 364800000,
	.min_grp3d_freq = 249600000,
	.set_grp3d_async = set_grp3d_async,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp3d_pclk_name = "grp_pclk",
#ifdef CONFIG_MSM_KGSL_2D
	.grp2d0_clk_name = "grp_2d_clk",
	.grp2d0_pclk_name = "grp_2d_pclk",
#else
	.grp2d0_clk_name = NULL,
#endif
	.idle_timeout_3d = HZ/20,
	.idle_timeout_2d = HZ/10,
	.nap_allowed = true,

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	.pt_va_size = SZ_32M,
	/* Maximum of 32 concurrent processes */
	.pt_max_count = 32,
#else
	.pt_va_size = SZ_128M,
	/* We only ever have one pagetable for everybody */
	.pt_max_count = 1,
#endif
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d0_reg_memory",
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_2d0_irq",
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1

#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qce_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static int mddi_toshiba_pmic_bl(int level)
{
	int ret = -EPERM;

	ret = pmic_set_led_intensity(LED_LCD, level);

	if (ret)
		printk(KERN_WARNING "%s: can't set lcd backlight!\n",
					__func__);
	return ret;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

/* FUJITSU:2011-05-02    bootbacklight ctrl add */
#ifdef CONFIG_MACH_F11K06
static bool backlight_init = false; 
static int  reset_skip = 2;
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */

/* FUJITSU:2011-03-03 display power start */
#ifdef CONFIG_MACH_F11K06
static int display_common_power(int on)
{
    int ret = 0;
    struct i2c_adapter *i2c_lcd;
    struct i2c_msg msg;
    u_int8_t buffer[8];
    int try = 0;

    //lcd power on
    msg.addr    = LED_2C_SLAVE_ADDR;
    msg.buf     = buffer;
    msg.len     = 2;
    msg.flags   = 0;
    
#if 0    
    //i2c qup adapter id=4
	    i2c_lcd = i2c_get_adapter(4);
#else
    //20110328 
	    i2c_lcd = i2c_get_adapter(0);
#endif
    
    pr_err("[LCD_atBRD]%s: Enter(%d) adapter(%p)\n", __func__,on,i2c_lcd);
    
    if (on) {
        //lcd driver
        //LDO1 and LDO2 Vout control
        //client register address
        buffer[0] = 0x14;
        //data  LDO2=2.8V LDO1=1.8V
        buffer[1] = 0xA4;
        ret = i2c_transfer(i2c_lcd, &msg, 1);
        if(ret < 0){
            pr_err("[DD](%s):I2C ERROR [%d] ret = %d *1*\n",__func__, __LINE__, ret);
            for (try = 0; try < 50; try++) {
                msleep(5);
                ret = i2c_transfer(i2c_lcd, &msg, 1);
                if (ret >= 0 ) {
                    pr_err("[DD](%s):I2C ERROR [%d] ret = %d *1* retry(%d):OK\n",__func__, __LINE__, ret,try);
                    break;
                }
            }
            if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *1* retry(%d):NG\n",__func__, __LINE__, ret,try);
                return ret;
            }
        }

        //touch driver
        //LDO3 and LDO4 Vout control
        //client register address
        buffer[0] = 0x15;
        //data  LDO4=2.8V LDO3=1.8V
        buffer[1] = 0xA4;
        ret = i2c_transfer(i2c_lcd, &msg, 1);
        if(ret < 0){
            pr_err("[DD](%s):I2C ERROR [%d] ret = %d *2*\n",__func__, __LINE__, ret);
            for (try = 0; try < 50; try++) {
                msleep(5);
                ret = i2c_transfer(i2c_lcd, &msg, 1);
                if (ret >= 0 ) {
                    pr_err("[DD](%s):I2C ERROR [%d] ret = %d *2* retry(%d):OK\n",__func__, __LINE__, ret,try);
                    break;
                }
            }
            if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *2* retry(%d):NG\n",__func__, __LINE__, ret,try);
                return ret;
            }
        }
        
        //LDO Power control
        //client register address
        buffer[0] = 0x13;
        //data LDO1 and LDO2 and LD03 and LD04 [ON]

        /* FUJITSU:2011-07-05 change LD04 disable start */
        if (system_rev >= 0x02)
        {
            buffer[1] = 0x03;
        }
        else
        {
        buffer[1] = 0x0F;
        }
        /* FUJITSU:2011-07-05 change LD04 disable end */

        ret = i2c_transfer(i2c_lcd, &msg, 1);
        if(ret < 0){
            pr_err("[DD](%s):I2C ERROR [%d] ret = %d *3*\n",__func__, __LINE__, ret);
            for (try = 0; try < 50; try++) {
                msleep(5);
                ret = i2c_transfer(i2c_lcd, &msg, 1);
                if (ret >= 0 ) {
                    pr_err("[DD](%s):I2C ERROR [%d] ret = %d *3* retry(%d):OK\n",__func__, __LINE__, ret,try);
                    break;
                }
            }
            if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *3* retry(%d):NG\n",__func__, __LINE__, ret,try);
                return ret;
            }
        }
        
/* FUJITSU:2011-09-30 reset_move start */
        if(reset_skip == 0) {
            printk("[LCD_atBRD]Reset on\n");
            msleep(50);
            gpio_set_value(33,1);
            msleep(10);
            gpio_set_value(33,0);
            msleep(11);
            gpio_set_value(33,1);
            msleep(20);
        }
        else {
            printk("[LCD_atBRD] Reset skip %d\n",reset_skip);
            reset_skip--;
        }
/* FUJITSU:2011-09-30 reset_move end */
        
/* FUJITSU:2011-05-02 bootbacklight ctrl start */
        if(!backlight_init){
            backlight_init = true;
            
            //backlight LED pin setting
            buffer[0] = 0x01;
            buffer[1] = 0x0E;
            ret = i2c_transfer(i2c_lcd, &msg, 1);
            if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *4*\n",__func__, __LINE__, ret);
                return ret;
            }
            
        //backlight on
            //Main LED Current setting
            //client register address
            buffer[0] = 0x03;
            //data  20mA
//            buffer[1] = 0x63;/* FUJITSU:2011-06-16 bootbacklight brightness chg */
            buffer[1] = 0x30;  /* FUJITSU:2011-06-16 bootbacklight brightness chg */
            ret = i2c_transfer(i2c_lcd, &msg, 1);
           if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *5*\n",__func__, __LINE__, ret);
                return ret;
            }

            //client register address
            buffer[0] = 0x02;
            //data  MLEDEN [ON]
            buffer[1] = 0x01;
            ret = i2c_transfer(i2c_lcd, &msg, 1);
            if(ret < 0){
                pr_err("[DD](%s):I2C ERROR [%d] ret = %d *6*\n",__func__, __LINE__, ret);
                return ret;
            }
        }//backlight_init
/* FUJITSU:2011-05-02 bootbacklight ctrl end */
    }
    else {
/* FUJITSU:2011-09-30 reset_move start */
        msleep(100);
        printk("[LCD_atBRD]Reset Low\n");
        gpio_set_value(33,0);
        msleep(15);
/* FUJITSU:2011-09-30 reset_move start */
        
        //LDO1 and LDO2 Power control
        //client register address
        buffer[0] = 0x13;
        //data LDO1 and LDO2 off. but keep LD04 [ON] (for touch drv)

/* FUJITSU:2011-07-29 bootbacklight ctrl end */
        if (system_rev >= 0x02) {
            buffer[1] = 0x01;
        }
        else {
            buffer[1] = 0x09;
        }
        ret = i2c_transfer(i2c_lcd, &msg, 1);
        if(ret < 0){
            pr_err("[DD](%s):I2C ERROR [%d] ret = %d *7*\n",__func__, __LINE__, ret);
            return ret;
        }
        
        msleep(7);
/* FUJITSU:2011-07-29 bootbacklight ctrl end */
        
        /* FUJITSU:2011-07-05 change LD04 disable start */
        if (system_rev >= 0x02)
        {
            buffer[1] = 0x00;
        }
        else
        {
        buffer[1] = 0x08;
        }
        /* FUJITSU:2011-07-05 change LD04 disable end */

        ret = i2c_transfer(i2c_lcd, &msg, 1);
        if(ret < 0){
            pr_err("[DD](%s):I2C ERROR [%d] ret = %d *8*\n",__func__, __LINE__, ret);
            return ret;
        }
    }
    pr_err("[LCD_atBRD]%s: Leave(%d)\n", __func__,on);

    return ret;
}
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */


static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */

int mdp_core_clk_rate_table[] = {
	122880000,
	122880000,
	122880000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 122880000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
};

/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvout_device", NULL);
#endif
}
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */


/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

// PMIC GPIO PORT
#define KO_PMICUART1_BT_TXD_21   (21-1)
#define KO_PMICUART1_BT_RXD_33   (33-1)
#define KO_PMICUART1_BT_PMIC_TXD_36 (36-1)
#define KO_PMICUART1_BT_PMIC_RXD_37 (37-1)

static unsigned bt_config_power_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
    GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
    GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
	GPIO_CFG(21,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PWON           */
	GPIO_CFG(96,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
	GPIO_CFG(38,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_RESET        */
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_PWON         */
	GPIO_CFG(20,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_HOSTWAKEUP   */
/* FUJITSU:2011-05-06 add BT HS start */
#ifdef MSM_UARTDM2_USE
	GPIO_CFG(49, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(50, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(51, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
#endif
/* FUJITSU:2011-05-06 add BT HS end */
	};

static unsigned bt_config_power_off[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
	GPIO_CFG(21,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PWON           */
	GPIO_CFG(96,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
	GPIO_CFG(38,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_RESET        */
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_PWON         */
	GPIO_CFG(20,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_HOSTWAKEUP   */
};

/* Status (Power) */
static int wlan_power_flg = 0;
static int bt_power_flg = 0;

/* Function List */
static void set_power_wlan_bt(int, int);

/* Function (set power) */
void set_power_wlan_bt(int on, int is_wlan)
{
	printk(KERN_DEBUG "%s S\n", __func__);

	if ( on )
	{
		printk(KERN_ERR "BT/WLAN-ON \n");
		if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
		{
			gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
			msleep(1);
			gpio_set_value(21, on); /* KO_BT_WAKEUP 		*/
			gpio_set_value(43, on); /* KO_BT_PWON    		*/
			gpio_set_value(39, on); /* KO_WLAN_PWON  */
		}
		
		if ( is_wlan )
		{
			wlan_power_flg = 1;
		}
		else
		{
			bt_power_flg = 1;
		}
	}
	else
	{
		printk(KERN_ERR "BT/WLAN-OFF \n");
		if ( ( wlan_power_flg ) || ( bt_power_flg ) )
		{
			if ( is_wlan )
			{
				wlan_power_flg = 0;
			}
			else
			{
				bt_power_flg = 0;
			}
			if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
			{
				gpio_set_value(39, on); /* KO_WLAN_PWON  */
				gpio_set_value(43, on); /* KO_BT_PWON    		*/
				gpio_set_value(21, on); /* KO_BT_WAKEUP 		*/
				gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
				msleep(1);
			}
		}
	}
	
	printk(KERN_DEBUG "%s E\n", __func__);
	
	return;
}


/* Function List */
int wlan_power(int on);
int wlan_set_carddetect(void);

/* Platform Data (WLAN) */
static struct wifi_platform_data wlan_wifi_control =
{
	.set_power			=	wlan_power,
	.set_reset			=	NULL,
	.set_carddetect			=	NULL,
	.mem_prealloc			=	NULL,
};
/* Resource (WLAN) */
static struct resource wlan_resources[] =
{
	[0] =
	{
		.name			=	"bcm4329_wlan_irq",
		.start			=	MSM_GPIO_TO_INT(20),
		.end			=	MSM_GPIO_TO_INT(20),
		.flags			=	IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};
/* Platform Device (WLAN) */
static struct platform_device wlan_device =
{
	.name				=	"bcm4329_wlan",
	.num_resources			=	ARRAY_SIZE(wlan_resources),
	.resource			=	wlan_resources,
	.dev				=
	{
		.platform_data		=	&wlan_wifi_control,
	},
};
int wlan_power(int on)
{
	int rc,pin;

	printk(KERN_DEBUG "%s S\n", __func__);

	if (on) {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: Err gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}

		set_power_wlan_bt(1,1);
		
		gpio_set_value(38, on); /* KO_WLAN_RESET */
		msleep(110);
	} else {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}

		// GPIO SET VALUE
		wlan_set_carddetect();
		msleep(130);
		gpio_set_value(38, on); /* KO_WLAN_RESET */
		msleep(110);
		
		set_power_wlan_bt(0,1);
	}
	printk(KERN_DEBUG "%s E\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(wlan_power);

void mmc_detect_change_for_bcm4329(unsigned long);

/* Function (set carddetect) */
int wlan_set_carddetect(void)
{
	mmc_detect_change_for_bcm4329(msecs_to_jiffies(2000));

	return 0;
}

static struct platform_device msm_wlan_power_device = {
	.name = "wlan_power",
};

static void __init wlan_power_init(void)
{
int pin,rc;
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: Err gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return ;
			}
		}

		// GPIO REQUEST

		rc = gpio_request(96, "KO_WLAN_BT_PWR_ONOFF");
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_BT_PWR_ONOFF\n");
		}
		rc = gpio_request(39, "KO_WLAN_PWON");
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_PWON\n");
		}
		rc = gpio_request(38, "KO_WLAN_RESET");
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_RESET\n");
		}
	
	msm_wlan_power_device.dev.platform_data = &wlan_power;
}

// PMIC GPIO 21
struct pm8058_gpio bt_txd_en_21 = {
 .direction      = PM_GPIO_DIR_OUT,
 .pull           = PM_GPIO_PULL_NO,
 .vin_sel        = PM_GPIO_VIN_S3,
 .function       = PM_GPIO_FUNC_2,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_MED,
 .output_value   = 0,
};

// PMIC GPIO 33
struct pm8058_gpio bt_rxd_en_33 = {
 .direction      = PM_GPIO_DIR_IN,
 .pull           = PM_GPIO_PULL_NO,
 .vin_sel        = PM_GPIO_VIN_S3,
 .function       = PM_GPIO_FUNC_1,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_MED,
 .output_value   = 0,
};

// PMIC GPIO 36
struct pm8058_gpio bt_pmic_txd_en_36 = {
 .direction      = PM_GPIO_DIR_IN,
 .pull           = PM_GPIO_PULL_NO,
 .vin_sel        = PM_GPIO_VIN_S3,
 .function       = PM_GPIO_FUNC_1,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_MED,
 .output_value   = 0,
};

// PMIC GPIO 37
struct pm8058_gpio bt_pmic_rxd_en_37 = {
 .direction      = PM_GPIO_DIR_OUT,
 .pull           = PM_GPIO_PULL_NO,
 .vin_sel        = PM_GPIO_VIN_S3,
 .function       = PM_GPIO_FUNC_2,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_MED,
 .output_value   = 0,
};


static int bluetooth_power(int on)
{
	int pin, rc;

	if (on) {

// PMIC_GPIO SETTING DISABLE
		// PMIC GPIO CONFIG

		// TX
		rc = pm8058_gpio_config(KO_PMICUART1_BT_TXD_21, &bt_txd_en_21);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
		return -EIO;
		}

		// RX
		rc = pm8058_gpio_config(KO_PMICUART1_BT_RXD_33, &bt_rxd_en_33);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		// PMIC
		rc = pm8058_misc_control(NULL, PM8058_UART_MUX_MASK, PM8058_UART_MUX_1);
		if (rc) {
			printk(KERN_ERR "%s: pm8058_misc_control failed=%d\n",__func__,rc);
		}
		rc = pm8058_gpio_config(KO_PMICUART1_BT_PMIC_TXD_36, &bt_pmic_txd_en_36);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8058_gpio_config(KO_PMICUART1_BT_PMIC_RXD_37, &bt_pmic_rxd_en_37);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		// GPIO TLMM CONFIG
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
					printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
					return -EIO;
				}
			}

		// GPIO SET VALUE
		set_power_wlan_bt(1,0);
		gpio_set_value(41, on); /* KO_BT_RESET   		*/
		msleep(110);
		
	} else {
		// GPIO SET VALUE
		gpio_set_value(41, on); /* KO_BT_RESET 			*/
		msleep(110);
		set_power_wlan_bt(0,0); 

		// PMIC GPIO CONFIG
		rc = pm8058_gpio_config(KO_PMICUART1_BT_TXD_21, &bt_txd_en_21);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD_21 config failed\n", __func__);
			return -EIO;
		}
		rc = pm8058_gpio_config(KO_PMICUART1_BT_RXD_33, &bt_rxd_en_33);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		rc = pm8058_gpio_config(KO_PMICUART1_BT_PMIC_TXD_36, &bt_pmic_txd_en_36);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8058_gpio_config(KO_PMICUART1_BT_PMIC_RXD_37, &bt_pmic_rxd_en_37);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_RXD config failed\n", __func__);
			return -EIO;
		}

		// GPIO TLMM CONFIG
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
				}
		}
	}
	return 0;
}


static void __init bt_power_init(void)
{
    msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 147,
		.end	= 147,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(147),
		.end	= MSM_GPIO_TO_INT(147),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct msm_psy_batt_pdata msm_psy_batt_data = {
/* FUJITSU:2011-05-10  mod Battery start */
#if 0
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
#else
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
#endif
/* FUJITSU:2011-05-10  mod Battery end */
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};


#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}


static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0002,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};

#endif /* CONFIG_MSM_SDIO_AL */

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
	&qsd_device_spi,
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
	&mddi_toshiba_device,
/* FUJITSU:2011-02-21 del LCDC start */
/* FUJITSU:2011-02-21 del LCDC end */
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
/* FUJITSU:2011-02-21 del LCDC start */
/* FUJITSU:2011-02-21 del LCDC end */
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,

/* FUJITSU:2011-05-06   add BT HS start */
#ifdef MSM_UARTDM2_USE
	&msm_device_uart_dm2,
#endif
/* FUJITSU:2011-05-06   add BT HS end */
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,
/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
/* FUJITSU:2011/05/12   end */
	&msm_bluesleep_device,
	&msm_device_kgsl,
#ifdef CONFIG_F11K06_CAMERA
  #ifdef CONFIG_CAM8MP
    &msm_camera_sensor_cam8mp,
#endif
#endif

	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
#if CONFIG_WIFI_CONTROL_FUNC
	&wlan_device,
#endif
	&msm_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,

/* FUJITSU:2011/05/12  TMS begin */
#ifdef CONFIG_MACH_F11K06
    &memread_dev,
#endif
/* FUJITSU:2011/05/12  TMS end */

    &msm_ebi1_thermal,
/* FUJITSU:2011_06_04   start */
	&fj_walkmotion_device,
	&i2cuart_device
/* FUJITSU:2011_06_04   end */
};

/* FUJITSU:2011/06/28   mod 16mA -> 2mA -> 6mA start */
static struct msm_gpio msm_i2c_gpios_hw[] = {
    { GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "i2c_scl" },
    { GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
    { GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "i2c_scl" },
    { GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "i2c_sda" },
};
/* FUJITSU:2011/06/20  end */


/* FUJITSU:2011/02/22   mod 16MA -> 2MA start */
static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};
/* FUJITSU:2011/02/22 end */

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#if 0   /* FUJITSU:2011/05/26 add Camera del start */
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
#endif  /* FUJITSU:2011/05/26 add Camera del end */
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#if 0   /* FUJITSU:2011/05/26 add Camera del start */
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
#endif
#endif  /* FUJITSU:2011/05/26 add Camera del end */
}
// CHECK:2011/02/22 param check 
static struct msm_i2c_platform_data msm_i2c_pdata = {

/* FUJITSU:2011/05/27   start */
//    .clk_freq = 100000,
    .clk_freq = 400000,
/* FUJITSU:2011/05/27   end */
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#if 0   /* FUJITSU:2011/05/26 add Camera del start */
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
#endif  /* FUJITSU:2011/05/26 add Camera del end */
}
/* FUJITSU:2011_06_04   start */
static struct msm_gpio pedo_gpios[] = {
	{ GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "i2u_rst" },
	{ GPIO_CFG(93,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_rst" },
	{ GPIO_CFG(42,  0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_irq" },
	{ GPIO_CFG(23,  2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_clk" },
/* FUJITSU:2011_06_06 add start */
	{ GPIO_CFG(115,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2u_irq" },
/* FUJITSU:2011_06_06 add end */
};

/* FUJITSU:2011_06_16 del start */
#if 0
static struct msm_gpio pedo_pseudo_gpios[] = {
	{ GPIO_CFG(1,   0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_scl" },
	{ GPIO_CFG(0,   0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_sda" },
};
#endif
/* FUJITSU:2011_06_16 del end */

static void __init pedo_init(void)
{
	int ret = 0;

	ret = msm_gpios_request( pedo_gpios, ARRAY_SIZE(pedo_gpios) );
	ret = msm_gpios_enable( pedo_gpios, ARRAY_SIZE(pedo_gpios) );
/* FUJITSU:2011_06_16 del start */
#if 0
    if(system_rev >= 0x0c) {
		ret = msm_gpios_request( pedo_pseudo_gpios, ARRAY_SIZE(pedo_pseudo_gpios) );
		ret = msm_gpios_enable( pedo_pseudo_gpios, ARRAY_SIZE(pedo_pseudo_gpios) );
	}
#endif
/* FUJITSU:2011_06_16 del end */
}
/* FUJITSU:2011_06_04   end */

#ifdef CONFIG_I2C_SSBI
static struct msm_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};

static struct msm_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static struct msm_acpu_clock_platform_data msm7x30_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

/* FUJUTSU:2011_06_17 GPIO115 Del */
static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
/*	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},*/
};

struct vreg *vreg_s3;
struct vreg *vreg_mmc; 

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
struct vreg *vreg_gp6;/*  SDCC */
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = {
};
static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},
};

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},// KO_WLAN_SD_CLK
/* FUJITSU:2011-09-28 change WLAN start */
//	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
//	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
//	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
//	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
//	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
/* FUJITSU:2011-09-01 change WLAN end */
};
#define SD_GPIONO_SDC4_CLK   58
#define SD_GPIONO_SDC4_CMD   59
#define SD_GPIONO_SDC4_D0    60
#define SD_GPIONO_SDC4_D1    61
#define SD_GPIONO_SDC4_D2    62
#define SD_GPIONO_SDC4_D3    63
#define SD_GPIONO_SD_RST    168
#define SD_GPIONO_SD_CLK     98
#define SD_GPIONO_SD_INT    169

#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},// KO_WLAN_SD_CLK
/* FUJITSU:2011-09-28 change WLAN start */
//	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
//	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
//	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
//	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
//	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},// KO_WLAN_SD_CMD
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_3"},// KO_WLAN_SD_DAT3
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_2"},// KO_WLAN_SD_DAT2
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_1"},// KO_WLAN_SD_DAT1
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_0"},// KO_WLAN_SD_DAT0
/* FUJITSU:2011-09-28 change WLAN end */
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
static struct msm_gpio sdc4_cfg_data[] = {
    { GPIO_CFG( SD_GPIONO_SDC4_CLK , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "SDC4_CLK" },
    { GPIO_CFG( SD_GPIONO_SDC4_CMD , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_CMD" },
    { GPIO_CFG( SD_GPIONO_SDC4_D0  , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D0" },
    { GPIO_CFG( SD_GPIONO_SDC4_D1  , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D1" },
    { GPIO_CFG( SD_GPIONO_SDC4_D2  , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D2" },
    { GPIO_CFG( SD_GPIONO_SDC4_D3  , 1, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D3" },
    { GPIO_CFG( SD_GPIONO_SD_RST   , 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "SD_RST" },
    { GPIO_CFG( SD_GPIONO_SD_CLK   , 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "gp_clk" },
    { GPIO_CFG( SD_GPIONO_SD_INT   , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "SD_INT" },
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}


static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (!enable || enabled_once[dev_id - 1])
		return 0;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#endif


#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1));
}
#endif

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.wpswitch    = msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif

/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
	    .msmsdcc_fmin   = 375000,
	    .msmsdcc_fmid   = 25000000,
	    .msmsdcc_fmax   = 48000000,
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */
	.nonremovable	= 0,
};
#endif

static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

/* FUJITSU:2011-04-07  mod SDCard start */
#ifdef CONFIG_MACH_F11K06
	vreg_mmc = vreg_get(NULL, "mmc");
#endif // CONFIG_MACH_F11EIF
/* FUJITSU:2011-04-07  mod SDCard end */

	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

/* FUJITSU:2011/05/12  SDCC Start */
#ifdef CONFIG_MACH_F11K06
    vreg_gp6 = vreg_get(NULL, "gp6");
    if (IS_ERR(vreg_gp6)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
               __func__, PTR_ERR(vreg_gp6));
        return;
    }
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  SDCC End */

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
/* FUJITSU:2011-04-07  mod SDCard start */
#ifdef CONFIG_MACH_F11K06
    sdcc_vreg_data[3].vreg_data = vreg_gp6; /*  SDCC */
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  SDCC End */

	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{
/* FUJITSU:2011-04-12  Add for KeyBackLight start */
#ifdef CONFIG_MACH_F11K06
    if (gpio_tlmm_config(GPIO_CFG(128, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
        pr_err("%s: gpio_tlmm_config (gpio=[%d]) failed\n", __func__, 128);
    if (gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
        pr_err("%s: gpio_tlmm_config (gpio=[%d]) failed\n", __func__, 129);
    if (gpio_tlmm_config(GPIO_CFG(130, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
        pr_err("%s: gpio_tlmm_config (gpio=[%d]) failed\n", __func__, 130);
    gpio_set_value(128, 0);
    gpio_set_value(129, 0);
    gpio_set_value(130, 0);
/* FUJITSU:2011-07-05 Add for LED start */
    platform_device_register(&notification_led);
/* FUJITSU:2011-07-05 Add for LED end */
    platform_device_register(&msm_device_pmic_leds);
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-12  Add for KeyBackLight end */
}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

/* FUJITSU:2011/05/24 F11EIF del  begin */
#if 0
static const char *vregs_isa1200_name[] = {
	"gp7",
	"gp10",
};


static const int vregs_isa1200_val[] = {
	1800,
	2600,
};
#endif

/* FUJITSU:2011/05/24 F11EIF del  begin */
#if 0
static struct vreg *vregs_isa1200[ARRAY_SIZE(vregs_isa1200_name)];

static int isa1200_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
		if (!vregs_isa1200[i]) {
			pr_err("%s: vreg_get %s failed (%d)\n",
				__func__, vregs_isa1200_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_isa1200[i]) :
			  vreg_disable(vregs_isa1200[i]);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed (%d)\n",
				__func__, vregs_isa1200_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	return 0;

vreg_fail:
	while (i)
		vreg_disable(vregs_isa1200[--i]);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int i, rc;

	if (enable == true) {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
			vregs_isa1200[i] = vreg_get(NULL,
						vregs_isa1200_name[i]);
			if (IS_ERR(vregs_isa1200[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_isa1200_name[i],
					PTR_ERR(vregs_isa1200[i]));
				rc = PTR_ERR(vregs_isa1200[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_isa1200[i],
					vregs_isa1200_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				goto vreg_get_fail;
			}
		}

		rc = gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO);
			goto vreg_get_fail;
		}

		rc = gpio_request(HAP_LVL_SHFT_MSM_GPIO, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO, rc);
			goto vreg_get_fail;
		}

		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);
	} else {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++)
			vreg_put(vregs_isa1200[i]);

		gpio_free(HAP_LVL_SHFT_MSM_GPIO);
	}

	return 0;
vreg_get_fail:
	while (i)
		vreg_put(vregs_isa1200[--i]);
	return rc;
}
static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	.pwm_ch_id = 1, /*channel id*/
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};
#endif

#if 0
static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};
#endif
/* FUJITSU:2011/05/12 F11EIF del end */

static int kp_flip_mpp_config(void)
{
	return pm8058_mpp_config_digital_in(PM_FLIP_MPP,
		PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DIN_TO_INT);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};

/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
static struct msm_gpio msm_104[] = {
    { GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_104" },
};
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */

static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t usb_hub_gpio_cfg_value = GPIO_CFG(56,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA);
	uint32_t soc_version = 0;

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);

	soc_version = socinfo_get_version();

	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);
	if (machine_is_msm7x30_surf() || machine_is_msm7x30_fluid())
		msm7x30_cfg_smsc911x();
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

/* FUJITSU:2011-05-06    add BT HS start */
#ifdef MSM_UARTDM2_USE
	msm_uart_dm2_pdata.wakeup_irq = gpio_to_irq(51);
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif


/* FUJITSU:2011-05-06    add BT HS end */

	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}
#ifdef CONFIG_USB_ANDROID
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		android_usb_pdata.product_id = 0x9028;
		android_usb_pdata.num_products =
			ARRAY_SIZE(fusion_usb_products);
		android_usb_pdata.products = fusion_usb_products;
	}
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));

/* FUJITSU:2011/07/13  USB start */
#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
/* FUJITSU:2011/07/13  USB end */
	msm7x30_init_mmc();
	msm7x30_init_nand();
	msm_qsd_spi_init();

#ifdef CONFIG_SPI_QSD
	if (machine_is_msm7x30_fluid())
		spi_register_board_info(lcdc_sharp_spi_board_info,
			ARRAY_SIZE(lcdc_sharp_spi_board_info));
	else
		spi_register_board_info(lcdc_toshiba_spi_board_info,
			ARRAY_SIZE(lcdc_toshiba_spi_board_info));


/* FUJITSU:2011/05/12  start */
#ifdef CONFIG_MACH_F11K06
    spi_register_board_info(k06_spi_board_info,
            ARRAY_SIZE(k06_spi_board_info));
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12  end */


#endif


/* FUJITSU:2011-05-10  add I2C setup start */
#ifdef CONFIG_MACH_F11K06
    msm_gpios_enable(msm_104, 1);
    gpio_set_value(104, 1);
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-05-10  add setup end */


	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
    msm_device_i2c_2_init();
	qup_device_i2c_init();
/* FUJITSU:2011_06_04   start */
	pedo_init();
/* FUJITSU:2011_06_04   end */

/* FUJITSU:2011/04/19   add bsp-sensor start */
#if 0
#ifdef CONFIG_MACH_F11K06
	pedo_init();
#endif // CONFIG_MACH_F11K06
#endif
/* FUJITSU:2011/04/19   add bsp-sensor end */

	buses_init();
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	aux_pcm_gpio_init();
#endif

/* FUJITSU:2011-03-17 audio gpio start */
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
/* FUJITSU:2011-03-17 audio gpio end */

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

/* FUJITSU:2011-03-03  touch pad start */
    i2c_register_board_info( 4, k06_cap_info, ARRAY_SIZE(k06_cap_info) );
/* FUJITSU:2011-03-03  touch pad end */

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	bt_power_init();
/* FUJITSU:2011/05/12   begin */
#ifdef CONFIG_MACH_F11K06
	wlan_power_init();
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011/05/12   end */

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

/* FUJITSU:2011/05/24 F11EIF del start */
#if 0
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));
#endif
/* FUJITSU:2011/05/24 F11EIF del end */

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);

/* FUJITSU:2011/05/09  start */
#ifdef CONFIG_MACH_F11K06
    platform_device_register(&keypad_device_f11k06);
#endif
/* FUJITSU:2011/05/09  end */

	pmic8058_leds_init();

/* FUJITSU:2011/05/09  start */
#ifdef CONFIG_MACH_F11K06
    captouch_init();
#endif
/* FUJITSU:2011/05/09  end */

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}

/* FUJITSU:2011_06_04   start */
/* FUJITSU:2011_06_16 del start */
#if 0
    if(system_rev >= 0x0c) {
		i2c_register_board_info(4, compassinfo,ARRAY_SIZE(compassinfo));
	} else {
#endif
/* FUJITSU:2011_06_16 del end */
		i2c_register_board_info(0, compassinfo,ARRAY_SIZE(compassinfo));
/* FUJITSU:2011_06_16 del start */
#if 0
	}
#endif
/* FUJITSU:2011_06_16 del end */
/* FUJITSU:2011_06_04   end */

/* FUJITSU:2011-04-19  VIB start */
#ifdef CONFIG_MACH_F11K06
    msm_init_pmic_vibrator();
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-19  VIB end   */

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static int __init fluid_pmem_adsp_size_setup(char *p)
{
	fluid_pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("fluid_pmem_adsp_size", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;
/*
   Request allocation of Hardware accessible PMEM regions
   at the beginning to make sure they are allocated in EBI-0.
   This will allow 7x30 with two mem banks enter the second
   mem bank into Self-Refresh State during Idle Power Collapse.

    The current HW accessible PMEM regions are
    1. Frame Buffer.
       LCDC HW can access msm_fb_resources during Idle-PC.

    2. Audio
       LPA HW can access android_pmem_audio_pdata during Idle-PC.
*/
	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	if machine_is_msm7x30_fluid()
		size = fluid_pmem_adsp_size;
	else
		size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
}


/* FUJITSU:2011-04-26   USB setting start */
#ifdef CONFIG_MACH_F11K06
int msm_usb_read_nvitem(unsigned int id, unsigned int *data)
{
	int rc;

	if(data == NULL)
		return -1;

	/* read NV*/
	rc = msm_proc_comm(PCOM_OEM_011, &id,(unsigned *)data);
	if(rc)
		printk("USB NV read error %d\n",rc);

	return rc;
}
EXPORT_SYMBOL(msm_usb_read_nvitem);

int msm_usb_write_nvitem(unsigned int id, unsigned int *data)
{
	int rc;
	if( data == NULL )
		return -1;

	/* write VN */
	rc = msm_proc_comm(PCOM_OEM_012, &id, (unsigned *)data);
	if( rc )
		printk("USB NV write error %d %d\n",rc,id);

	return rc;
}
EXPORT_SYMBOL(msm_usb_write_nvitem);
#endif // CONFIG_MACH_F11K06
/* FUJITSU:2011-04-26  USB setting end */

/* FUJITSU:2011-06-06 DEL@proximity s*/
/* FUJITSU:2011-05-09 ADD@proximity s*/
#if 0
#ifdef CONFIG_MACH_F11K06
int msm_prox_write_nvitem(unsigned int id, unsigned short *data)
{
	int rc;
        if( data == NULL )
                return -1;

        /* write VN */
        rc = msm_proc_comm(PCOM_NV_WRITE, &id, (unsigned *)data);
        if( rc )
        {
		printk("NV write error %d %d\n",rc,id);
        }
        return rc;
}
EXPORT_SYMBOL(msm_prox_write_nvitem);

int msm_prox_read_nvitem(unsigned int id)
{
	int rc;
	unsigned int prox_read_data;
        
	/* read NV*/
	rc = msm_proc_comm(PCOM_NV_READ, &id, &prox_read_data);
	if(rc)
	{
		printk("PROX NV read error %d\n",rc);
	    return -1;
	}
    else
    {
        return prox_read_data;
    }
}
EXPORT_SYMBOL(msm_prox_read_nvitem);
#endif // CONFIG_MACH_F11K06
#endif
/* FUJITSU:2011-05-09 ADD@proximity e*/
/* FUJITSU:2011-06-06 DEL@proximity e*/

/* FUJITSU:2011-02-17 add hardware start */
MACHINE_START(F11K06, "F11K06")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* FUJITSU:2011-02-17 add hardware end */

