/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#ifndef SENSOR_LIGHT_TEST_IF_H
#define SENSOR_LIGHT_TEST_IF_H

#define	SENSOR_LIGHT_IOCTL_BASE	'T'

#define SENSOR_LIGHT_TESTMODE_START        _IO(SENSOR_LIGHT_IOCTL_BASE, 1)
#define SENSOR_LIGHT_TESTMODE_END          _IO(SENSOR_LIGHT_IOCTL_BASE, 2)
#define SENSOR_LIGHT_TESTMODE_MSM_ADC      _IOR(SENSOR_LIGHT_IOCTL_BASE, 3, int)
#define SENSOR_LIGHT_TESTMODE_LED_DRV_ADC  _IO(SENSOR_LIGHT_IOCTL_BASE, 4)
#define SENSOR_LIGHT_TESTMODE_END_2        _IO(SENSOR_LIGHT_IOCTL_BASE, 5)
#define SENSOR_LIGHT_TESTMODE_SET_LCD_BL   _IO(SENSOR_LIGHT_IOCTL_BASE, 6)
#define SENSOR_LIGHT_TESTMODE_SET_1K       _IO(SENSOR_LIGHT_IOCTL_BASE, 7)
#define SENSOR_LIGHT_TESTMODE_SET_7K       _IO(SENSOR_LIGHT_IOCTL_BASE, 8)

#endif /* SENSOR_LIGHT_TEST_IF_H */
