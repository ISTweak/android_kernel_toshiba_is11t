/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#ifndef PROX_I2C_GPIO_H
#define PROX_I2C_GPIO_H

/* I2C Control Informatino Sturcture */
typedef struct{
    uint8_t     slave_addr;     /* slave address                      */
    uint8_t     reg;            /* command register                   */
    uint8_t     *data;          /* pointer to buffer in caller space  */
    uint16_t    len;            /* count of bytes to transfer         */
} PROX_I2C_INFO;

/* prototypes                       */
#ifdef __cplusplus
extern "C" {
#endif

extern bool prox_i2c_init(void);
extern bool prox_i2c_read( PROX_I2C_INFO *cmd_ptr );
extern bool prox_i2c_write( PROX_I2C_INFO *cmd_ptr );

#ifdef __cplusplus
}
#endif


#endif /* PROX_I2C_GPIO_H */
