#ifndef _BL_TS_H_ 
#define _BL_TS_H_
#include "bl_chip_common.h"
#include "fw_update.h"
#include <nrfx_gpiote.h>
#if defined(BTL_FACTORY_TEST_SUPPORT)
#include "bl_factory.h"
#endif
void ctp_delay_ms(unsigned short time);

#define   MDELAY(n)	    ctp_delay_ms(n)	
#define   UDELAY(n)	    ctp_delay_us(n)


#ifdef   BL_POWER_CONTROL_SUPPORT
#define	CTP_LDO_PIN		GPIO10
#define CTP_SET_LDO_OUTPUT	    gpio_set_direction(CTP_LDO_PIN,GPIO_OUT)
#define CTP_SET_LDO_HIGH		GPIO_WriteIO(CTP_LDO_PIN,1)
#define CTP_SET_LDO_LOW		    GPIO_WriteIO(CTP_LDO_PIN,0)
#endif

#ifdef RESET_PIN_WAKEUP
#define	CTP_RESET_PIN							25
#define CTP_SET_RESET_PIN_OUTPUT	nrf_gpio_cfg_output(CTP_RESET_PIN)
#define CTP_SET_RESET_PIN_HIGH		nrf_gpio_pin_set(CTP_RESET_PIN)
#define CTP_SET_RESET_PIN_LOW			nrf_gpio_pin_clear(CTP_RESET_PIN)
#endif

#ifdef   GPIO_EINT
#define  CTP_EINT_PIN							24
#define CTP_SET_I2C_EINT_OUTPUT		nrf_gpio_cfg_output(CTP_EINT_PIN)
#define CTP_SET_I2C_EINT_INPUT		nrf_gpio_cfg_input(CTP_EINT_PIN,NRF_GPIO_PIN_NOPULL)
#define CTP_SET_I2C_EINT_HIGH			nrf_gpio_pin_set(CTP_EINT_PIN)
#define CTP_SET_I2C_EINT_LOW			nrf_gpio_pin_clear(CTP_EINT_PIN)	
#endif

#ifdef   CTP_USE_SW_I2C
//#define  CTP_I2C_DELAY			50
//#define CTP_I2C_DATA_PIN		GPIO11 
//#define CTP_I2C_CLK_PIN			GPIO10 

//#define CTP_WRITE		CTP_SLAVE_ADDR
//#define CTP_READ		(CTP_SLAVE_ADDR+1)
//#define CTP_ACK_COUNTER	10

//#define CTP_SET_I2C_CLK_OUTPUT		gpio_set_direction(CTP_I2C_CLK_PIN,GPIO_OUT)
//#define CTP_SET_I2C_CLK_HIGH		gpio_output_set(CTP_I2C_CLK_PIN,1)
//#define CTP_SET_I2C_CLK_LOW			gpio_output_set(CTP_I2C_CLK_PIN,0)

//#define CTP_SET_I2C_DATA_OUTPUT		gpio_set_direction(CTP_I2C_DATA_PIN,GPIO_OUT)
//#define CTP_SET_I2C_DATA_INPUT		gpio_set_direction(CTP_I2C_DATA_PIN,GPIO_IN)
//#define CTP_SET_I2C_DATA_HIGH		gpio_output_set(CTP_I2C_DATA_PIN,1)
//#define CTP_SET_I2C_DATA_LOW		gpio_output_set(CTP_I2C_DATA_PIN,0)
//#define CTP_GET_I2C_DATA_BIT		gpio_input_get(CTP_I2C_DATA_PIN)

//#define CTP_I2C_START_BIT \
//	{ \
//		volatile unsigned int j; \
//		CTP_SET_I2C_CLK_OUTPUT; \
//		CTP_SET_I2C_DATA_OUTPUT; \
//		CTP_SET_I2C_CLK_HIGH; \
//		CTP_SET_I2C_DATA_HIGH; \
//		for(j=0;j<CTP_I2C_DELAY;j++);\
//		CTP_SET_I2C_DATA_LOW; \
//		for(j=0;j<CTP_I2C_DELAY;j++);\
//		CTP_SET_I2C_CLK_LOW; \
//	}

//	#define CTP_I2C_STOP_BIT \
//	{ \
//		volatile unsigned int j; \
//		CTP_SET_I2C_CLK_OUTPUT; \
//		CTP_SET_I2C_DATA_OUTPUT; \
//		CTP_SET_I2C_CLK_LOW; \
//		CTP_SET_I2C_DATA_LOW; \
//		for(j=0;j<CTP_I2C_DELAY;j++);\
//		CTP_SET_I2C_CLK_HIGH; \
//		for(j=0;j<CTP_I2C_DELAY;j++);\
//		CTP_SET_I2C_DATA_HIGH; \
//		for(j=0;j<CTP_I2C_DELAY;j++);\
//	}
#endif
int bl_get_chip_sub_id(unsigned char *buf);
int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len,unsigned char rw);
int bl_read_fw(unsigned char i2c_addr,unsigned char reg_addr, unsigned char *buf, int len);
extern int CTP_FLASH_I2C_WRITE(unsigned char i2c_addr, unsigned char *value, unsigned short len);
extern int CTP_FLASH_I2C_READ(unsigned char i2c_addr, unsigned char *value, unsigned short len);
extern int ctp_bl_ts_init(void);
void bl6133_hisr(void);
void ctp_enter_sleep(void);
void ctp_exit_sleep(void);
#if((UPDATE_MODE==I2C_UPDATE_MODE_NEW)||(UPDATE_MODE==I2C_UPDATE_MODE_OLD))
void bl_enter_update_with_i2c(void);
void bl_exit_update_with_i2c(void);
#endif
#if(UPDATE_MODE==INT_UPDATE_MODE)
void bl_enter_update_with_int(void);
void bl_exit_update_with_int(void);
#endif
#ifdef  RESET_PIN_WAKEUP
void bl_ts_reset_wakeup(void);
#endif

#if((UPDATE_MODE==I2C_UPDATE_MODE_NEW)||(UPDATE_MODE==I2C_UPDATE_MODE_OLD))
#define   SET_WAKEUP_HIGH    bl_exit_update_with_i2c()	 
#define   SET_WAKEUP_LOW	 bl_enter_update_with_i2c()	 
#endif

#if(UPDATE_MODE==INT_UPDATE_MODE)
#define   SET_WAKEUP_HIGH    bl_exit_update_with_int()	 
#define   SET_WAKEUP_LOW	 bl_enter_update_with_int()	 
#endif
#endif
