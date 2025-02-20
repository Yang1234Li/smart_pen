#include "tp_ts.h"
#include <nrfx_twi.h>
#include <nrfx_uart.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "app_error.h"

extern nrf_drv_twi_t *hi2c1;
extern volatile bool m_xfer_done;
#define TP_DATA_BYTE         (96)

void ctp_delay_us(unsigned int time)
{
	unsigned int i,j,k;
	for(i=0;i< time;i++)
	{
		for(j=0;j<100;j++)
		{
			k=0;
		}
	}
}

void ctp_delay_ms(unsigned short time)
{
    nrf_delay_ms(time);
}

void bl_ts_set_intmode(char mode)
{
	if(0 == mode)
	{//GPIO mode
		CTP_SET_I2C_EINT_OUTPUT;
	}
	else if(1 == mode)
	{//INT mode
		CTP_SET_I2C_EINT_INPUT;
	}
}

void bl_ts_set_intup(char level)
{
	if(level==1)
		CTP_SET_I2C_EINT_HIGH;
	else if(level==0)
		CTP_SET_I2C_EINT_LOW;
}

#ifdef INT_PIN_WAKEUP
void bl_ts_int_wakeup(void)
{
	bl_ts_set_intmode(0);
	bl_ts_set_intup(1);
	ctp_delay_ms(20);
	bl_ts_set_intup(0);
	ctp_delay_ms(1);
	bl_ts_set_intup(1);
	ctp_delay_ms(20);
	bl_ts_set_intmode(1);
}
#endif

#ifdef  RESET_PIN_WAKEUP
void bl_ts_reset_wakeup(void)
{
	CTP_SET_RESET_PIN_OUTPUT;
	CTP_SET_RESET_PIN_HIGH;
	ctp_delay_ms(20);
	CTP_SET_RESET_PIN_LOW;
	ctp_delay_ms(20);
	CTP_SET_RESET_PIN_HIGH;
	ctp_delay_ms(20);
}
#endif

#ifdef BL_POWER_CONTROL_SUPPORT
void bl_ts_reset_poweroff(void)
{
    //ALL gpio set as output, and out 0
    //GPIO_ModeSetup(CTP_RESET_PIN, 0);
	CTP_SET_RESET_PIN_OUTPUT;
	CTP_SET_RESET_PIN_LOW;
    //GPIO_ModeSetup(CTP_EINT_PIN, 0);
	CTP_SET_I2C_EINT_OUTPUT;
	CTP_SET_I2C_EINT_LOW;
    //GPIO_ModeSetup(CTP_I2C_CLK_PIN, 0);
	CTP_SET_I2C_CLK_OUTPUT;
	CTP_SET_I2C_CLK_LOW;
    //GPIO_ModeSetup(CTP_I2C_DATA_PIN, 0);
	CTP_SET_I2C_DATA_OUTPUT;
	CTP_SET_I2C_DATA_LOW;

	ctp_delay_ms(10);
    //Supply of touch IC is off
    //GPIO_ModeSetup(CTP_LDO_PIN, 0);
	CTP_SET_LDO_OUTPUT;
	CTP_SET_LDO_LOW;	
    ctp_delay_ms(20);
}

void bl_ts_reset_poweron(void)
{
    //ALL gpio set as output, and out 0
    //GPIO_ModeSetup(CTP_RESET_PIN, 0);
	CTP_SET_RESET_PIN_OUTPUT;
	CTP_SET_RESET_PIN_LOW;
    //GPIO_ModeSetup(CTP_EINT_PIN, 0);
	CTP_SET_I2C_EINT_OUTPUT;
	CTP_SET_I2C_EINT_LOW;
    //GPIO_ModeSetup(CTP_I2C_CLK_PIN, 0);
	CTP_SET_I2C_CLK_OUTPUT;
	CTP_SET_I2C_CLK_LOW;
    //GPIO_ModeSetup(CTP_I2C_DATA_PIN, 0);
	CTP_SET_I2C_DATA_OUTPUT;
	CTP_SET_I2C_DATA_LOW;

	ctp_delay_ms(10);
    //Supply of touch IC is on
    //GPIO_ModeSetup(CTP_LDO_PIN, 0);
	CTP_SET_LDO_OUTPUT;
	CTP_SET_LDO_HIGH;	
    ctp_delay_ms(50);
	
    //ALL gpio set as output, and out 1
    //GPIO_ModeSetup(CTP_RESET_PIN, 0);
	CTP_SET_RESET_PIN_OUTPUT;
	CTP_SET_RESET_PIN_HIGH;
    //GPIO_ModeSetup(CTP_EINT_PIN, 0);
	CTP_SET_I2C_EINT_OUTPUT;
	CTP_SET_I2C_EINT_HIGH;
    //GPIO_ModeSetup(CTP_I2C_CLK_PIN, 0);
	CTP_SET_I2C_CLK_OUTPUT;
	CTP_SET_I2C_CLK_HIGH;
    //GPIO_ModeSetup(CTP_I2C_DATA_PIN, 0);
	CTP_SET_I2C_DATA_OUTPUT;
	CTP_SET_I2C_DATA_HIGH;
    ctp_delay_ms(20);
	bl_ts_set_intmode(1);
}

void bl_ts_reset_powerup(void)
{
	bl_ts_reset_poweroff();
    bl_ts_reset_poweron();
}
#endif

#ifdef	CTP_USE_SW_I2C 
void ctp_i2c_init(void)
{
	CTP_SET_I2C_CLK_OUTPUT;
	CTP_SET_I2C_CLK_HIGH;
	CTP_SET_I2C_DATA_OUTPUT;
	CTP_SET_I2C_DATA_HIGH;
	
}

char CTP_I2C_send_byte(char send_byte)
{
	volatile signed char i;
	volatile unsigned int j;
	volatile unsigned int k=0;
	char ack;

	for (i=7;i>=0;i--)
	{	/* data bit 7~0 */
		if (send_byte & (1<<i))
		{
			CTP_SET_I2C_DATA_HIGH;
		}
		else
		{
			CTP_SET_I2C_DATA_LOW;
		}
		for(j=0;j<CTP_I2C_DELAY;j++);
		CTP_SET_I2C_CLK_HIGH;
		for(j=0;j<CTP_I2C_DELAY;j++);
		CTP_SET_I2C_CLK_LOW;
		for(j=0;j<CTP_I2C_DELAY;j++);
	}

	CTP_SET_I2C_DATA_INPUT;
	CTP_SET_I2C_CLK_HIGH;
	for(j=0;j<CTP_I2C_DELAY;j++);

	for(k=0;k<CTP_ACK_COUNTER;k++)
	{
		if(CTP_GET_I2C_DATA_BIT == 0)
		{
			ack=1;
			break;
		}
		else
		{
			ack=0;
		}
	}

	CTP_SET_I2C_CLK_LOW;
	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_DATA_OUTPUT;
	ctp_delay_us(2);
	return ack;
}	

unsigned char CTP_I2C_get_byte_with_ack(unsigned char uc_ack_lvl)
{
	volatile signed char i;
	volatile unsigned int j;
	unsigned char get_byte=0;

	CTP_SET_I2C_DATA_INPUT;
	for(j=0;j<CTP_I2C_DELAY;j++);
	for (i=7;i>=0;i--)
	{       /* data bit 7~0 */
		CTP_SET_I2C_CLK_LOW;
		for(j=0;j<CTP_I2C_DELAY;j++);
		CTP_SET_I2C_CLK_HIGH;
		for(j=0;j<CTP_I2C_DELAY;j++);
		if (CTP_GET_I2C_DATA_BIT)
			get_byte |= (1<<i);
		for(j=0;j<CTP_I2C_DELAY;j++);
	}
	/* don't care bit, 9th bit */

	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_CLK_LOW;
	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_DATA_OUTPUT;

	if (uc_ack_lvl == 1)
	{
		CTP_SET_I2C_DATA_HIGH;
	}
	else
	{
		CTP_SET_I2C_DATA_LOW;
	}

	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_CLK_HIGH;

	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_CLK_LOW;

	for(j=0;j<CTP_I2C_DELAY;j++);
	CTP_SET_I2C_DATA_LOW;
	ctp_delay_us(2);
	return get_byte;
}	

int CTP_FLASH_I2C_WRITE(unsigned char i2c_addr, unsigned char *value, unsigned short len) 
{
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CTP_SLAVE_ADDR, value, len, false);
    while (!m_xfer_done);  // 等待传输完成

    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    return CPS_OK;

/*
	volatile unsigned int j;
	unsigned char get_byte = 0;
	unsigned short i;

	CTP_I2C_START_BIT;       
	for(j=0;j<CTP_I2C_DELAY;j++);

	//if(!CTP_I2C_send_byte(reg<<1))	//FLASH_ADDR + WRITE
	if(!CTP_I2C_send_byte(i2c_addr))	//FLASH_ADDR + WRITE
		return(1);
	for(j=0;j<CTP_I2C_DELAY;j++);

	//for(i = 0; i< len - 1; i++)
	for(i = 0; i< len ; i++)
	{
		if(!CTP_I2C_send_byte(*value++)) //FLASH COMMAND
			return(1);
		for(j=0;j<CTP_I2C_DELAY;j++);
	}

	CTP_I2C_STOP_BIT;
	for(j=0;j<CTP_I2C_DELAY;j++);

	return 0;*/
}

int CTP_FLASH_I2C_READ(unsigned char i2c_addr, unsigned char *value, unsigned short len) 
{
    ret_code_t err_code;

    // 读取寄存器值
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, CTP_SLAVE_ADDR, value, len);
    while (m_xfer_done == false);
    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;
    return CPS_OK;
/*
	volatile unsigned int j;
	unsigned char get_byte = 0;
	unsigned short i;

	CTP_I2C_START_BIT;       
	//if(!CTP_I2C_send_byte( (reg<<1) | 1))	// FLASH READ
	if(!CTP_I2C_send_byte( i2c_addr | 0x01) )	// FLASH READ
		return(1);

	if (len <= 1)
	{
		//*value =CTP_I2C_get_byte();
		*value =CTP_I2C_get_byte_with_ack(1);
		for(j=0;j<CTP_I2C_DELAY;j++);
	}
	else
	{
		for (i = 0; i< len - 1; i++)
		{
			*value++ =CTP_I2C_get_byte_with_ack(0);
		}
		*value =CTP_I2C_get_byte_with_ack(1);
	}
	CTP_I2C_STOP_BIT;
	return 0;*/
}
#endif

#ifdef CTP_USE_HW_I2C
void ctp_i2c_init(void)
{
    //i2c_initialize(I2C_MASTER_CI2C1);
}

int CTP_FLASH_I2C_READ(unsigned char i2c_addr, unsigned char *value, unsigned short len) 
{
    ret_code_t err_code;

    // 读取寄存器值
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, CTP_SLAVE_ADDR, value, len);
    while (m_xfer_done == false);
    if (err_code != NRF_SUCCESS)
        return err_code;
    return err_code;

	//unsigned char regAddr = 0xe7;
	
	//return IxI_I2cRead_u8(value, len, i2c_addr, regAddr);
	
}

int CTP_FLASH_I2C_WRITE(unsigned char i2c_addr, unsigned char *value, unsigned short len) 
{
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CTP_SLAVE_ADDR, value, len, false);
    while (!m_xfer_done);  // 等待传输完成

    if (err_code != NRF_SUCCESS)
        return err_code;

    return err_code;

    //unsigned char regAddr = value[0];
	//value++;
	//return IxI_I2cWrite_u8(value, len - 1, i2c_addr, regAddr);
}
#endif

void bl6133_hisr(void)
{
    unsigned char buf[TP_DATA_BYTE];
    unsigned char readPointCmd = TD_STAT_ADDR;

    CTP_FLASH_I2C_WRITE(CTP_SLAVE_ADDR,&readPointCmd, 1);
    CTP_FLASH_I2C_READ(CTP_SLAVE_ADDR, buf, sizeof(buf));
	#ifdef BTL_FACTORY_TEST_SUPPORT
    if(intTestMode) {
        intFlag++;
    }
    #endif

    return;
}

void ctp_enter_sleep(void)
{
	unsigned char sleepCmd[2] = {0xa5,0x03};
	
	CTP_FLASH_I2C_WRITE(CTP_SLAVE_ADDR,sleepCmd, sizeof(sleepCmd));
	ctp_delay_ms(100);
	bl_log_trace( "ctp_enter_sleep");
}

void ctp_exit_sleep(void)
{
	#ifdef  RESET_PIN_WAKEUP
	bl_ts_reset_wakeup();
	#endif
	#ifdef INT_PIN_WAKEUP
	bl_ts_int_wakeup();
	#endif
	
	bl_log_trace( "ctp_exit_sleep");
}

int ctp_bl_ts_init(void)
{
	char  ret = 0;	
    #ifdef BTL_CHECK_CHIPID
	unsigned char chipID = 0x00;
	int i = 0;
	int retry = 5;
	#endif
	#ifdef  RESET_PIN_WAKEUP
	bl_ts_reset_wakeup();
	#endif
	#ifdef INT_PIN_WAKEUP
	bl_ts_int_wakeup();
	#endif
		
	ctp_i2c_init();
	MDELAY(20);
	
    #ifdef  BTL_CHECK_CHIPID
	for(i = 0; i < retry; i++)
    {
        SET_WAKEUP_LOW;
        ret = bl_get_chip_sub_id(&chipID);
        SET_WAKEUP_HIGH;
        if((ret < 0)|| (chipID != 0x66))
        {
            bl_log_trace("ctp_bl_ts_init:Read chipID Fail:chipID = %x;;retry = %d\n",chipID, retry);
        }
        else
        {
            bl_log_trace("ctp_bl_ts_init:Read chipID succes:chipID = %x\n",chipID); 
            break;
        }
    }
    #endif
	
	#ifdef  BL_AUTO_UPDATE_FARMWARE	

	bl_auto_update_fw();
	ctp_delay_ms(10);
	#endif

	return !ret;
}
