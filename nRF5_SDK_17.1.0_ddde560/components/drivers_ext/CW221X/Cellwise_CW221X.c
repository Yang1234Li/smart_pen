
#include "Cellwise_CW221X.h"
#include <nrfx_twi.h>
#include <nrfx_uart.h>
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_GPIO_CONFIG         0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C
#define IC_TEMP_READY           0x08
#define IC_VOL_CUR_READY        0x04

#define SIZE_OF_PROFILE         80

#define USER_RSENSE (10 * 1000)  //rsense * 1000

#define INIT_TEST_TIME      50 /*must >= 50 , can not modify */

#define CW221X_ERROR_IIC      -1
#define CW221X_ERROR_CHIP_ID  -2
#define CW221X_ERROR_TIME_OUT -3
#define CW221X_NOT_ACTIVE          1
#define CW221X_PROFILE_NOT_READY   2
#define CW221X_PROFILE_NEED_UPDATE 3

#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */
#define CW_SLEEP_COUNTS         50

#define CW2215_MARK             0x80
#define CW2217_MARK             0x40
#define CW2218_MARK             0x00

#define CW221X_ADDR			(0xC8U >> 1)//(0x64)

extern nrf_drv_twi_t *hi2c1;
extern volatile bool m_xfer_done;

/**************************global**********************************/
static unsigned char config_profile_info[SIZE_OF_PROFILE] = {
	0x5A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0xB9,0xB6,0xC2,0xBD,0xC4,0xC1,0x95,0x5B,
	0x2C,0xFF,0xFF,0xE1,0xBF,0x7F,0x6A,0x5B,
	0x51,0x4C,0x46,0x84,0xC2,0xD9,0x9E,0xD5,
	0xCF,0xCE,0xCD,0xCA,0xC9,0xB1,0xE0,0xAE,
	0xBD,0xC6,0xAB,0x98,0x8C,0x83,0x7C,0x6C,
	0x63,0x65,0x80,0x91,0xA2,0x73,0x61,0x53,
	0x00,0x00,0x57,0x10,0x00,0x40,0xF6,0x00,
	0x00,0x00,0x64,0x1F,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF4,
};

int cw_read(unsigned char PointReg,unsigned char *pData);
int cw_write(unsigned char PointReg,unsigned char *pData);
int cw_read_nbyte(unsigned char point_reg,unsigned char *r_pdata, unsigned char len);

int cw_write(unsigned char PointReg,unsigned char *pData)
{
		ret_code_t err_code;
		m_xfer_done = false;
    uint8_t buffer[2] = { PointReg, *pData };
    err_code = nrf_drv_twi_tx(hi2c1, CW221X_ADDR, buffer, sizeof(buffer), false);
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("TX Error: %d", err_code);
    }

    return err_code;
}

int cw_read(unsigned char PointReg,unsigned char *pData)
{
    ret_code_t err_code;

		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CW221X_ADDR, &PointReg, 1, true);
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("TX Error: %d", err_code);
        return err_code;
    }

		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, CW221X_ADDR, pData, 1);
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("RX Error: %d", err_code);
    }

    return err_code;
}

int cw_read_nbyte(unsigned char point_reg,unsigned char *r_pdata, unsigned char len)
{
    ret_code_t err_code;
		
		for(int i = 0; i < len; i++)
		{
			err_code = cw_read(point_reg, r_pdata-i);
			if (err_code != NRF_SUCCESS) {
					return err_code;
			}
		}

    return err_code;
}

void cw_delay10us(unsigned int us)
{
	unsigned char a, b;
	unsigned int i;
	for(i = 0; i < us; i++){
		for(b=1; b>0; b--)
		{
			for(a=2; a>0; a--);
		}
	}
}

int cw221x_read_word(unsigned char point_reg, unsigned int *r_pdata)
{
	int ret = 0;
	unsigned char reg_val[2] = {0 , 0};
	unsigned int temp_val_buff = 0;
	unsigned int temp_val_second = 0;
	
	ret = cw_read_nbyte(point_reg, reg_val, 2);
	if(ret)
		return CW221X_ERROR_IIC;
	temp_val_buff = (reg_val[0] << 8) + reg_val[1];
	cw_delay10us(100 * 4); /*sleep  >= 4 ms must*/
	ret = cw_read_nbyte(point_reg, reg_val, 2);
	if(ret)
		return CW221X_ERROR_IIC;
	temp_val_second = (reg_val[0] << 8) + reg_val[1];
	
	if (temp_val_buff != temp_val_second){
		cw_delay10us(100 * 4); /*sleep  >= 4 ms must*/
		ret = cw_read_nbyte(point_reg, reg_val, 2);
		if(ret)
			return CW221X_ERROR_IIC;
		temp_val_buff = (reg_val[0] << 8) + reg_val[1];
	}
	
	*r_pdata = temp_val_buff;
	return 0;
}

int cw221x_get_chip_id(int *chip_id)
{
	int ret;
	unsigned char reg_val;
	
	ret = cw_read(REG_CHIP_ID, &reg_val);
	if(ret)
		return CW221X_ERROR_IIC;
	*chip_id = reg_val;
	return 0;
}

static int cw221x_get_state(void)
{
	int ret;
	unsigned char reg_val;
	int i;
	
	ret = cw_read(REG_MODE_CONFIG, &reg_val);
	if(ret)
		return CW221X_ERROR_IIC;
	if (reg_val != CONFIG_MODE_ACTIVE)
		return CW221X_NOT_ACTIVE;
	
	ret = cw_read(REG_SOC_ALERT, &reg_val);
	if (ret)
		return CW221X_ERROR_IIC;
	if (0x00 == (reg_val & CONFIG_UPDATE_FLG))
		return CW221X_PROFILE_NOT_READY;
	
	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_read((REG_BAT_PROFILE + i), &reg_val);
		if (ret)
			return CW221X_ERROR_IIC;
		if (config_profile_info[i] != reg_val)
			break;
	}
	if ( i != SIZE_OF_PROFILE)
		return CW221X_PROFILE_NEED_UPDATE;
	
	return 0;
}

static int cw221x_write_profile(unsigned char buf[])
{
	int ret;
	int i;

	for (i = 0; i < SIZE_OF_PROFILE; i++) {
		ret = cw_write(REG_BAT_PROFILE + i, &buf[i]);
		if (ret)
			return CW221X_ERROR_IIC;
	}

	return 0;
}

int cw221x_write_temperature(int temperature)
{
	int ret;
	unsigned char A0_value;
	unsigned char A1_value;
	
	if(temperature < -40 || temperature > 87){
		return -1; //write error temperature
	}
	A0_value = (temperature + 40) * 2;
	ret = cw_read(REG_T_HOST_L, &A1_value);
	if (ret)
		return CW221X_ERROR_IIC;
	A1_value = ~A1_value;
	
	ret = cw_write(REG_T_HOST_H, &A0_value);
	if (ret)
		return CW221X_ERROR_IIC;
	ret = cw_write(REG_T_HOST_L, &A1_value);
	if (ret)
		return CW221X_ERROR_IIC;
	return 0;
}

int cw221x_sleep(void)
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	ret = cw_write(REG_MODE_CONFIG, &reg_val);
	if (ret)
		return CW221X_ERROR_IIC;
	cw_delay10us(2 * 1000); /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_SLEEP;
	ret = cw_write(REG_MODE_CONFIG, &reg_val);
	if (ret)
		return CW221X_ERROR_IIC;
	cw_delay10us(1 * 1000); 

	return 0;
}

static int cw221x_active()
{
	int ret;
	unsigned char reg_val = CONFIG_MODE_RESTART;

	ret = cw_write(REG_MODE_CONFIG, &reg_val);
	if (ret)
		return CW221X_ERROR_IIC;
	cw_delay10us(2 * 1000); /* Here delay must >= 20 ms */

	reg_val = CONFIG_MODE_ACTIVE;
	ret = cw_write(REG_MODE_CONFIG, &reg_val);
	if (ret < 0)
		return CW221X_ERROR_IIC;
	cw_delay10us(1 * 1000); 

	return 0;
}

/*CW221X update profile function, Often called during initialization*/
static int cw221x_config_start_ic()
{
	int ret;
	unsigned char reg_val;
	int count = 0;

	ret = cw221x_sleep();
	if (ret < 0)
		return ret;

	/* update new battery info */
	ret = cw221x_write_profile(config_profile_info);
	if (ret < 0)
		return ret;

	/* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
	reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;   
	ret = cw_write(REG_SOC_ALERT, &reg_val);
	if (ret)
		return CW221X_ERROR_IIC;

	/*close all interruptes*/
	reg_val = 0; 
	ret = cw_write(REG_GPIO_CONFIG, &reg_val); 
	if (ret)
		return CW221X_ERROR_IIC;

	ret = cw221x_active();
	if (ret < 0) 
		return ret;
		
	while(1){
		cw_delay10us(10 * 1000);
		ret = cw_read(REG_IC_STATE, &reg_val);
		if (IC_READY_MARK == (reg_val & IC_READY_MARK))
			break;
		count++;
		if (count >= CW_SLEEP_COUNTS) {
			cw221x_sleep();
			return -1;
		}
	}
	
	return 0;
}

static int cw221x_init(void)
{
	int ret;
	int chip_id;

	ret = cw221x_get_chip_id(&chip_id);
	if(ret){
		return ret;
	}
	if(chip_id != IC_VCHIP_ID){
		return CW221X_ERROR_CHIP_ID;
	}

	ret = cw221x_get_state();
	if (ret < 0)
		return ret;

	if (ret != 0) {
		ret = cw221x_config_start_ic();
		if (ret < 0)
			return ret;
	}

	return 0;
}

int cw221x_get_vol(unsigned int *lp_vol)
{
	int ret = 0;
	unsigned int temp_val_buff = 0;
	unsigned int ad_value = 0;

	ret = cw221x_read_word(REG_VCELL_H, &temp_val_buff);
	if(ret)
		return CW221X_ERROR_IIC;
	
	ad_value = temp_val_buff * 5 / 16;
	*lp_vol = ad_value;
	
	return 0; 
}

#define UI_FULL     100
int cw221x_get_capacity(int *lp_uisoc)
{
	int ret = 0;
	unsigned int temp_val_buff = 0;
	int soc = 0;
	int soc_decimal = 0;
//	int remainder = 0;
	unsigned int UI_SOC = 0;

	ret = cw221x_read_word(REG_SOC_INT, &temp_val_buff);
	if(ret)
		return CW221X_ERROR_IIC;
	
	soc = temp_val_buff >> 8;
	soc_decimal = temp_val_buff & 0xFF;

 	UI_SOC = (((unsigned long)soc * 256 + soc_decimal) * 100)/ (UI_FULL * 256);
//	remainder = ((((unsigned long)soc * 256 + soc_decimal) * 100 * 100) / (UI_FULL * 256)) % 100;
	
	if(UI_SOC >= 100){
		UI_SOC = 100;
	}
	*lp_uisoc = UI_SOC;
	
	return 0;
}

int cw221x_get_temp(int *lp_temp)
{
	int ret = 0;
	unsigned char reg_val = 0;
	int temp = 0;
	
	ret = cw_read(REG_TEMP, &reg_val);
	if(ret)
		return CW221X_ERROR_IIC;

	temp = (int)reg_val * 10 / 2 - 400;
	*lp_temp = temp;
	
	return 0;
}

long get_complement_code(unsigned short raw_code)
{
	long complement_code = 0;
	int dir = 0;

	if (0 != (raw_code & 0x8000)){
		dir = -1;
		raw_code =  (0XFFFF - raw_code) + 1;
	}
	else{
		dir = 1;
	}

	complement_code = (long)raw_code * dir;

	return complement_code;
}

int cw221x_get_current(long *lp_current)
{
	int ret = 0;
	unsigned int temp_val_buff = 0;
	long cw_current = 0;
	unsigned char fw_version = 0;
	
	ret = cw_read(REG_FW_VERSION, &fw_version);
	if(ret)
		return CW221X_ERROR_IIC;
	
	ret = cw221x_read_word(REG_CURRENT_H, &temp_val_buff);
	if(ret)
		return CW221X_ERROR_IIC;

	cw_current = get_complement_code(temp_val_buff);
	
	if(((fw_version & CW2215_MARK) != 0) || ((fw_version & CW2217_MARK) != 0)){
		cw_current = cw_current * 1600 / USER_RSENSE;
	}else if((fw_version != 0) && ((fw_version & 0xC0) == CW2218_MARK)){
		cw_current = cw_current * 3815 / USER_RSENSE;
	}else{
		cw_current = 123456;
		/*Error read FW Version*/
	}

	*lp_current = cw_current;
	return 0;
}

int cw221x_get_cycle_count(int *lp_count)
{
	int ret = 0;
	unsigned int temp_val_buff = 0;
	int cycle_buff = 0;

	ret = cw221x_read_word(REG_CYCLE_H, &temp_val_buff);
	if(ret)
		return CW221X_ERROR_IIC;

	cycle_buff = temp_val_buff/16;
	*lp_count = cycle_buff;
	return 0;
}

int cw221x_get_soh(int *lp_soh)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned char SOH = 0;
	
	ret = cw_read(REG_SOH, &reg_val);
	if(ret)
		return CW221X_ERROR_IIC;

	SOH = reg_val;
	*lp_soh = SOH;
	
	return 0;
}

int cw221x_dump_register(void)
{
	int ret = 0;
	unsigned char reg_val = 0;
	int i = 0;

	for(i = 0; i <= 0xF; i++){
		ret = cw_read(i, &reg_val);
		/*Please add print if use*/
		/*printf("0x%2x = 0x%2x\n", i, (int)reg_val);*/
	}
	for(i = 0xA0; i <= 0xAB; i++){
		ret = cw_read(i, &reg_val);
		/*Please add print if use*/
		/*printf("0x%2x = 0x%2x\n", i, (int)reg_val);*/
	}
	return ret;
}

int cw221x_bat_init(void)
{
	int ret;
	int loop = 0;
	
	ret = cw221x_init();
	while((loop++ < 3) && (ret != 0))
	{
		ret = cw221x_init();
	}
	
	return ret;
}

