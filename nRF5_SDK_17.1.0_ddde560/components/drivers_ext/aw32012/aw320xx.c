/* stm32_aw320xx.c
 * Hardware driver for the line-charger
 *
 * Author: Alan <wangshuaijie@awinic.com>
 * Support list:
 *  AW32001CSR
 *  AW32001ACSR
 *  AW32001BCSR
 *  AW32001ECSR
 *  AW32002CSR
 *  AW32002ACSR
 *  AW32012CSR
 */

#include <stdio.h>
//#include "i2c.h"
//#include "usart.h"
#include <nrfx_twi.h>
#include <nrfx_uart.h>
#include "nrf_drv_twi.h"
#include <aw320xx.h>
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define AW320XX_DRIVER_VERSION "V0.5.0"
struct aw320xx g_aw320xx;
extern nrf_drv_twi_t *hi2c1;
extern volatile bool m_xfer_done;
 /******************************************************
 *
 * aw320xx i2c write/read/write_bit
 *
 ******************************************************/
//static HAL_StatusTypeDef aw320xx_i2c_read(struct aw320xx *aw320xx, uint8_t reg_addr, uint8_t *reg_data)
//{
//	unsigned char cnt = 0;
//	uint8_t slave_data[2] = { 0 };
//	HAL_StatusTypeDef status;

//	while (cnt < AW320XX_I2C_RW_RETRIES) {
//		status = HAL_I2C_Mem_Read(aw320xx->hi2cx,
//					  AW320XX_ADDR << 1,
//					  reg_addr,
//					  I2C_MEMADD_SIZE_8BIT,
//					  slave_data,
//					  1,
//					  100);
//		//AWLOGD("%s:read reg:0x%02x = 0x%02x\n",
//			__func__, reg_addr, slave_data[0]);
//		if (status == NRF_SUCCESS) {
//			*reg_data = slave_data[0];
//			return status;
//		} else {
//			cnt++;
//			//AWLOGD("%s:read data fail !!!\n", __func__);
//			HAL_Delay(2);
//		}
//	}

//	return status;
//}

//static HAL_StatusTypeDef aw320xx_i2c_write(struct aw320xx *aw320xx, uint8_t reg_addr, uint8_t reg_data)
//{
//	unsigned char cnt = 0;
//	HAL_StatusTypeDef status;
//	uint8_t slave_data[2] = {0};

//	slave_data[0] = reg_data;

//	while (cnt < AW320XX_I2C_RW_RETRIES) {
//			status = HAL_I2C_Mem_Write(aw320xx->hi2cx,
//						   AW320XX_ADDR<<1,
//						   reg_addr,
//						   I2C_MEMADD_SIZE_8BIT,
//						   slave_data,
//						   1,
//						   100);
//		if (status == NRF_SUCCESS) {
//			//AWLOGD("%s:write  data NRF_SUCCESS\n", __func__);
//			return status;
//		} else if (status == HAL_BUSY) {
//			//AWLOGD("%s:write  data HAL_BUSY\n", __func__);
//			return HAL_BUSY;
//		} else {
//			cnt++;
//			HAL_Delay(2);
//		}
//	}
//	//AWLOGD("%s:write  data HAL_ERROR\n", __func__);
//	return HAL_ERROR;
//}

// I²C 写函数
ret_code_t aw320xx_i2c_write(struct aw320xx *aw320xx, uint8_t reg_addr, uint8_t reg_val)
{
		ret_code_t err_code;
		m_xfer_done = false;
    uint8_t buffer[2] = { reg_addr, reg_val };
    err_code = nrf_drv_twi_tx(hi2c1, AW320XX_ADDR, buffer, sizeof(buffer), false);//（发送STOP信号为false）
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("TX Error: %d", err_code);
    }

    return err_code;
}

ret_code_t aw320xx_i2c_read(struct aw320xx *aw320xx, uint8_t reg_addr, uint8_t *reg_val)
{
    ret_code_t err_code;

	// 发送寄存器地址（不发送STOP信号为true）
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, AW320XX_ADDR, &reg_addr, 1, true);
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("TX Error: %d", err_code);
        return err_code;
    }

    // 读取寄存器值（使用相同设备地址）
		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, AW320XX_ADDR, reg_val, 1);
		while (m_xfer_done == false)
			;
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_INFO("RX Error: %d", err_code);
    }

    return err_code;
}

// I²C 写入指定位
ret_code_t aw320xx_i2c_write_bits(struct aw320xx *aw320xx, uint8_t reg_addr, uint8_t mask, uint8_t reg_data)
{
    ret_code_t err_code;
    uint8_t reg_val;

    // 读取当前寄存器值
    err_code = aw320xx_i2c_read(aw320xx, reg_addr, &reg_val);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // 修改寄存器值
    reg_val &= ~mask;                 // 清除 mask 指定的位
    reg_val |= (reg_data & mask);     // 设置新的值

    // 写入寄存器
    err_code = aw320xx_i2c_write(aw320xx, reg_addr, reg_val);
    return err_code;
}

/*
 * platform stm32
 * function: write register value
*/
#if 0
void aw320xx_test_write_function(struct aw320xx *aw320xx)
{
	uint8_t reg_val = 0;
	uint8_t reg_data = 0;

	aw320xx_i2c_read(aw320xx, AW320XX_REG0_SCR, &reg_val);
	HAL_Delay(5);
	reg_data = 0x7e;
	aw320xx_i2c_write(aw320xx, AW320XX_REG0_SCR, reg_data);
	HAL_Delay(5);
	aw320xx_i2c_read(aw320xx, AW320XX_REG0_SCR, &reg_val);
	HAL_Delay(5);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG0_SCR,
			       AW320XX_BIT_SCR_IIN_LMT_MASK,
			       0x0c);
	HAL_Delay(5);
	aw320xx_i2c_read(aw320xx, AW320XX_REG0_SCR, &reg_val);
	HAL_Delay(5);
}


/*
 * platform stm32
 * function: read register value
*/
void aw320xx_test_read_function(struct aw320xx *aw320xx)
{
	uint8_t reg_val = 0;

	aw320xx_i2c_read(aw320xx, AW320XX_REG0_SCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG1_POCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG2_CCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG3_CCR2, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG4_CVR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG5_TIMCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG6_MCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG7_SVCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG8_STATR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG9_FLTR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGA_ID, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGB_CCR3, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGC_RCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGC_RCR, &reg_val);
	/*HAL_Delay(5);*/
}
#endif

void aw320xx_drump_reg(struct aw320xx *aw320xx)
{
	uint8_t reg_val = 0;

	aw320xx_i2c_read(aw320xx, AW320XX_REG0_SCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG1_POCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG2_CCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG3_CCR2, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG4_CVR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG5_TIMCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG6_MCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG7_SVCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG8_STATR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REG9_FLTR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGA_ID, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGB_CCR3, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGC_RCR, &reg_val);
	aw320xx_i2c_read(aw320xx, AW320XX_REGD_FCR, &reg_val);
	/*HAL_Delay(5);*/
}

void aw320xx_reg_value_init(struct aw320xx *aw320xx)
{
	//AWLOGD("%s\n", __func__);

	if (aw320xx->device_id == AW32012_REV_ID) {
		aw320xx_i2c_write_bits(aw320xx,
				AW320XX_REGD_FCR,
				(uint8_t)AW320XX_BIT_VIN_OVP_MASK,
				aw320xx->vin_ovp);
	}
	if (aw320xx->icc > AW320XX_ICC_MAX_NORMAL) {
		aw320xx_i2c_write_bits(aw320xx,
				      AW320XX_REG0_SCR,
				      AW320XX_BIT_SCR_IIN_LMT_MASK,
				      AW320XX_BIT_SCR_IIN_LMT_500MA);
		aw320xx_i2c_write_bits(aw320xx,
				      AW320XX_REGC_RCR,
				      (uint8_t)AW320XX_BIT_RCR_EN0P55_MASK,
				      AW320XX_BIT_RCR_EN0P55_ENABLE);
	}
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG2_CCR,
			       AW320XX_BIT_CCR_ICC_MASK,
			       aw320xx->icc);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REGB_CCR3,
			       (uint8_t)AW320XX_BIT_CCR3_EN_ICC_DIVD_MASK,
			       aw320xx->en_icc_divd);

	//AWLOGD("aw320xx set ICHG = %dmA!!!!\n", (aw320xx->icc + 1) * 8);

	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG1_POCR,
			       AW320XX_BIT_POCR_VBAT_UVLO_MASK,
			       aw320xx->vbat_uvlo);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG3_CCR2,
			       (uint8_t)AW320XX_BIT_CCR2_IDSCHG_MASK,
			       aw320xx->idschg);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG7_SVCR,
			       AW320XX_BIT_SVCR_TJ_REG_MASK,
			       aw320xx->tj_reg);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG7_SVCR,
			       AW320XX_BIT_SVCR_VSYS_REG_MASK,
			       aw320xx->vsys_reg);

	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG0_SCR,
			       AW320XX_BIT_SCR_IIN_LMT_MASK,
			       aw320xx->iin_lmt);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG0_SCR,
			       (uint8_t)AW320XX_BIT_SCR_VIN_MIN_MASK,
			       aw320xx->vin_dpm);

	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REGC_RCR,
			       AW320XX_BIT_RCR_PRETO_MASK,
			       aw320xx->preto);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG5_TIMCR,
			       AW320XX_BIT_TIMCR_CHG_TMR_MASK,
			       aw320xx->chg_tmr);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REGC_RCR,
			       AW320XX_BIT_RCR_ITERMDEG_MASK,
			       aw320xx->itermdeg);

	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG4_CVR,
			       AW320XX_BIT_CVR_VBAT_PRE_MASK,
			       aw320xx->vbat_pre);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REGB_CCR3,
			       AW320XX_BIT_CCR3_EN_IPRE_SET_MASK,
			       AW320XX_BIT_CCR3_EN_IPRE_SET_ENABLE);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REGB_CCR3,
			       AW320XX_BIT_CCR3_IPRE_MASK,
			       aw320xx->ipre);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG4_CVR,
			       (uint8_t)AW320XX_BIT_CVR_VBAT_REG_MASK,
			       aw320xx->vbat_reg);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG3_CCR2,
			       AW320XX_BIT_CCR2_ITERM_MASK,
			       aw320xx->iterm);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG4_CVR,
			       AW320XX_BIT_CVR_VRECH_MASK,
			       aw320xx->vrech);
}

void aw320xx_reg_data_init(struct aw320xx *aw320xx)
{
	//AWLOGD("%s\n", __func__);
	aw320xx->fault_flag = AW320XX_IRQ_OK;
	aw320xx->vbat_uvlo = PARM_VBAT_UVLO;
	aw320xx->idschg = PARM_IDSCHG;
	aw320xx->vin_ovp =  PARM_VIN_OVP;
	aw320xx->tj_reg = PARM_TJ_REG;
	aw320xx->vsys_reg = PARM_VSYS_REG;

	aw320xx->iin_lmt = PARM_IIN_LMT;
	aw320xx->vin_dpm = PARM_VIN_DPM;

	aw320xx->preto = PARM_PRETO;
	aw320xx->chg_tmr = PARM_CHG_TMR;
	aw320xx->itermdeg = PARM_ITERMDEG;

	aw320xx->vbat_pre = PARM_VBAT_PRE;
	aw320xx->ipre = PARM_IBAT_PRE;
	aw320xx->icc = PARM_ICC;
	aw320xx->vbat_reg = PARM_VBAT_REG;
	aw320xx->iterm = PARM_ITERM;
	aw320xx->vrech = PARM_VRECH;

	aw320xx->en_icc_divd = PARM_EN_ICC_DIVD;
	aw320xx_reg_value_init(aw320xx);
}

/*****************************************************
* check chip id
*****************************************************/
static int32_t aw320xx_read_chipid(struct aw320xx *aw320xx)
{
	uint8_t cnt = 0;
	uint8_t chipid = 0xFF;
	uint8_t val = 0xFF;
	ret_code_t status;

	while (cnt < AW320XX_I2C_RW_RETRIES) {
		status = aw320xx_i2c_read(aw320xx, AW320XX_REGA_ID, &val);
		NRF_LOG_INFO("read status = %d\n",status);
		if (status == NRF_SUCCESS)
			chipid = val;
		if (chipid == AW320XX_CHIP_ID)
			return 0;
		cnt++;
		/* HAL_Delay(10); */
		//AWLOGD("%s [%d]: read chipid error\n", __func__, __LINE__);
		NRF_LOG_INFO("read chipid error. chipid = %d\n",chipid);
	}
	NRF_LOG_INFO("chipid = %d\n",chipid);
	return -AW320XX_CHIPID_FAILD;
}

/*****************************************************
* check device id
*****************************************************/
static int32_t aw320xx_check_device_id(struct aw320xx *aw320xx)
{
	uint8_t cnt = 0;
	uint8_t val = 0xFF;
	ret_code_t status;

	while (cnt < AW320XX_I2C_RW_RETRIES) {
		status = aw320xx_i2c_read(aw320xx, AW320XX_REG8_STATR, &val);
		if (status == NRF_SUCCESS) {
			val &= AW320XX_BIT_REV_ID_MASK;
			val >>= AW320XX_BIT_REV_ID_SHIFT;
			aw320xx->device_id = val;
			if (aw320xx->device_id == AW3200X_REV_ID) {
				//AWLOGD("device is aw3200X\n");
				return 0;
			} else if (aw320xx->device_id == AW32012_REV_ID) {
				//AWLOGD("device is aw32012\n");
				return 0;
			} else
				;//AWLOGD("device is no found!!!!\n");
		}
		cnt++;
		/* HAL_Delay(10); */
		//AWLOGD("%s [%d]; check device_id error\n", __func__, __LINE__);
	}
	return -AW320XX_DEVICE_ID_FAILD;
}

/*
 * @check status
* @retval 1:success  0:fail
*/
static int aw320xx_check_status(struct aw320xx *aw320xx)
{
	uint8_t reg_val = 0;
	unsigned char cnt = 0;
	ret_code_t status;

	while (cnt < 3) {
		status = aw320xx_i2c_read(aw320xx, AW320XX_REG9_FLTR, &reg_val);
		if ((status == NRF_SUCCESS) && ((reg_val & 0x3F) == 0)) {
				//AWLOGD("%s: success!!!\n", __func__);
				return AW320XX_STATUS_IS_OK;
		} else
			//AWLOGD("%s: status failed! REG9:0x%02X\n", __func__, reg_val);
		/* HAL_Delay(2); */
		cnt++;
	}

	return AW320XX_STATUS_IS_ERROR;
}

void aw320xx_charging_enable(struct aw320xx *aw320xx)
{
	//AWLOGD("%s enter\n", __func__);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG1_POCR,
			       AW320XX_BIT_POCR_CEB_MASK,
			       AW320XX_BIT_POCR_CEB_ENABLE);
}

void aw320xx_charging_disable(struct aw320xx *aw320xx)
{
	//AWLOGD("%s enter\n", __func__);
	aw320xx_i2c_write_bits(aw320xx,
			       AW320XX_REG1_POCR,
			       AW320XX_BIT_POCR_CEB_MASK,
			       AW320XX_BIT_POCR_CEB_DISABLE);
}

/*****************************************************
* get charge status
*****************************************************/
static void aw320xx_get_charge_status(struct aw320xx *aw320xx)
{
	uint8_t cnt = 0;
	uint8_t val = 0xFF;
	ret_code_t status;

	while (cnt < AW320XX_I2C_RW_RETRIES) {
		status = aw320xx_i2c_read(aw320xx, AW320XX_REG8_STATR, &val);
		if (status == NRF_SUCCESS) {
			val &= AW320XX_BIT_CHG_STAT_MASK;
			val = (val >> AW320XX_BIT_CHG_STAT_SHIFT);
			aw320xx->chg_stat = val;
			switch (val) {
			case AW320XX_NOT_CHARGING:
				//AWLOGD("chg_stat is not charging\n");
				break;
			case AW320XX_PRE_CHARGE:
				//AWLOGD("chg_stat is pre charge\n");
				break;
			case AW320XX_CHARGE:
				//AWLOGD("chg_stat is charge\n");
				break;
			case AW320XX_CHARGE_DONE:
				//AWLOGD("chg_stat is charge done\n");
				break;
			}
			break;
		} else
			//AWLOGD("get_charge_status failed\n");
		nrf_delay_ms(1);;
		cnt++;
	}
}

static int32_t aw320xx_is_charging_cvst(struct aw320xx *aw320xx)
{
	ret_code_t status;
	uint8_t reg_val = 0;

	//AWLOGD("%s enter\n", __func__);
	status = aw320xx_i2c_read(aw320xx, AW320XX_REG10_START2, &reg_val);
	if(status == NRF_SUCCESS) {
		if (reg_val & AW320XX_BIT_STATR2_CVST_NORMAL) {
			//AWLOGD("AW320XX_IN_CVST\n");
			return AW320XX_IN_CVST;
		} else {
			//AWLOGD("AW320XX_NOT_IN_CVST\n");
			return AW320XX_NOT_IN_CVST;
		}
	} else {
		//AWLOGD("get AW320XX_REG10_START2 failed\n");
		return -AW320XX_CVST_FAILD;
	}
}

static void aw32012_set_detection_vin_sequence_func(struct aw320xx *aw320xx, int enable)
{
	uint8_t reg_val = 0;

	//AWLOGD("%s enter\n", __func__);
	aw320xx_i2c_write(aw320xx, AW32012_REG_WPRT, AW32012_WR_PRO_CLOSE);
	if (enable)
		reg_val = AW32012_BIT_DET_VIN_SEQ_OPEN;
	else
		reg_val = AW32012_BIT_DET_VIN_SEQ_CLOSE;
	aw320xx_i2c_write_bits(aw320xx, AW32012_REG_DUMMY1,
				AW32012_BIT_DET_VIN_SEQ_MASK, reg_val);
	aw320xx_i2c_write(aw320xx, AW32012_REG_WPRT, AW32012_WR_PRO_OPEN);
}
/*
 * Interrupt function
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t reg8_val = 0xFF;
	uint8_t reg9_val = 0xFF;
	ret_code_t status;

	//AWLOGD("%s start, pin:%d\n", __func__, GPIO_Pin);
	switch(GPIO_Pin) {
		case AW320XX_IRQ_PIN:
			aw320xx_get_charge_status(&g_aw320xx);
			status = aw320xx_i2c_read(&g_aw320xx, AW320XX_REG9_FLTR, &reg9_val);
			status |= aw320xx_i2c_read(&g_aw320xx, AW320XX_REG8_STATR, &reg8_val);
			if (status == NRF_SUCCESS) {
				if ((reg9_val & AW320XX_BIT_FLTR_FAULT_MASK) != 0) {
					g_aw320xx.fault_flag = AW320XX_IRQ_ERROR;
					//AWLOGD("%s aw320xx IC :irq error. REG9 = 0x%02X\n",__func__, reg9_val & AW320XX_BIT_FLTR_FAULT_MASK);
				} else {
					//AWLOGD("%s aw320xx IC is normal running\n", __func__);
				}
				if ((reg8_val & AW320XX_BIT_PG_STAT_MASK) ==
						AW320XX_BIT_PG_STAT_POWER_GOOD) {
					g_aw320xx.fault_flag = AW320XX_IRQ_OK;
					aw320xx_charging_enable(&g_aw320xx);
				}
				//AWLOGD("%s. REG8:0x%02X\n", __func__, reg8_val);

			} else {
				//AWLOGD("%s aw320xx read REG9 or REG8 error!\n", __func__);
			}

			//AWLOGD("%s end\n", __func__);
			break;

		default:
			break;
	}
}

/*
 *Before it goes into charge,Check the register 0X09 of BIT5~BIT0.
 *If normal,initializing the register.
 *If the state is abnormal£¬Prohibit charging.
 *
*/

void aw320xx_enter(void)
{
	int ret = -1;
	g_aw320xx.hi2cx = hi2c1;
//aw320xx_i2c_write(&g_aw320xx, AW32012_REG_WPRT, AW32012_WR_PRO_OPEN);
	//AWLOGD("%s\n", __func__);
	if (!aw320xx_read_chipid(&g_aw320xx)) {
		if (!aw320xx_check_device_id(&g_aw320xx)) {
			aw320xx_reg_data_init(&g_aw320xx);
			ret = aw320xx_check_status(&g_aw320xx);
			if (ret == AW320XX_STATUS_IS_OK) {
				aw320xx_charging_enable(&g_aw320xx);
			} else
				aw320xx_charging_disable(&g_aw320xx);
		}
	}
	//aw320xx_reg_data_init(&g_aw320xx);
	//AWLOGD("%s: exit!!!\n", __func__);
	NRF_LOG_INFO("AW.\n");
}

void aw320xx_loop(void)
{
	if (g_aw320xx.fault_flag == AW320XX_IRQ_OK) {
		aw320xx_drump_reg(&g_aw320xx);
		aw320xx_get_charge_status(&g_aw320xx);
		aw320xx_is_charging_cvst(&g_aw320xx);
		nrf_delay_ms(10);
	}
}

/* Entry function */
#ifdef AW320XX
extern void awinic_single_enter(void)
{
	aw320xx_enter();
}

extern void awinic_loop_enter(void)
{
	aw320xx_loop();
}

#endif

