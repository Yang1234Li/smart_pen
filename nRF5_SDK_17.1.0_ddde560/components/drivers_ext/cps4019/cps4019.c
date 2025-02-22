#include "cps4019.h"
#include <nrfx_twi.h>
#include <nrfx_uart.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static cps_config_t m_config;
static cps_charger_data_t m_current_data;
static volatile bool m_initialized = false;
extern nrf_drv_twi_t *hi2c1;
extern volatile bool m_xfer_done;

/* cps_wls_driver.c - 新增固件升级实现 */
// 状态变量
//static uint8_t m_fw_buffer[CPS_FW_PAGE_SIZE];

// 示例：固件数据作为常量数组存储
unsigned char BOOTLOADER_DATA[CPS_BOOTLOADER_SIZE] = {
    // CPS4019_BL_00_3A_V0.2_CRC5B31
		0x02, 0x07, 0xD6, 0xEE, 0x30, 0xE7, 0x07, 0xC3,
};

unsigned char FIRMWARE_DATA[CPS_FW_MAX_SIZE] = {
     // CPS4019_00_3A_V0.3_CRC0E1A
    0x02, 0x07, 0xD6, 0xEE, 0x30, 0xE7, 0x07, 0xC3,
};

// TWI回调
//static void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
//{
//	if (p_event->type == NRF_DRV_TWI_EVT_DONE)
//	{
//			m_transfer_done = true;
//	}
//}

// GPIO中断处理
//static void gpio_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//    if (pin == CPS_INT_PIN)
//    {
//        cps_charger_data_t data;
//        if (cps_poll_data(&data) == CPS_OK && m_config.event_handler)
//        {
//            m_config.event_handler(&data);
//        }
//    }
//}

cps_err_t cps_init(void)
{
//    if (config == NULL || config->event_handler == NULL)
//    {
//        return CPS_ERR_INVALID_PARAM;
//    }

//    memcpy(&m_config, config, sizeof(cps_config_t));

//    // 初始化TWI
//    const nrf_drv_twi_config_t twi_config = {
//        .scl = m_config.twi_instance.pin_scl,
//        .sda = m_config.twi_instance.pin_sda,
//        .frequency = NRF_DRV_TWI_FREQ_400K,
//        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//        .clear_bus_init = false};

//    ret_code_t err_code = nrf_drv_twi_init(&m_config.twi_instance, &twi_config, twi_handler, NULL);
//    if (err_code != NRF_SUCCESS)
//    {
//        return CPS_ERR_COMM;
//    }
//    nrf_drv_twi_enable(&m_config.twi_instance);

//    // 初始化GPIO中断
//    if (!nrfx_gpiote_is_init())
//    {
//        nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
//    }

//    nrfx_gpiote_in_config_t int_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
//    int_cfg.pull = NRF_GPIO_PIN_PULLUP;

//    err_code = nrfx_gpiote_in_init(CPS_INT_PIN, &int_cfg, gpio_handler);
//    if (err_code != NRFX_SUCCESS)
//    {
//        return CPS_ERR_COMM;
//    }
//    nrfx_gpiote_in_event_enable(CPS_INT_PIN, true);

//    ret_code_t err_code;
//    uint8_t reg_addr;  // 确保寄存器地址格式正确

//    // 发送寄存器地址，不发送 STOP
//    m_xfer_done = false;
//    err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, &reg_addr, 1, false);
//    while (m_xfer_done == false);  // 等待传输完成
//    if (err_code != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    // 读取寄存器值
//    m_xfer_done = false;
//    err_code = nrf_drv_twi_rx(hi2c1, CPS_I2C_ADDRESS, data, len);
//    while (m_xfer_done == false);
//    if (err_code != NRF_SUCCESS)
//        return CPS_ERR_COMM;

    // 验证通信
    uint16_t chip_id;
    if (cps_get_chip_info(&chip_id, NULL, NULL) != CPS_OK)
    {
        return CPS_ERR_COMM;
    }

    m_initialized = true;
		
//        cps_charger_data_t data;
//        cps_poll_data(&data);
    return CPS_OK;
}

static cps_err_t read_register(cps_registers_t reg, uint8_t *data, uint8_t len)
{
//    if (!m_initialized)
//        return CPS_ERR_NOT_INIT;

//    uint8_t reg_addr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};

//    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TXRX(
//        CPS_I2C_ADDRESS, reg_addr, 2, data, len);

//    ret_code_t err_code = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (err_code != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    return CPS_OK;
	
//    if (!m_initialized)
//        return CPS_ERR_NOT_INIT;
    ret_code_t err_code;
    uint8_t reg_addr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};  // 确保寄存器地址格式正确

    // 发送寄存器地址，不发送 STOP
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, reg_addr, sizeof(reg_addr), false);
    while (m_xfer_done == false);  // 等待传输完成
    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    // 读取寄存器值
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, CPS_I2C_ADDRESS, data, len);
    while (m_xfer_done == false);
    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;
    return CPS_OK;
}

static cps_err_t write_register(cps_registers_t reg, const uint8_t *data, uint8_t len)
{
//    if (!m_initialized)
//        return CPS_ERR_NOT_INIT;

//    uint8_t buffer[len + 2];
//    buffer[0] = (uint8_t)(reg >> 8);
//    buffer[1] = (uint8_t)reg;
//    memcpy(buffer + 2, data, len);

//    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(
//        CPS_I2C_ADDRESS, buffer, len + 2);

//    ret_code_t err_code = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (err_code != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    return CPS_OK;
	
//    if (!m_initialized)
//        return CPS_ERR_NOT_INIT;

    ret_code_t err_code;
    uint8_t buffer[len + 1];

    // 只保留低8位寄存器地址
    buffer[0] = (uint8_t)reg;
    memcpy(buffer + 1, data, len);

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, buffer, len + 1, false);
    while (!m_xfer_done);  // 等待传输完成

    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    return CPS_OK;
}

// 32位寄存器写操作
cps_err_t cps_write_32(uint32_t reg_addr, uint32_t value)
{
    uint8_t buffer[8] = {
        (reg_addr >> 24) & 0xFF,
        (reg_addr >> 16) & 0xFF,
        (reg_addr >> 8) & 0xFF,
        reg_addr & 0xFF,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF};
		
			// 传输开始
			ret_code_t err_code;
			m_xfer_done = false;
			err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, buffer, sizeof(buffer), false); // 发送完 STOP 信号
			while (!m_xfer_done);  // 等待传输完成

			if (err_code != NRF_SUCCESS)
					return CPS_ERR_COMM;

			return CPS_OK;

//    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(
//        CPS_I2C_ADDRESS, buffer, 8);

//    ret_code_t err = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (err != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    return CPS_OK;
}

uint32_t big_little_endian_convert(uint32_t dat)
{
    uint8_t *p = (uint8_t *)(&dat);
    return ((uint32_t)p[3] << 24) | 
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] << 8)  |
           (uint32_t)p[0];
}

cps_err_t cps_read_32(uint32_t reg_addr, uint32_t *value)
{
    ret_code_t err_code;
    uint8_t addr_buf[4] = {
        (reg_addr >> 24) & 0xFF,
        (reg_addr >> 16) & 0xFF,
        (reg_addr >> 8) & 0xFF,
        reg_addr & 0xFF};

    uint8_t data_buf[4];

    // 发送寄存器地址（不发送 STOP 信号）
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, addr_buf, sizeof(addr_buf), true);
    while (!m_xfer_done);  // 等待传输完成
    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    // 读取寄存器值
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(hi2c1, CPS_I2C_ADDRESS, data_buf, sizeof(data_buf));
    while (!m_xfer_done);  // 等待读取完成
    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    // 解析数据
    *value = ((uint32_t)data_buf[0] << 24) |
             ((uint32_t)data_buf[1] << 16) |
             ((uint32_t)data_buf[2] << 8) |
             data_buf[3];

    return CPS_OK;
	
//    uint8_t addr_buf[4] = {
//        (reg_addr >> 24) & 0xFF,
//        (reg_addr >> 16) & 0xFF,
//        (reg_addr >> 8) & 0xFF,
//        reg_addr & 0xFF};

//    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(
//        CPS_I2C_ADDRESS, addr_buf, 4);

//    ret_code_t err = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (err != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    uint8_t data_buf[4];
//    xfer_desc = NRF_DRV_TWI_XFER_DESC_RX(CPS_I2C_ADDRESS, data_buf, 4);
//    err = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (err != NRF_SUCCESS)
//        return CPS_ERR_COMM;

//    *value = ((uint32_t)data_buf[0] << 24) |
//             ((uint32_t)data_buf[1] << 16) |
//             ((uint32_t)data_buf[2] << 8) |
//             data_buf[3];
//    return CPS_OK;
}

cps_err_t cps_set_vout(uint16_t voltage_mv)
{
    if (voltage_mv < 3000 || voltage_mv > 12000)
        return CPS_ERR_INVALID_PARAM;

    uint8_t data[2] = {(uint8_t)(voltage_mv >> 8), (uint8_t)voltage_mv};
    return write_register(CPS_REG_VOUT_SET, data, 2);
}

cps_err_t cps_set_current_limit(uint16_t current_ma)
{
    if (current_ma < 100 || current_ma > 3000)
        return CPS_ERR_INVALID_PARAM;

    uint8_t data = (uint8_t)(current_ma / 50); // 1 LSB = 50mA
    return write_register(CPS_REG_ILIM_SET, &data, 1);
}

cps_err_t cps_poll_data(cps_charger_data_t *data)
{
    uint8_t raw_data[2];

    // 读取VRECT
    if (read_register(CPS_REG_ADC_VRECT, raw_data, 2) != CPS_OK)
        return CPS_ERR_COMM;
    data->vrect_mv = (raw_data[0] << 8) | raw_data[1];

    // 读取VOUT
    if (read_register(CPS_REG_ADC_VOUT, raw_data, 2) != CPS_OK)
        return CPS_ERR_COMM;
    data->vout_mv = (raw_data[0] << 8) | raw_data[1];

    // 读取IOUT
    if (read_register(CPS_REG_ADC_IOUT, raw_data, 2) != CPS_OK)
        return CPS_ERR_COMM;
    data->iout_ma = (raw_data[0] << 8) | raw_data[1];

    // 读取温度
    if (read_register(CPS_REG_DIE_TEMP, raw_data, 2) != CPS_OK)
        return CPS_ERR_COMM;
    data->die_temp = (int8_t)((raw_data[0] << 8) | raw_data[1]);

    // 状态判断（简化逻辑）
    if (data->vout_mv < 1000)
    {
        data->state = CPS_STATE_DISCONNECTED;
    }
    else if (data->iout_ma > 50)
    {
        data->state = CPS_STATE_CHARGING;
    }
    else
    {
        data->state = CPS_STATE_FULL;
    }

    return CPS_OK;
}

cps_err_t cps_enable_charging(bool enable)
{
    if (!m_initialized)
        return CPS_ERR_NOT_INIT;

    // 通过控制VOUT_SET寄存器实现充电使能
    return enable ? cps_set_vout(m_current_data.vout_mv) : cps_set_vout(0);
}

cps_err_t cps_get_chip_info(uint16_t *chip_id, uint16_t *fw_major, uint16_t *fw_minor)
{
    uint8_t data[2];
    if (chip_id)
    {
        if (read_register(CPS_REG_CHIP_ID, data, 2) != CPS_OK)
            return CPS_ERR_COMM;
        *chip_id = (data[0] << 8) | data[1];
    }

    if (fw_major)
    {
        if (read_register(CPS_REG_FW_MAJOR, data, 2) != CPS_OK)
            return CPS_ERR_COMM;
        *fw_major = (data[0] << 8) | data[1];
    }

    if (fw_minor)
    {
        if (read_register(CPS_REG_FW_MINOR, data, 2) != CPS_OK)
            return CPS_ERR_COMM;
        *fw_minor = (data[0] << 8) | data[1];
    }

    return CPS_OK;
}

//cps_err_t cps_clear_fault(void)
//{
//    // 写任意值到INT_CLEAR寄存器
//    uint8_t clear_cmd[2] = {0xFF, 0xFF};
//    return write_register(CPS_REG_INT_CLEAR, clear_cmd, 2);
//}

// 私有函数
static cps_err_t enter_bootloader_mode(void)
{
    // 进入bootloader模式
    CPS_CHECK(cps_write_32(0xFFFFFF00, big_little_endian_convert(0x0E000000)));   // 使能32位模式
    CPS_CHECK(cps_write_32(0x40040070, big_little_endian_convert(0x0000A061)));   // 密码
    CPS_CHECK(cps_write_32(0x40040004, big_little_endian_convert(0x00000008)));   // 复位并暂停MCU

    nrf_delay_ms(10);

    return CPS_OK;
}

// crc校验
static cps_err_t cps_crc_test(void)
{
    // CRC校验
    CPS_CHECK(cps_write_32(ADDR_CMD, CACL_CRC_TEST));

    uint32_t timeout = CPS_PROGRAM_TIMEOUT_MS;
    uint32_t status;
    do
    {
        CPS_CHECK(cps_read_32(ADDR_FLAG, &status));
        if ((status & 0xFF) == PASS)
            break;
        nrf_delay_ms(1);
    } while (--timeout);
    if (timeout == 0)
        return CPS_ERR_TIMEOUT;

    return CPS_OK;
}

// 完成固件升级
cps_err_t cps_fw_update_finalize(void)
{
    // CRC校验
    CPS_CHECK(cps_write_32(ADDR_CMD, CACL_CRC_APP));

    uint32_t timeout = CPS_PROGRAM_TIMEOUT_MS;
    uint32_t status;
    do
    {
        CPS_CHECK(cps_read_32(ADDR_FLAG, &status));
        if ((status & 0xFF) == PASS)
            break;
        nrf_delay_ms(1);
    } while (--timeout);
    if (timeout == 0)
        return CPS_ERR_TIMEOUT;

    // 设置启动标志
    CPS_CHECK(cps_write_32(ADDR_CMD, big_little_endian_convert(PGM_WR_FLAG)));

    // 系统复位
    CPS_CHECK(cps_write_32(0x40040004, big_little_endian_convert(0x00000001)));
    nrf_delay_ms(100);

    return CPS_OK;
}

cps_err_t cps_wls_send_ask_packet(uint8_t *data, uint8_t data_len)
{
    if (!m_initialized)
        return CPS_ERR_NOT_INIT;

    cps_registers_t base_reg = (cps_registers_t)CPS_RX_REG_PPP_HEADER;

    // 写入数据到连续寄存器
    for (uint8_t i = 0; i < data_len; i++)
    {
        cps_err_t ret = write_register((cps_registers_t)(base_reg + i), &data[i], 1);
        if (ret != CPS_OK)
        {
            return ret; // 直接返回驱动错误码
        }
    }

    // 读取并更新命令寄存器
    uint8_t cmd_data[2];
    cps_err_t ret = read_register((cps_registers_t)CPS_COMM_REG_CMD1, cmd_data, 2);
    if (ret != CPS_OK)
    {
        return ret;
    }

    uint16_t cmd = (cmd_data[0] << 8) | cmd_data[1];
    cmd |= RX_CMD_SEND_DATA; // 设置发送标志位

    // 写回命令寄存器
    uint8_t updated_cmd[2] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
    return write_register((cps_registers_t)CPS_COMM_REG_CMD1, updated_cmd, 2);
}

cps_err_t test_send(void)
{
    uint8_t packet_data[] = {0xAA, 0xBB, 0xCC};
    cps_err_t ret = cps_wls_send_ask_packet(packet_data, sizeof(packet_data));
    if (ret == CPS_OK)
    {
        // 发送成功
    }
    else
    {
        // 处理错误
    }
		return ret;
}

// 公共接口
cps_err_t cps_fw_update_init(void)
{
    //CPS_SAFE_CALL(enter_bootloader_mode());
		enter_bootloader_mode();
    return CPS_OK;
}

static cps_err_t cps_wls_program_wait_cmd_done(void)
{
    // 等待完成
    uint32_t timeout = CPS_PROGRAM_TIMEOUT_MS;
    uint32_t status;
    do
    {
        CPS_CHECK(cps_read_32(ADDR_FLAG, &status));
        if ((status & 0xFF) == PASS)
            break;
        nrf_delay_ms(1);
    } while (--timeout);
    if (timeout == 0)
        return CPS_ERR_TIMEOUT;

    return CPS_OK;
}

// 写入固件页
cps_err_t cps_fw_write_page()
{
    uint8_t buff0_flag = 0, buff1_flag = 0;
    uint8_t result;
    int firmware_length;
    unsigned char *data;
    int k;
    uint32_t *p_convert;
    unsigned char *firmware_buf;

    firmware_load(&firmware_buf, &firmware_length); // load

    p_convert = (uint32_t *)firmware_buf;
    for (k = 0; k < 16 * 1024 / 4; k++)
    {
        p_convert[k] = big_little_endian_convert(p_convert[k]);
    }

    data = (unsigned char *)p_convert;

    for (uint32_t offset = 0; offset < firmware_length; offset += CPS_FW_PAGE_SIZE)
    {
        if (buff0_flag == 0)
        {
						ret_code_t err_code;
					
            // 写入缓冲区
            uint8_t buffer[CPS_FW_PAGE_SIZE + 4];
            buffer[0] = (ADDR_BUFFER0 >> 24) & 0xFF;
            buffer[1] = (ADDR_BUFFER0 >> 16) & 0xFF;
            buffer[2] = (ADDR_BUFFER0 >> 8) & 0xFF;
            buffer[3] = ADDR_BUFFER0 & 0xFF;
            memcpy(buffer + 4, data, CPS_FW_PAGE_SIZE);
					
						// **4. 发送 I2C 数据**
						m_xfer_done = false;
						err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, buffer, sizeof(buffer), false); // 发送 STOP 信号

						while (!m_xfer_done)
							;  // 等待传输完成

						if (err_code != NRF_SUCCESS) 
						{
								return CPS_ERR_COMM;
						}

//            nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(
//                CPS_I2C_ADDRESS, buffer, sizeof(buffer));

//            ret_code_t err = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//            if (err != NRF_SUCCESS)
//                return CPS_ERR_COMM;

            data = data + CPS_FW_PAGE_SIZE;
            if (buff1_flag == 1)
            {
                // wait finish
                result = cps_wls_program_wait_cmd_done();
                if (result != CPS_OK)
                {
                    return CPS_ERR_NOT_INIT;
                }
                buff1_flag = 0;
            }

            // write buff 0 CMD
            CPS_CHECK(cps_write_32(ADDR_CMD, PGM_BUFFER0));
            buff0_flag = 1;
            continue;
        }
        if (buff1_flag == 0)
        {
						ret_code_t err_code;
            // 写入缓冲区
            uint8_t buffer[CPS_FW_PAGE_SIZE + 4];
            buffer[0] = (ADDR_BUFFER1 >> 24) & 0xFF;
            buffer[1] = (ADDR_BUFFER1 >> 16) & 0xFF;
            buffer[2] = (ADDR_BUFFER1 >> 8) & 0xFF;
            buffer[3] = ADDR_BUFFER1 & 0xFF;
            memcpy(buffer + 4, data, CPS_FW_PAGE_SIZE);
					
						// **4. 发送 I2C 数据**
						m_xfer_done = false;
						err_code = nrf_drv_twi_tx(hi2c1, CPS_I2C_ADDRESS, buffer, sizeof(buffer), false); // 发送 STOP 信号

						while (!m_xfer_done)
							;  // 等待传输完成

						if (err_code != NRF_SUCCESS) 
						{
								return CPS_ERR_COMM;
						}

//            nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(
//                CPS_I2C_ADDRESS, buffer, sizeof(buffer));

//            ret_code_t err = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//            if (err != NRF_SUCCESS)
//                return CPS_ERR_COMM;

            data = data + CPS_FW_PAGE_SIZE;
            if (buff0_flag == 1)
            {
                // wait finish
                cps_wls_program_wait_cmd_done();
                buff0_flag = 0;
            }

            CPS_CHECK(cps_write_32(ADDR_CMD, PGM_BUFFER1));
            buff1_flag = 1;
            continue;
        }
    }
    if (buff0_flag == 1)
    {
        // wait finish
        cps_wls_program_wait_cmd_done();
        buff0_flag = 0;
    }

    if (buff1_flag == 1)
    {
        // wait finish
        cps_wls_program_wait_cmd_done();
        buff1_flag = 0;
    }

    return CPS_OK;
}

static int bootloader_load(unsigned char **bootloader, int *bootloader_length)
{
    // 直接返回BOOTLOADER_DATA的指针和长度
    *bootloader = BOOTLOADER_DATA;
    *bootloader_length = CPS_BOOTLOADER_SIZE;
    return 0;
}

static int firmware_load(unsigned char **firmeware, int *firmeware_length)
{
    // 直接返回BOOTLOADER_DATA的指针和长度
    *firmeware = FIRMWARE_DATA;
    *firmeware_length = sizeof(FIRMWARE_DATA);
    return 0;
}

static cps_err_t cps_wls_program_sram(uint32_t addr, const uint8_t *data, int len)
{
		ret_code_t err_code;
	
    // 将地址和数据打包成一个缓冲区
    uint8_t buffer[len + 4];
    buffer[0] = (addr >> 24) & 0xFF;
    buffer[1] = (addr >> 16) & 0xFF;
    buffer[2] = (addr >> 8) & 0xFF;
    buffer[3] = addr & 0xFF;
    memcpy(buffer + 4, data, len);
	
    // 传输开始
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_config.twi_instance, CPS_I2C_ADDRESS, buffer, sizeof(buffer), false); // 发送 STOP 信号

    while (!m_xfer_done);  // 等待传输完成

    if (err_code != NRF_SUCCESS)
        return CPS_ERR_COMM;

    return CPS_OK;

//    int ret;
//    nrf_drv_twi_xfer_desc_t xfer_desc;
//	
//    // 设置TWI传输描述符
//    xfer_desc = NRF_DRV_TWI_XFER_DESC_TX(CPS_I2C_ADDRESS, buffer, len + 4);

//    // 执行TWI传输
//    ret = nrf_drv_twi_xfer(&m_config.twi_instance, &xfer_desc, 0);
//    if (ret != NRF_SUCCESS)
//    {
//        NRF_LOG_ERROR("I2C write error!");
//        return CPS_ERR_COMM;
//    }

//    return CPS_OK;
}

void perform_firmware_update(void)
{
    uint32_t ret;
    uint32_t timeout = CPS_PROGRAM_TIMEOUT_MS;
    unsigned char *bootloader_buf;
    int bootloader_length;
    uint32_t status;

    // 加载bootloader数据
    ret = bootloader_load(&bootloader_buf, &bootloader_length);
    if (ret != 0)
    {
        NRF_LOG_ERROR("Bootloader load error: %d", ret);
        return;
    }

    // 初始化升级
    if (cps_fw_update_init() != CPS_OK)
    {
        NRF_LOG_ERROR("Bootloader entry failed");
        return;
    }

    // 将bootloader数据写入SRAM
    if (CPS_ERR_COMM == cps_wls_program_sram(0x20000000, bootloader_buf, bootloader_length))
    {
        NRF_LOG_ERROR("SRAM programming failed");
        goto update_fail;
    }

    CPS_CHECK(cps_write_32(0x40040010, big_little_endian_convert(0x00000001))); // 使能32位模式
    CPS_CHECK(cps_write_32(0x40040004, big_little_endian_convert(0x00000066))); // 密码
    nrf_delay_ms(10);
    CPS_CHECK(cps_write_32(0xFFFFFF00, big_little_endian_convert(0x0E000000))); // 复位并暂停MCU
    nrf_delay_ms(10);

    if (cps_crc_test() != CPS_OK)
    {
        NRF_LOG_ERROR("cps_bootloader_update fail");
        return;
    }

    NRF_LOG_INFO("bootloader update success");

    CPS_CHECK(cps_write_32(ADDR_CMD, big_little_endian_convert(CPS_PROGRAM_BUFFER_SIZE)));

    CPS_CHECK(cps_write_32(ADDR_CMD, PGM_ERASER_0));

    do
    {
        CPS_CHECK(cps_read_32(ADDR_FLAG, &status));
        if ((status & 0xFF) == PASS)
            break;
        nrf_delay_ms(1);
    } while (--timeout);
    if (timeout == 0)
        return;

    // 写入firmware
    cps_fw_write_page();
   
    if (cps_fw_update_finalize() != CPS_OK)
    {
        NRF_LOG_ERROR("firmware update failed");
        return;
    }

    NRF_LOG_INFO("Firmware update successful");

    return;

update_fail:
//    nrf_drv_twi_disable(&m_config.twi_instance);
//    nrfx_gpiote_in_uninit(CPS_INT_PIN);
    if (bootloader_buf)
        free(bootloader_buf);
}
