#ifndef CPS_WLS_NRF52832_DRIVER_H
#define CPS_WLS_NRF52832_DRIVER_H
#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "nrfx_gpiote.h"

#define CPS_TWI_INSTANCE_ID 0
#define CPS_I2C_ADDRESS 0x30                // 根据实际修改
#define CPS_INT_PIN NRF_GPIO_PIN_MAP(0, 12) // 中断引脚
#define CPS_POLL_INTERVAL_MS 1000           // 轮询间隔
#define CPS_FW_PAGE_SIZE 128                // 根据芯片实际情况修改
#define CPS_BOOTLOADER_SIZE 128
#define CPS_FW_MAX_SIZE (64)
#define CPS_PROGRAM_BUFFER_SIZE 64

#define CPS_PROGRAM_TIMEOUT_MS 50

// 固件升级特殊寄存器
#define ADDR_CMD 0x200005FC
#define ADDR_FLAG 0x200005F8
#define ADDR_BUF_SIZE 0x200005F4
#define ADDR_BUFFER0 0x20000600
#define ADDR_BUFFER1 0x20000700

// 命令定义
#define PGM_BUFFER0 0x10
#define PGM_ERASER_0 0x60
#define CACL_CRC_APP 0x90
#define CACL_CRC_TEST 0xB0
#define PGM_WR_FLAG 0x80

#define RUNNING 0x66
#define PASS 0x55
#define FAIL 0xAA
#define ILLEGAL 0x40

#define CPS_COMM_REG_CMD1 0x0F     // 命令寄存器地址
#define CPS_RX_REG_PPP_HEADER 0x25 
#define RX_CMD_SEND_DATA (0x01 << 0)

// 错误检查宏
#define CPS_CHECK(x)
//do                                                                      \
//{                                                                       \
//    cps_err_t _err = (expr);                                            \
//    if (_err != CPS_OK)                                                 \
//    {                                                                   \
//        NRF_LOG_ERROR("Error 0x%X at %s:%d", _err, __FILE__, __LINE__); \
//        return _err;                                                    \
//    }                                                                   \
//} while (0)

typedef enum
{
    CPS_STATE_DISCONNECTED = 0,
    CPS_STATE_CHARGING,
    CPS_STATE_FULL,
    CPS_STATE_FAULT
} cps_charge_state_t;

typedef struct
{
    uint16_t vrect_mv; // 整流电压(mV)
    uint16_t vout_mv;  // 输出电压(mV)
    uint16_t iout_ma;  // 输出电流(mA)
    int8_t die_temp;   // 芯片温度(℃)
    cps_charge_state_t state;
} cps_charger_data_t;

typedef void (*cps_event_handler_t)(cps_charger_data_t *data);

typedef struct
{
    nrf_drv_twi_t twi_instance; // 替换nrfx_twim_t
    nrfx_gpiote_in_config_t int_config;
    cps_event_handler_t event_handler;
} cps_config_t;

// 寄存器定义
typedef enum
{
    CPS_REG_CHIP_ID = 0x0000,
    CPS_REG_FW_MAJOR = 0x0002,
    CPS_REG_FW_MINOR = 0x0004,
    CPS_REG_INT_STATUS = 0x0009,
    CPS_REG_VOUT_SET = 0x0013,
    CPS_REG_ILIM_SET = 0x0016,
    CPS_REG_ADC_VOUT = 0x0017,
    CPS_REG_ADC_VRECT = 0x0019,
    CPS_REG_ADC_IOUT = 0x001B,
    CPS_REG_DIE_TEMP = 0x001D
} cps_registers_t;

// 错误码
typedef enum
{
    CPS_OK = 0,
    CPS_ERR_COMM,
    CPS_ERR_INVALID_PARAM,
    CPS_ERR_NOT_INIT,
    CPS_ERR_TIMEOUT
} cps_err_t;

static int firmware_load(unsigned char **firmeware, int *firmeware_length);
// 初始化驱动
cps_err_t cps_init(void);

// 设置输出电压（mV）
cps_err_t cps_set_vout(uint16_t voltage_mv);

// 设置电流限制（mA）
cps_err_t cps_set_current_limit(uint16_t current_ma);

// 手动触发数据采集
cps_err_t cps_poll_data(cps_charger_data_t *data);

// 启用/禁用充电
cps_err_t cps_enable_charging(bool enable);

// 获取芯片信息
cps_err_t cps_get_chip_info(uint16_t *chip_id, uint16_t *fw_major, uint16_t *fw_minor);

// 清除故障状态
cps_err_t cps_clear_fault(void);

// 新增函数原型
cps_err_t cps_fw_update_init(void);
cps_err_t cps_fw_update_finalize(void);

#endif // CPS_WLS_NRF52832_DRIVER_H
