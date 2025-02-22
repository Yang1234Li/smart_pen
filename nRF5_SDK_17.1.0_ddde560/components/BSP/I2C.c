#include "nrf_drv_twi.h"
#include "pen_board.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
const nrf_drv_twi_t *hi2c1 = &m_twi;
/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief I2C initialization.
 */
void pen_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t pen_twi_config = {
       .scl                = MCU_SCL0_PIN,
       .sda                = MCU_SDA0_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &pen_twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
