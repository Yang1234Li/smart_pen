#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "pen_board.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static void wireless_power_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void charge_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void voltmeter_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void gsensor_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void haptic_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void touch_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}

static void code_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void pen_gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
	
		//wireless power pin set
    nrf_drv_gpiote_in_config_t wireless_power_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    wireless_power_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MCU_INT_4019A_PIN, &wireless_power_config, wireless_power_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MCU_INT_4019A_PIN, true);
	
		//charge pin set
    nrf_drv_gpiote_in_config_t charge_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    charge_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MCU_INT_Charger_PIN, &charge_int_config, charge_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MCU_INT_Charger_PIN, true);
		
		//voltmeter pin set
    nrf_drv_gpiote_in_config_t voltmeter_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    voltmeter_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(MCU_INT_CW2215_PIN, &voltmeter_int_config, voltmeter_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MCU_INT_CW2215_PIN, true);
		
		//gsensor pin set
    nrf_drv_gpiote_in_config_t gsensor_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    gsensor_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(IMU_INT_PIN, &gsensor_int_config, gsensor_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(IMU_INT_PIN, true);
	
		//HapTic RST set
    nrf_drv_gpiote_out_config_t haptic_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(HapTic_RST_PIN, &haptic_out_config);
    APP_ERROR_CHECK(err_code);
		
		//HapTic INT set
    nrf_drv_gpiote_in_config_t haptic_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    haptic_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(HapTic_INT_PIN, &haptic_int_config, haptic_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(HapTic_INT_PIN, true);
		
		//touch RST set
    nrf_drv_gpiote_out_config_t touch_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(TOUCH_RST_PIN, &touch_out_config);
    APP_ERROR_CHECK(err_code);
		
		//touch INT set
    nrf_drv_gpiote_in_config_t touch_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    touch_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(TOUCH_INT_PIN, &touch_int_config, touch_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(TOUCH_INT_PIN, true);
		
		//code RST set
    nrf_drv_gpiote_out_config_t code_out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(RST_5168_PIN, &code_out_config);
    APP_ERROR_CHECK(err_code);
		
		//code INT set
    nrf_drv_gpiote_in_config_t code_int_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    code_int_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(INT_5168_PIN, &code_int_config, code_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_5168_PIN, true);
		
		//code V33 en set
    nrf_drv_gpiote_out_config_t code_V33_EN_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(V33_EN_5168_PIN, &code_V33_EN_config);
    APP_ERROR_CHECK(err_code);
		
		//code V18 en set
    nrf_drv_gpiote_out_config_t code_V18_EN_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(V18_EN_5168_PIN, &code_V18_EN_config);
    APP_ERROR_CHECK(err_code);
		
		//DC V33 en set
    nrf_drv_gpiote_out_config_t V33_EN_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(DC_3V3_EN_PIN, &V33_EN_config);
    APP_ERROR_CHECK(err_code);
		
		//touch V33 en set
    nrf_drv_gpiote_out_config_t touch_V33_EN_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(TOUCH_3V3_EN_PIN, &touch_V33_EN_config);
    APP_ERROR_CHECK(err_code);
	
}


