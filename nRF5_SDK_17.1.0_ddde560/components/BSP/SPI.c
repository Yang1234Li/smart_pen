#include "nrf_drv_spi.h"
#include "pen_board.h"
#include "lsm6dso_reg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static nrf_drv_spi_t pen_gsensor_spi1_drv = NRF_DRV_SPI_INSTANCE(1);
static uint8_t g_lx_board_spi_is_init = 0;

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
}

void pen_spi_init(void)
{
    ret_code_t err_code;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    
    if(g_lx_board_spi_is_init)
    {
        return;
    }
		
    spi_config.ss_pin  = IMU_SPI1_CS_PIN;
    spi_config.sck_pin = IMU_SPI1_CLK_PIN;
    spi_config.miso_pin = IMU_SPI1_MISO_PIN;
    spi_config.mosi_pin = IMU_SPI1_MOSI_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    err_code = nrf_drv_spi_init(&pen_gsensor_spi1_drv, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
    g_lx_board_spi_is_init = 1;
}

void pen_spi_deinit(void)
{
    nrf_drv_spi_uninit(&pen_gsensor_spi1_drv);
    g_lx_board_spi_is_init = 0;
}

uint32_t pen_spi_read(uint8_t* read_buffer, uint16_t length)
{
    ret_code_t err_code;
    
		spi_xfer_done = false;
    err_code = nrf_drv_spi_transfer(&pen_gsensor_spi1_drv, NULL, 0, read_buffer, length);
		while (spi_xfer_done == false)
			;
	
    if(err_code == NRF_SUCCESS)
    {
        return 1;
    }
    
    return 0;
}

uint32_t pen_spi_write(uint8_t* write_buffer, uint16_t length)
{
    ret_code_t err_code;
    
		spi_xfer_done = false;
    err_code = nrf_drv_spi_transfer(&pen_gsensor_spi1_drv, write_buffer, length, NULL, 0);
		while (spi_xfer_done == false)
			;
		
    if(err_code == NRF_SUCCESS)
    {
        return 1;
    }
    
    return 0;
}

void pen_gsensor_init(void)
{
		stmdev_ctx_t dev_ctx;
		uint8_t whoamI,rst;
		lsm6dso_pin_int2_route_t int2_route;
	
		lsm6dso_device_id_get(&dev_ctx, &whoamI);

		if (whoamI != LSM6DSO_ID)
			while (1);
		
		/* Restore default configuration */
		lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

		do {
			lsm6dso_reset_get(&dev_ctx, &rst);
		} while (rst);
		
		/* Disable I3C interface */
		lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
		/* Set SPI interface */
		lsm6dso_aux_spi_mode_set(&dev_ctx, LSM6DSO_AUX_SPI_4_WIRE);
		/* Set XL Output Data Rate to 416 Hz */
		lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_417Hz);
		/* Set 2g full XL scale */
		lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
		/* Apply high-pass digital filter on Wake-Up function */
		lsm6dso_xl_hp_path_internal_set(&dev_ctx, LSM6DSO_USE_SLOPE);
		/* Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6 */
		lsm6dso_wkup_threshold_set(&dev_ctx, 2);
		/* Uncomment interrupt generation on Wake-Up INT1 pin */
		//lsm6dso_pin_int1_route_get(&dev_ctx, &int1_route);
		//int1_route.wake_up = PROPERTY_ENABLE;
		//lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);
		/* Enable if interrupt generation on Wake-Up INT2 pin */
		lsm6dso_pin_int2_route_get(&dev_ctx, NULL, &int2_route);
		int2_route.wake_up = PROPERTY_ENABLE;
		lsm6dso_pin_int2_route_set(&dev_ctx, NULL, int2_route);
}

void gsensor_event_get(void)
{
		stmdev_ctx_t dev_ctx;
	
    lsm6dso_all_sources_t all_source;
    /* Check if Wake-Up events */
    lsm6dso_all_sources_get(&dev_ctx, &all_source);
}

