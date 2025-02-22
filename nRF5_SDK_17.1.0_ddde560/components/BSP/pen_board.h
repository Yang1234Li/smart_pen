#include <nrfx_gpiote.h>

#define MCU_INT_4019A_PIN   				 	0
#define MCU_INT_Charger_PIN   				1
#define MCU_INT_CW2215_PIN   				 	2
#define Tail_FS_OUT_PIN   				 		5
#define MCU_SCL0_PIN   				 				6
#define MCU_SDA0_PIN   				 				7
#define MCU_SCL2_PIN   				 				8
#define MCU_SDA2_PIN   				 				9
#define IMU_SPI1_MISO_PIN   				 10
#define IMU_SPI1_MOSI_PIN   				 11
#define IMU_SPI1_CLK_PIN  					 12
#define IMU_SPI1_CS_PIN  						 13
#define IMU_INT_PIN  								 14
#define UART_TX_PIN   				 			 15
#define UART_RX_PIN   				 			 16
#define NOR_SPI1_MISO_PIN  					 17
#define NOR_SPI1_MOSI_PIN  					 18
#define NOR_SPI1_CS_PIN  						 19
#define NOR_SPI1_SCLK_PIN   				 20
#define HapTic_RST_PIN   				 		 21
#define HapTic_INT_PIN  					   22
#define TOUCH_RST_PIN  						   23
#define TOUCH_INT_PIN  							 24
#define RST_5168_PIN   				 			 25
#define INT_5168_PIN   				 			 26
#define V33_EN_5168_PIN  					   27
#define V18_EN_5168_PIN  					   28
#define MCU_ADC_TEST_PIN  					 29
#define DC_3V3_EN_PIN   				 		 30
#define TOUCH_3V3_EN_PIN   				 	 31

void pen_spi_init(void);
void pen_spi_deinit(void);
uint32_t pen_spi_read(uint8_t* read_buffer, uint16_t length);
uint32_t pen_spi_write(uint8_t* write_buffer, uint16_t length);
void gsensor_event_get(void);
void pen_gsensor_init(void);

void pen_twi_init (void);
