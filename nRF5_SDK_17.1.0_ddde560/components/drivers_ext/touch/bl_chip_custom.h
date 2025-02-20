#ifndef BL_CHIP_CUSTOM_H
#define BL_CHIP_CUSTOM_H
#define     TS_CHIP          BL6XX6          //配置使用触摸IC型号所属系列
#define     CTP_SLAVE_ADDR		(0x2C)     		 //配置IIC地址
#define     BTL_CHECK_CHIPID                 //开启读取触摸IC硬件识别ID流程
#define     CTP_USE_HW_I2C                   //开启硬件IIC,需关闭CTP_USE_SW_I2C
//#define     CTP_USE_SW_I2C                 //开启软件IIC，需关闭CTP_USE_HW_I2C
#define     GPIO_EINT
#define     RESET_PIN_WAKEUP
//#define     INT_PIN_WAKEUP
//#define     BL_UPDATE_FIRMWARE_ENABLE      //开启升级总开关
//#define     BL_DEBUG
//#define     BTL_FACTORY_TEST_SUPPORT
#endif
