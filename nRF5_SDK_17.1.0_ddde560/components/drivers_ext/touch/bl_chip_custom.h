#ifndef BL_CHIP_CUSTOM_H
#define BL_CHIP_CUSTOM_H
#define     TS_CHIP          BL6XX6          //����ʹ�ô���IC�ͺ�����ϵ��
#define     CTP_SLAVE_ADDR		(0x2C)     		 //����IIC��ַ
#define     BTL_CHECK_CHIPID                 //������ȡ����ICӲ��ʶ��ID����
#define     CTP_USE_HW_I2C                   //����Ӳ��IIC,��ر�CTP_USE_SW_I2C
//#define     CTP_USE_SW_I2C                 //�������IIC����ر�CTP_USE_HW_I2C
#define     GPIO_EINT
#define     RESET_PIN_WAKEUP
//#define     INT_PIN_WAKEUP
//#define     BL_UPDATE_FIRMWARE_ENABLE      //���������ܿ���
//#define     BL_DEBUG
//#define     BTL_FACTORY_TEST_SUPPORT
#endif
