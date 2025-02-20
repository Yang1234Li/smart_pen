#ifndef _USER_AW320XX_H_
#define _USER_AW320XX_H_
//#include "main.h"
//#include "i2c.h"
#include <stdio.h>
#include <nrfx_twi.h>
#include <nrfx_uart.h>
#include <aw320xx.h>

//#define AWINIC_UART_DEBUG
#ifdef AWINIC_UART_DEBUG
//#define AWLOGD(format, arg...) printf(format, ##arg)
#else
//#define AWLOGD(format, arg...) (do {} while (0))
#endif

#define AW320XX_IRQ_PIN 2
#define AW320XX_ADDR			(0x92U >> 1)//(0x49)
//#define AW320XX_READ_ADDR	(0x92U >> 1)

#define AW320XX_REG0_SCR	(0x00)
#define AW320XX_REG1_POCR	(0x01)
#define AW320XX_REG2_CCR	(0x02)
#define AW320XX_REG3_CCR2	(0x03)
#define AW320XX_REG4_CVR	(0x04)
#define AW320XX_REG5_TIMCR	(0x05)
#define AW320XX_REG6_MCR	(0x06)
#define AW320XX_REG7_SVCR	(0x07)
#define AW320XX_REG8_STATR	(0x08)
#define AW320XX_REG9_FLTR	(0x09)
#define AW320XX_REGA_ID		(0x0a)
#define AW320XX_REGB_CCR3	(0x0b)
#define AW320XX_REGC_RCR	(0x0c)
#define AW320XX_REGD_FCR	(0x0d)
#define AW320XX_REG10_START2	(0x10)

#define AW32012_REG_DUMMY1	(0x21)
#define AW32012_REG_WPRT	(0x69)

#define AW32012_WR_PRO_CLOSE	(0xC2)
#define AW32012_WR_PRO_OPEN		(0x00)
#define AW32012_BIT_DET_VIN_SEQ_MASK	(~(1<<7))
#define AW32012_BIT_DET_VIN_SEQ_OPEN	(0<<7)
#define AW32012_BIT_DET_VIN_SEQ_CLOSE	(1<<7)

#define AW320XX_I2C_RW_RETRIES		(5)
#define AW320XX_REGADD_SIZE_8BIT	(1)
#define AW320XX_REGADD_SIZE_16BIT	(2)
#define AW3200X_REV_ID	  	2
#define AW32012_REV_ID	  	3
#define AW320XX_CHIP_ID		(0x49)

#define AW320XX_IN_CVST				(1)
#define AW320XX_NOT_IN_CVST		(0)

#define AW320XX_IRQ_ERROR		1
#define AW320XX_IRQ_OK			0

#define AW320XX_ICC_MAX_NORMAL	(0x38)		/* 456mA */
/********************AW320XX Set config start*******************/
#define PARM_IIN_LMT		(0x0F << 0)						/* REG0:IIN_LMT = 500mA */
#define PARM_VIN_DPM		((4520-3880)/80 << 4)			/* REG0:VIN_DPM = 4520mV,Step 80mV,Base 3880mV,max 5080mV */

#define PARM_VBAT_UVLO		(0x04 << 0)						/* REG1:VBAT_UVLO = 2.76V */

#define PARM_ICC			((128-8)/8 << 0)				/* REG2:ICC = 128mA,Step 8mA,Base 8mV,max 512mA */

#define PARM_ITERM			((3-1)/2 << 0)					/* REG3:ITERM = 3mA,Step 2mA,Base 1mA,max 31mA */
#define PARM_IDSCHG			((2000/200 -1) << 4)			/* REG3:IDSCHG = 2000mA,Step 200mA,Base 200mA,max 3200mA */

#define PARM_VRECH			(1 << 0)						/* REG4:0-VRECH= 100mV 1-VRECH = 200mV */
#define PARM_VBAT_PRE		(1 << 1)						/* REG4:0-VBAT_PRE = 2.8V, 1-VBAT_PRE = 3.0V*/
#define PARM_VBAT_REG		((4200-3600)/15 << 2)			/* REG4:VBAT_REG = 4200mV,Step 15mV,Base 3600mV,max 4545mV */

#define PARM_CHG_TMR		(0 << 1)						/* REG5:0-CHG_TMR = 3h */

#define PARM_VSYS_REG		((4600-4200)/50 << 0)			/* REG7:VSYS_REG = 4600mV,Step 50mV,Base 4200mV,max 4950mV */
#define PARM_TJ_REG			(((120-60)/20) << 4)			/* REG7:TJ_REG = 120C,Step 20mA,Base 60mA,max 120mA */

															/* NOTE: need setting REGB:EN_IPRE_SET=1 */
#define PARM_IBAT_PRE		((3-1)/2 << 1)					/* REGB:IBAT_PRE = 3mA,Step 2mA,Base 1mA,max 31mA */
#define PARM_EN_ICC_DIVD	(0 << 7)						/* REGB */

#define PARM_PRETO			(0 << 3)						/* REGC:0-PRETO = 1h */
#define PARM_ITERMDEG		(0 << 6)						/* REGC:0-ITERMDEG = 3s */

#define PARM_VIN_OVP		(0 << 5)						/* AW32012: REGD:0-VIN_OVP = 6.0V 1-VIN_OVP = 7.5V */
/********************AW320XX Set config end*******************/


/* reg:0x00 */
#define AW320XX_BIT_SCR_VIN_MIN_MASK                 (~(15<<4))
#define AW320XX_BIT_SCR_IIN_LMT_MASK                 (~(15<<0))
#define AW320XX_BIT_SCR_IIN_LMT_500MA                (0X0F)

/* reg:0x01 */
#define AW320XX_BIT_POCR_RST_DEG_MASK                (~(3<<6))
#define AW320XX_BIT_POCR_RST_DEG_8S                  (0<<6)
#define AW320XX_BIT_POCR_RST_DEG_12S                 (1<<6)
#define AW320XX_BIT_POCR_RST_DEG_16S                 (2<<6)
#define AW320XX_BIT_POCR_RST_DEG_20S                 (3<<6)
#define AW320XX_BIT_POCR_RST_DUR_MASK                (~(1<<5))
#define AW320XX_BIT_POCR_RST_DUR_2S                  (0<<5)
#define AW320XX_BIT_POCR_RST_DUR_4S                  (1<<5)
#define AW320XX_BIT_POCR_ENHIZ_MASK                  (~(1<<4))
#define AW320XX_BIT_POCR_ENHIZ_ENABLE                (1<<4)
#define AW320XX_BIT_POCR_ENHIZ_DISABLE               (0<<4)
#define AW320XX_BIT_POCR_CEB_MASK                    (~(1<<3))
#define AW320XX_BIT_POCR_CEB_ENABLE                  (0<<3)
#define AW320XX_BIT_POCR_CEB_DISABLE                 (1<<3)
#define AW320XX_BIT_POCR_VBAT_UVLO_MASK              (~(7<<0))
/* reg:0x02 */
#define AW320XX_BIT_CCR_SOFT_RST_MASK                (~(1<<7))
#define AW320XX_BIT_CCR_SOFT_RST_NORMAL              (0<<7)
#define AW320XX_BIT_CCR_SOFT_RST_RESET               (1<<7)
#define AW320XX_BIT_CCR_WD_RST_MASK                  (~(1<<6))
#define AW320XX_BIT_CCR_WD_RST_NORMAL                (0<<6)
#define AW320XX_BIT_CCR_WD_RST_RESET                 (1<<6)
#define AW320XX_BIT_CCR_ICC_MASK                     (~(63<<0))
/* reg:0x03 */
#define AW320XX_BIT_CCR2_IDSCHG_MASK                 (~(15<<4))
#define AW320XX_BIT_CCR2_ITERM_MASK                  (~(15<<0))
/* reg:0x04 */
#define AW320XX_BIT_CVR_VBAT_REG_MASK                (~(63<<2))
#define AW320XX_BIT_CVR_VBAT_PRE_MASK                (~(1<<1))
#define AW320XX_BIT_CVR_VBAT_PRE_2P8V                (0<<1)
#define AW320XX_BIT_CVR_VBAT_PRE_3P0V                (1<<1)
#define AW320XX_BIT_CVR_VRECH_MASK                   (~(1<<0))
#define AW320XX_BIT_CVR_VRECH_100MV                  (0<<0)
#define AW320XX_BIT_CVR_VRECH_200MV                  (1<<0)
/* reg:0x05 */
#define AW320XX_BIT_TIMCR_EN_WD_DSCHG_MASK           (~(1<<7))
#define AW320XX_BIT_TIMCR_EN_WD_DSCHG_DISABLE        (0<<7)
#define AW320XX_BIT_TIMCR_EN_WD_DSCHG_ENABLE         (1<<7)
#define AW320XX_BIT_TIMCR_WD_CFG_MASK                (~(3<<5))
#define AW320XX_BIT_TIMCR_WD_CFG_DISABLE             (0<<5)
#define AW320XX_BIT_TIMCR_WD_CFG_40S                 (1<<5)
#define AW320XX_BIT_TIMCR_WD_CFG_80S                 (2<<5)
#define AW320XX_BIT_TIMCR_WD_CFG_160S                (3<<5)
#define AW320XX_BIT_TIMCR_EN_TERM_MASK               (~(1<<4))
#define AW320XX_BIT_TIMCR_EN_TERM_DISABLE            (0<<4)
#define AW320XX_BIT_TIMCR_EN_TERM_ENABLE             (1<<4)
#define AW320XX_BIT_TIMCR_EN_TMR_MASK                (~(1<<3))
#define AW320XX_BIT_TIMCR_EN_TMR_DISABLE             (0<<3)
#define AW320XX_BIT_TIMCR_EN_TMR_ENABLE              (1<<3)
#define AW320XX_BIT_TIMCR_CHG_TMR_MASK               (~(3<<1))
#define AW320XX_BIT_TIMCR_CHG_TMR_3HRS               (0<<1)
#define AW320XX_BIT_TIMCR_CHG_TMR_5HRS               (1<<1)
#define AW320XX_BIT_TIMCR_CHG_TMR_8HRS               (2<<1)
#define AW320XX_BIT_TIMCR_CHG_TMR_12HRS              (3<<1)
#define AW320XX_BIT_TIMCR_TERM_TMR_MASK              (~(1<<0))
#define AW320XX_BIT_TIMCR_TERM_TMR_DISABLE           (0<<0)
#define AW320XX_BIT_TIMCR_TERM_TMR_ENABLE            (1<<0)
/* reg:0x06 */
#define AW320XX_BIT_MCR_EN_NTC_MASK                  (~(1<<7))
#define AW320XX_BIT_MCR_EN_NTC_DISABLE               (0<<7)
#define AW320XX_BIT_MCR_EN_NTC_ENABLE                (1<<7)
#define AW320XX_BIT_MCR_TMR2X_EN_MASK                (~(1<<6))
#define AW320XX_BIT_MCR_TMR2X_EN_DISABLE             (0<<6)
#define AW320XX_BIT_MCR_TMR2X_EN_ENABLE              (1<<6)
#define AW320XX_BIT_MCR_FET_DIS_MASK                 (~(1<<5))
#define AW320XX_BIT_MCR_FET_DIS_ON                   (0<<5)
#define AW320XX_BIT_MCR_FET_DIS_OFF                  (1<<5)
#define AW320XX_BIT_MCR_PG_INT_CTR_MASK              (~(1<<4))
#define AW320XX_BIT_MCR_PG_INT_CTR_ON                (0<<4)
#define AW320XX_BIT_MCR_PG_INT_CTR_OFF               (1<<4)
#define AW320XX_BIT_MCR_EOC_INT_CTR_MASK             (~(1<<3))
#define AW320XX_BIT_MCR_EOC_INT_CTR_ON               (0<<3)
#define AW320XX_BIT_MCR_EOC_INT_CTR_OFF              (1<<3)
#define AW320XX_BIT_MCR_CS_INT_CTR_MASK              (~(1<<2))
#define AW320XX_BIT_MCR_CS_INT_CTR_ON                (0<<2)
#define AW320XX_BIT_MCR_CS_INT_CTR_OFF               (1<<2)
#define AW320XX_BIT_MCR_NTC_INT_CTR_MASK             (~(1<<1))
#define AW320XX_BIT_MCR_NTC_INT_CTR_ON               (0<<1)
#define AW320XX_BIT_MCR_NTC_INT_CTR_OFF              (1<<1)
#define AW320XX_BIT_MCR_BOVP_INT_CTR_MASK            (~(1<<0))
#define AW320XX_BIT_MCR_BOVP_INT_CTR_ON              (0<<0)
#define AW320XX_BIT_MCR_BOVP_INT_CTR_OFF             (1<<0)
/* reg:0x07 */
#define AW320XX_BIT_SVCR_EN_PCB_OTP_MASK             (~(1<<7))
#define AW320XX_BIT_SVCR_EN_PCB_OTP_DISABLE          (1<<7)
#define AW320XX_BIT_SVCR_EN_PCB_OTP_ENABLE           (0<<7)
#define AW320XX_BIT_SVCR_EN_VINLOOP_MASK             (~(1<<6))
#define AW320XX_BIT_SVCR_EN_VINLOOP_DISABLE          (1<<6)
#define AW320XX_BIT_SVCR_EN_VINLOOP_ENABLE           (0<<6)
#define AW320XX_BIT_SVCR_TJ_REG_MASK                 (~(3<<4))
#define AW320XX_BIT_SVCR_TJ_REG_60                   (0<<4)
#define AW320XX_BIT_SVCR_TJ_REG_80                   (1<<4)
#define AW320XX_BIT_SVCR_TJ_REG_100                  (2<<4)
#define AW320XX_BIT_SVCR_TJ_REG_120                  (3<<4)
#define AW320XX_BIT_SVCR_VSYS_REG_MASK               (~(15<<0))
/* reg:0x08 */
#define AW320XX_BIT_REV_ID_MASK	                     (3<<5)
#define AW320XX_BIT_REV_ID_SHIFT                     (5)
#define AW320XX_BIT_CHG_STAT_MASK	                   (3<<3)
#define AW320XX_BIT_CHG_STAT_SHIFT	                 (3)
#define AW320XX_BIT_PG_STAT_MASK	                 (1<<1)
#define AW320XX_BIT_PG_STAT_POWER_FAIL               (0<<1)
#define AW320XX_BIT_PG_STAT_POWER_GOOD               (1<<1)
/* reg:0x09 */
#define AW320XX_BIT_FLTR_EN_SHIP_DEG_MASK            (~(3<<6))
#define AW320XX_BIT_FLTR_TJ_REG_1S                   (0<<6)
#define AW320XX_BIT_FLTR_TJ_REG_2S                   (1<<6)
#define AW320XX_BIT_FLTR_TJ_REG_4S                   (2<<6)
#define AW320XX_BIT_FLTR_TJ_REG_8S                   (3<<6)
#define AW320XX_BIT_FLTR_FAULT_MASK                  (0x3F)
/* reg:0x0a is DEV_ID*/
/* reg:0x0b */
#define AW320XX_BIT_CCR3_EN_ICC_DIVD_MASK            (~(1<<7))
#define AW320XX_BIT_CCR3_EN_ICC_DIVD_DISABLE         (0<<7)
#define AW320XX_BIT_CCR3_EN_ICC_DIVD_ENABLE          (1<<7)
#define AW320XX_BIT_CCR3_EN_IPRE_SET_MASK            (~(1<<5))
#define AW320XX_BIT_CCR3_EN_IPRE_SET_DISABLE         (0<<5)
#define AW320XX_BIT_CCR3_EN_IPRE_SET_ENABLE          (1<<5)
#define AW320XX_BIT_CCR3_IPRE_MASK                   (~(15<<1))
#define AW320XX_BIT_CCR3_EXSHIP_DEG_MASK             (~(1<<0))
#define AW320XX_BIT_CCR3_EXSHIP_DEG_2S               (0<<0)
#define AW320XX_BIT_CCR3_EXSHIP_DEG_100MS            (1<<0)
/* reg:0x0c */
#define AW320XX_BIT_RCR_EN0P55_MASK                  (~(1<<7))
#define AW320XX_BIT_RCR_EN0P55_DISABLE               (0<<7)
#define AW320XX_BIT_RCR_EN0P55_ENABLE                (1<<7)
#define AW320XX_BIT_RCR_ITERMDEG_MASK                (~(1<<6))
#define AW320XX_BIT_RCR_ITERMDEG_3S                  (0<<6)
#define AW320XX_BIT_RCR_ITERMDEG_1S                  (1<<6)
#define AW320XX_BIT_RCR_PRETO_MASK                   (~(1<<3))
#define AW320XX_BIT_RCR_PRETO_1H                     (0<<3)
#define AW320XX_BIT_RCR_PRETO_2H                     (1<<3)
#define AW320XX_BIT_RCR_EN10KNTC_MASK                (~(1<<1))
#define AW320XX_BIT_RCR_EN10KNTC_DISABLE             (0<<1)
#define AW320XX_BIT_RCR_EN10KNTC_ENABLE              (1<<1)
#define AW320XX_BIT_RCR_RSTDLY_MASK                  (~(1<<0))
#define AW320XX_BIT_RCR_RSTDLY_0S                    (0<<0)
#define AW320XX_BIT_RCR_RSTDLY_2S                    (1<<0)
/* reg:0x0d */
#define AW320XX_BIT_VIN_OVP_MASK                  (~(1<<7))
#define AW320XX_BIT_VIN_OVP_6V		              (0<<7)
#define AW320XX_BIT_VIN_OVP_7P5V              		  (1<<7)
/* reg:0x10 */
#define AW320XX_BIT_STATR2_CVST_NORMAL                  (1<<1)

enum aw320xx_charge_state{
	AW320XX_NOT_CHARGING =0,
	AW320XX_PRE_CHARGE,
	AW320XX_CHARGE,
	AW320XX_CHARGE_DONE,
};

struct aw320xx {
	uint16_t vin_dpm;
	uint16_t iin_lmt;
	uint16_t rst_deg;
	uint16_t rst_dur;
	uint16_t enhiz;
	uint16_t ceb;
	uint16_t vbat_uvlo;
	uint16_t soft_rst;
	uint16_t wd_rst;
	uint16_t icc;
	uint16_t idschg;
	uint16_t iterm;
	uint16_t vbat_reg;
	uint16_t vbat_pre;
	uint16_t vrech;
	uint16_t en_wd_dschg;
	uint16_t wd_cfg;
	uint16_t en_term;
	uint16_t en_tmr;
	uint16_t chg_tmr;
	uint16_t term_tmr;
	uint16_t en_ntc;
	uint16_t tmr2x_en;
	uint16_t fet_dis;
	uint16_t pg_int_ctr;
	uint16_t eoc_int_ctr;
	uint16_t cs_int_crt;
	uint16_t ntc_int_crt;
	uint16_t bovp_int_ctr;
	uint16_t en_pcb_otp;
	uint16_t en_vinloop;
	uint16_t tj_reg;
	uint16_t vsys_reg;
	uint16_t en_ship_deg;
	uint16_t en_icc_divd;
	uint16_t en_ipre_set;
	uint16_t ipre;
	uint16_t exship_deg;
	uint16_t itermdeg;
	uint16_t preto;
	uint16_t en10kntc;
	uint16_t rstdly;
	uint16_t device_id;
	uint16_t vin_ovp;
	uint16_t chg_stat;
	uint16_t fault_flag;
	nrf_drv_twi_t *hi2cx;
};
extern void aw320xx_enter(void);

#define AW320XX_STATUS_IS_OK	1
enum aw320xx_error{
	AW320XX_STATUS_IS_ERROR = 0,
	AW320XX_CHIPID_FAILD,
	AW320XX_DEVICE_ID_FAILD,
	AW320XX_CVST_FAILD,
};
#endif
