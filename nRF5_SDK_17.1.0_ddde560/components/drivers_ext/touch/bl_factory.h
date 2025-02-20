#ifndef _BL_FACTORY_H_
#define _BL_FACTORY_H_

/**********common definition***************/
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define VERSION_SIZE                   3
#define PROJECT_CODE_SIZE              20
#define ENTER_WORK_FACTORY_RETRIES     3
#define FACTORY_TEST_DELAY             20
#define FACTORY_SCAN_RETRIES           10
#define FACTORY_CALI_RETRIES           20
#define MULTIFLYING_POWER_VALUE        10000
#define BTL_THROW_FRAME_CNT            3
#define BTL_BASE_FRAME_CNT             5
#define BTL_DIFF_FRAME_CNT             20
#define BTL_RANGE_OF_KEY_CB            14
#define BTL_WAIT_SHORT_TEST_TIME       200
#define BTL_WAIT_OPEN_TEST_TIME        500


/**********reg definition******************/
//common reg definition
#define BTL_REG_MODE_SEL               0xf7
#define BTL_REG_SCAN_DATA              0xc0
#define BTL_REG_SCAN_SHORT             0xc4
#define BTL_REG_VERSION                0xb6
#define BTL_REG_PROJECT_CODE           0xb4
#define BTL_REG_INT_TEST               0x9d
//SC reg definition
#define BTL_REG_SC_RX_CHN              0x15
#define BTL_REG_SC_FACTORY_ID          0xb4
#define BTL_REG_SC_CB_CALI             0x9c
#define BTL_REG_SC_CB                  0x11
#define BTL_REG_SC_RAWDATA             0x7b
#define BTL_REG_SC_SHORT               0x7b
#define BTL_REG_SCAN_VOL               0x0c
#define BTL_REG_SCAN_CF                0x0d
extern int intFlag;

#endif
