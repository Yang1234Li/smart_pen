//#if defined(BTL_FACTORY_TEST_SUPPORT)
#include "tp_ts.h"
#include "bl_factory.h"
int intFlag = 0;
int intTestMode = 0;
unsigned char CB[RX_NUM_MAX] = {0};
//采样CB即CBCali需由配置文件提供传入
unsigned char CBCali[RX_NUM_MAX] = {45, 45, 45, 42, 45, 41, 46, 41, 46, 46, 49, 45, 49, 46, 49, 46, 48, 47, 43, 47, 42, 42, 43, 43, 43, 44, 45, 52, 57, 85, 46, 46, 47, 47, 48, 50, 42, 43, 45, 49, 72};
short CBBase[RX_NUM_MAX] = {9782,10227,9782,9545,9782,9318,10000,9318,10000,10454,10652,10227,10652,10454,10652,10454,10434,10681,9347,10681,9130,9545,9347,9772,9347,10000,9782,11818,12391,19318,10000,10454,10217,10681,10434,11363,9130,9772,9782,11136,15652};
unsigned short rawData[RX_NUM_MAX] = {0};
unsigned short adcShort[RX_NUM_MAX] = {0};
unsigned short adcOpen[RX_NUM_MAX] = {0};
short diff[RX_NUM_MAX] = {0};
unsigned char rxChnNum = 0;

static void btl_set_vol(unsigned char vol)
{
    unsigned char cmd[2] = {0};
	cmd[0] = BTL_REG_SCAN_VOL;
	cmd[1] = vol;
    bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
}

static void btl_set_cf(unsigned char cf)
{
    unsigned char cmd[2] = {0};
	cmd[0] = BTL_REG_SCAN_CF;
	cmd[1] = cf;
    bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
}

static int btl_enter_factory_mode(void)
{
    int ret = 0;
    unsigned char mode = 0;
	unsigned char cmdWrite[2] = {0};
    int i = 0;
    int j = 0;

	cmdWrite[0] = BTL_REG_MODE_SEL;
	cmdWrite[1] = 0x40;

    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_MODE_SEL, &mode, 1);
    if ((ret == 0) && (0x40 == (mode & 0x7f)))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmdWrite, sizeof(cmdWrite), I2C_WRITE);
        if (ret >= 0) {
            MDELAY(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_MODE_SEL, &mode, 1);
                if ((ret == 0) && (0x40 == (mode & 0x7f))) {
                    MDELAY(50);
                    return 0;
                } else
                    MDELAY(FACTORY_TEST_DELAY);
            }
        }

        MDELAY(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        bl_log_trace("Enter factory mode fail\r\n");
        return -1;
    }

    return 0;

}

static int btl_start_scan(unsigned char value)
{
    int ret = 0;
    unsigned char cmdWrite[2] = {0};
	unsigned char val = 0;
    int times = 0;
	int j = 0;

	cmdWrite[0] = BTL_REG_MODE_SEL;
	cmdWrite[1] = value;

    while (times++ < FACTORY_SCAN_RETRIES) {
        ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmdWrite, sizeof(cmdWrite), I2C_WRITE); 
        if (ret == 0) {
            MDELAY(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
 				ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_MODE_SEL, &val, 1);
                if ((ret == 0) && ((val & 0x84)== 0x00)) {
                    MDELAY(50);
                    return 0;
                } else
                    MDELAY(FACTORY_TEST_DELAY);
            }
        }

        MDELAY(50);
    }

    if (times >= FACTORY_SCAN_RETRIES) {
        bl_log_trace("scan timeout\r\n");
        return -1;
    }

    return 0;
}

static int btl_cali_cb(unsigned char mode)
{
    int ret = 0;
    int i = 0;
    int j = 0;
	unsigned char cmdWrite[2] = {0};

    cmdWrite[0] = BTL_REG_SC_CB_CALI;
	cmdWrite[1] = mode;
	
    for (i = 0; i < FACTORY_CALI_RETRIES; i++) {
		ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmdWrite, sizeof(cmdWrite), I2C_WRITE);
        if (ret == 0) {
            MDELAY(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
 				ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_CB_CALI, &mode, 1);
                if ((ret == 0) && (0x00 == mode)) {
                    MDELAY(50);
                    return 0;
                } else
                    MDELAY(FACTORY_TEST_DELAY);
            }
        }

        MDELAY(50);
    }

    if (i >= FACTORY_CALI_RETRIES) {
        bl_log_trace("Enter factory mode fail\r\n");
        return -1;
    }

    return 0;
}

static unsigned char btl_get_rx_chn_num(void)
{

    int ret = 0;
    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_RX_CHN, &rxChnNum, 1);
    if(ret < 0) {
        bl_log_trace("read rx channel num fail!");
    } else {
        bl_log_trace("read rx channel num success:%d", rxChnNum);
    }
    return rxChnNum;
}

static int btl_get_factory_id(void)
{

    int ret = 0;
    unsigned char factoryID = 0;
	ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_FACTORY_ID, &factoryID, 1);
	if(ret < 0) {
		bl_log_trace("read factory ID fail!");
	} else {
		bl_log_trace("read factory ID success:%d", factoryID);
	}
	return factoryID;
}

static void btl_get_firmware_version(unsigned char * version)
{
    int ret = 0;
    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_VERSION, version, VERSION_SIZE);
    if(ret < 0) {
        bl_log_trace("read firmware version ID fail!");
    } else {
        bl_log_trace("read firmware version ID success:0x%x-0x%x-0x%x!", version[0], version[1], version[2]);
    }
}

static void btl_get_project_code(unsigned char * code)
{
    int ret = 0;
    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_PROJECT_CODE, code, PROJECT_CODE_SIZE);
    if(ret < 0) {
        bl_log_trace("read firmware version ID fail!");
    } else {
        bl_log_trace("read firmware version ID success:%s!", code);
    }
}

static int btl_factory_rst_test(void)
{
    int ret = 0;
	unsigned char mode = 0;

    ret = btl_enter_factory_mode();
	if(ret == 0) {
        bl_ts_reset_wakeup();
		MDELAY(50);
	    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_MODE_SEL, &mode, 1);
        if((mode == 0) && (ret == 0)) {
            ret = 0;
        } else {            
            ret = -1;
        }
    }
    if(ret == 0) {
        bl_log_trace("reset test PASS!");
    } else {
        bl_log_trace("reset test NG!");
    }
}

static int btl_factory_int_test(void)
{
    int ret = 0;
	unsigned char cmd[2] = {BTL_REG_INT_TEST, 0x02};
    intTestMode = 1;
    ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
	if(ret == 0) {
        MDELAY(50);
        if(intFlag == 2) {
            ret = 0;
        } else {
            ret = -1;
        }
    }
    if(ret == 0) {
        bl_log_trace("int test PASS!");
    } else {
        bl_log_trace("int test NG!");
    }
	intFlag = 0;
	intTestMode = 0;
}

static void btl_sc_cb_to_slope(unsigned char *cb, unsigned char rx_num, unsigned char key_num, unsigned char* cbCali, short *result)
{
    int i = 0;
	unsigned char channel = rx_num + key_num;
	unsigned char oddPosCnt = 0;
	unsigned char oddNegCnt = 0;
	int oddPosSum = 0;
	int oddNegSum = 0;
	int oddAvg = 0;
	unsigned char evenPosCnt = 0;
	unsigned char evenNegCnt = 0;
	int evenAvg = 0;
	int evenPosSum = 0;
	int evenNegSum = 0;
	int diff = 0;
	int evenMinDiff = 0;
	int oddMinDiff = 0;
	int tmpCb[RX_NUM_MAX] = {0x00};
	
    //统计奇偶分组数据正负数据个数以及累加和
    for(i = 0; i < rx_num; i++)
    {
        diff = cb[i] - cbCali[i];
        if((i % 2) == 0)
        {
            if(diff >=0)
            {
                evenPosCnt++;
                evenPosSum += diff;
            }
            else
            {
                evenNegCnt++;
                evenNegSum += diff;
            }
        }
        else
        {
            if(diff >=0)
            {
                oddPosCnt++;
                oddPosSum += diff;
            }
            else
            {
                oddNegCnt++;
                oddNegSum += diff;
            }
        }
    }
		
    //根据上面统计值计算奇偶分组数据的平均值
    if(evenPosCnt > evenNegCnt)
    {
        evenAvg = evenPosSum / evenPosCnt;
    }
    else
    {
        evenAvg = evenNegSum / evenNegCnt;
    }
    
    if(oddPosCnt > oddNegCnt)
    {
        oddAvg = oddPosSum / oddPosCnt;
    }
    else
    {
        oddAvg = oddNegSum / oddNegCnt;
    }

    //奇偶分组数据对应减去上面计算的均值
    for(i = 0; i < rx_num; i++)
    {
        if((i % 2) == 0)
        {
            tmpCb[i] = cb[i] - evenAvg;
        }
        else
        {
            tmpCb[i] = cb[i] - oddAvg;
        }
    }

    for(i = 0; i < key_num; i++)
    {
        tmpCb[rx_num + i] = cb[rx_num + i];
    }
	
    //根据奇偶分组数据计算最小diff
    for(i = 0; i < rx_num; i++)
    {
        diff = tmpCb[i] - cbCali[i];
        if((i % 2) == 0)
        {
            if(i == 0)
            {
                evenMinDiff = diff;

            }
			if(ABS(diff) < ABS(evenMinDiff))
			{
				evenMinDiff = diff;
			}
        }
        else
        {
            if(i == 1)
            {
                oddMinDiff = diff;
            }
			if(ABS(diff) < ABS(oddMinDiff))
			{
				oddMinDiff = diff;
			}
        }
    }
	
    //根据奇偶分组数据统计落在threshHold区域的数据进行统计并且计算均值
    evenPosCnt = 0;
	evenPosSum = 0;
	oddPosCnt = 0;
	oddPosSum = 0;
    for(i = 0; i < rx_num; i++)
    {
        if((i % 2) == 0)
        {
            diff = tmpCb[i] - cbCali[i] - evenMinDiff;
            if(ABS(diff) < BTL_RANGE_OF_KEY_CB)
            {
                evenPosCnt = evenPosCnt + 1;
                evenPosSum += tmpCb[i] - evenMinDiff;
            }
        }
        else
        {
            diff = tmpCb[i] - cbCali[i] - oddMinDiff;
            if(ABS(diff) < BTL_RANGE_OF_KEY_CB)
            {
                oddPosCnt = oddPosCnt + 1;
                oddPosSum += tmpCb[i] - oddMinDiff;
            }
        }
    }

    //根据奇偶分组数据计算均值
    if(0x00 == evenPosCnt)
	{
        evenAvg = tmpCb[0];
	}
	else
	{
        evenAvg = evenPosSum / evenPosCnt;
	}

    if(0x00 == oddPosCnt)
	{
        oddAvg = tmpCb[0];
	}
	else
	{
        oddAvg = oddPosSum / oddPosCnt;
	}

	//所有通道CB根据奇偶分组数据分别计算斜率输出
    for(i = 0; i < channel; i++)
    {
        if((i % 2) == 0)
        {
            result[i] = tmpCb[i] * MULTIFLYING_POWER_VALUE / evenAvg;
        }
        else
        {
            result[i] = tmpCb[i] * MULTIFLYING_POWER_VALUE / oddAvg;
        }
    }
}

static int btl_sc_read_cb_data(unsigned char rxNum, unsigned char keyNum, unsigned char* cb, unsigned char* cbCali, short* base)
{
    int i = 0;
	int ret = 0;
	int byteNum = keyNum + rxNum;
    short tempCb[RX_NUM_MAX] = {0};
	
    ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_CB, cb, byteNum);
    if(ret < 0)
    {
    	bl_log_trace("get cb value fail\r\n");
    	goto test_err;
    }
	
    for (i = 0; i < byteNum; i++) 
    {		
    	tempCb[i] = cb[i]; 
    }
    
    btl_sc_cb_to_slope(cb, rxNum, keyNum, cbCali, tempCb);
	
    for (i = 0; i < byteNum; i++) {
        tempCb[i] = tempCb[i] - base[i];
    }
	
test_err:
    return ret;
}

static void btl_sc_get_cb_base_value(unsigned char *cbCali)
{
    btl_sc_cb_to_slope(cbCali, rxChnNum, 0, cbCali, CBBase);
}

static void btl_cb_test(void)
{
    int ret = 0;

	ret = btl_enter_factory_mode();
    if (ret < 0) {
        goto err;
    }
	
    ret = btl_cali_cb(0x02);
    if(ret < 0)
    {
        goto err;
    }

    ret = btl_sc_read_cb_data(rxChnNum, 0, CB, CBCali, CBBase);
    if(ret < 0)
    {
        goto err;
    }
err:
	bl_ts_reset_wakeup();	
	return;
}

static void btl_rawdata_test(void)
{
	int ret = 0;
	int i = 0;
	int byteNum = 2 * rxChnNum;
	unsigned char data[RX_NUM_MAX * 2] = {0x00};

    ret = btl_enter_factory_mode();
    if (ret < 0) {
        goto err;
    }

    ret = btl_start_scan(0xc0);
    if(ret < 0)
    {
        goto err;
    }

	ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_RAWDATA, data, byteNum);
	if(ret < 0)
	{
		goto err;	  
	}

	for (i = 0; i < byteNum; i = i + 2) {
		rawData[i >> 1] = (unsigned short)((data[i+1] << 8) + data[i]);
	}
err:
	bl_ts_reset_wakeup();	
	return;
}

void btl_short_test(void)
{
	int ret = 0;
	int i = 0;
	unsigned char data[RX_NUM_MAX] = {0x00};
	int byteNum = 0;

	byteNum = 2 * rxChnNum;
	
    ret = btl_enter_factory_mode();
    if (ret < 0) {
        goto err;
    }
	
    ret = btl_start_scan(0xc4);
    if(ret < 0)
    {
        goto err;
    }
    MDELAY(BTL_WAIT_SHORT_TEST_TIME);
	
	ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_SHORT, data, byteNum);
	if(ret < 0)
	{
		goto err;
	}
	for (i = 0; i < byteNum; i = i + 2) {
		adcShort[i >> 1] = (unsigned short)((data[i+1] << 8) + data[i]);
	}
err:
    bl_ts_reset_wakeup();	
    return;
}

static void btl_set_open_test_para(unsigned char vol, unsigned char cf)
{

    btl_set_vol(vol);
    btl_set_cf(cf);
}

void btl_open_test(void)
{
	int ret = 0;
	int i = 0;
	unsigned char data[RX_NUM_MAX] = {0x00};
	int byteNum = 0;

	byteNum = 2 * rxChnNum;
	
    ret = btl_enter_factory_mode();
    if (ret < 0) {
        goto err;
    }

    btl_set_open_test_para(0x07, 0x7f);

    ret = btl_cali_cb(0x02);
    if(ret < 0)
    {
        goto err;
    }
	
    ret = btl_start_scan(0xc2);
    if(ret < 0)
    {
        goto err;
    }
    MDELAY(BTL_WAIT_OPEN_TEST_TIME);
	
	ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_SHORT, data, byteNum);
	if(ret < 0)
	{
		goto err;
	}
	for (i = 0; i < byteNum; i = i + 2) {
		adcOpen[i >> 1] = (unsigned short)((data[i+1] << 8) + data[i]);
	}
err:
    bl_ts_reset_wakeup();	
    return;
}

static int btl_throw_frame(void)
{
    int i = 0;
	int ret = 0;
	for(i = 0; i < BTL_THROW_FRAME_CNT; i++)
	{
		ret = btl_start_scan(0xc0);
	}
    return ret;
}

static int btl_get_base(unsigned short * base)
{
    int i = 0;
    int j = 0;
    int ret = 0;
    int sumBase[RX_NUM_MAX] = {0x0};
	int byteNum = 2 * rxChnNum;
	unsigned char data[RX_NUM_MAX] = {0x00};
    for(i = 0; i < BTL_BASE_FRAME_CNT; i++) {
        ret = btl_start_scan(0xc0);
        if(ret == 0)
        {
            ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_RAWDATA, data, byteNum);
            if(ret == 0)
            {
                for(j = 0; j < rxChnNum; j++) {
					sumBase[i] += data[i];
                }
                
            }
        }
    }

    for(i = 0; i < rxChnNum; i++) {
        base[i] = (short)(sumBase[i] / BTL_BASE_FRAME_CNT);
    }
    return ret;
}

static short btl_get_diff(unsigned short * base, short * diff)
{
    int i = 0;
    int ret = 0;
    unsigned char data[RX_NUM_MAX] = {0};
    int byteNum = 2 * rxChnNum;
    short subValue[RX_NUM_MAX] = {0};

    for(i = 0; i < BTL_DIFF_FRAME_CNT; i++) {
        ret = btl_start_scan(0xc0);
        if(ret == 0)
        {
            ret = bl_read_fw(CTP_SLAVE_ADDR, BTL_REG_SC_RAWDATA, data, byteNum);
            if(ret == 0)
            {
                for(i = 0; i < rxChnNum; i++) {
					subValue[i] = data[i] - base[i];
                    diff[i] += subValue[i];
                }
                
            }
        }
    }    
    return ret;
}

void btl_diff_test(void)
{
    int ret = 0;
	unsigned short base[RX_NUM_MAX] = {0x00};
    ret = btl_enter_factory_mode();
    if (ret < 0) {
        goto err;
    }

    ret = btl_throw_frame();
    if(ret < 0) {
        goto err;
    }

	ret = btl_get_base(base);
    if(ret < 0) {
        goto err;
    }

    ret = btl_get_diff(base, diff);
    if(ret < 0) {
        goto err;
    }
err:
	bl_ts_reset_wakeup();	
    return;
}
//#endif
