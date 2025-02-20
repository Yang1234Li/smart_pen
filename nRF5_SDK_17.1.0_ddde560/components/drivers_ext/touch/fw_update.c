#include "TP_ts.h"
#include "bl_fw.h"

#ifdef CTP_USE_SW_I2C
int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len,unsigned char rw)
{
	int ret;
		
    switch(rw)
    {
        case I2C_WRITE:
			ret = CTP_FLASH_I2C_WRITE(i2c_addr, buf, len);
			break;
		case I2C_READ:
			ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
			break;			
    }
	if(ret){
		bl_log_trace("bl_i2c_transfer:i2c transfer error___\n");
		return -1;
	}

	return 0;
}

int bl_read_fw(unsigned char i2c_addr,unsigned char reg_addr, unsigned char *buf, int len)
{
	int ret;

    ret = CTP_FLASH_I2C_WRITE(i2c_addr, &reg_addr, 1);
	if(ret)
    {
		goto IIC_COMM_ERROR;
    }
	ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
	if(ret)
    {
		goto IIC_COMM_ERROR;
    }	

IIC_COMM_ERROR:	
	if(ret){
		bl_log_trace("bl_read_fw:i2c transfer error___\n");
		return -1;
	}

	return 0;
}
#endif

#ifdef CTP_USE_HW_I2C
int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len,unsigned char rw)
{
	int ret;
    switch(rw)
    {
        case I2C_WRITE:
			ret = CTP_FLASH_I2C_WRITE(i2c_addr, buf, len);
			break;
		case I2C_READ:
			ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
			break;			
    }
	if(ret<0){
		bl_log_trace("bl_i2c_transfer:i2c transfer error___\n");
		return -1;
	}

	return 0;
}

int bl_read_fw(unsigned char i2c_addr,unsigned char reg_addr, unsigned char *buf, int len)
{
	int ret;

	ret = CTP_FLASH_I2C_WRITE(i2c_addr, &reg_addr, 1);
	UDELAY(100);
	if(ret<0)
       {
		goto IIC_COMM_ERROR;
       }
	ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
	if(ret<0)
       {
		goto IIC_COMM_ERROR;
       }	

IIC_COMM_ERROR:	
	if(ret<0){
		bl_log_trace("bl_read_fw:i2c transfer error___\n");
		return -1;
	}		
	return 0;
}
#endif

int bl_get_chip_id(unsigned char *buf)
{

	unsigned char cmd[3];
	int ret = 0x00;
	bl_log_trace("bl_get_chip_id\n");

    cmd[0] = RW_REGISTER_CMD;
    cmd[1] = ~cmd[0];
    cmd[2] = CHIP_ID_REG;
    
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd,3,I2C_WRITE);
    if(ret < 0){
    	bl_log_trace("bl_get_chip_id:i2c write flash error___\n");
    	goto GET_CHIP_ID_ERROR;
    }
    
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, buf,1,I2C_READ); 
    if(ret < 0){
    	bl_log_trace("bl_get_chip_id:i2c read flash error___\n");
    	goto GET_CHIP_ID_ERROR;
    }
    
    bl_log_trace("bl_get_chip_id:buf = %x\n",*buf);

GET_CHIP_ID_ERROR:
	return ret;
}

int bl_get_chip_sub_id(unsigned char *buf)
{

	unsigned char cmd[6];
	int ret = 0x00;
	bl_log_trace("bl_get_chip_sub_id\n");

    cmd[0] = 0x0e;
    cmd[1] = ~cmd[0];
    cmd[2] = 0x02;
	cmd[3] = 0x88;
    cmd[4] = 0x02;
	cmd[5] = 0x88;
	
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd,sizeof(cmd),I2C_WRITE);
    if(ret < 0){
    	bl_log_trace("bl_get_chip_sub_id:i2c write flash error___\n");
    	goto GET_CHIP_ID_ERROR;
    }
    
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, buf,1,I2C_READ); 
    if(ret < 0){
    	bl_log_trace("bl_get_chip_sub_id:i2c read flash error___\n");
    	goto GET_CHIP_ID_ERROR;
    }
    
    bl_log_trace("bl_get_chip_sub_id:buf = %x\n",*buf);

GET_CHIP_ID_ERROR:
	return ret;
}

#if((UPDATE_MODE == I2C_UPDATE_MODE_OLD) || (UPDATE_MODE == I2C_UPDATE_MODE_NEW))
#if(UPDATE_MODE == I2C_UPDATE_MODE_NEW)
void bl_enter_update_with_i2c(void)
{
    unsigned char buf[4] = {0x63, 0x75, 0x69, 0x33};
	unsigned char cmd[16] = {0};
	int i = 4;
	int ret = 0;
    #if defined(RESET_PIN_WAKEUP)
	bl_ts_reset_wakeup();
	#endif
	for(i = 0; i < 4; i++) 
    {
        cmd[4 * i] = buf[0];
		cmd[4 * i + 1] = buf[1];
		cmd[4 * i + 2] = buf[2];
		cmd[4 * i + 3] = buf[3];
    }

	ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
	if(ret < 0)
    {
        bl_log_trace("bl_enter_update_with_i2c failed:send i2c cmd error___\r\n");
        goto error;
    }
	MDELAY(50);

error:
	return ;
}
#endif

#if(UPDATE_MODE == I2C_UPDATE_MODE_OLD)
void bl_enter_update_with_i2c(void)
{
    unsigned char cmd[200] = {0x00};
	int i = 0;
	int ret = 0;
    #if defined(RESET_PIN_WAKEUP)
		bl_ts_reset_wakeup();
    #endif

	for(i = 0; i < sizeof(cmd); i += 2)
    {
        cmd[i] = 0x5a;
		cmd[i + 1] = 0xa5;
    }

	ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
	if(ret < 0)
    {
        bl_log_trace("bl_enter_update_with_i2c failed:send 5a a5 error___\n");
        goto error;
    }
	MDELAY(50);

error:
	return ;
}
#endif

void bl_exit_update_with_i2c(void)
{
    int ret = 0;
	unsigned char cmd[2] = {0x5a, 0xa5};
    MDELAY(20);
	ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
	if(ret < 0)
    {
        bl_log_trace("bl_exit_update_with_i2c failed:send 5a a5 error___\n");
    }

	MDELAY(20);
	#if defined(RESET_PIN_WAKEUP)
	bl_ts_reset_wakeup();
	MDELAY(30);
	#endif
	return;
}
#endif

#if(UPDATE_MODE == INT_UPDATE_MODE)
void bl_enter_update_with_int(void)
{
    bl_ts_set_intmode(0);
    bl_ts_set_intup(0);
	#if defined(RESET_PIN_WAKEUP)
	bl_ts_reset_wakeup();
    #endif
	MDELAY(50);
}

void bl_exit_update_with_int(void)
{
    MDELAY(20);
	bl_ts_set_intup(1);
	MDELAY(20);
	bl_ts_set_intmode(1);
    #if defined(RESET_PIN_WAKEUP)
	bl_ts_reset_wakeup();
    #endif
}
#endif

int bl_get_prj_id(unsigned char *buf)
{
	bl_log_trace("bl_get_prj_id\n");
	return bl_read_fw(CTP_SLAVE_ADDR,BL_PRJ_ID_REG, buf, 1);    
}

int bl_get_fwArgPrj_id(unsigned char *buf)
{
	bl_log_trace("bl_get_fwArgPrj_id\n");
	return bl_read_fw(CTP_SLAVE_ADDR,BL_FWVER_PJ_ID_REG, buf, 3);
}


#ifdef BL_UPDATE_FIRMWARE_ENABLE
static int bl_get_fw_checksum(unsigned short *fw_checksum)
{
	unsigned char buf[3];
	unsigned char checksum_ready = 0;
	int retry = 5;
	int ret = 0x00;

	bl_log_trace("bl_get_fw_checksum\n");

	buf[0] = CHECKSUM_CAL_REG;
	buf[1] = CHECKSUM_CAL;
	ret = bl_i2c_transfer(CTP_SLAVE_ADDR, buf,2,I2C_WRITE);
	if(ret < 0){
		bl_log_trace("bl_get_fw_checksum:write checksum cmd error___\n");
		return -1;
	}

	while((retry--) && (checksum_ready != CHECKSUM_READY)){

		MDELAY(50);
		ret = bl_read_fw(CTP_SLAVE_ADDR,CHECKSUM_REG, buf, 3);
		if(ret < 0){
			bl_log_trace("bl_get_fw_checksum:read checksum error___\n");
			return -1;
		}

		checksum_ready = buf[0];
	}
	
	if(checksum_ready != CHECKSUM_READY){
		bl_log_trace("bl_get_fw_checksum:read checksum fail___\n");
		return -1;
	}
	*fw_checksum = (buf[1]<<8)+buf[2];

	return 0;
}

static void bl_get_fw_bin_checksum_for_self_ctp(unsigned char *fw_data,unsigned short *fw_bin_checksum, int fw_size)
{
	int i = 0;
	int temp_checksum = 0x0;

    for(i = 0; i < BL_ARGUMENT_BASE_OFFSET; i++)
    {
        temp_checksum += fw_data[i];
    }
    for(i = BL_ARGUMENT_BASE_OFFSET; i < BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET; i++)
    {
    	temp_checksum += fw_data[i];
    }
    for(i = BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET; i < BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET + 4; i++)
    {
    	temp_checksum += fw_data[i];
    }
    for(i = BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET + 4; i < fw_size; i++)
    {
    	temp_checksum += fw_data[i];
    }

    for(i = fw_size; i < MAX_FLASH_SIZE; i++)
    {
    	temp_checksum += 0xff;
    }
    
    *fw_bin_checksum = temp_checksum & 0xffff;
}

static int bl_erase_flash(void)
{
	unsigned char cmd[2];
	
	bl_log_trace("bl_erase_flash\n");
	
	cmd[0] = ERASE_ALL_MAIN_CMD; 
	cmd[1] = ~cmd[0];

	return bl_i2c_transfer(BL_FLASH_I2C_ADDR,cmd, 0x02,I2C_WRITE);	
}

static int bl_write_flash_no_blm18(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
	unsigned char cmd_buf[6+FLASH_WSIZE];
	unsigned int flash_end_addr;
	int ret;
		
	bl_log_trace("bl_write_flash_no_blm18\n");
	
	if(!len){
		bl_log_trace("___write flash len is 0x00,return___\n");
		return -1;	
	}

	flash_end_addr = flash_start_addr + len -1;

	if(flash_end_addr >= MAX_FLASH_SIZE){
		bl_log_trace("___write flash end addr is overflow,return___\n");
		return -1;	
	}

	cmd_buf[0] = cmd;
	cmd_buf[1] = ~cmd;
	cmd_buf[2] = flash_start_addr >> 0x08;
	cmd_buf[3] = flash_start_addr & 0xff;
	cmd_buf[4] = flash_end_addr >> 0x08;
	cmd_buf[5] = flash_end_addr & 0xff;

	memcpy(&cmd_buf[6],buf,len);

	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,cmd_buf, len+6,I2C_WRITE);	
	if(ret < 0){
		bl_log_trace("i2c transfer error___\n");
		return -1;
	}

	return 0;
}

static int bl_write_flash_blm18(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
	unsigned char cmd_buf[8+FLASH_WSIZE];
	unsigned int flash_end_addr;
	int ret;
		
	bl_log_trace("bl_write_flash_blm18\n");
	
	if(!len){
		bl_log_trace("___write flash len is 0x00,return___\n");
		return -1;	
	}

	flash_end_addr = flash_start_addr + len -1;

	if(flash_end_addr >= MAX_FLASH_SIZE){
		bl_log_trace("___write flash end addr is overflow,return___\n");
		return -1;	
	}

	cmd_buf[0] = cmd;
	cmd_buf[1] = ~cmd;
	cmd_buf[2] = flash_start_addr >> 16;
	cmd_buf[3] = flash_start_addr >> 8;
	cmd_buf[4] = flash_start_addr & 0xff;
	cmd_buf[5] = flash_end_addr >> 16;
	cmd_buf[6] = flash_end_addr >> 8;
	cmd_buf[7] = flash_end_addr & 0xff;

	memcpy(&cmd_buf[8],buf,len);

	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,cmd_buf, len+8,I2C_WRITE);	
	if(ret < 0){
		bl_log_trace("i2c transfer error___\n");
		return -1;
	}

	return 0;
}

static int bl_write_flash(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
    if(MAX_FLASH_SIZE > 0x10000)
		return bl_write_flash_blm18(cmd, flash_start_addr, buf, len);
	else
	    return bl_write_flash_no_blm18(cmd, flash_start_addr, buf, len);
}

static int bl_read_flash_no_blm18(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
	char ret =0;
	unsigned char cmd_buf[6];
	unsigned int flash_end_addr;

	flash_end_addr = flash_start_addr + len -1;
	cmd_buf[0] = cmd;
	cmd_buf[1] = ~cmd;
	cmd_buf[2] = flash_start_addr >> 0x08;
	cmd_buf[3] = flash_start_addr & 0xff;
	cmd_buf[4] = flash_end_addr >> 0x08;
	cmd_buf[5] = flash_end_addr & 0xff;
	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,cmd_buf,6,I2C_WRITE);
	if(ret < 0)
	{
	    bl_log_trace("bl_read_flash_no_blm18:i2c transfer write error\n");
		return -1;
	}
	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,buf,len,I2C_READ);
	if(ret < 0)
	{
	    bl_log_trace("bl_read_flash_no_blm18:i2c transfer read error\n");
		return -1;
	}

	return 0;
}

static int bl_read_flash_blm18(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
	char ret =0;
	unsigned char cmd_buf[8];
	unsigned int flash_end_addr;

	flash_end_addr = flash_start_addr + len -1;
	cmd_buf[0] = cmd;
	cmd_buf[1] = ~cmd;
	cmd_buf[2] = flash_start_addr >> 16;
	cmd_buf[3] = flash_start_addr >> 8;
	cmd_buf[4] = flash_start_addr & 0xff;
	cmd_buf[5] = flash_end_addr >> 16;
	cmd_buf[6] = flash_end_addr >> 8;
	cmd_buf[7] = flash_end_addr & 0xff;
	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,cmd_buf,8,I2C_WRITE);
	if(ret < 0)
	{
	    bl_log_trace("bl_read_flash_blm18:i2c transfer write error\n");
		return -1;
	}
	ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR,buf,len,I2C_READ);
	if(ret < 0)
	{
	    bl_log_trace("bl_read_flash_blm18:i2c transfer read error\n");
		return -1;
	}

	return 0;
}

static int bl_read_flash(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
    if(MAX_FLASH_SIZE > 0x10000)
	    return bl_read_flash_blm18(cmd, flash_start_addr, buf, len);
	else
	    return bl_read_flash_no_blm18(cmd, flash_start_addr, buf, len);
}
static int bl_download_fw_for_self_ctp(unsigned char *pfwbin,int specificArgAddr, int fwsize)
{
	unsigned int i;
	unsigned int size,len;
	unsigned int addr;
    unsigned char verifyBuf[4] = {0xff, 0xff, 0xff, 0xff};
	bl_log_trace("bl_download_fw_for_self_ctp\n");
	
	verifyBuf[2] = pfwbin[BL_ARGUMENT_BASE_OFFSET+VERTIFY_START_OFFSET+2];
	verifyBuf[3] = pfwbin[BL_ARGUMENT_BASE_OFFSET+VERTIFY_START_OFFSET+3];	
	bl_log_trace("bl_download_fw:verifyBuf = %x %x %x %x\n",verifyBuf[0],verifyBuf[1],verifyBuf[2],verifyBuf[3]);
	if(bl_erase_flash()){
		bl_log_trace("___erase flash fail___\n");
		return -1;
	}

	MDELAY(50);

    //Write data before BL_ARGUMENT_BASE_OFFSET
	for(i=0;i< BL_ARGUMENT_BASE_OFFSET;)
	{
		size = BL_ARGUMENT_BASE_OFFSET - i;
		if(size > FLASH_WSIZE){
			len = FLASH_WSIZE;
		}else{
			len = size;
		}

		addr = i;
	
		if(bl_write_flash(WRITE_MAIN_CMD,addr, &pfwbin[i],len)){
			return -1;
		}
		i += len;
		MDELAY(5);
	}

    //Write the data from BL_ARGUMENT_BASE_OFFSET to VERTIFY_START_OFFSET
    for(i=BL_ARGUMENT_BASE_OFFSET;i< (VERTIFY_START_OFFSET+BL_ARGUMENT_BASE_OFFSET);)
    {
    	size = VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET - i;
    	if(size > FLASH_WSIZE){
    		len = FLASH_WSIZE;
    	}else{
    		len = size;
    	}
    
    	addr = i;
    
    	if(bl_write_flash(WRITE_MAIN_CMD,addr, &pfwbin[i+specificArgAddr-BL_ARGUMENT_BASE_OFFSET],len)){
    		return -1;
    	}
    	i += len;
    	MDELAY(5);
    }

    //Write the four bytes verifyBuf from VERTIFY_START_OFFSET
    for(i=(VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET);i< (VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET + sizeof(verifyBuf));)
    {
    	size = VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET + sizeof(verifyBuf) - i;
    	if(size > FLASH_WSIZE){
    		len = FLASH_WSIZE;
    	}else{
    		len = size;
    	}
    
    	addr = i;
    
    	if(bl_write_flash(WRITE_MAIN_CMD,addr, &verifyBuf[i-VERTIFY_START_OFFSET-BL_ARGUMENT_BASE_OFFSET],len)){
    		return -1;
    	}
    	i += len;
    	MDELAY(5);
    }

    //Write data after verityBuf from VERTIFY_START_OFFSET + 4
    for(i=(BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET + 4);i< fwsize;)
    {
    	size = fwsize - i;
    	if(size > FLASH_WSIZE){
    		len = FLASH_WSIZE;
    	}else{
    		len = size;
    	}
    
    	addr = i;
    
    	if(bl_write_flash(WRITE_MAIN_CMD,addr, &pfwbin[i],len)){
    		return -1;
    	}
    	i += len;
    	MDELAY(5);
    }

	return 0;	
}

static int bl_read_flash_vertify(unsigned char *pfwbin)
{
	unsigned char cnt = 0;
	int ret = 0;
	unsigned char vertify[2] = {0};
	unsigned char vertify1[2] = {0};
	
	memcpy(vertify,&pfwbin[BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET],sizeof(vertify));
	bl_log_trace("bl_read_flash_vertify: vertify:%x %x\n",vertify[0],vertify[1]);

	SET_WAKEUP_LOW;
	while(cnt < 3)
	{
	    cnt++;
        ret = bl_read_flash(READ_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify1, sizeof(vertify1));
		if(ret < 0)
		{
			bl_log_trace("bl_write_flash_vertify: read fail\n");
			continue;
		}

		if(memcmp(vertify, vertify1, sizeof(vertify)) == 0)
		{
			ret = 0;
			break;
		}
		else
        {
            ret = -1;
        }
	}
	SET_WAKEUP_HIGH;
	return ret;
}

static int bl_write_flash_vertify(unsigned char *pfwbin)
{
	unsigned char cnt = 0;
	int ret = 0;
	unsigned char vertify[2] = {0};
	unsigned char vertify1[2] = {0};

	memcpy(vertify,&pfwbin[BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET],sizeof(vertify));
	bl_log_trace("bl_write_flash_vertify: vertify:%x %x\n",vertify[0],vertify[1]);

	SET_WAKEUP_LOW;
	while(cnt < 3)
	{
	    cnt++;
	    ret = bl_write_flash(WRITE_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify, sizeof(vertify));
		if(ret < 0)
		{
			bl_log_trace("bl_write_flash_vertify: write fail\n");
			continue;
		}
		
		MDELAY(10);

        ret = bl_read_flash(READ_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify1, sizeof(vertify1));
		if(ret < 0)
		{
			bl_log_trace("bl_write_flash_vertify: read fail\n");
			continue;
		}

		if(memcmp(vertify, vertify1, sizeof(vertify)) == 0)
		{
			ret = 0;
			break;
		}
		else
        {
            ret = -1;
        }
	}
	SET_WAKEUP_HIGH;
	return ret;
}

static int bl_update_flash_for_self_ctp(unsigned char update_type, unsigned char *pfwbin,int fwsize)
{
	int retry = 0;
	int ret = 0;
	unsigned short fw_checksum = 0x0;
	unsigned short fw_bin_checksum = 0x0;
	retry =3;
	while(retry--)
	{
		SET_WAKEUP_LOW;

	    ret = bl_download_fw_for_self_ctp(pfwbin,BL_ARGUMENT_BASE_OFFSET,fwsize);

		if(ret<0)
		{
			bl_log_trace("bl_update_flash_for_self_ctp:bl_download_fw_for_self_ctp error retry=%d\n",retry);
			continue;
		}
		
		MDELAY(50);

		SET_WAKEUP_HIGH;
	
	    bl_get_fw_bin_checksum_for_self_ctp(pfwbin,&fw_bin_checksum, fwsize);
	    ret = bl_get_fw_checksum(&fw_checksum);
		fw_checksum -= 0xff;
		bl_log_trace("bl_download_fw_for_self_ctp:fw checksum = 0x%x,fw_bin_checksum =0x%x\n",fw_checksum, fw_bin_checksum);

		
		if((ret < 0) || ((update_type == FW_ARG_UPDATE)&&(fw_checksum != fw_bin_checksum)))
		{
			bl_log_trace("bl_download_fw_for_self_ctp:bl_get_fw_checksum error");
			continue;
		}

		if((update_type == FW_ARG_UPDATE)&&(fw_checksum == fw_bin_checksum))
		{
            ret = bl_write_flash_vertify(pfwbin);
			if(ret < 0)
				continue;
		}
		break;
	}

	if(retry < 0)
	{
		bl_log_trace("bl_download_fw_for_self_ctp error\n");
		return -1;
	}

	bl_log_trace("bl_download_fw_for_self_ctp success___\n");	

	return 0;
}

static unsigned char choose_update_type_for_self_ctp(unsigned char isBlank, unsigned char* fw_data, unsigned short fwChecksum, unsigned short fwBinChecksum)
{
    unsigned char update_type = NONE_UPDATE;
    if(isBlank)
	{
		update_type = FW_ARG_UPDATE;
		bl_log_trace("Update case 0:FW_ARG_UPDATE\n");
	}
	else
	{
        if(fwChecksum != fwBinChecksum)
	    {
            update_type = FW_ARG_UPDATE;
		    bl_log_trace("Update case 1:FW_ARG_UPDATE\n");
	    }
	    else
	    {
            update_type = NONE_UPDATE;
            bl_log_trace("Update case 2:NONE_UPDATE\n");
	    }
	}
    return update_type;
}

static int bl_update_fw_for_self_ctp(unsigned char* fw_data, int fw_size)
{
	int ret = 0x00;
	unsigned char isBlank = 0x0;    //Indicate the IC have any firmware
	unsigned short fw_checksum = 0x0;  //The checksum for firmware in IC
	unsigned short fw_bin_checksum = 0x0;  //The checksum for firmware in file
	unsigned char update_type = NONE_UPDATE;  
    bl_log_trace("bl_update_fw_for_self_ctp start\n");	

    bl_get_fw_bin_checksum_for_self_ctp(fw_data, &fw_bin_checksum, fw_size);
    ret = bl_get_fw_checksum(&fw_checksum);
    if((ret < 0) || (fw_checksum != fw_bin_checksum)){
    	bl_log_trace("bl_update_fw_for_self_ctp:Read checksum fail fw_checksum = %x\n",fw_checksum);
    	isBlank = 1;
    } else {
        isBlank = 0;
    }
    bl_log_trace("bl_update_fw_for_self_ctp:isBlank = %d fw_checksum = 0x%x,fw_bin_checksum = 0x%x___\n",isBlank, fw_checksum, fw_bin_checksum);
    
    update_type = choose_update_type_for_self_ctp(isBlank, fw_data, fw_checksum, fw_bin_checksum);

//Step 8:Start Update depend condition
    if(update_type != NONE_UPDATE)
    {
        ret = bl_update_flash_for_self_ctp(update_type, fw_data, fw_size);
        if(ret < 0)
        {
            bl_log_trace("bl_update_fw_for_self_ctp:bl_update_flash failed\n");
        }
    }
	bl_log_trace("bl_update_fw_for_self_ctp exit\n");
	return ret;
}

#ifdef BL_AUTO_UPDATE_FARMWARE
int bl_auto_update_fw(void)
{
	int ret = 0;
    unsigned int fwLen = sizeof(fwbin);

	bl_log_trace("bl_auto_update_fw:fwLen = %x\n",fwLen); 
	ret = bl_update_fw_for_self_ctp((unsigned char *)fwbin, fwLen);
	
	if(ret < 0)
	{
		bl_log_trace("bl_auto_update_fw: bl_update_fw fail\n");  
	}
	else
	{
		bl_log_trace("bl_auto_update_fw: bl_update_fw success\n");  
	}
	return ret;
}
#endif
#endif
