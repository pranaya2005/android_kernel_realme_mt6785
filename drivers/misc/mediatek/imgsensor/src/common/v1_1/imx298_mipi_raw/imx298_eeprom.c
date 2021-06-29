#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"



#define PFX "imx298_pdafotp"
#define LOG_INF(format, args...)		pr_debug(PFX "[%s] " format, __func__, ##args)

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define imx298_EEPROM_READ_ID  0xB0
#define imx298_EEPROM_WRITE_ID   0xB1
#define imx298_I2C_SPEED        100  
#define imx298_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

BYTE imx298_DCC_data[96]= {0};
BYTE imx298_SPC_data[252]= {0};


static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > imx298_MAX_OFFSET)
        return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, imx298_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_imx298_eeprom(kal_uint16 addr, BYTE* data, int size ){
	int i = 0;
	int offset = addr;
	LOG_INF("enter _read_eeprom size = %d\n",size);
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}


bool read_imx298_SPC(BYTE* data)
{
	int addr = 0x0901;
	int size = 252;
	LOG_INF("read imx298 SPC, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx298_eeprom(addr, imx298_SPC_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	memcpy(data, imx298_SPC_data , size);
	return true;
}


void read_imx298_DCC( kal_uint16 addr,BYTE* data, kal_uint32 size){

	addr = 0x0A63;
	size = 96;
	
		LOG_INF("read imx298 DCC, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx298_eeprom(addr, imx298_DCC_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;

		}
	}

	memcpy(data, imx298_DCC_data , size);
   
}


bool IMX298CheckLensVersion(int id)
{
    kal_uint8 otp_flag = 0;   
    kal_uint8 data[8] = { 0 };

    LOG_INF("IMX298CheckLensVersion enter\n");
    _read_imx298_eeprom(0x0000,&otp_flag,1);
    LOG_INF("[lpl]read imx298 otp flag = %d\n", otp_flag);
       
    if(!otp_flag)
    {
        LOG_INF("[lpl]read otp failed!\n");
        return false;
    }
    
    _read_imx298_eeprom(0x0001,data,7);
    
    LOG_INF("[lpl]read imx298 otp year = %d\n", data[0]);
    LOG_INF("[lpl]read imx298 otp month = %d\n", data[1]);
    LOG_INF("[lpl]read imx298 otp day = %d\n", data[2]);
    LOG_INF("[lpl]read imx298 otp mid = %d\n", data[3]);
    LOG_INF("[lpl]read imx298 otp Lens_ID = %d\n", data[4]);
    LOG_INF("[lpl]read imx298 otp VCM_ID = %d\n", data[5]);
    LOG_INF("[lpl]read imx298 otp VCM_Driver_ID = %d\n", data[6]);

    return true;
}
