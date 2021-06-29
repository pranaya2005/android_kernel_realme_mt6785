#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#if defined(AGOLD_HW_COMPATIBLE)
#include "agold_camera_info.h" 
#endif
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"

#define PFX "S5KGM1KW_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

#define Sleep(ms)          mdelay(ms)
static int s5kGM1_write_id;

#define s5kgm1kw_WRITE_ID 	0xA0
static int eeprom_id = s5kgm1kw_WRITE_ID;

static int BGr_ratio_Typical = 646;
static int RGr_ratio_Typical = 502;
static int GbGr_ratio_Typical = 1026;

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_RED_ADDR      0x0210
#define GAIN_GREEN2_ADDR   0x0214

typedef struct s5kgm1kw_MIPI_otp_struct{
	kal_uint16 R_val;
	kal_uint16 B_val;
	kal_uint16 Gb_val;
	kal_uint16 Gr_val;
	kal_uint16 RGr_ratio;
	kal_uint16 BGr_ratio;
	kal_uint16 GbGr_ratio;
}s5kgm1kw_OTP_TYPE;

#ifdef AGOLD_OTP_ID_COMPATIBLE
const static unsigned short slaveid[3]={0xA0,0xB0,0xFF}; 
#endif

/**************************************************************/
extern int g_cur_cam_sensor;
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


static int read_s5kgm1kw_eeprom(kal_uint32 addr,u8* data)
{
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    return iReadRegI2C(puSendCmd , 2, data, 1, eeprom_id);
}
static kal_uint8 eeprom_read(kal_uint32 addr)
{
    kal_uint8 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, eeprom_id);
    return get_byte;
}
/***********************************************************************/
static kal_uint8 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint8 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, eeprom_id);
    return get_byte;
}
static int read_cmos_sensor_size(u16 offset, u8* data,int size)
{
	int i = 0;
	for(i = 0; i < size; i++){
		if(read_s5kgm1kw_eeprom(offset+i, data+i) != 0)
			return -1;
	}
	return 0;
}
/***********************************************************************/

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para,int write_id)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, write_id);
}
#if 1
static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    
    iWriteRegI2C(pu_send_cmd, 3, eeprom_id);
}
#endif
/**********************************************************************/


static int s5kgm1kw_MIPI_read_otp_wb(s5kgm1kw_OTP_TYPE *otp)
{	
   	kal_uint16 RGr_ratio, BGr_ratio, GbGr_ratio;
	kal_uint8 otp_flag = 0;   
	
	otp_flag = read_cmos_sensor(0x0070);
    LOG_INF("read s5kGM1 AWB otp flag = %d\n", otp_flag); 
    #ifndef AGOLD_XB_S5KGM1_OTP  
    if(otp_flag != 1)
    {
        LOG_INF("read awb otp failed!\n");
        read_cmos_sensor(0x0000);//[zhangpeng] add for eeprom [20170602]
        return 0;
    }
    #endif
    

  	RGr_ratio = (eeprom_read(0x0072)<<8) | eeprom_read(0x0071);
  	BGr_ratio = (eeprom_read(0x0074)<<8) | eeprom_read(0x0073);
  	GbGr_ratio = (eeprom_read(0x0076)<<8) | eeprom_read(0x0075);

  	LOG_INF("RGr_ratio = 0x%x,BGr_ratio = 0x%x,GbGr_ratio = 0x%x\n",RGr_ratio,BGr_ratio,GbGr_ratio);
  	
   	otp->RGr_ratio = RGr_ratio;
   	otp->BGr_ratio = BGr_ratio;	
   	otp->GbGr_ratio = GbGr_ratio;
   	return 1;
}

void s5kgm1kw_MIPI_write_otp_wb(s5kgm1kw_OTP_TYPE *otp)
{
    int R_gain, B_gain, Gb_gain, Gr_gain, Base_gain;
    kal_uint16 RGr_ratio, BGr_ratio, GbGr_ratio;
   
    RGr_ratio = otp->RGr_ratio;
    BGr_ratio = otp->BGr_ratio;
    GbGr_ratio = otp->GbGr_ratio;
   
   	#if defined (AGOLD_HW_COMPATIBLE)	
	BGr_ratio_Typical=agold_get_bg_ratio(g_cur_cam_sensor-1);
	RGr_ratio_Typical=agold_get_rg_ratio(g_cur_cam_sensor-1);
    #endif
    
    #ifdef AGOLD_S5KGM1_GOLDEN_TYPICAL
  	RGr_ratio_Typical = (eeprom_read(0x0078)<<8) | eeprom_read(0x0077);
  	BGr_ratio_Typical = (eeprom_read(0x007a)<<8) | eeprom_read(0x0079);
  	GbGr_ratio_Typical = (eeprom_read(0x007c)<<8) | eeprom_read(0x007b);  
  	#endif
  	LOG_INF("BGr_ratio_Typical = %x,RGr_ratio_Typical = %x,GbGr_ratio_Typical = %x\n",BGr_ratio_Typical,RGr_ratio_Typical,GbGr_ratio_Typical);
    R_gain = (RGr_ratio_Typical*1000) / RGr_ratio;
	B_gain = (BGr_ratio_Typical*1000) / BGr_ratio;
	Gb_gain = (GbGr_ratio_Typical*1000) / GbGr_ratio;
	Gr_gain = 1000;
	Base_gain = R_gain;
	
	if(Base_gain>B_gain) Base_gain = B_gain;
	if(Base_gain>Gb_gain) Base_gain = Gb_gain;
	if(Base_gain>Gr_gain) Base_gain = Gr_gain;
	
	R_gain = 0x100 * R_gain / Base_gain;
	B_gain = 0x100 * B_gain / Base_gain;
	Gb_gain = 0x100 * Gb_gain / Base_gain;
	Gr_gain = 0x100 * Gr_gain / Base_gain;
	
	write_cmos_sensor_byte(0x3058,0x01);

	if(Gr_gain>0x100)
		{
		    write_cmos_sensor(0x020E,Gr_gain>>8,s5kGM1_write_id);
            write_cmos_sensor(0x020F,Gr_gain&0xff,s5kGM1_write_id);
		}
	if(R_gain>0x100)
		{
		    write_cmos_sensor(0x0210,R_gain>>8,s5kGM1_write_id);
            write_cmos_sensor(0x0211,R_gain&0xff,s5kGM1_write_id);
		}
	if(B_gain>0x100)
		{
		    write_cmos_sensor(0x0212,B_gain>>8,s5kGM1_write_id);
            write_cmos_sensor(0x0213,B_gain&0xff,s5kGM1_write_id);
		}
	if(Gb_gain>0x100)
		{
		    write_cmos_sensor(0x0214,Gb_gain>>8,s5kGM1_write_id);
            write_cmos_sensor(0x0215,Gb_gain&0xff,s5kGM1_write_id);
		}
		
  	LOG_INF("S5KGM1KW_OTP:Gr_gain=0x%x\n",Gr_gain);
	LOG_INF("S5KGM1KW_OTP:R_gain=0x%x\n",R_gain);
	LOG_INF("S5KGM1KW_OTP:B_gain=0x%x\n",B_gain);
	LOG_INF("S5KGM1KW_OTP:Gb_gain=0x%x\n",Gb_gain);
	LOG_INF("S5KGM1KW_OTP:End.\n");
	return ;
}

void s5kgm1kw_MIPI_update_wb_register_from_otp(void)
{
   s5kgm1kw_OTP_TYPE current_otp;  
   
   if(s5kgm1kw_MIPI_read_otp_wb(&current_otp))
   		s5kgm1kw_MIPI_write_otp_wb(&current_otp);
   	return;
}



bool s5kgm1kwCheckLensVersion(int id)
{
    kal_uint8 otp_flag = 0; 
    kal_uint8 data[9] = { 0 };
	
	s5kGM1_write_id = id;
	LOG_INF("otp sensor id = 0x%x\n",s5kGM1_write_id);
	
	#ifdef AGOLD_OTP_ID_COMPATIBLE
		for(i = 0; slaveid[i] != 0xFF; i++){
			eeprom_id = slaveid[i];
			otp_flag = read_cmos_sensor(0x0000);
			if(otp_flag != 1)
				continue;
			else
				break;
		}
	#else
    	otp_flag = read_cmos_sensor(0x0000);
    #endif
    
    LOG_INF("read s5kgm1kw otp module information flag = %d\n", otp_flag);   
    if(otp_flag != 1)
    {
        LOG_INF("read otp failed!\n");
        read_cmos_sensor(0x0000);
        return false;
    }
                 
   	read_cmos_sensor_size(0x0001,data,9);

   	agold_camera_info[g_cur_cam_sensor].mf_id = data[0];
	agold_camera_info[g_cur_cam_sensor].lens_id = data[6];
	agold_camera_info[g_cur_cam_sensor].sen_id = data[5];
    agold_camera_info[g_cur_cam_sensor].date[0] = data[2];
    agold_camera_info[g_cur_cam_sensor].date[1] = data[3];
    agold_camera_info[g_cur_cam_sensor].date[2] = data[4];
    
    LOG_INF("read s5kgm1kw otp module = %d\n", data[0]);
    LOG_INF("read s5kgm1kw otp platform = %d\n", data[1]);    
    LOG_INF("read s5kgm1kw otp year = %d\n", data[2]);
    LOG_INF("read s5kgm1kw otp month = %d\n", data[3]);
    LOG_INF("read s5kgm1kw otp day = %d\n", data[4]);
    LOG_INF("read s5kgm1kw otp Lens_ID = 0x%x\n", data[6]);
	LOG_INF("read s5kgm1kw otp sen_id = 0x%x\n", data[5]);
	LOG_INF("read s5kgm1kw otp drv_id = 0x%x\n", data[8]);

	read_cmos_sensor_size(0x07F7,data,2);
	LOG_INF("read s5kgm1kw otp AF = %d\n", data[1]);

    return true;
}

