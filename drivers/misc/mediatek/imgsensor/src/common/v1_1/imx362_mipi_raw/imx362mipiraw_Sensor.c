/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx362mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_camera_typedef.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx362mipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/

int b_imx362open = 0;

#define PFX "imx362_camera_sensor"
#ifdef CONFIG_FAKE_DUAL_CAMERA
#include "fake_camera.h"
#endif

#define LOG_INF(format, args...)    printk(PFX "[%s] " format, __FUNCTION__, ##args)

#define LOG_1 LOG_INF("imx362,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

//#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_INF(format, args...)	

static DEFINE_SPINLOCK(imgsensor_drv_lock);
extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

extern int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,u16 a_sizeRecvData, u16 i2cId, u16 timing);

extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);
extern int iWriteRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId, u16 timing);

static kal_uint8 mode_change;
static struct imgsensor_info_struct imgsensor_info = { 
	.sensor_id = IMX362_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x245a1c49,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 231270000,				//record different mode's pclk
		.linelength = 4620,				//record different mode's linelength
		.framelength = 1610,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2016,		//record different mode's width of grabwindow
		.grabwindow_height = 1512,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 278000000,
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 840000000,
		.linelength = 8488,
		.framelength = 3298,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.mipi_pixel_rate = 513000000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 432000000,
		.linelength = 4620,
		.framelength = 3118,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.mipi_pixel_rate = 516000000,
		.max_framerate = 300,
	},

	.normal_video = {
		.pclk = 840000000,
		.linelength = 8488,
		.framelength = 3298,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.mipi_pixel_rate = 516000000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 231270000,				//record different mode's pclk
		.linelength = 4620,				//record different mode's linelength
		.framelength = 1610,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2016,		//record different mode's width of grabwindow
		.grabwindow_height = 1512,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 281000000,
		.max_framerate = 300,	
	},
	.slim_video = {
		.pclk = 231270000,				//record different mode's pclk
		.linelength = 4620,				//record different mode's linelength
		.framelength = 1610,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2016,		//record different mode's width of grabwindow
		.grabwindow_height = 1512,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 281000000,
		.max_framerate = 300,	
	},
	.margin = 10,		/* sensor framelength & shutter margin */
	.min_shutter = 2,	/* min shutter */
	.min_gain = 64, /*1x gain*/
	.max_gain = 1024, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 3,	/* enter preview delay frame num */
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num
	
	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2


	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,


	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x20, 0x34, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
	.i2c_speed = 100,	/* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {

	.mirror = IMAGE_NORMAL,//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,	/* current shutter */
	.gain = 0x200,		/* current gain */
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,//record current sensor's i2c write id
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 4032, 3024,	0, 	0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512,	0,	0, 2016,  1512}, // Preview 
 { 4032, 3024,	0, 	0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024,	0,	0, 4032,  3024}, // capture 
 { 4032, 3024,	0, 	0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024,	0,	0, 4032,  3024},  // video 
 { 4032, 3024,	0, 	0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512,	0,	0, 2016,  1512}, //hight speed video 
 { 4032, 3024,	0, 	0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512,	0,	0, 2016,  1512}};// slim video 




//extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
//extern int iWriteRegI2C(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/*static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	  kal_uint16 get_byte=0;
      iReadReg((u16) addr ,(u8*)&get_byte, imgsensor.i2c_write_id);
      return get_byte;
}*/

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}
//#define write_cmos_sensor(addr, para) iWriteRegI2C((u16) addr , (u32) para , 1,  imgsensor.i2c_write_id)
static int write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	return iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
static void set_dummy(void)
{
	//LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0104, 0x01);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0x00);
}	/*	set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0016) << 8) | read_cmos_sensor(0x0017));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	pr_info("framerate = %d, min framelength should enable(%d)\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);

	imgsensor.frame_length =
		 (frame_length > imgsensor.min_frame_length)
		? frame_length : imgsensor.min_frame_length;

	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;
	/* dummy_line = frame_length - imgsensor.min_frame_length;
	 * if (dummy_line < 0)
	 * imgsensor.dummy_line = 0;
	 * else
	 * imgsensor.dummy_line = dummy_line;
	 * imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	 */

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

#define MAX_SHUTTER	12103350	/* 120s long exposure time */
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 line_length = 0;
	kal_uint16 long_exp_times = 0;
	kal_uint16 long_exp_shift = 0;

	/* limit max exposure time to be 120s */
	if (shutter > MAX_SHUTTER)
		shutter = MAX_SHUTTER;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	pr_info("enter shutter =%d\n", shutter);
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	if (shutter >
	    (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		long_exp_times = shutter /
			(imgsensor_info.max_frame_length
			 - imgsensor_info.margin);

		if (shutter %
		    (imgsensor_info.max_frame_length - imgsensor_info.margin))
			long_exp_times++;

		if (long_exp_times > 128)
			long_exp_times = 128;
		if (long_exp_times < 1)
			long_exp_times = 1;

		long_exp_shift = fls(long_exp_times) - 1;

		if (long_exp_times & (~(1 << long_exp_shift))) {
			/* fix for coding style */
			long_exp_shift++;
		}

		long_exp_times = 1 << long_exp_shift;
		write_cmos_sensor(0x3004, long_exp_shift);
		shutter = shutter / long_exp_times;

		if (shutter > (imgsensor_info.max_frame_length -
			       imgsensor_info.margin)) {
			line_length = shutter * 4296 /
				(imgsensor_info.max_frame_length
				 - imgsensor_info.margin);

			line_length = (line_length + 1) / 2 * 2;
		}

		spin_lock(&imgsensor_drv_lock);
		if (shutter >
		    imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length =
				shutter + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);

		/* line_length range is 4296 <-> 32766 */

		if (line_length > 32766) {
			/* 20171116ken : fix for coding style */
			line_length = 32766;
		}

		if (line_length < 4296) {
			/* 20171116ken : fix for coding style */
			line_length = 4296;
		}

		write_cmos_sensor(0x0342, line_length >> 8);
		write_cmos_sensor(0x0343, line_length & 0xFF);
	}

	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;

	shutter = (shutter >
		   (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341,
					  imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}

	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0350, 0x01); /* enable auto extend */

	/* Update Shutter */
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	pr_info(
	"Exit! shutter=%d, framelength=%d, long_exp_line_length=%d, long_exp_shift:%d auto_extend=%d\n",
		shutter, imgsensor.frame_length, line_length,
		long_exp_shift, read_cmos_sensor(0x0350));
}	/*	write_shutter  */

static void set_shutter_frame_length(
				kal_uint16 shutter, kal_uint16 frame_length,
				kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
/* if shutter bigger than frame_length, should extend frame length first */

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		  ? imgsensor_info.min_shutter : shutter;

	shutter =
	  (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	  ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
		/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		      write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter  & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	pr_info(
	"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor(0x0350));

}	/* set_shutter_frame_length */

#define imx362MIPI_MaxGainIndex (255)

kal_uint16 imx362MIPI_sensorGainMapping[imx362MIPI_MaxGainIndex][2] = {
	{64, 0},
	{65, 6},
	{66, 12},
	{67, 20},
	{68, 27},
	{69, 34},
	{70, 43},
	{71, 51},
	{72, 55},
	{73, 63},
	{74, 67},
	{75, 75},
	{76, 79},
	{77, 85},
	{78, 92},
	{79, 96},
	{80, 100},
	{81, 106},
	{82, 112},
	{83, 116},
	{84, 122},
	{85, 125},
	{86, 130},
	{87, 136},
	{88, 139},
	{89, 144},
	{90, 146},
	{91, 152},
	{92, 154},
	{93, 159},
	{94, 162},
	{95, 167},
	{96, 169},
	{97, 173},
	{98, 176},
	{100, 184},
	{101, 186},
	{102, 190},
	{103, 193},
	{104, 196},
	{105, 200},
	{106, 202},
	{107, 206},
	{108, 208},
	{110, 213},
	{111, 216},
	{112, 220},
	{113, 221},
	{114, 224},
	{115, 226},
	{116, 230},
	{117, 231},
	{118, 234},
	{120, 239},
	{121, 242},
	{122, 243},
	{123, 246},
	{124, 247},
	{125, 249},
	{126, 251},
	{127, 253},
	{128, 255},
	{130, 259},
	{131, 261},
	{132, 263},
	{133, 265},
	{134, 267},
	{135, 269},
	{136, 271},
	{137, 272},
	{138, 274},
	{140, 278},
	{141, 279},
	{142, 281},
	{143, 283},
	{144, 284},
	{145, 286},
	{146, 287},
	{147, 289},
	{148, 290},
	{150, 293},
	{151, 295},
	{152, 296},
	{153, 298},
	{154, 299},
	{155, 300},
	{156, 302},
	{157, 303},
	{158, 304},
	{160, 307},
	{161, 308},
	{162, 310},
	{163, 311},
	{164, 312},
	{165, 313},
	{166, 315},
	{167, 316},
	{168, 317},
	{170, 319},
	{171, 320},
	{172, 321},
	{173, 323},
	{174, 324},
	{175, 325},
	{176, 326},
	{177, 327},
	{178, 328},
	{180, 330},
	{181, 331},
	{182, 332},
	{183, 333},
	{184, 334},
	{185, 335},
	{186, 336},
	{187, 337},
	{188, 338},
	{191, 340},
	{192, 341},
	{193, 342},
	{194, 343},
	{195, 344},
	{196, 345},
	{197, 346},
	{199, 347},
	{200, 348},
	{202, 350},
	{204, 351},
	{205, 352},
	{206, 353},
	{207, 354},
	{209, 355},
	{210, 356},
	{211, 357},
	{213, 358},
	{216, 360},
	{217, 361},
	{218, 362},
	{220, 363},
	{221, 364},
	{223, 365},
	{224, 366},
	{226, 367},
	{228, 368},
	{229, 369},
	{231, 370},
	{232, 371},
	{234, 372},
	{236, 373},
	{237, 374},
	{239, 375},
	{241, 376},
	{243, 377},
	{245, 378},
	{246, 379},
	{248, 380},
	{250, 381},
	{252, 382},
	{254, 383},
	{256, 384},
	{258, 385},
	{260, 386},
	{262, 387},
	{264, 388},
	{266, 389},
	{269, 390},
	{271, 391},
	{273, 392},
	{275, 393},
	{278, 394},
	{280, 395},
	{282, 396},
	{285, 397},
	{287, 398},
	{290, 399},
	{293, 400},
	{295, 401},
	{298, 402},
	{301, 403},
	{303, 404},
	{306, 405},
	{309, 406},
	{312, 407},
	{315, 408},
	{318, 409},
	{321, 410},
	{324, 411},
	{328, 412},
	{331, 413},
	{334, 414},
	{338, 415},
	{341, 416},
	{345, 417},
	{349, 418},
	{352, 419},
	{356, 420},
	{360, 421},
	{364, 422},
	{368, 423},
	{372, 424},
	{377, 425},
	{381, 426},
	{386, 427},
	{390, 428},
	{395, 429},
	{400, 430},
	{405, 431},
	{410, 432},
	{415, 433},
	{420, 434},
	{426, 435},
	{431, 436},
	{437, 437},
	{443, 438},
	{449, 439},
	{455, 440},
	{462, 441},
	{468, 442},
	{475, 443},
	{482, 444},
	{489, 445},
	{496, 446},
	{504, 447},
	{512, 448},
	{520, 449},
	{529, 450},
	{537, 451},
	{546, 452},
	{555, 453},
	{565, 454},
	{575, 455},
	{585, 456},
	{596, 457},
	{607, 458},
	{618, 459},
	{630, 460},
	{643, 461},
	{655, 462},
	{669, 463},
	{683, 464},
	{697, 465},
	{712, 466},
	{728, 467},
	{745, 468},
	{762, 469},
	{780, 470},
	{799, 471},
	{819, 472},
	{840, 473},
	{862, 474},
	{886, 475},
	{910, 476},
	{936, 477},
	{964, 478},
	{993, 479},
	{1024, 480},
};

#if 0
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	/* gain = 64 = 1x real gain */
	reg_gain = 512 - (512 * 64 / gain);
	return (kal_uint16) reg_gain;
}
#else

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI = 0;

	for (iI = 0; iI < imx362MIPI_MaxGainIndex; iI++) {
		if (gain <= imx362MIPI_sensorGainMapping[iI][0])
			return imx362MIPI_sensorGainMapping[iI][1];
	}
	pr_info("exit imx362MIPI_sensorGainMapping function\n");
	return imx362MIPI_sensorGainMapping[iI-1][1];
}
#endif

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_info("Error gain setting");
		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_info("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	/*
	 * WORKAROUND! stream on after set shutter/gain, which will get
	 * first valid capture frame.
	 */
	if (mode_change && (imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE)) {
		write_cmos_sensor(0x0100, 0x01);
		mode_change = 0;
	}
	return gain;
}	/*	set_gain  */

#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101, 0X00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101, 0X01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101, 0X02);
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101, 0X03);
			break;
		default:
		LOG_INF("Error image_mirror setting\n");
}
}
#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");
	//write_cmos_sensor(0x0100,0x01);  //wake up
	write_cmos_sensor(0x0136, 0x18);
	write_cmos_sensor(0x0137, 0x00); 
	write_cmos_sensor(0x31A3, 0x00); 
	write_cmos_sensor(0x64D4, 0x01);
    write_cmos_sensor(0x64D5, 0xAA);
    write_cmos_sensor(0x64D6, 0x01); 
    write_cmos_sensor(0x64D7, 0xA9); 
    write_cmos_sensor(0x64D8, 0x01);
    write_cmos_sensor(0x64D9, 0xA5); 
    write_cmos_sensor(0x64DA, 0x01); 
    write_cmos_sensor(0x64DB, 0xA1); 
    write_cmos_sensor(0x720A, 0x24);
    write_cmos_sensor(0x720B, 0x89); 
    write_cmos_sensor(0x720C, 0x85); 
    write_cmos_sensor(0x720D, 0xA1); 
    write_cmos_sensor(0x720E, 0x6E);
    write_cmos_sensor(0x729C, 0x59); 
    write_cmos_sensor(0x817C, 0xFF); 
    write_cmos_sensor(0x817D, 0x80); 
    write_cmos_sensor(0x9348, 0x96);
    write_cmos_sensor(0x934B, 0x8C); 
    write_cmos_sensor(0x934C, 0x82); 
    write_cmos_sensor(0x9353, 0xAA); 
    write_cmos_sensor(0x9354, 0xAA);
    write_cmos_sensor(0x0100, 0x00);
}	/*	sensor_init  */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	write_cmos_sensor(0x0112, 0x0A); //
	write_cmos_sensor(0x0113, 0x0A); 
	write_cmos_sensor(0x0114, 0x03); 
	write_cmos_sensor(0x0220, 0x00);
    write_cmos_sensor(0x0221, 0x11); 
    write_cmos_sensor(0x0340, 0x0C); 
    write_cmos_sensor(0x0341, 0xe2); 
    write_cmos_sensor(0x0342, 0x21);
    write_cmos_sensor(0x0343, 0x28); 
    write_cmos_sensor(0x0381, 0x01); 
    write_cmos_sensor(0x0383, 0x01); 
    write_cmos_sensor(0x0385, 0x01);
    write_cmos_sensor(0x0387, 0x01); 
    write_cmos_sensor(0x0900, 0x00); 
    write_cmos_sensor(0x0901, 0x11); 
    write_cmos_sensor(0x30F4, 0x00);
    write_cmos_sensor(0x30F5, 0x14); 
    write_cmos_sensor(0x30F6, 0x03); 
    write_cmos_sensor(0x30F7, 0x20); 
    write_cmos_sensor(0x31A0, 0x02);
    write_cmos_sensor(0x31A5, 0x00); 
    write_cmos_sensor(0x31A6, 0x00); 
    write_cmos_sensor(0x560F, 0x05); 
    write_cmos_sensor(0x5856, 0x04);
    write_cmos_sensor(0x58D0, 0x0E); 
    write_cmos_sensor(0x734A, 0x23); 
    write_cmos_sensor(0x734F, 0x64); 
    write_cmos_sensor(0x7441, 0x5A);
    write_cmos_sensor(0x7914, 0x02); 
    write_cmos_sensor(0x7928, 0x08); 
    write_cmos_sensor(0x7929, 0x08); 
    write_cmos_sensor(0x793F, 0x02);
    write_cmos_sensor(0xBC7B, 0x2C); 
    write_cmos_sensor(0x0344, 0x00); 
    write_cmos_sensor(0x0345, 0x00); 
    write_cmos_sensor(0x0346, 0x00);
    write_cmos_sensor(0x0347, 0x00); 
    write_cmos_sensor(0x0348, 0x0F); 
    write_cmos_sensor(0x0349, 0xBF); 
    write_cmos_sensor(0x034A, 0x0B);
    write_cmos_sensor(0x034B, 0xCF); 
    write_cmos_sensor(0x034C, 0x0F); 
    write_cmos_sensor(0x034D, 0xC0); 
    write_cmos_sensor(0x034E, 0x0B);
    write_cmos_sensor(0x034F, 0xD0); 
    write_cmos_sensor(0x0408, 0x00); 
    write_cmos_sensor(0x0409, 0x00); 
    write_cmos_sensor(0x040A, 0x00);
    write_cmos_sensor(0x040B, 0x00); 
    write_cmos_sensor(0x040C, 0x0F); 
    write_cmos_sensor(0x040D, 0xC0); 
    write_cmos_sensor(0x040E, 0x0B);
    write_cmos_sensor(0x040F, 0xD0); 
    write_cmos_sensor(0x0301, 0x03); 
    write_cmos_sensor(0x0303, 0x02); 
    write_cmos_sensor(0x0305, 0x04);
    write_cmos_sensor(0x0306, 0x00); 
    write_cmos_sensor(0x0307, 0xd2); 
    write_cmos_sensor(0x0309, 0x0A); 
    write_cmos_sensor(0x030B, 0x01);
    write_cmos_sensor(0x030D, 0x04); 
    write_cmos_sensor(0x030E, 0x00); 
    write_cmos_sensor(0x030F, 0xc8); 
    write_cmos_sensor(0x0310, 0x01);
    write_cmos_sensor(0x0202, 0x0C); 
    write_cmos_sensor(0x0203, 0x26); 
    write_cmos_sensor(0x0224, 0x01); 
    write_cmos_sensor(0x0225, 0xF4);
    write_cmos_sensor(0x0204, 0x00); 
    write_cmos_sensor(0x0205, 0x00); 
    write_cmos_sensor(0x0216, 0x00); 
    write_cmos_sensor(0x0217, 0x00);
    write_cmos_sensor(0x020E, 0x01); 
    write_cmos_sensor(0x020F, 0x00); 
    write_cmos_sensor(0x0226, 0x00); 
    write_cmos_sensor(0x0227, 0x00);
	write_cmos_sensor(0x0100,0x01);
		
}


static void preview_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0112, 0x0A); 
	write_cmos_sensor(0x0113, 0x0A); 
	write_cmos_sensor(0x0114, 0x03); 
	write_cmos_sensor(0x0220, 0x00);
    write_cmos_sensor(0x0221, 0x11); 
    write_cmos_sensor(0x0340, 0x08); 
    write_cmos_sensor(0x0341, 0x58); 
    write_cmos_sensor(0x0342, 0x11);
    write_cmos_sensor(0x0343, 0x10); 
    write_cmos_sensor(0x0381, 0x01); 
    write_cmos_sensor(0x0383, 0x01); 
    write_cmos_sensor(0x0385, 0x01);
    write_cmos_sensor(0x0387, 0x01); 
    write_cmos_sensor(0x0900, 0x01); 
    write_cmos_sensor(0x0901, 0x22); 
    write_cmos_sensor(0x30F4, 0x02);
    write_cmos_sensor(0x30F5, 0xBC); 
    write_cmos_sensor(0x30F6, 0x01); 
    write_cmos_sensor(0x30F7, 0x18); 
    write_cmos_sensor(0x31A0, 0x02);
    write_cmos_sensor(0x31A5, 0x00); 
    write_cmos_sensor(0x31A6, 0x01); 
    write_cmos_sensor(0x560F, 0x14); 
    write_cmos_sensor(0x5856, 0x04);
    write_cmos_sensor(0x58D0, 0x0E); 
    write_cmos_sensor(0x734A, 0x23); 
    write_cmos_sensor(0x734F, 0x64); 
    write_cmos_sensor(0x7441, 0x5A);
    write_cmos_sensor(0x7914, 0x02); 
    write_cmos_sensor(0x7928, 0x08); 
    write_cmos_sensor(0x7929, 0x08); 
    write_cmos_sensor(0x793F, 0x02);
    write_cmos_sensor(0xBC7B, 0x2C); 
    write_cmos_sensor(0x0344, 0x00); 
    write_cmos_sensor(0x0345, 0x00); 
    write_cmos_sensor(0x0346, 0x00);
    write_cmos_sensor(0x0347, 0x00); 
    write_cmos_sensor(0x0348, 0x0F); 
    write_cmos_sensor(0x0349, 0xBF); 
    write_cmos_sensor(0x034A, 0x0B);
    write_cmos_sensor(0x034B, 0xCF); 
    write_cmos_sensor(0x034C, 0x07); 
    write_cmos_sensor(0x034D, 0xE0); 
    write_cmos_sensor(0x034E, 0x05);
    write_cmos_sensor(0x034F, 0xE8); 
    write_cmos_sensor(0x0408, 0x00); 
    write_cmos_sensor(0x0409, 0x00); 
    write_cmos_sensor(0x040A, 0x00);
    write_cmos_sensor(0x040B, 0x00); 
    write_cmos_sensor(0x040C, 0x07); 
    write_cmos_sensor(0x040D, 0xE0); 
    write_cmos_sensor(0x040E, 0x05);
    write_cmos_sensor(0x040F, 0xE8); 
    write_cmos_sensor(0x0301, 0x03); 
    write_cmos_sensor(0x0303, 0x02); 
    write_cmos_sensor(0x0305, 0x04);
    write_cmos_sensor(0x0306, 0x00); 
    write_cmos_sensor(0x0307, 0x46); 
    write_cmos_sensor(0x0309, 0x0A); 
    write_cmos_sensor(0x030B, 0x04);
    write_cmos_sensor(0x030D, 0x03); 
    write_cmos_sensor(0x030E, 0x00); 
    write_cmos_sensor(0x030F, 0xC2); 
    write_cmos_sensor(0x0310, 0x01);
    write_cmos_sensor(0x0202, 0x08); 
    write_cmos_sensor(0x0203, 0x4A); 
    write_cmos_sensor(0x0224, 0x01); 
    write_cmos_sensor(0x0225, 0xF4);
    write_cmos_sensor(0x0204, 0x00); 
    write_cmos_sensor(0x0205, 0x00); 
    write_cmos_sensor(0x0216, 0x00); 
    write_cmos_sensor(0x0217, 0x00);
    write_cmos_sensor(0x020E, 0x01); 
    write_cmos_sensor(0x020F, 0x00); 
    write_cmos_sensor(0x0226, 0x01); 
    write_cmos_sensor(0x0227, 0x00);
	write_cmos_sensor(0x0100,0x01);   
	
}	/*	preview_setting  */

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0112, 0x0A); 
	write_cmos_sensor(0x0113, 0x0A); 
	write_cmos_sensor(0x0114, 0x03); 
	write_cmos_sensor(0x0220, 0x00);
    write_cmos_sensor(0x0221, 0x11); 
    write_cmos_sensor(0x0340, 0x0C); 
    write_cmos_sensor(0x0341, 0xe2); 
    write_cmos_sensor(0x0342, 0x21);
    write_cmos_sensor(0x0343, 0x28); 
    write_cmos_sensor(0x0381, 0x01); 
    write_cmos_sensor(0x0383, 0x01); 
    write_cmos_sensor(0x0385, 0x01);
    write_cmos_sensor(0x0387, 0x01); 
    write_cmos_sensor(0x0900, 0x00); 
    write_cmos_sensor(0x0901, 0x11); 
    write_cmos_sensor(0x30F4, 0x00);
    write_cmos_sensor(0x30F5, 0x14); 
    write_cmos_sensor(0x30F6, 0x03); 
    write_cmos_sensor(0x30F7, 0x20); 
    write_cmos_sensor(0x31A0, 0x02);
    write_cmos_sensor(0x31A5, 0x00); 
    write_cmos_sensor(0x31A6, 0x00); 
    write_cmos_sensor(0x560F, 0x05); 
    write_cmos_sensor(0x5856, 0x04);
    write_cmos_sensor(0x58D0, 0x0E); 
    write_cmos_sensor(0x734A, 0x23); 
    write_cmos_sensor(0x734F, 0x64); 
    write_cmos_sensor(0x7441, 0x5A);
    write_cmos_sensor(0x7914, 0x02); 
    write_cmos_sensor(0x7928, 0x08); 
    write_cmos_sensor(0x7929, 0x08); 
    write_cmos_sensor(0x793F, 0x02);
    write_cmos_sensor(0xBC7B, 0x2C); 
	//output size setting
    write_cmos_sensor(0x0344, 0x00); 
    write_cmos_sensor(0x0345, 0x00); 
    write_cmos_sensor(0x0346, 0x00);
    write_cmos_sensor(0x0347, 0x00); 
    write_cmos_sensor(0x0348, 0x0F); 
    write_cmos_sensor(0x0349, 0xBF); 
    write_cmos_sensor(0x034A, 0x0B);
    write_cmos_sensor(0x034B, 0xCF); 
    write_cmos_sensor(0x034C, 0x0F); 
    write_cmos_sensor(0x034D, 0xC0); 
    write_cmos_sensor(0x034E, 0x0B);
    write_cmos_sensor(0x034F, 0xD0); 
    write_cmos_sensor(0x0408, 0x00); 
    write_cmos_sensor(0x0409, 0x00); 
    write_cmos_sensor(0x040A, 0x00);
    write_cmos_sensor(0x040B, 0x00); 
    write_cmos_sensor(0x040C, 0x0F); 
    write_cmos_sensor(0x040D, 0xC0); 
    write_cmos_sensor(0x040E, 0x0B);
    write_cmos_sensor(0x040F, 0xD0);
	//clock setting	
    write_cmos_sensor(0x0301, 0x03); 
    write_cmos_sensor(0x0303, 0x02); 
    write_cmos_sensor(0x0305, 0x04);
    write_cmos_sensor(0x0306, 0x00); 
    write_cmos_sensor(0x0307, 0xd2); 
    write_cmos_sensor(0x0309, 0x0A); 
    write_cmos_sensor(0x030B, 0x01);
    write_cmos_sensor(0x030D, 0x04); 
    write_cmos_sensor(0x030E, 0x00); 
    write_cmos_sensor(0x030F, 0xc8); 
    write_cmos_sensor(0x0310, 0x01);
	//intergration time setting
    write_cmos_sensor(0x0202, 0x0C); 
    write_cmos_sensor(0x0203, 0x26); 
    write_cmos_sensor(0x0224, 0x01); 
    write_cmos_sensor(0x0225, 0xF4);
	//gain setting
    write_cmos_sensor(0x0204, 0x00); 
    write_cmos_sensor(0x0205, 0x00); 
    write_cmos_sensor(0x0216, 0x00); 
    write_cmos_sensor(0x0217, 0x00);
    write_cmos_sensor(0x020E, 0x01); 
    write_cmos_sensor(0x020F, 0x00); 
    write_cmos_sensor(0x0226, 0x00); 
    write_cmos_sensor(0x0227, 0x00);   
	write_cmos_sensor(0x0100,0x01);
	
}
static void hs_video_setting(void)
{
	LOG_INF("E\n");
	
	write_cmos_sensor(0x0112, 0x0A); 
	write_cmos_sensor(0x0113, 0x0A); 
	write_cmos_sensor(0x0114, 0x03); 
	write_cmos_sensor(0x0220, 0x00);
    write_cmos_sensor(0x0221, 0x11); 
    write_cmos_sensor(0x0340, 0x08); 
    write_cmos_sensor(0x0341, 0x58); 
    write_cmos_sensor(0x0342, 0x11);
    write_cmos_sensor(0x0343, 0x10); 
    write_cmos_sensor(0x0381, 0x01); 
    write_cmos_sensor(0x0383, 0x01); 
    write_cmos_sensor(0x0385, 0x01);
    write_cmos_sensor(0x0387, 0x01); 
    write_cmos_sensor(0x0900, 0x01); 
    write_cmos_sensor(0x0901, 0x22); 
    write_cmos_sensor(0x30F4, 0x02);
    write_cmos_sensor(0x30F5, 0xBC); 
    write_cmos_sensor(0x30F6, 0x01); 
    write_cmos_sensor(0x30F7, 0x18); 
    write_cmos_sensor(0x31A0, 0x02);
    write_cmos_sensor(0x31A5, 0x00); 
    write_cmos_sensor(0x31A6, 0x01); 
    write_cmos_sensor(0x560F, 0x14); 
    write_cmos_sensor(0x5856, 0x04);
    write_cmos_sensor(0x58D0, 0x0E); 
    write_cmos_sensor(0x734A, 0x23); 
    write_cmos_sensor(0x734F, 0x64); 
    write_cmos_sensor(0x7441, 0x5A);
    write_cmos_sensor(0x7914, 0x02); 
    write_cmos_sensor(0x7928, 0x08); 
    write_cmos_sensor(0x7929, 0x08); 
    write_cmos_sensor(0x793F, 0x02);
    write_cmos_sensor(0xBC7B, 0x2C); 
    write_cmos_sensor(0x0344, 0x00); 
    write_cmos_sensor(0x0345, 0x00); 
    write_cmos_sensor(0x0346, 0x00);
    write_cmos_sensor(0x0347, 0x00); 
    write_cmos_sensor(0x0348, 0x0F); 
    write_cmos_sensor(0x0349, 0xBF); 
    write_cmos_sensor(0x034A, 0x0B);
    write_cmos_sensor(0x034B, 0xCF); 
    write_cmos_sensor(0x034C, 0x07); 
    write_cmos_sensor(0x034D, 0xE0); 
    write_cmos_sensor(0x034E, 0x05);
    write_cmos_sensor(0x034F, 0xE8); 
    write_cmos_sensor(0x0408, 0x00); 
    write_cmos_sensor(0x0409, 0x00); 
    write_cmos_sensor(0x040A, 0x00);
    write_cmos_sensor(0x040B, 0x00); 
    write_cmos_sensor(0x040C, 0x07); 
    write_cmos_sensor(0x040D, 0xE0); 
    write_cmos_sensor(0x040E, 0x05);
    write_cmos_sensor(0x040F, 0xE8); 
    write_cmos_sensor(0x0301, 0x03); 
    write_cmos_sensor(0x0303, 0x02); 
    write_cmos_sensor(0x0305, 0x04);
    write_cmos_sensor(0x0306, 0x00); 
    write_cmos_sensor(0x0307, 0x46); 
    write_cmos_sensor(0x0309, 0x0A); 
    write_cmos_sensor(0x030B, 0x04);
    write_cmos_sensor(0x030D, 0x03); 
    write_cmos_sensor(0x030E, 0x00); 
    write_cmos_sensor(0x030F, 0xC2); 
    write_cmos_sensor(0x0310, 0x01);
    write_cmos_sensor(0x0202, 0x08); 
    write_cmos_sensor(0x0203, 0x4A); 
    write_cmos_sensor(0x0224, 0x01); 
    write_cmos_sensor(0x0225, 0xF4);
    write_cmos_sensor(0x0204, 0x00); 
    write_cmos_sensor(0x0205, 0x00); 
    write_cmos_sensor(0x0216, 0x00); 
    write_cmos_sensor(0x0217, 0x00);
    write_cmos_sensor(0x020E, 0x01); 
    write_cmos_sensor(0x020F, 0x00); 
    write_cmos_sensor(0x0226, 0x01); 
    write_cmos_sensor(0x0227, 0x00);
	write_cmos_sensor(0x0100,0x01);   
	
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	
	write_cmos_sensor(0x0112, 0x0A); 
	write_cmos_sensor(0x0113, 0x0A); 
	write_cmos_sensor(0x0114, 0x03); 
	write_cmos_sensor(0x0220, 0x00);
    write_cmos_sensor(0x0221, 0x11); 
    write_cmos_sensor(0x0340, 0x08); 
    write_cmos_sensor(0x0341, 0x58); 
    write_cmos_sensor(0x0342, 0x11);
    write_cmos_sensor(0x0343, 0x10); 
    write_cmos_sensor(0x0381, 0x01); 
    write_cmos_sensor(0x0383, 0x01); 
    write_cmos_sensor(0x0385, 0x01);
    write_cmos_sensor(0x0387, 0x01); 
    write_cmos_sensor(0x0900, 0x01); 
    write_cmos_sensor(0x0901, 0x22); 
    write_cmos_sensor(0x30F4, 0x02);
    write_cmos_sensor(0x30F5, 0xBC); 
    write_cmos_sensor(0x30F6, 0x01); 
    write_cmos_sensor(0x30F7, 0x18); 
    write_cmos_sensor(0x31A0, 0x02);
    write_cmos_sensor(0x31A5, 0x00); 
    write_cmos_sensor(0x31A6, 0x01); 
    write_cmos_sensor(0x560F, 0x14); 
    write_cmos_sensor(0x5856, 0x04);
    write_cmos_sensor(0x58D0, 0x0E); 
    write_cmos_sensor(0x734A, 0x23); 
    write_cmos_sensor(0x734F, 0x64); 
    write_cmos_sensor(0x7441, 0x5A);
    write_cmos_sensor(0x7914, 0x02); 
    write_cmos_sensor(0x7928, 0x08); 
    write_cmos_sensor(0x7929, 0x08); 
    write_cmos_sensor(0x793F, 0x02);
    write_cmos_sensor(0xBC7B, 0x2C); 
    write_cmos_sensor(0x0344, 0x00); 
    write_cmos_sensor(0x0345, 0x00); 
    write_cmos_sensor(0x0346, 0x00);
    write_cmos_sensor(0x0347, 0x00); 
    write_cmos_sensor(0x0348, 0x0F); 
    write_cmos_sensor(0x0349, 0xBF); 
    write_cmos_sensor(0x034A, 0x0B);
    write_cmos_sensor(0x034B, 0xCF); 
    write_cmos_sensor(0x034C, 0x07); 
    write_cmos_sensor(0x034D, 0xE0); 
    write_cmos_sensor(0x034E, 0x05);
    write_cmos_sensor(0x034F, 0xE8); 
    write_cmos_sensor(0x0408, 0x00); 
    write_cmos_sensor(0x0409, 0x00); 
    write_cmos_sensor(0x040A, 0x00);
    write_cmos_sensor(0x040B, 0x00); 
    write_cmos_sensor(0x040C, 0x07); 
    write_cmos_sensor(0x040D, 0xE0); 
    write_cmos_sensor(0x040E, 0x05);
    write_cmos_sensor(0x040F, 0xE8); 
    write_cmos_sensor(0x0301, 0x03); 
    write_cmos_sensor(0x0303, 0x02); 
    write_cmos_sensor(0x0305, 0x04);
    write_cmos_sensor(0x0306, 0x00); 
    write_cmos_sensor(0x0307, 0x46); 
    write_cmos_sensor(0x0309, 0x0A); 
    write_cmos_sensor(0x030B, 0x04);
    write_cmos_sensor(0x030D, 0x03); 
    write_cmos_sensor(0x030E, 0x00); 
    write_cmos_sensor(0x030F, 0xC2); 
    write_cmos_sensor(0x0310, 0x01);
    write_cmos_sensor(0x0202, 0x08); 
    write_cmos_sensor(0x0203, 0x4A); 
    write_cmos_sensor(0x0224, 0x01); 
    write_cmos_sensor(0x0225, 0xF4);
    write_cmos_sensor(0x0204, 0x00); 
    write_cmos_sensor(0x0205, 0x00); 
    write_cmos_sensor(0x0216, 0x00); 
    write_cmos_sensor(0x0217, 0x00);
    write_cmos_sensor(0x020E, 0x01); 
    write_cmos_sensor(0x020F, 0x00); 
    write_cmos_sensor(0x0226, 0x01); 
    write_cmos_sensor(0x0227, 0x00);
	write_cmos_sensor(0x0100,0x01);   
	
}
//
//kal_uint8  test_pattern_flag_imx=0;

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if(imgsensor.current_scenario_id != MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG)
	   {
		   if(enable) 
		   {   
			   //1640 x 1232
			   // enable color bar
			   //test_pattern_flag_imx =TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   //1640 x 1232
			  // test_pattern_flag_imx=FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x00); 	 
			   write_cmos_sensor(0x0624, 0x06); //W:3280---h
			   write_cmos_sensor(0x0625, 0x68); //		  l
			   write_cmos_sensor(0x0626, 0x04); //H:2464   h
			   write_cmos_sensor(0x0627, 0xd0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x06); //W 		h
			   write_cmos_sensor(0x613D, 0x68); //		   l
			   write_cmos_sensor(0x613E, 0x04); //H 		h
			   write_cmos_sensor(0x613F, 0xd0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   }
	   }
	   else
	   {
		   if(enable) 
		   {   
			   //3280 x 2464
			   // enable color bar
			   //test_pattern_flag_imx=TRUE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
		   } 
		   else 
		   {   
			   //test_pattern_flag_imx=FALSE;
			   write_cmos_sensor(0x0600, 0x00); 
			   write_cmos_sensor(0x0601, 0x02); 	 
			   write_cmos_sensor(0x0624, 0x0C); //W:3280---h
			   write_cmos_sensor(0x0625, 0xD0); //		  l
			   write_cmos_sensor(0x0626, 0x09); //H:2464   h
			   write_cmos_sensor(0x0627, 0xA0); //		  l
			   write_cmos_sensor(0x6128, 0x00); 
			   write_cmos_sensor(0x6129, 0x02); 		 
			   write_cmos_sensor(0x613C, 0x0C); //W 		h
			   write_cmos_sensor(0x613D, 0xD0); //		   l
			   write_cmos_sensor(0x613E, 0x09); //H 		h
			   write_cmos_sensor(0x613F, 0xA0); //			   l
			   write_cmos_sensor(0x6506, 0x00); 
			   write_cmos_sensor(0x6507, 0x00);
	
	
		   }
	   }
		   
	   return ERROR_NONE;


	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 20;

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, i2c write id: 0x%x id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 20;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 20;
	kal_uint32 sensor_id = 0; 
	LOG_1;
	LOG_2;

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				printk("imx362 i2c write id: 0x%x, retry = %d, sensor id: 0x%x\n", imgsensor.i2c_write_id, retry, sensor_id);
				break;
			}	
			printk("imx362 Read sensor id:0x%x  fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 20;
	}		 

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
#ifdef CONFIG_FAKE_DUAL_CAMERA
    if(openYuvSensor() < 0)
      printk("open yuv sensor fail!\n");
   msleep(50);
#endif
	/* initail sequence write in  */
	sensor_init();
	
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	mode_change = 1;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate)
		pr_info("capture30fps: use cap30FPS's setting: %d fps!\n",
			imgsensor.current_fps / 10);

	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_info("%s.\n", __func__);
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;

	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_info("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			  (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
		    / framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength) : 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
		frame_length = imgsensor_info.cap.pclk
			/ framerate * 10 / imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.cap.framelength)
		? (frame_length - imgsensor_info.cap.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.slim_video.framelength)
		? (frame_length - imgsensor_info.slim_video.framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	default:		/* coding with	preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		pr_info("error scenario_id = %d, we use preview scenario\n",
			scenario_id);

		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}



static kal_uint32 streaming_control(kal_bool enable)
{
	pr_info("streaming_enable(0=Sw tandby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
//	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	//LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            //LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_en = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(
		    (UINT16) (*feature_data), (UINT16) (*(feature_data + 1)),
			(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_info("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;

	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_info("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
		case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
					imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
					= imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			break;
#ifdef CONFIG_FAKE_DUAL_CAMERA
    case SENSOR_FEATURE_GET_YUV_SENSOR_BV:
      printk("SENSOR_FEATURE_GET_YUV_SENSOR_BV\n");
      GetYUVSensorBV(feature_data_32);
      break;
#endif
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX362_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
