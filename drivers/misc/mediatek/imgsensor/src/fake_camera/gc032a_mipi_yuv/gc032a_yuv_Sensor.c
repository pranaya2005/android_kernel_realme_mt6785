/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC032Amipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
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
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "gc032a_yuv_Sensor.h"
#include "fake_camera_define.h"

#define Sleep(ms) mdelay(ms)

UINT16 read_cmos_sensor(kal_uint8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC032A_READ_ID))
  {
    YUV_DBG("ERROR: gc032aMIPI_read_cmos_sensor \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C(puSendCmd , 2, GC032A_WRITE_ID);
}

UINT16 read_cmos_sensor2(kal_uint8 addr)
{
  UINT16 get_byte = 0;
  char puSendCmd = { (char)(addr & 0xFF) };
  kdSetI2CSpeed(100);
  if(0 != iReadRegI2C2(&puSendCmd , 1, (u8*)&get_byte, 1, GC032A_READ_ID))
  {
    YUV_DBG("ERROR: gc032aMIPI_read_cmos_sensor2 \n");
    return 0;
  }
  if(get_byte == 0)  //for ata yuv sensor shutter
  {
    get_byte = 1;
  }
  return get_byte;
}

static void write_cmos_sensor2(kal_uint8 addr, kal_uint8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
  kdSetI2CSpeed(100);
  iWriteRegI2C2(puSendCmd , 2, GC032A_WRITE_ID);
}

UINT16 GC032A_MIPI_GetYUVSensorBV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  //write_cmos_sensor(0xfe,0x01);
  //light = GC032A_read_cmos_sensor(0x14);

  //write_cmos_sensor(0xfe,0x00);
  temp_reg1 = read_cmos_sensor(0x04);
  temp_reg2 = read_cmos_sensor(0x03);

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
  write_cmos_sensor(0xfe, 0x00);
  light = read_cmos_sensor(0xef);
  YUV_DBG("gc032a light = %d shutter = %d\n",light,shutter);

  return light;
}
UINT16 GC032A_MIPI_GetYUVSensor2BV(void)
{
  UINT8 temp_reg1, temp_reg2;
  UINT16 shutter,light;

  //write_cmos_sensor(0xfe,0x01);
  //light = GC032A_read_cmos_sensor(0x14);

  //write_cmos_sensor(0xfe,0x00);
  temp_reg1 = read_cmos_sensor2(0x04);
  temp_reg2 = read_cmos_sensor2(0x03);

  shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);
  write_cmos_sensor2(0xfe, 0x00);
  light = read_cmos_sensor2(0xef);
  YUV_DBG("gc032a light = %d shutter = %d\n",light,shutter);

  return light;
}
static void sensor_init(void)
{
	write_cmos_sensor(0xf3,0xff);
	write_cmos_sensor(0xf5,0x06);
	write_cmos_sensor(0xf7,0x01);
	write_cmos_sensor(0xf8,0x03);
	write_cmos_sensor(0xf9,0xce); 
	write_cmos_sensor(0xfa,0x00);
	write_cmos_sensor(0xfc,0x02);
	write_cmos_sensor(0xfe,0x02);
	write_cmos_sensor(0x81,0x03); 

	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x77,0x64);
	write_cmos_sensor(0x78,0x40);
	write_cmos_sensor(0x79,0x60);
	/*ANALOG & CISCTL*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x03,0x01);
	write_cmos_sensor(0x04,0xce);
	write_cmos_sensor(0x05,0x01);
	write_cmos_sensor(0x06,0xad);
	write_cmos_sensor(0x07,0x00);
	write_cmos_sensor(0x08,0x10);
	write_cmos_sensor(0x0a,0x00);
	write_cmos_sensor(0x0c,0x00);
	write_cmos_sensor(0x0d,0x01);
	write_cmos_sensor(0x0e,0xe8);
	write_cmos_sensor(0x0f,0x02);
	write_cmos_sensor(0x10,0x88);
	write_cmos_sensor(0x17,0x54);
	write_cmos_sensor(0x19,0x08);
	write_cmos_sensor(0x1a,0x0a);
	write_cmos_sensor(0x1f,0x40);
	write_cmos_sensor(0x20,0x30);
	write_cmos_sensor(0x2e,0x80);
	write_cmos_sensor(0x2f,0x2b);
	write_cmos_sensor(0x30,0x1a);
	write_cmos_sensor(0xfe,0x02);
	write_cmos_sensor(0x03,0x02);
	write_cmos_sensor(0x05,0xd7);
	write_cmos_sensor(0x06,0x60);
	write_cmos_sensor(0x08,0x80);
	write_cmos_sensor(0x12,0x89);

	/*blk*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x18,0x02);
	write_cmos_sensor(0xfe,0x02);
	write_cmos_sensor(0x40,0x22);
	write_cmos_sensor(0x45,0x00);
	write_cmos_sensor(0x46,0x00);
	write_cmos_sensor(0x49,0x20);
	write_cmos_sensor(0x4b,0x3c);
	write_cmos_sensor(0x50,0x20);
	write_cmos_sensor(0x42,0x10);

	/*isp*/
	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0x0a,0xc5);
	write_cmos_sensor(0x45,0x00);
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x40,0xff);
	write_cmos_sensor(0x41,0x25);
	write_cmos_sensor(0x42,0xcf);
	write_cmos_sensor(0x43,0x10);
	write_cmos_sensor(0x44,0x83);
	write_cmos_sensor(0x46,0x22);
	write_cmos_sensor(0x49,0x03);
	write_cmos_sensor(0x52,0x02);
	write_cmos_sensor(0x54,0x00);
	write_cmos_sensor(0xfe,0x02);
	write_cmos_sensor(0x22,0xf6);

	/*Shading*/
	write_cmos_sensor(0xfe,0x01);	
	write_cmos_sensor(0xc1,0x38);
	write_cmos_sensor(0xc2,0x4c);
	write_cmos_sensor(0xc3,0x00);
	write_cmos_sensor(0xc4,0x32);
	write_cmos_sensor(0xc5,0x24);
	write_cmos_sensor(0xc6,0x16);
	write_cmos_sensor(0xc7,0x08);
	write_cmos_sensor(0xc8,0x08);
	write_cmos_sensor(0xc9,0x00);
	write_cmos_sensor(0xca,0x20);
	write_cmos_sensor(0xdc,0x8a);
	write_cmos_sensor(0xdd,0xa0);
	write_cmos_sensor(0xde,0xa6);
	write_cmos_sensor(0xdf,0x75);

	/*AWB*/
	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0x7c,0x09);
	write_cmos_sensor(0x65,0x06);
	write_cmos_sensor(0x7c,0x08);
	write_cmos_sensor(0x56,0xf4); 
	write_cmos_sensor(0x66,0x0f); 
	write_cmos_sensor(0x67,0x84);
	write_cmos_sensor(0x6b,0x80);
	write_cmos_sensor(0x6d,0x12);
	write_cmos_sensor(0x6e,0xb0); 
	write_cmos_sensor(0x86,0x00);
	write_cmos_sensor(0x87,0x00);
	write_cmos_sensor(0x88,0x00);
	write_cmos_sensor(0x89,0x00);
	write_cmos_sensor(0x8a,0x00);
	write_cmos_sensor(0x8b,0x00);
	write_cmos_sensor(0x8c,0x00);
	write_cmos_sensor(0x8d,0x00);
	write_cmos_sensor(0x8e,0x00);
	write_cmos_sensor(0x8f,0x00);
	write_cmos_sensor(0x90,0x00);
	write_cmos_sensor(0x91,0x00);
	write_cmos_sensor(0x92,0xf4);
	write_cmos_sensor(0x93,0xd5);
	write_cmos_sensor(0x94,0x50);
	write_cmos_sensor(0x95,0x0f);
	write_cmos_sensor(0x96,0xf4);
	write_cmos_sensor(0x97,0x2d);
	write_cmos_sensor(0x98,0x0f);
	write_cmos_sensor(0x99,0xa6);
	write_cmos_sensor(0x9a,0x2d);
	write_cmos_sensor(0x9b,0x0f);
	write_cmos_sensor(0x9c,0x59);
	write_cmos_sensor(0x9d,0x2d);
	write_cmos_sensor(0x9e,0xaa);
	write_cmos_sensor(0x9f,0x67);
	write_cmos_sensor(0xa0,0x59);
	write_cmos_sensor(0xa1,0x00);
	write_cmos_sensor(0xa2,0x00);
	write_cmos_sensor(0xa3,0x0a);
	write_cmos_sensor(0xa4,0x00);
	write_cmos_sensor(0xa5,0x00);
	write_cmos_sensor(0xa6,0xd4);
	write_cmos_sensor(0xa7,0x9f);
	write_cmos_sensor(0xa8,0x55);
	write_cmos_sensor(0xa9,0xd4);
	write_cmos_sensor(0xaa,0x9f);
	write_cmos_sensor(0xab,0xac);
	write_cmos_sensor(0xac,0x9f);
	write_cmos_sensor(0xad,0x55);
	write_cmos_sensor(0xae,0xd4);
	write_cmos_sensor(0xaf,0xac);
	write_cmos_sensor(0xb0,0xd4);
	write_cmos_sensor(0xb1,0xa3);
	write_cmos_sensor(0xb2,0x55);
	write_cmos_sensor(0xb3,0xd4);
	write_cmos_sensor(0xb4,0xac);
	write_cmos_sensor(0xb5,0x00);
	write_cmos_sensor(0xb6,0x00);
	write_cmos_sensor(0xb7,0x05);
	write_cmos_sensor(0xb8,0xd6);
	write_cmos_sensor(0xb9,0x8c);

	/*CC*/
	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0xd0,0x40);
	write_cmos_sensor(0xd1,0xf8);
	write_cmos_sensor(0xd2,0x00);
	write_cmos_sensor(0xd3,0xfa);
	write_cmos_sensor(0xd4,0x45);
	write_cmos_sensor(0xd5,0x02);

	write_cmos_sensor(0xd6,0x30);
	write_cmos_sensor(0xd7,0xfa);
	write_cmos_sensor(0xd8,0x08);
	write_cmos_sensor(0xd9,0x08);
	write_cmos_sensor(0xda,0x58);
	write_cmos_sensor(0xdb,0x02);
	write_cmos_sensor(0xfe,0x00);

	/*Gamma*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0xba,0x00);
	write_cmos_sensor(0xbb,0x04);
	write_cmos_sensor(0xbc,0x0a);
	write_cmos_sensor(0xbd,0x0e);
	write_cmos_sensor(0xbe,0x22);
	write_cmos_sensor(0xbf,0x30);
	write_cmos_sensor(0xc0,0x3d);
	write_cmos_sensor(0xc1,0x4a);
	write_cmos_sensor(0xc2,0x5d);
	write_cmos_sensor(0xc3,0x6b);
	write_cmos_sensor(0xc4,0x7a);
	write_cmos_sensor(0xc5,0x85);
	write_cmos_sensor(0xc6,0x90);
	write_cmos_sensor(0xc7,0xa5);
	write_cmos_sensor(0xc8,0xb5);
	write_cmos_sensor(0xc9,0xc2);
	write_cmos_sensor(0xca,0xcc);
	write_cmos_sensor(0xcb,0xd5);
	write_cmos_sensor(0xcc,0xde);
	write_cmos_sensor(0xcd,0xea);
	write_cmos_sensor(0xce,0xf5);
	write_cmos_sensor(0xcf,0xff);

	/*Auto Gamma*/                      
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x5a,0x08);
	write_cmos_sensor(0x5b,0x0f);
	write_cmos_sensor(0x5c,0x15);
	write_cmos_sensor(0x5d,0x1c);
	write_cmos_sensor(0x5e,0x28);
	write_cmos_sensor(0x5f,0x36);
	write_cmos_sensor(0x60,0x45);
	write_cmos_sensor(0x61,0x51);
	write_cmos_sensor(0x62,0x6a);
	write_cmos_sensor(0x63,0x7d);
	write_cmos_sensor(0x64,0x8d);
	write_cmos_sensor(0x65,0x98);
	write_cmos_sensor(0x66,0xa2);
	write_cmos_sensor(0x67,0xb5);
	write_cmos_sensor(0x68,0xc3);
	write_cmos_sensor(0x69,0xcd);
	write_cmos_sensor(0x6a,0xd4);
	write_cmos_sensor(0x6b,0xdc);
	write_cmos_sensor(0x6c,0xe3);
	write_cmos_sensor(0x6d,0xf0);
	write_cmos_sensor(0x6e,0xf9);
	write_cmos_sensor(0x6f,0xff);

	/*Gain*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x70,0x50);

	/*AEC*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x4f,0x01);
	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0x0d,0x00);
	write_cmos_sensor(0x12,0xa0);
	write_cmos_sensor(0x13,0x3a);	
	write_cmos_sensor(0x44,0x04);
	write_cmos_sensor(0x1f,0x30);
	write_cmos_sensor(0x20,0x40);	
	write_cmos_sensor(0x26,0x9a);
	write_cmos_sensor(0x3e,0x20);
	write_cmos_sensor(0x3f,0x2d);
	write_cmos_sensor(0x40,0x40);
	write_cmos_sensor(0x41,0x5b);
	write_cmos_sensor(0x42,0x82);
	write_cmos_sensor(0x43,0xb7);
	write_cmos_sensor(0x04,0x0a);
	write_cmos_sensor(0x02,0x79);
	write_cmos_sensor(0x03,0xc0);

	/*measure window*/
	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0xcc,0x08);
	write_cmos_sensor(0xcd,0x08);
	write_cmos_sensor(0xce,0xa4);
	write_cmos_sensor(0xcf,0xec);

	/*DNDD*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x81,0xb8);
	write_cmos_sensor(0x82,0x12);
	write_cmos_sensor(0x83,0x0a);
	write_cmos_sensor(0x84,0x01);
	write_cmos_sensor(0x86,0x50);
	write_cmos_sensor(0x87,0x18);
	write_cmos_sensor(0x88,0x10);
	write_cmos_sensor(0x89,0x70);
	write_cmos_sensor(0x8a,0x20);
	write_cmos_sensor(0x8b,0x10);
	write_cmos_sensor(0x8c,0x08);
	write_cmos_sensor(0x8d,0x0a);

	/*Intpee*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x8f,0xaa);
	write_cmos_sensor(0x90,0x9c);
	write_cmos_sensor(0x91,0x52);
	write_cmos_sensor(0x92,0x03);
	write_cmos_sensor(0x93,0x03);
	write_cmos_sensor(0x94,0x08);
	write_cmos_sensor(0x95,0x44);
	write_cmos_sensor(0x97,0x00);
	write_cmos_sensor(0x98,0x00);

	/*ASDE*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0xa1,0x30);
	write_cmos_sensor(0xa2,0x41);
	write_cmos_sensor(0xa4,0x30);
	write_cmos_sensor(0xa5,0x20);
	write_cmos_sensor(0xaa,0x30);
	write_cmos_sensor(0xac,0x32);

	/*YCP*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0xd1,0x3c);
	write_cmos_sensor(0xd2,0x3c);
	write_cmos_sensor(0xd3,0x38);
	write_cmos_sensor(0xd6,0xf4);
	write_cmos_sensor(0xd7,0x1d);
	write_cmos_sensor(0xdd,0x73);
	write_cmos_sensor(0xde,0x84);

	/*Banding*/
	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0x05,0x01);
	write_cmos_sensor(0x06,0xad);
	write_cmos_sensor(0x07,0x00);
	write_cmos_sensor(0x08,0x10);

	write_cmos_sensor(0xfe,0x01);
	write_cmos_sensor(0x25,0x00);
	write_cmos_sensor(0x26,0x9a);

	write_cmos_sensor(0x27,0x01);
	write_cmos_sensor(0x28,0xce);
	write_cmos_sensor(0x29,0x03);
	write_cmos_sensor(0x2a,0x02);
	write_cmos_sensor(0x2b,0x04);
	write_cmos_sensor(0x2c,0x36);
	write_cmos_sensor(0x2d,0x07);
	write_cmos_sensor(0x2e,0xd2);
	write_cmos_sensor(0x2f,0x0b);
	write_cmos_sensor(0x30,0x6e);
	write_cmos_sensor(0x31,0x0e);
	write_cmos_sensor(0x32,0x70);
	write_cmos_sensor(0x33,0x12);
	write_cmos_sensor(0x34,0x0c);
	write_cmos_sensor(0x3c,0x30);
	write_cmos_sensor(0xfe,0x00);
}    /*    sensor_init  */
static void sensor2_init(void)
{
    write_cmos_sensor2(0xf3,0xff);
    write_cmos_sensor2(0xf5,0x06);
    write_cmos_sensor2(0xf7,0x01);
    write_cmos_sensor2(0xf8,0x03);
    write_cmos_sensor2(0xf9,0xce); 
    write_cmos_sensor2(0xfa,0x00);
    write_cmos_sensor2(0xfc,0x02);
    write_cmos_sensor2(0xfe,0x02);
    write_cmos_sensor2(0x81,0x03); 

    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x77,0x64);
    write_cmos_sensor2(0x78,0x40);
    write_cmos_sensor2(0x79,0x60);
    /*ANALOG & CISCTL*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x03,0x01);
    write_cmos_sensor2(0x04,0xce);
    write_cmos_sensor2(0x05,0x01);
    write_cmos_sensor2(0x06,0xad);
    write_cmos_sensor2(0x07,0x00);
    write_cmos_sensor2(0x08,0x10);
    write_cmos_sensor2(0x0a,0x00);
    write_cmos_sensor2(0x0c,0x00);
    write_cmos_sensor2(0x0d,0x01);
    write_cmos_sensor2(0x0e,0xe8);
    write_cmos_sensor2(0x0f,0x02);
    write_cmos_sensor2(0x10,0x88);
    write_cmos_sensor2(0x17,0x54);
    write_cmos_sensor2(0x19,0x08);
    write_cmos_sensor2(0x1a,0x0a);
    write_cmos_sensor2(0x1f,0x40);
    write_cmos_sensor2(0x20,0x30);
    write_cmos_sensor2(0x2e,0x80);
    write_cmos_sensor2(0x2f,0x2b);
    write_cmos_sensor2(0x30,0x1a);
    write_cmos_sensor2(0xfe,0x02);
    write_cmos_sensor2(0x03,0x02);
    write_cmos_sensor2(0x05,0xd7);
    write_cmos_sensor2(0x06,0x60);
    write_cmos_sensor2(0x08,0x80);
    write_cmos_sensor2(0x12,0x89);

    /*blk*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x18,0x02);
    write_cmos_sensor2(0xfe,0x02);
    write_cmos_sensor2(0x40,0x22);
    write_cmos_sensor2(0x45,0x00);
    write_cmos_sensor2(0x46,0x00);
    write_cmos_sensor2(0x49,0x20);
    write_cmos_sensor2(0x4b,0x3c);
    write_cmos_sensor2(0x50,0x20);
    write_cmos_sensor2(0x42,0x10);

    /*isp*/
    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0x0a,0xc5);
    write_cmos_sensor2(0x45,0x00);
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x40,0xff);
    write_cmos_sensor2(0x41,0x25);
    write_cmos_sensor2(0x42,0xcf);
    write_cmos_sensor2(0x43,0x10);
    write_cmos_sensor2(0x44,0x83);
    write_cmos_sensor2(0x46,0x22);
    write_cmos_sensor2(0x49,0x03);
    write_cmos_sensor2(0x52,0x02);
    write_cmos_sensor2(0x54,0x00);
    write_cmos_sensor2(0xfe,0x02);
    write_cmos_sensor2(0x22,0xf6);

    /*Shading*/
    write_cmos_sensor2(0xfe,0x01);   
    write_cmos_sensor2(0xc1,0x38);
    write_cmos_sensor2(0xc2,0x4c);
    write_cmos_sensor2(0xc3,0x00);
    write_cmos_sensor2(0xc4,0x32);
    write_cmos_sensor2(0xc5,0x24);
    write_cmos_sensor2(0xc6,0x16);
    write_cmos_sensor2(0xc7,0x08);
    write_cmos_sensor2(0xc8,0x08);
    write_cmos_sensor2(0xc9,0x00);
    write_cmos_sensor2(0xca,0x20);
    write_cmos_sensor2(0xdc,0x8a);
    write_cmos_sensor2(0xdd,0xa0);
    write_cmos_sensor2(0xde,0xa6);
    write_cmos_sensor2(0xdf,0x75);

    /*AWB*/
    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0x7c,0x09);
    write_cmos_sensor2(0x65,0x06);
    write_cmos_sensor2(0x7c,0x08);
    write_cmos_sensor2(0x56,0xf4); 
    write_cmos_sensor2(0x66,0x0f); 
    write_cmos_sensor2(0x67,0x84);
    write_cmos_sensor2(0x6b,0x80);
    write_cmos_sensor2(0x6d,0x12);
    write_cmos_sensor2(0x6e,0xb0); 
    write_cmos_sensor2(0x86,0x00);
    write_cmos_sensor2(0x87,0x00);
    write_cmos_sensor2(0x88,0x00);
    write_cmos_sensor2(0x89,0x00);
    write_cmos_sensor2(0x8a,0x00);
    write_cmos_sensor2(0x8b,0x00);
    write_cmos_sensor2(0x8c,0x00);
    write_cmos_sensor2(0x8d,0x00);
    write_cmos_sensor2(0x8e,0x00);
    write_cmos_sensor2(0x8f,0x00);
    write_cmos_sensor2(0x90,0x00);
    write_cmos_sensor2(0x91,0x00);
    write_cmos_sensor2(0x92,0xf4);
    write_cmos_sensor2(0x93,0xd5);
    write_cmos_sensor2(0x94,0x50);
    write_cmos_sensor2(0x95,0x0f);
    write_cmos_sensor2(0x96,0xf4);
    write_cmos_sensor2(0x97,0x2d);
    write_cmos_sensor2(0x98,0x0f);
    write_cmos_sensor2(0x99,0xa6);
    write_cmos_sensor2(0x9a,0x2d);
    write_cmos_sensor2(0x9b,0x0f);
    write_cmos_sensor2(0x9c,0x59);
    write_cmos_sensor2(0x9d,0x2d);
    write_cmos_sensor2(0x9e,0xaa);
    write_cmos_sensor2(0x9f,0x67);
    write_cmos_sensor2(0xa0,0x59);
    write_cmos_sensor2(0xa1,0x00);
    write_cmos_sensor2(0xa2,0x00);
    write_cmos_sensor2(0xa3,0x0a);
    write_cmos_sensor2(0xa4,0x00);
    write_cmos_sensor2(0xa5,0x00);
    write_cmos_sensor2(0xa6,0xd4);
    write_cmos_sensor2(0xa7,0x9f);
    write_cmos_sensor2(0xa8,0x55);
    write_cmos_sensor2(0xa9,0xd4);
    write_cmos_sensor2(0xaa,0x9f);
    write_cmos_sensor2(0xab,0xac);
    write_cmos_sensor2(0xac,0x9f);
    write_cmos_sensor2(0xad,0x55);
    write_cmos_sensor2(0xae,0xd4);
    write_cmos_sensor2(0xaf,0xac);
    write_cmos_sensor2(0xb0,0xd4);
    write_cmos_sensor2(0xb1,0xa3);
    write_cmos_sensor2(0xb2,0x55);
    write_cmos_sensor2(0xb3,0xd4);
    write_cmos_sensor2(0xb4,0xac);
    write_cmos_sensor2(0xb5,0x00);
    write_cmos_sensor2(0xb6,0x00);
    write_cmos_sensor2(0xb7,0x05);
    write_cmos_sensor2(0xb8,0xd6);
    write_cmos_sensor2(0xb9,0x8c);

    /*CC*/
    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0xd0,0x40);
    write_cmos_sensor2(0xd1,0xf8);
    write_cmos_sensor2(0xd2,0x00);
    write_cmos_sensor2(0xd3,0xfa);
    write_cmos_sensor2(0xd4,0x45);
    write_cmos_sensor2(0xd5,0x02);

    write_cmos_sensor2(0xd6,0x30);
    write_cmos_sensor2(0xd7,0xfa);
    write_cmos_sensor2(0xd8,0x08);
    write_cmos_sensor2(0xd9,0x08);
    write_cmos_sensor2(0xda,0x58);
    write_cmos_sensor2(0xdb,0x02);
    write_cmos_sensor2(0xfe,0x00);

    /*Gamma*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0xba,0x00);
    write_cmos_sensor2(0xbb,0x04);
    write_cmos_sensor2(0xbc,0x0a);
    write_cmos_sensor2(0xbd,0x0e);
    write_cmos_sensor2(0xbe,0x22);
    write_cmos_sensor2(0xbf,0x30);
    write_cmos_sensor2(0xc0,0x3d);
    write_cmos_sensor2(0xc1,0x4a);
    write_cmos_sensor2(0xc2,0x5d);
    write_cmos_sensor2(0xc3,0x6b);
    write_cmos_sensor2(0xc4,0x7a);
    write_cmos_sensor2(0xc5,0x85);
    write_cmos_sensor2(0xc6,0x90);
    write_cmos_sensor2(0xc7,0xa5);
    write_cmos_sensor2(0xc8,0xb5);
    write_cmos_sensor2(0xc9,0xc2);
    write_cmos_sensor2(0xca,0xcc);
    write_cmos_sensor2(0xcb,0xd5);
    write_cmos_sensor2(0xcc,0xde);
    write_cmos_sensor2(0xcd,0xea);
    write_cmos_sensor2(0xce,0xf5);
    write_cmos_sensor2(0xcf,0xff);

    /*Auto Gamma*/                      
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x5a,0x08);
    write_cmos_sensor2(0x5b,0x0f);
    write_cmos_sensor2(0x5c,0x15);
    write_cmos_sensor2(0x5d,0x1c);
    write_cmos_sensor2(0x5e,0x28);
    write_cmos_sensor2(0x5f,0x36);
    write_cmos_sensor2(0x60,0x45);
    write_cmos_sensor2(0x61,0x51);
    write_cmos_sensor2(0x62,0x6a);
    write_cmos_sensor2(0x63,0x7d);
    write_cmos_sensor2(0x64,0x8d);
    write_cmos_sensor2(0x65,0x98);
    write_cmos_sensor2(0x66,0xa2);
    write_cmos_sensor2(0x67,0xb5);
    write_cmos_sensor2(0x68,0xc3);
    write_cmos_sensor2(0x69,0xcd);
    write_cmos_sensor2(0x6a,0xd4);
    write_cmos_sensor2(0x6b,0xdc);
    write_cmos_sensor2(0x6c,0xe3);
    write_cmos_sensor2(0x6d,0xf0);
    write_cmos_sensor2(0x6e,0xf9);
    write_cmos_sensor2(0x6f,0xff);

    /*Gain*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x70,0x50);

    /*AEC*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x4f,0x01);
    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0x0d,0x00);
    write_cmos_sensor2(0x12,0xa0);
    write_cmos_sensor2(0x13,0x3a);   
    write_cmos_sensor2(0x44,0x04);
    write_cmos_sensor2(0x1f,0x30);
    write_cmos_sensor2(0x20,0x40);   
    write_cmos_sensor2(0x26,0x9a);
    write_cmos_sensor2(0x3e,0x20);
    write_cmos_sensor2(0x3f,0x2d);
    write_cmos_sensor2(0x40,0x40);
    write_cmos_sensor2(0x41,0x5b);
    write_cmos_sensor2(0x42,0x82);
    write_cmos_sensor2(0x43,0xb7);
    write_cmos_sensor2(0x04,0x0a);
    write_cmos_sensor2(0x02,0x79);
    write_cmos_sensor2(0x03,0xc0);

    /*measure window*/
    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0xcc,0x08);
    write_cmos_sensor2(0xcd,0x08);
    write_cmos_sensor2(0xce,0xa4);
    write_cmos_sensor2(0xcf,0xec);

    /*DNDD*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x81,0xb8);
    write_cmos_sensor2(0x82,0x12);
    write_cmos_sensor2(0x83,0x0a);
    write_cmos_sensor2(0x84,0x01);
    write_cmos_sensor2(0x86,0x50);
    write_cmos_sensor2(0x87,0x18);
    write_cmos_sensor2(0x88,0x10);
    write_cmos_sensor2(0x89,0x70);
    write_cmos_sensor2(0x8a,0x20);
    write_cmos_sensor2(0x8b,0x10);
    write_cmos_sensor2(0x8c,0x08);
    write_cmos_sensor2(0x8d,0x0a);

    /*Intpee*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x8f,0xaa);
    write_cmos_sensor2(0x90,0x9c);
    write_cmos_sensor2(0x91,0x52);
    write_cmos_sensor2(0x92,0x03);
    write_cmos_sensor2(0x93,0x03);
    write_cmos_sensor2(0x94,0x08);
    write_cmos_sensor2(0x95,0x44);
    write_cmos_sensor2(0x97,0x00);
    write_cmos_sensor2(0x98,0x00);

    /*ASDE*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0xa1,0x30);
    write_cmos_sensor2(0xa2,0x41);
    write_cmos_sensor2(0xa4,0x30);
    write_cmos_sensor2(0xa5,0x20);
    write_cmos_sensor2(0xaa,0x30);
    write_cmos_sensor2(0xac,0x32);

    /*YCP*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0xd1,0x3c);
    write_cmos_sensor2(0xd2,0x3c);
    write_cmos_sensor2(0xd3,0x38);
    write_cmos_sensor2(0xd6,0xf4);
    write_cmos_sensor2(0xd7,0x1d);
    write_cmos_sensor2(0xdd,0x73);
    write_cmos_sensor2(0xde,0x84);

    /*Banding*/
    write_cmos_sensor2(0xfe,0x00);
    write_cmos_sensor2(0x05,0x01);
    write_cmos_sensor2(0x06,0xad);
    write_cmos_sensor2(0x07,0x00);
    write_cmos_sensor2(0x08,0x10);

    write_cmos_sensor2(0xfe,0x01);
    write_cmos_sensor2(0x25,0x00);
    write_cmos_sensor2(0x26,0x9a);

    write_cmos_sensor2(0x27,0x01);
    write_cmos_sensor2(0x28,0xce);
    write_cmos_sensor2(0x29,0x03);
    write_cmos_sensor2(0x2a,0x02);
    write_cmos_sensor2(0x2b,0x04);
    write_cmos_sensor2(0x2c,0x36);
    write_cmos_sensor2(0x2d,0x07);
    write_cmos_sensor2(0x2e,0xd2);
    write_cmos_sensor2(0x2f,0x0b);
    write_cmos_sensor2(0x30,0x6e);
    write_cmos_sensor2(0x31,0x0e);
    write_cmos_sensor2(0x32,0x70);
    write_cmos_sensor2(0x33,0x12);
    write_cmos_sensor2(0x34,0x0c);
    write_cmos_sensor2(0x3c,0x30);
    write_cmos_sensor2(0xfe,0x00);
}    /*    sensor_init  */


UINT32 gc032a_read_id(void)
{
  volatile signed char i;
  UINT16 sensor_id = 0,sensor2_id = 0;
  for(i=0;i<3;i++)
  {
    sensor_id = ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
    if(sensor_id==GC032A_YUV_ID)
      break;
  }
  for(i=0;i<3;i++)
  {
    sensor2_id = ((read_cmos_sensor2(0xf0) << 8) | read_cmos_sensor2(0xf1));
    if(sensor2_id==GC032A_YUV_ID)
      break;
  }

  YUV_DBG("yuv sensor_id = 0x%x,sensor2_id = 0x%x\n",sensor_id,sensor2_id);
  return (((sensor_id==GC032A_YUV_ID)&&(sensor2_id==GC032A_YUV_ID))?1:0);
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void gc032a_open(void)
{
  YUV_DBG("%s\n",__func__);

    sensor_init();
    sensor2_init();
}
YUV_SENSOR_FUNC gc032a_sensor_func = {

  gc032a_open,
  GC032A_MIPI_GetYUVSensorBV,
  GC032A_MIPI_GetYUVSensor2BV,
};

UINT32 GC032A_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
  if(NULL != pfFunc)
    *pfFunc = &gc032a_sensor_func;
  YUV_DBG("%s\n",__func__);
  return gc032a_read_id();
}
