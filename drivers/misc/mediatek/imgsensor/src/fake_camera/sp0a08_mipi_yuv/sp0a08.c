/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   SP0A08yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.2.3
 *
 * Author:
 * -------
 *   Leo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2012.02.29  kill bugs
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GCoreinc. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "sp0a08.h"
#include "fake_camera_define.h"

static UINT16 SP0A08_read_cmos_sensor(UINT8 addr)
{
	UINT16 get_byte = 0;
	char puSendCmd = { (char)(addr & 0xFF) };
	if(0 != iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, SP0A08_READ_ID))
	{
		YUV_DBG("ERROR: SP0A08MIPI_read_cmos_sensor \n");
		return 0;
	}
	if(get_byte == 0)  //for ata yuv sensor shutter
	{
		get_byte = 1;
	}
	return get_byte;
}

static void SP0A08_write_cmos_sensor(UINT8 addr, UINT8 para)
{
	char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 2, SP0A08_WRITE_ID);
}

UINT16 SP0A08_MIPI_GetYUVSensorBV(void)
{
	UINT8 temp_reg1, temp_reg2;
	UINT16 shutter,light;

	SP0A08_write_cmos_sensor(0xfd,0x00);
	temp_reg1 = SP0A08_read_cmos_sensor(0x04); //LOW 8
	temp_reg2 = SP0A08_read_cmos_sensor(0x03); //HI 3

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	SP0A08_write_cmos_sensor(0xfd,0x01);
	light = SP0A08_read_cmos_sensor(0xf2);
	YUV_DBG("light = %d shutter = %d\n",light,shutter);

	return light;
}

void SP0A08_Sensor_Init(void)
{
	SP0A08_write_cmos_sensor(0xfd,0x00);	
	SP0A08_write_cmos_sensor(0x36,0xa0);
	SP0A08_write_cmos_sensor(0xe7,0x03);     
	SP0A08_write_cmos_sensor(0xe7,0x00);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x7c,0x6c);
	SP0A08_write_cmos_sensor(0xfd,0x00);	
	SP0A08_write_cmos_sensor(0x0e,0x01);  
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x60,0x0a);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x63,0x2a);
	SP0A08_write_cmos_sensor(0x64,0x02);
	SP0A08_write_cmos_sensor(0x65,0x80);   //88   SOC  window   2016.6.21
	SP0A08_write_cmos_sensor(0x69,0x07);
	SP0A08_write_cmos_sensor(0x66,0x73);
	SP0A08_write_cmos_sensor(0x6c,0x01);
	SP0A08_write_cmos_sensor(0x6d,0x01);
	SP0A08_write_cmos_sensor(0x6e,0x03);
	SP0A08_write_cmos_sensor(0x6f,0x02);
	SP0A08_write_cmos_sensor(0x71,0x01);
	SP0A08_write_cmos_sensor(0x72,0x04);
	SP0A08_write_cmos_sensor(0x78,0xff);
	SP0A08_write_cmos_sensor(0x79,0xff);
	SP0A08_write_cmos_sensor(0x7d,0x01);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0x31,0x10);      //Ĭ��0x10   ��50��  buf_comm_ctrl  Upside downʹ��  2016.6.22
	SP0A08_write_cmos_sensor(0x32,0x00);  
	SP0A08_write_cmos_sensor(0x0f,0x40);
	SP0A08_write_cmos_sensor(0x10,0x40);
	SP0A08_write_cmos_sensor(0x11,0x10);
	SP0A08_write_cmos_sensor(0x12,0xa0);
	SP0A08_write_cmos_sensor(0x13,0x0a);
	SP0A08_write_cmos_sensor(0x14,0x10);
	SP0A08_write_cmos_sensor(0x15,0x00); 
	SP0A08_write_cmos_sensor(0x16,0x08);
	SP0A08_write_cmos_sensor(0x1A,0x37);
	SP0A08_write_cmos_sensor(0x1B,0x17);
	SP0A08_write_cmos_sensor(0x1C,0x2f);
	SP0A08_write_cmos_sensor(0x1d,0x00);
	SP0A08_write_cmos_sensor(0x1E,0x57);
	SP0A08_write_cmos_sensor(0x21,0x34);
	SP0A08_write_cmos_sensor(0x22,0x12);
	SP0A08_write_cmos_sensor(0x24,0x80);
	SP0A08_write_cmos_sensor(0x25,0x02);
	SP0A08_write_cmos_sensor(0x26,0x03);
	SP0A08_write_cmos_sensor(0x27,0xeb);
	SP0A08_write_cmos_sensor(0x28,0x5f);
	SP0A08_write_cmos_sensor(0x5f,0x02);
	SP0A08_write_cmos_sensor(0xfb,0x33);  
	SP0A08_write_cmos_sensor(0xf4,0x09);  
	SP0A08_write_cmos_sensor(0xe7,0x03);  
	SP0A08_write_cmos_sensor(0xe7,0x00);  
	SP0A08_write_cmos_sensor(0x65,0x00);    //18  gb_suboffset    2016.6.21
	SP0A08_write_cmos_sensor(0x66,0x00);    //18  blue_suboffset
	SP0A08_write_cmos_sensor(0x67,0x00);    //18  red_suboffset
	SP0A08_write_cmos_sensor(0x68,0x00);    //18  gr_suboffset
	SP0A08_write_cmos_sensor(0xfd,0x00);  
	SP0A08_write_cmos_sensor(0x03,0x00);  
	SP0A08_write_cmos_sensor(0x04,0xf0);  
	SP0A08_write_cmos_sensor(0x05,0x00);  
	SP0A08_write_cmos_sensor(0x06,0x00);  
	SP0A08_write_cmos_sensor(0x09,0x02);  
	SP0A08_write_cmos_sensor(0x0a,0x50);  
	SP0A08_write_cmos_sensor(0xf0,0x50);  
	SP0A08_write_cmos_sensor(0xf1,0x00);  
	SP0A08_write_cmos_sensor(0xfd,0x01);  
	SP0A08_write_cmos_sensor(0x90,0x06);  
	SP0A08_write_cmos_sensor(0x92,0x01);   
	SP0A08_write_cmos_sensor(0x98,0x50);  
	SP0A08_write_cmos_sensor(0x99,0x00);  
	SP0A08_write_cmos_sensor(0x9a,0x01);  
	SP0A08_write_cmos_sensor(0x9b,0x00);  
	SP0A08_write_cmos_sensor(0xfd,0x01);  
	SP0A08_write_cmos_sensor(0xce,0xe0);  
	SP0A08_write_cmos_sensor(0xcf,0x01);  
	SP0A08_write_cmos_sensor(0xd0,0xe0);  
	SP0A08_write_cmos_sensor(0xd1,0x01);  
	SP0A08_write_cmos_sensor(0xfd,0x00);  
	SP0A08_write_cmos_sensor(0xfd,0x01);  
	SP0A08_write_cmos_sensor(0xc4,0x6c);  
	SP0A08_write_cmos_sensor(0xc5,0x7c);  
	SP0A08_write_cmos_sensor(0xca,0x30);  
	SP0A08_write_cmos_sensor(0xcb,0x45);  
	SP0A08_write_cmos_sensor(0xcc,0x60);  
	SP0A08_write_cmos_sensor(0xcd,0x60);   
	SP0A08_write_cmos_sensor(0xfd,0x00);  
	SP0A08_write_cmos_sensor(0x45,0x00);  
	SP0A08_write_cmos_sensor(0x46,0x99); 
	SP0A08_write_cmos_sensor(0x79,0xff);  
	SP0A08_write_cmos_sensor(0x7a,0xff);  
	SP0A08_write_cmos_sensor(0x7b,0x10);  
	SP0A08_write_cmos_sensor(0x7c,0x10);  
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x35,0x14);
	SP0A08_write_cmos_sensor(0x36,0x14);
	SP0A08_write_cmos_sensor(0x37,0x1c);
	SP0A08_write_cmos_sensor(0x38,0x1c);
	SP0A08_write_cmos_sensor(0x39,0x10);
	SP0A08_write_cmos_sensor(0x3a,0x10);
	SP0A08_write_cmos_sensor(0x3b,0x1a);
	SP0A08_write_cmos_sensor(0x3c,0x1a);
	SP0A08_write_cmos_sensor(0x3d,0x10);
	SP0A08_write_cmos_sensor(0x3e,0x10);
	SP0A08_write_cmos_sensor(0x3f,0x15);
	SP0A08_write_cmos_sensor(0x40,0x20);
	SP0A08_write_cmos_sensor(0x41,0x0a);
	SP0A08_write_cmos_sensor(0x42,0x08);
	SP0A08_write_cmos_sensor(0x43,0x08);
	SP0A08_write_cmos_sensor(0x44,0x0a);
	SP0A08_write_cmos_sensor(0x45,0x00);
	SP0A08_write_cmos_sensor(0x46,0x00);
	SP0A08_write_cmos_sensor(0x47,0x00);
	SP0A08_write_cmos_sensor(0x48,0xfd);
	SP0A08_write_cmos_sensor(0x49,0x00);
	SP0A08_write_cmos_sensor(0x4a,0x00);
	SP0A08_write_cmos_sensor(0x4b,0x04);
	SP0A08_write_cmos_sensor(0x4c,0xfd);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xa1,0x20);
	SP0A08_write_cmos_sensor(0xa2,0x20);
	SP0A08_write_cmos_sensor(0xa3,0x20);
	SP0A08_write_cmos_sensor(0xa4,0xff);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0xde,0x0f);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0x57,0x08);
	SP0A08_write_cmos_sensor(0x58,0x0e);
	SP0A08_write_cmos_sensor(0x56,0x10);
	SP0A08_write_cmos_sensor(0x59,0x10);
	SP0A08_write_cmos_sensor(0x89,0x08);
	SP0A08_write_cmos_sensor(0x8a,0x0e);
	SP0A08_write_cmos_sensor(0x9c,0x10);
	SP0A08_write_cmos_sensor(0x9d,0x10);
	SP0A08_write_cmos_sensor(0x81,0xd8);
	SP0A08_write_cmos_sensor(0x82,0x98);
	SP0A08_write_cmos_sensor(0x83,0x80);
	SP0A08_write_cmos_sensor(0x84,0x80);
	SP0A08_write_cmos_sensor(0x85,0xe0);
	SP0A08_write_cmos_sensor(0x86,0xa0);
	SP0A08_write_cmos_sensor(0x87,0x80);
	SP0A08_write_cmos_sensor(0x88,0x80);
	SP0A08_write_cmos_sensor(0x5a,0xff);
	SP0A08_write_cmos_sensor(0x5b,0xb8);
	SP0A08_write_cmos_sensor(0x5c,0xa0);
	SP0A08_write_cmos_sensor(0x5d,0xa0);
	SP0A08_write_cmos_sensor(0xa7,0xff);
	SP0A08_write_cmos_sensor(0xa8,0xff);
	SP0A08_write_cmos_sensor(0xa9,0xff);
	SP0A08_write_cmos_sensor(0xaa,0xff);
	SP0A08_write_cmos_sensor(0x9e,0x10);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0xe2,0x30);
	SP0A08_write_cmos_sensor(0xe4,0xa0);
	SP0A08_write_cmos_sensor(0xe5,0x08);
	SP0A08_write_cmos_sensor(0xd3,0x10);
	SP0A08_write_cmos_sensor(0xd7,0x08);
	SP0A08_write_cmos_sensor(0xe6,0x08);
	SP0A08_write_cmos_sensor(0xd4,0x10);
	SP0A08_write_cmos_sensor(0xd8,0x08);
	SP0A08_write_cmos_sensor(0xe7,0x10);
	SP0A08_write_cmos_sensor(0xd5,0x10);
	SP0A08_write_cmos_sensor(0xd9,0x10);
	SP0A08_write_cmos_sensor(0xd2,0x10);
	SP0A08_write_cmos_sensor(0xd6,0x10);
	SP0A08_write_cmos_sensor(0xda,0x10);
	SP0A08_write_cmos_sensor(0xe8,0x28);
	SP0A08_write_cmos_sensor(0xec,0x38);
	SP0A08_write_cmos_sensor(0xe9,0x20);
	SP0A08_write_cmos_sensor(0xed,0x38);
	SP0A08_write_cmos_sensor(0xea,0x10);
	SP0A08_write_cmos_sensor(0xef,0x20);
	SP0A08_write_cmos_sensor(0xeb,0x10);
	SP0A08_write_cmos_sensor(0xf0,0x10);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0xa0,0x8a);
	SP0A08_write_cmos_sensor(0xa1,0xfb);
	SP0A08_write_cmos_sensor(0xa2,0xfc);
	SP0A08_write_cmos_sensor(0xa3,0xfe);
	SP0A08_write_cmos_sensor(0xa4,0x93);
	SP0A08_write_cmos_sensor(0xa5,0xf0);
	SP0A08_write_cmos_sensor(0xa6,0x0b);
	SP0A08_write_cmos_sensor(0xa7,0xd8);
	SP0A08_write_cmos_sensor(0xa8,0x9d);
	SP0A08_write_cmos_sensor(0xa9,0x3c);
	SP0A08_write_cmos_sensor(0xaa,0x33);
	SP0A08_write_cmos_sensor(0xab,0x0c);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0x8b,0x00);
	SP0A08_write_cmos_sensor(0x8c,0x0c);
	SP0A08_write_cmos_sensor(0x8d,0x19);
	SP0A08_write_cmos_sensor(0x8e,0x2c);
	SP0A08_write_cmos_sensor(0x8f,0x49);
	SP0A08_write_cmos_sensor(0x90,0x61);
	SP0A08_write_cmos_sensor(0x91,0x77);
	SP0A08_write_cmos_sensor(0x92,0x8a);
	SP0A08_write_cmos_sensor(0x93,0x9b);
	SP0A08_write_cmos_sensor(0x94,0xa9);
	SP0A08_write_cmos_sensor(0x95,0xb5);
	SP0A08_write_cmos_sensor(0x96,0xc0);
	SP0A08_write_cmos_sensor(0x97,0xca);
	SP0A08_write_cmos_sensor(0x98,0xd4);
	SP0A08_write_cmos_sensor(0x99,0xdd);
	SP0A08_write_cmos_sensor(0x9a,0xe6);
	SP0A08_write_cmos_sensor(0x9b,0xef);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x8d,0xf7);
	SP0A08_write_cmos_sensor(0x8e,0xff);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x28,0x80);
	SP0A08_write_cmos_sensor(0x29,0x80);
	SP0A08_write_cmos_sensor(0x11,0x13);
	SP0A08_write_cmos_sensor(0x12,0x13);
	SP0A08_write_cmos_sensor(0x2e,0x13);
	SP0A08_write_cmos_sensor(0x2f,0x13);
	SP0A08_write_cmos_sensor(0x16,0x1c);
	SP0A08_write_cmos_sensor(0x17,0x1a);
	SP0A08_write_cmos_sensor(0x18,0x1a);
	SP0A08_write_cmos_sensor(0x19,0x54);
	SP0A08_write_cmos_sensor(0x1a,0xa5);
	SP0A08_write_cmos_sensor(0x1b,0x9a);
	SP0A08_write_cmos_sensor(0x2a,0xef);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xe0,0x3a);
	SP0A08_write_cmos_sensor(0xe1,0x2c);
	SP0A08_write_cmos_sensor(0xe2,0x26);
	SP0A08_write_cmos_sensor(0xe3,0x22);
	SP0A08_write_cmos_sensor(0xe4,0x22);
	SP0A08_write_cmos_sensor(0xe5,0x20);
	SP0A08_write_cmos_sensor(0xe6,0x20);
	SP0A08_write_cmos_sensor(0xe8,0x20);
	SP0A08_write_cmos_sensor(0xe9,0x20);
	SP0A08_write_cmos_sensor(0xea,0x20);
	SP0A08_write_cmos_sensor(0xeb,0x1e);
	SP0A08_write_cmos_sensor(0xf5,0x1e);
	SP0A08_write_cmos_sensor(0xf6,0x1e);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x94,0x60);
	SP0A08_write_cmos_sensor(0x95,0x1e);
	SP0A08_write_cmos_sensor(0x9c,0x60);
	SP0A08_write_cmos_sensor(0x9d,0x1e);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xed,0x94);
	SP0A08_write_cmos_sensor(0xf7,0x90);
	SP0A08_write_cmos_sensor(0xf8,0x88);
	SP0A08_write_cmos_sensor(0xec,0x84);
	SP0A08_write_cmos_sensor(0xef,0x88);
	SP0A08_write_cmos_sensor(0xf9,0x84);
	SP0A08_write_cmos_sensor(0xfa,0x7c);
	SP0A08_write_cmos_sensor(0xee,0x78);
	SP0A08_write_cmos_sensor(0xfd,0x01);
	SP0A08_write_cmos_sensor(0x30,0x40);
	SP0A08_write_cmos_sensor(0x31,0x70);
	SP0A08_write_cmos_sensor(0x32,0x20);
	SP0A08_write_cmos_sensor(0x33,0xef);
	SP0A08_write_cmos_sensor(0x34,0x02);
	SP0A08_write_cmos_sensor(0x4d,0x40);
	SP0A08_write_cmos_sensor(0x4e,0x15);
	SP0A08_write_cmos_sensor(0x4f,0x13);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xbe,0x5a);
	SP0A08_write_cmos_sensor(0xc0,0xff);
	SP0A08_write_cmos_sensor(0xc1,0xff);
	SP0A08_write_cmos_sensor(0xd3,0x78);
	SP0A08_write_cmos_sensor(0xd4,0x78);
	SP0A08_write_cmos_sensor(0xd6,0x70);
	SP0A08_write_cmos_sensor(0xd7,0x60);
	SP0A08_write_cmos_sensor(0xd8,0x78);
	SP0A08_write_cmos_sensor(0xd9,0x78);
	SP0A08_write_cmos_sensor(0xda,0x70);
	SP0A08_write_cmos_sensor(0xdb,0x60);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xdc,0x00);
	SP0A08_write_cmos_sensor(0xdd,0x88);
	SP0A08_write_cmos_sensor(0xde,0x90);
	SP0A08_write_cmos_sensor(0xdf,0x80);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0xc2,0x08);
	SP0A08_write_cmos_sensor(0xc3,0x08);
	SP0A08_write_cmos_sensor(0xc4,0x08);
	SP0A08_write_cmos_sensor(0xc5,0x10);
	SP0A08_write_cmos_sensor(0xc6,0x80);
	SP0A08_write_cmos_sensor(0xc7,0x80);
	SP0A08_write_cmos_sensor(0xc8,0x80);
	SP0A08_write_cmos_sensor(0xc9,0x80);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0x34,0x00);
	SP0A08_write_cmos_sensor(0x33,0x00);
	SP0A08_write_cmos_sensor(0x35,0x20);
	SP0A08_write_cmos_sensor(0xfd,0x00);
	SP0A08_write_cmos_sensor(0x37,0x00);
	SP0A08_write_cmos_sensor(0x38,0x00);
	SP0A08_write_cmos_sensor(0x39,0x01);
	SP0A08_write_cmos_sensor(0x3a,0xe0);
	SP0A08_write_cmos_sensor(0x3b,0x00);
	SP0A08_write_cmos_sensor(0x3c,0x00);
	SP0A08_write_cmos_sensor(0x3d,0x02);
	SP0A08_write_cmos_sensor(0x3e,0x80);
	SP0A08_write_cmos_sensor(0xfd,0x00);
}

/*************************************************************************
 * FUNCTION
 *	SP0A08Open
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
UINT32 sp0a08_read_id(void)
{
	volatile signed char i;
	UINT16 sensor_id = 0;
	for(i = 0;i < 3;i++)
	{
		SP0A08_write_cmos_sensor(0xfd,0x00);
		sensor_id = SP0A08_read_cmos_sensor(0x02);
		if(sensor_id == SP0A08_YUV_ID)
			break;
	}
	YUV_DBG("sp0a08 read yuv sensor_id=0x%x\n",sensor_id);
	return ((sensor_id == SP0A08_YUV_ID)?1:0);
}

void sp0a08_open(void)
{
	YUV_DBG("%s\n",__func__);
	SP0A08_Sensor_Init();
}

YUV_SENSOR_FUNC sp0a08_sensor_func = 
{
	sp0a08_open,
	SP0A08_MIPI_GetYUVSensorBV,
};

UINT32 SP0A08_YUV_SENSOR_INIT(PYUV_SENSOR_FUNC *pfFunc)
{
	if(NULL != pfFunc)
		*pfFunc = &sp0a08_sensor_func;
	YUV_DBG("%s\n",__func__);
	return sp0a08_read_id();
}
