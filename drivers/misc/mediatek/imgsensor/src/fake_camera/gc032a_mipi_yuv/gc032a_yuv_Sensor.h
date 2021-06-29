/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc032amipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/10/25 Firsty Released By Mormo;
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/

 
#ifndef __GC032A_SENSOR_H
#define __GC032A_SENSOR_H

#include "kd_camera_typedef.h"

#define GC032A_WRITE_ID							    0x42
#define GC032A_READ_ID								0x43

extern void kdSetI2CSpeed(u16 i2cSpeed);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int iReadRegI2C2(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C2(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
UINT32 gc032a_read_id(void);
void gc032a_open(void);
UINT16 GC032a_MIPI_GetYUVSensorBV(void);

#endif
