/*
 * ibus.c
 *
 *  Created on: Sep 8, 2023
 *      Author: oguzk
 */

#include "ibus.h"



void iBus_Parsing(unsigned char *data ,FSiA6B_iBus *iBus)
{
		iBus->RH = (data[2] | data[3]<<8) & 0x0fff;
		iBus->RV = (data[4] | data[5]<<8) & 0x0fff;
		iBus->LV = (data[6] | data[7]<<8) & 0x0fff;
		iBus->LH = (data[8] | data[9]<<8) & 0x0fff;
		iBus->SwA = (data[10] | data[11]<<8) & 0x0fff;
		iBus->SwC = (data[12] | data[13]<<8) & 0x0fff;

		iBus->SwD = (data[14] | data[15]<<8) & 0x0fff;

		iBus->FailSafe = iBus->SwD == 1500;

	//	iBus->FailSafe = (data[13] >> 4);
}






unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len)
{
	unsigned short chksum = 0xffff;

	for(int i=0;i<len-2;i++)
	{
		chksum = chksum - data[i];
	}

	return ((chksum&0x00ff)==data[30]) && ((chksum>>8)==data[31]);
}
