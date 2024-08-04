/*
 * ibus.h
 *
 *  Created on: Sep 8, 2023
 *      Author: oguzk
 */

#ifndef INC_IBUS_H_
#define INC_IBUS_H_

#include "stm32f4xx.h"

extern UART_HandleTypeDef huart1;

struct FSiA6B_iBus_Msg
{
	unsigned short Roll;
	unsigned short Pitch;
	unsigned short Yaw;
	unsigned short Throttle;
	unsigned short SwA;
	unsigned short SwC;

	unsigned char FailSafe;

};

typedef struct FSiA6B_iBus
{
	unsigned short RH;
	unsigned short RV;
	unsigned short LV;
	unsigned short LH;
	unsigned short SwA;
	unsigned short SwB;
	unsigned short SwC;
	unsigned short SwD;
	unsigned short VrA;
	unsigned short VrB;

	unsigned char FailSafe;
}FSiA6B_iBus;

unsigned char iBus_Check_CHKSUM(unsigned char* data, unsigned char len);
void iBus_Parsing(unsigned char *data ,FSiA6B_iBus *iBus);






#endif /* INC_IBUS_H_ */
