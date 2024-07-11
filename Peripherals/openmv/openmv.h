/*
 * openmv.h
 *
 *  Created on: Mar 12, 2024
 *      Author: zeng
 */

#ifndef INC_OPENMV_H_
#define INC_OPENMV_H_

#include "stm32f4xx_hal.h"



void  Openmv_Receive_Data(int16_t data);

extern signed int Bias;
extern signed int Bias_Sign;
extern signed int label_value;
extern signed int Ch;

#endif /* INC_OPENMV_H_ */
