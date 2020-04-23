/*
 * recorder.h
 *
 *  Created on: Apr 23, 2020
 *      Author: stach
 */

#ifndef INC_RECORDER_H_
#define INC_RECORDER_H_


#include <stdio.h>
#include "stm32f4xx.h"
#include "main.h"
#include "ff.h"

int AddWaveHeader(char* file_path);
int SaveChunk(char* file_path,int16_t* data[]);

#endif /* INC_RECORDER_H_ */
