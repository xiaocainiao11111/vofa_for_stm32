#ifndef _VOFA_H
#define _VOFA_H

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>

void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);

#endif
