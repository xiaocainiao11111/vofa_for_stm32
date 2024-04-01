#ifndef _USER_MAIN_H
#define _USER_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif
/*------------- C Scope -------------*/
#include "stdint.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "gpio.h"
#include "usart.h"
#include "vofa.h"


    void user_main(void);

#ifdef __cplusplus
}
/*-------------- C++ Scope --------*/
/*cpp用的和带<cstdint>的放下面*/
#include <cstdio>
#include "math_utils.h"
#include <cmath>
#include "encoder.h"
// #include "encoder.h"
// #include "driver.h"
// #include "pid.h"
// #include "low_side_current_sense.h"
// #include "math_utils.h"
#include "motor.h"

#endif
#endif
