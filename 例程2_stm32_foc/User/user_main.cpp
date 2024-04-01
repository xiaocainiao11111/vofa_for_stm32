#include "user_main.h"

#include <stdio.h>

extern uint8_t cmd;
extern bool enabled;
extern float phaseResistance;
extern int polePairs; // 极对数
extern float voltageA, voltageB, voltageC;
extern float estimateAngle;
extern float electricalAngle;
extern float estimateVelocity;
extern float setPointCurrent;
extern float setPointVelocity;
extern float setPointAngle;
extern uint64_t openLoopTimestamp;

extern float deltaT;

extern float _angle;
extern float angleLast;

extern float t0, t1, t2;
extern float dutyA;
extern float dutyB;
extern float dutyC;

extern float voltagePowerSupply;

extern uint16_t PERIOD_COUNT;

extern uint64_t t;

void user_main(void)
{
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  Encoder_Init();
  HAL_Delay(5);

  while (1)
  {
    // VelocityOpenLoopTick(6);
    VelocityOpenLoopTick(3.14);

    HAL_Delay(1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    // _angle = GetRawAngle();
    // Update();

    // Motor_Tick();

    float ff[10] = {voltageA, voltageB, voltageC, estimateAngle, t0, t1, t2, _angle, electricalAngle, voltage.q};
    Vofa_JustFloat(ff, 4);

    // Vofa_FireWater("%f,%f,%f,%f,%f,%f,%f\r\n",voltageA,voltageB, voltageC, estimateAngle, t0, t1, t2);
  }
}
