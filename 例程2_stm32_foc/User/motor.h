#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "user_main.h"

float Constraint(float _val);

float VelocityOpenLoopTick(float _target);
float GetEstimateVelocity();
void FocOutputTick();
void Motor_Tick();
struct State_t
{
    float rawAngle{};
    float estAngle{};
    float rawVelocity{};
    float estVelocity{};
};
extern State_t state;

enum ControlMode_t
{
    TORQUE, // 力矩
    VELOCITY,
    ANGLE,
    VELOCITY_OPEN_LOOP,
    ANGLE_OPEN_LOOP
};
extern ControlMode_t controlMode;

#endif