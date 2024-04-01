#include "motor.h"
#include "encoder.h"
#include "lowpass_filter.h"
#include "pid.h"

uint8_t cmd = 0;
bool enabled = false;
float phaseResistance = -12345.0;
int polePairs = 7; // 极对数
float voltageA, voltageB, voltageC;
float estimateAngle;
float electricalAngle;
float estimateVelocity;
float setPointCurrent;
float setPointVelocity;
float setPointAngle;
uint64_t openLoopTimestamp = 0;

float deltaT;

float _angle = 0;

float t0, t1, t2;
float dutyA = 0;
float dutyB = 0;
float dutyC = 0;

float voltagePowerSupply = 12;

uint16_t PERIOD_COUNT = 4199;

uint64_t t;

float target = 0;

State_t state{};
ControlMode_t controlMode = ANGLE;

// 限幅，百分比
float Constraint(float _val)
{
    if (_val > 1)
        _val = 1;
    else if (_val < 0)
        _val = 0;

    return _val;
}

// 输入三个百分比，输出pwm
void SetPwmDutyByRegister(float _dutyA, float _dutyB, float _dutyC)
{
    _dutyA = Constraint(_dutyA);
    _dutyB = Constraint(_dutyB);
    _dutyC = Constraint(_dutyC);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(_dutyA * PERIOD_COUNT));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(_dutyB * PERIOD_COUNT));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(_dutyC * PERIOD_COUNT));
}

// 输入三相电压，转化为百分比
void SetVoltage(float _voltageA, float _voltageB, float _voltageC)
{
    _voltageA = CONSTRAINT(_voltageA, 0, voltagePowerSupply);
    _voltageB = CONSTRAINT(_voltageB, 0, voltagePowerSupply);
    _voltageC = CONSTRAINT(_voltageC, 0, voltagePowerSupply);

    dutyA = _voltageA / voltagePowerSupply;
    dutyB = _voltageB / voltagePowerSupply;
    dutyC = _voltageC / voltagePowerSupply;

    SetPwmDutyByRegister(dutyA, dutyB, dutyC);
}

// 设置相电压，两相电压加电角度
void SetPhaseVoltage(float _voltageQ, float _voltageD, float _angleElectrical)
{
    float uOut;

    uOut = _voltageQ / 12;
    _angleElectrical = Normalize(_angleElectrical + _PI_2);

    uint8_t sec = (int)(std::floor(_angleElectrical / _PI_3)) + 1;
    t1 = _SQRT3 * SinApprox((float)(sec)*_PI_3 - _angleElectrical) * uOut;
    t2 = _SQRT3 * SinApprox(_angleElectrical - ((float)(sec)-1.0f) * _PI_3) * uOut;
    t0 = 1 - t1 - t2;

    float tA, tB, tC;
    switch (sec)
    {
    case 1:
        tA = t1 + t2 + t0 / 2;
        tB = t2 + t0 / 2;
        tC = t0 / 2;
        break;
    case 2:
        tA = t1 + t0 / 2;
        tB = t1 + t2 + t0 / 2;
        tC = t0 / 2;
        break;
    case 3:
        tA = t0 / 2;
        tB = t1 + t2 + t0 / 2;
        tC = t2 + t0 / 2;
        break;
    case 4:
        tA = t0 / 2;
        tB = t1 + t0 / 2;
        tC = t1 + t2 + t0 / 2;
        break;
    case 5:
        tA = t2 + t0 / 2;
        tB = t0 / 2;
        tC = t1 + t2 + t0 / 2;
        break;
    case 6:
        tA = t1 + t2 + t0 / 2;
        tB = t0 / 2;
        tC = t1 + t0 / 2;
        break;
    default:
        tA = 0;
        tB = 0;
        tC = 0;
    }

    // calculate the phase voltages and center
    voltageA = tA * 12;
    voltageB = tB * 12;
    voltageC = tC * 12;

    // currentSense->pwmDutyA = tA;
    // currentSense->pwmDutyB = tB;
    // currentSense->pwmDutyC = tC;

    SetVoltage(voltageA, voltageB, voltageC);
}

// 开环速度
float VelocityOpenLoopTick(float _target)
{
    t = micros();
    deltaT = (float)(t - openLoopTimestamp) * 1e-6f;
    // Quick fix for strange cases (micros overflow or timestamp not defined)
    if (deltaT <= 0 || deltaT > 0.5f)
        deltaT = 1e-3f; // 限幅

    estimateAngle = Normalize(estimateAngle + _target * deltaT); // 限幅0~2PI
    estimateVelocity = _target;

    float voltageQ = 1.5;
    SetPhaseVoltage(voltageQ, 0, Normalize(estimateAngle) * (float)polePairs);

    openLoopTimestamp = t;

    return voltageQ;
}

//..........闭环...............
float GetEstimateAngle()
{

    state.rawAngle = (float)countDirection * GetFullAngle();
    state.estAngle = lpfAngle(state.rawAngle);

    return state.estAngle;
}

float GetEstimateVelocity()
{
    state.rawVelocity = (float)countDirection * GetVelocity();
    state.estVelocity = lpfVelocity(state.rawVelocity);

    return state.estVelocity;
}

void CloseLoopControlTick()
{

    estimateAngle = GetEstimateAngle();

    estimateVelocity = GetEstimateVelocity();

    controlMode = VELOCITY;

    switch (controlMode)
    {
    // case ControlMode_t::TORQUE:
    //     voltage.q = ASSERT(phaseResistance) ? target * phaseResistance : target;
    //     voltage.d = 0;
    //     setPointCurrent = voltage.q;
    //     break;
    case ANGLE:
        setPointAngle = target;
        setPointVelocity = pidAngle(setPointAngle - estimateAngle);
        setPointCurrent = pidVelocity(setPointVelocity - estimateVelocity); // 只赋值p
        break;
    case VELOCITY:
        setPointVelocity = target;
        setPointCurrent = pidVelocity(setPointVelocity - estimateVelocity);
        break;
    // case ControlMode_t::VELOCITY_OPEN_LOOP:
    //     setPointVelocity = target;
    //     voltage.q = VelocityOpenLoopTick(setPointVelocity);
    //     voltage.d = 0;
    //     break;
    // case ControlMode_t::ANGLE_OPEN_LOOP:
    //     setPointAngle = target;
    //     voltage.q = AngleOpenLoopTick(setPointAngle);
    //     voltage.d = 0;
    //     break;
    default:
        break;
    }
}

float GetElectricalAngle()
{
    // If no sensor linked return previous value (for open-loop case)
    return Normalize((float)(polePairs)*angleLast);
}

void FocOutputTick()
{
    Update();

    electricalAngle = GetElectricalAngle();

    voltage.q = setPointCurrent;
    voltage.d = 0;

    SetPhaseVoltage(voltage.q, voltage.d, electricalAngle);
}

void Motor_Tick()
{
    CloseLoopControlTick();
    FocOutputTick();
}