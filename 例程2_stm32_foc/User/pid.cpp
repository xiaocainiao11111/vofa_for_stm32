#include "pid.h"
PidController pidCurrentQ = PidController{3, 300.0f, 0.0f, 0, 12.0f};
PidController pidCurrentD = PidController{3, 300.0f, 0.0f, 0, 12.0f};
PidController pidVelocity = PidController{0.5f, 10.0f, 0.0f, 1000.0f, 12.0f};
PidController pidAngle = PidController{20.0f, 0, 0, 0, 20.0f};

// 输入误差，pid计算的结果
float PidController::operator()(float error)
{
    auto time = micros();
    float dt = (float)(time - timeStamp) * 1e-6f; // 时间差
    // Quick fix for strange cases (micros overflow)
    if (dt <= 0 || dt > 0.5f)
        dt = 1e-3f; // 0.001

    float pTerm = p * error;
    float iTerm = integralLast + i * dt * 0.5f * (error + errorLast); // 误差累加
    iTerm = CONSTRAINT(iTerm, -limit, limit);
    float dTerm = d * (error - errorLast) / dt; // 瞬时误差的倍率

    float output = pTerm + iTerm + dTerm;
    output = CONSTRAINT(output, -limit, limit);

    // If output ramp defined
    if (outputRamp > 0)
    {
        // Limit the acceleration by ramping the output
        float outputRate = (output - outputLast) / dt;
        if (outputRate > outputRamp)
            output = outputLast + outputRamp * dt;
        else if (outputRate < -outputRamp)
            output = outputLast - outputRamp * dt;
    }

    integralLast = iTerm;
    outputLast = output;
    errorLast = error;
    timeStamp = time;

    return output;
}
