#include "lowpass_filter.h"

LowPassFilter lpfAngle = LowPassFilter{0.03f};
LowPassFilter lpfVelocity = LowPassFilter{0.1f};
// 对输入数进行滤波
float LowPassFilter::operator()(float _input)
{
    unsigned long time = micros();
    float dt = ((float)time - (float)timeStamp) * 1e-6f; // 时间差

    if (dt < 0.0f)
        dt = 1e-3f;
    else if (dt > 0.3f)
    {
        outputLast = _input;
        timeStamp = time;
        return _input;
    }

    float alpha = timeConstant / (timeConstant + dt);
    float output = alpha * outputLast + (1.0f - alpha) * _input; // 滤波
    outputLast = output;
    timeStamp = time;

    return output;
}
