#include <Arduino.h>

//..........................................................................................
// PWM输出引脚
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
float voltage_power_supply = 12.6;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;
float Ts = 0;

float Uq = voltage_power_supply / 2;
float rad = 3.14;

void setup()
{
    Serial.begin(115200);
    // PWM设置
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);
    ledcAttachPin(pwmA, 0);
    ledcAttachPin(pwmB, 1);
    ledcAttachPin(pwmC, 2);
    ledcSetup(0, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(1, 30000, 8);
    ledcSetup(2, 30000, 8);
    Serial.println("完成PWM初始化设置");
    delay(3000);
}

// 电角度求解，机械角度*极对数
float _electricalAngle(float shaft_angle, int pole_pairs)
{
    return (shaft_angle * pole_pairs);
}

// 归一角度到 [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc)
{
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    // 写入PWM到PWM 0 1 2 通道
    ledcWrite(0, dc_a * 255);
    ledcWrite(1, dc_b * 255);
    ledcWrite(2, dc_c * 255);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    // 帕克逆变换
    Ualpha = -Uq * sin(angle_el);
    Ubeta = Uq * cos(angle_el);

    // 克拉克逆变换
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPwm(Ua, Ub, Uc);
}

// 开环速度函数
float velocityOpenloop(float target_velocity)
{
    unsigned long now_us = micros(); // 获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值

    Ts = (now_us - open_loop_timestamp) * 1e-6f;

    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 1));

    open_loop_timestamp = now_us; // 用于计算下一个时间间隔

    return Uq;
}

// 参数打印
void _print()
{
    // Serial.println(
    //     " rad/s:" + String(rad) +
    //     " open_loop_timestamp:" + String(open_loop_timestamp) +
    //     " time:" + String(Ts * 1000) +
    //     " shaft_angle:" + String(shaft_angle) +
    //     " Uq:" + String(Uq) +
    //     " Ualpha:" + String(Ualpha) +
    //     " Ubeta:" + String(Ubeta) +
    //     " Ua:" + String(Ua) +
    //     " Ub:" + String(Ub) +
    //     " Uc:" + String(Uc)

    //     );
    // Serial.print("rad/s:");
    // Serial.println(rad, 2);
    // Serial.print("open_loop_timestamp:");
    // Serial.println(open_loop_timestamp, 4);
    // Serial.print("time:");
    // Serial.println(Ts, 4);
    // Serial.print("shaft_angle:");
    // Serial.println(shaft_angle, 4);
    // Serial.print("Uq:");
    // Serial.println(Uq, 4);
    // Serial.print("Ualpha:");
    // Serial.println(Ualpha, 4);
    // Serial.print("Ua:");
    // Serial.println(Ua, 4);
    // Serial.print("Ub:");
    Serial.print(rad, 4);
    Serial.print(",");
    Serial.print(open_loop_timestamp, 4);
    Serial.print(",");
    Serial.print(Ts * 100, 4);
    Serial.print(",");
    Serial.print(shaft_angle, 4);
    Serial.print(",");
    Serial.print(Uq, 4);
    Serial.print(",");
    Serial.print(Ualpha, 4);
    Serial.print(",");
    Serial.print(Ubeta, 4);
    Serial.print(",");
    Serial.print(Ua, 4);
    Serial.print(",");
    Serial.print(Ub, 4);
    Serial.print(",");
    Serial.println(Uc, 4);
}

void loop()
{
    // 输入弧度每秒
    velocityOpenloop(3.14);
    _print();
    // delay(1);
}
