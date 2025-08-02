#include "openloop.h"

float velocityOpenloop(float target_velocity);
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
void Set_Volt(float Ua, float Ub, float Uc)
{

    // 计算占空比
    // 限制占空比从0到1
    float dc_a, dc_b, dc_c;

    dc_a = Clamp(Ua / POWER_VOLTAGE, 0.0f, 1.0f);
    dc_b = Clamp(Ub / POWER_VOLTAGE, 0.0f, 1.0f);
    dc_c = Clamp(Uc / POWER_VOLTAGE, 0.0f, 1.0f);

    set_dtc_a((uint16_t)(dc_a * PWM_ARR)); // 设置通道1的占空比
    set_dtc_b((uint16_t)(dc_b * PWM_ARR)); // 设置通道2的占空比
    set_dtc_c((uint16_t)(dc_c * PWM_ARR)); // 设置通道3的占空比
}
// 电角度求解
float _electricalAngle(float shaft_angle)
{
    return (shaft_angle * POLE_PAIRS);
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2 * PI); // 取余运算可以用于归一化，列出特殊值例子算便知
    return a >= 0 ? a : (a + 2 * PI);
    // fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
    // 例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

void OpenLoopSetPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    // 帕克逆变换
    Ualpha = -Uq * sin(angle_el);
    Ubeta  = Uq * cos(angle_el);

    // 克拉克逆变换
    Ua = Ualpha + POWER_VOLTAGE / 2;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + POWER_VOLTAGE / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + POWER_VOLTAGE / 2;
    Set_Volt(Ua, Ub, Uc);
}
// 开环速度函数
float velocityOpenloop(float target_velocity)
{
    static float Ts = 0.001;
    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    // 以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    // 如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的POWER_VOLTAGE的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = POWER_VOLTAGE/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = POWER_VOLTAGE / 3;

    OpenLoopSetPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle));

    return Uq;
}
