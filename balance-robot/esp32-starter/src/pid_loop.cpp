#include "pid_loop.h"
#include <Arduino.h>
#include <cmath>

// 获取当前时间（秒）
static double nowSeconds() {
    return millis() / 1000.0;
}

// 构造函数
PIDLoop::PIDLoop(double kp, double ki, double kd, double targetValue) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_target = targetValue;
    m_minOut = -80.0;
    m_maxOut = 80.0;
    m_lastError = 0.0;
    m_integral = 0.0;
    m_sampleTime = 0.01;
    m_lastTime = nowSeconds();
    m_lastOutput = 0.0;
    m_isYaw = false;
}

// 设置PID参数
void PIDLoop::tunings(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

// 设置目标值
void PIDLoop::target(double targetValue) {
    m_target = targetValue;
}

// 设置采样时间
void PIDLoop::sampleTime(double sampleTime) {
    if (sampleTime > 0) {
        m_sampleTime = sampleTime;
    }
}

// 设置输出限制
void PIDLoop::limits(double min, double max) {
    if (min >= max) return;
    m_minOut = min;
    m_maxOut = max;
}

// 设置是否为偏航角控制
void PIDLoop::treatAsYaw(bool isYaw) {
    m_isYaw = isYaw;
}

// 限制值在指定范围内
double constrainValue(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 计算PID输出
double PIDLoop::run(double current) {
    double currentTime = nowSeconds();
    double timeChange = currentTime - m_lastTime;

    // 检查是否已经过了足够的时间
    if (timeChange >= m_sampleTime) {
        // 计算误差
        double error = m_target - current;

        // 特殊处理偏航角（循环连续性）
        if (m_isYaw) {
            if (error > M_PI) {
                error -= 2 * M_PI;
            } else if (error < -M_PI) {
                error += 2 * M_PI;
            }
        }
        
        // 计算积分项
        m_integral += error * timeChange;
        
        // 限制积分项，防止积分饱和
        m_integral = constrainValue(m_integral, m_minOut, m_maxOut);

        // 计算微分项
        double derivative = (error - m_lastError) / timeChange;

        // 计算总输出
        double out = (m_kp * error) + (m_ki * m_integral) + (m_kd * derivative);
        
        // 限制输出在设定范围内
        out = constrainValue(out, m_minOut, m_maxOut);

        // 存储状态用于下次计算
        m_lastError = error;
        m_lastTime = currentTime;
        m_lastOutput = out;

        return out;
    }

    // 如果时间不足，返回上次输出值
    return (m_lastOutput == 0.0) ? 0.0 : m_lastOutput;
} 