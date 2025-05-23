#include "pid_loop.h"
#include <Arduino.h>
#include <cmath>

// ��ȡ��ǰʱ�䣨�룩
static double nowSeconds() {
    return millis() / 1000.0;
}

// ���캯��
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

// ����PID����
void PIDLoop::tunings(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

// ����Ŀ��ֵ
void PIDLoop::target(double targetValue) {
    m_target = targetValue;
}

// ���ò���ʱ��
void PIDLoop::sampleTime(double sampleTime) {
    if (sampleTime > 0) {
        m_sampleTime = sampleTime;
    }
}

// �����������
void PIDLoop::limits(double min, double max) {
    if (min >= max) return;
    m_minOut = min;
    m_maxOut = max;
}

// �����Ƿ�Ϊƫ���ǿ���
void PIDLoop::treatAsYaw(bool isYaw) {
    m_isYaw = isYaw;
}

// ����ֵ��ָ����Χ��
double constrainValue(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ����PID���
double PIDLoop::run(double current) {
    double currentTime = nowSeconds();
    double timeChange = currentTime - m_lastTime;

    // ����Ƿ��Ѿ������㹻��ʱ��
    if (timeChange >= m_sampleTime) {
        // �������
        double error = m_target - current;

        // ���⴦��ƫ���ǣ�ѭ�������ԣ�
        if (m_isYaw) {
            if (error > M_PI) {
                error -= 2 * M_PI;
            } else if (error < -M_PI) {
                error += 2 * M_PI;
            }
        }
        
        // ���������
        m_integral += error * timeChange;
        
        // ���ƻ������ֹ���ֱ���
        m_integral = constrainValue(m_integral, m_minOut, m_maxOut);

        // ����΢����
        double derivative = (error - m_lastError) / timeChange;

        // ���������
        double out = (m_kp * error) + (m_ki * m_integral) + (m_kd * derivative);
        
        // ����������趨��Χ��
        out = constrainValue(out, m_minOut, m_maxOut);

        // �洢״̬�����´μ���
        m_lastError = error;
        m_lastTime = currentTime;
        m_lastOutput = out;

        return out;
    }

    // ���ʱ�䲻�㣬�����ϴ����ֵ
    return (m_lastOutput == 0.0) ? 0.0 : m_lastOutput;
} 