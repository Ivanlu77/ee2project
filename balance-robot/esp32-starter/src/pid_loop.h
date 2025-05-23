#ifndef PID_LOOP_H
#define PID_LOOP_H

/**
 * PID控制器类 - 用于实现比例-积分-微分控制
 */
class PIDLoop {
public:
    /**
     * 构造函数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param targetValue 目标值
     */
    PIDLoop(double kp, double ki, double kd, double targetValue);
    
    /**
     * 设置PID参数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     */
    void tunings(double kp, double ki, double kd);
    
    /**
     * 设置目标值
     * @param targetValue 新的目标值
     */
    void target(double targetValue);
    
    /**
     * 设置采样时间
     * @param sampleTime 采样时间(秒)
     */
    void sampleTime(double sampleTime);
    
    /**
     * 设置输出限制
     * @param min 最小输出值
     * @param max 最大输出值
     */
    void limits(double min, double max);
    
    /**
     * 设置是否为偏航角控制
     */
    void treatAsYaw(bool isYaw);
    
    /**
     * 计算输出值
     * @param current 当前传感器值
     * @return 计算的输出值
     */
    double run(double current);

private:
    // PID参数
    double m_kp;          // 比例增益
    double m_ki;          // 积分增益
    double m_kd;          // 微分增益
    double m_target;      // 目标值
    
    // 输出限制
    double m_minOut;      // 最小输出限制
    double m_maxOut;      // 最大输出限制
    
    // 状态变量
    double m_lastError;   // 上一次误差
    double m_integral;    // 积分项
    double m_lastTime;    // 上次计算时间
    double m_lastOutput;  // 上次输出值
    
    // 配置
    double m_sampleTime;  // 采样时间(秒)
    bool m_isYaw;         // 是否为偏航角控制
};

#endif // PID_LOOP_H
