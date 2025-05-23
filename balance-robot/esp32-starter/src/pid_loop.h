#ifndef PID_LOOP_H
#define PID_LOOP_H

/**
 * PID�������� - ����ʵ�ֱ���-����-΢�ֿ���
 */
class PIDLoop {
public:
    /**
     * ���캯��
     * @param kp ��������
     * @param ki ��������
     * @param kd ΢������
     * @param targetValue Ŀ��ֵ
     */
    PIDLoop(double kp, double ki, double kd, double targetValue);
    
    /**
     * ����PID����
     * @param kp ��������
     * @param ki ��������
     * @param kd ΢������
     */
    void tunings(double kp, double ki, double kd);
    
    /**
     * ����Ŀ��ֵ
     * @param targetValue �µ�Ŀ��ֵ
     */
    void target(double targetValue);
    
    /**
     * ���ò���ʱ��
     * @param sampleTime ����ʱ��(��)
     */
    void sampleTime(double sampleTime);
    
    /**
     * �����������
     * @param min ��С���ֵ
     * @param max ������ֵ
     */
    void limits(double min, double max);
    
    /**
     * �����Ƿ�Ϊƫ���ǿ���
     */
    void treatAsYaw(bool isYaw);
    
    /**
     * �������ֵ
     * @param current ��ǰ������ֵ
     * @return ��������ֵ
     */
    double run(double current);

private:
    // PID����
    double m_kp;          // ��������
    double m_ki;          // ��������
    double m_kd;          // ΢������
    double m_target;      // Ŀ��ֵ
    
    // �������
    double m_minOut;      // ��С�������
    double m_maxOut;      // ����������
    
    // ״̬����
    double m_lastError;   // ��һ�����
    double m_integral;    // ������
    double m_lastTime;    // �ϴμ���ʱ��
    double m_lastOutput;  // �ϴ����ֵ
    
    // ����
    double m_sampleTime;  // ����ʱ��(��)
    bool m_isYaw;         // �Ƿ�Ϊƫ���ǿ���
};

#endif // PID_LOOP_H
