#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct ImuReading {
    double temperature{0.0};

    double accX{0.0};
    double accY{0.0};
    double accZ{0.0};

    double gyroX{0.0};
    double gyroY{0.0};
    double gyroZ{0.0};
};

class ImuSensor {
public:
    ImuSensor() = default;
    void begin();
    void fetch(ImuReading& out);

private:
    Adafruit_MPU6050 m_sensor;
};

#endif // SENSOR_IMU_H
