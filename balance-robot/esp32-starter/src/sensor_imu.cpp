#include "sensor_imu.h"
#include <Arduino.h>

void ImuSensor::begin() {
    if (!m_sensor.begin()) {
        Serial.println(F("MPU-6050 not detected"));
        while (true) { delay(10); }
    }

    Serial.println(F("MPU-6050 ready"));

    m_sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
    m_sensor.setGyroRange(MPU6050_RANGE_500_DEG);
    m_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void ImuSensor::fetch(ImuReading& out) {
    sensors_event_t a, g, t;
    m_sensor.getEvent(&a, &g, &t);

    out.temperature = t.temperature;
    out.accX = a.acceleration.x;
    out.accY = a.acceleration.y;
    out.accZ = a.acceleration.z;

    out.gyroX = g.gyro.x;
    out.gyroY = g.gyro.y;
    out.gyroZ = g.gyro.z;
}
