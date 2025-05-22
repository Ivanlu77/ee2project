#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create MPU6050 object
Adafruit_MPU6050 mpu;

// Print interval (milliseconds)
const int PRINT_INTERVAL = 200;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  
  Serial.println("MPU6050 Test Program");

  // Try to initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip, check your connections!");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 initialization successful!");

  // Configure sensor settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G: Serial.println("°¿2G"); break;
    case MPU6050_RANGE_4_G: Serial.println("°¿4G"); break;
    case MPU6050_RANGE_8_G: Serial.println("°¿8G"); break;
    case MPU6050_RANGE_16_G: Serial.println("°¿16G"); break;
  }
  
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyroscope range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG: Serial.println("°¿250 deg/s"); break;
    case MPU6050_RANGE_500_DEG: Serial.println("°¿500 deg/s"); break;
    case MPU6050_RANGE_1000_DEG: Serial.println("°¿1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("°¿2000 deg/s"); break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ: Serial.println("94 Hz"); break;
    case MPU6050_BAND_44_HZ: Serial.println("44 Hz"); break;
    case MPU6050_BAND_21_HZ: Serial.println("21 Hz"); break;
    case MPU6050_BAND_10_HZ: Serial.println("10 Hz"); break;
    case MPU6050_BAND_5_HZ: Serial.println("5 Hz"); break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  // Get new sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print acceleration data (m/s^2)
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // Print gyroscope data (rad/s)
  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  // Print temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °„C");

  Serial.println("");
  
  // Wait for a while before reading the next set of data
  delay(PRINT_INTERVAL);
} 