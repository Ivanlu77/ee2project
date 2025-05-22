#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <step.h>
#include <math.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

// Timing constants
const int PRINT_INTERVAL    = 100;    // Debug print interval (ms)
const int CONTROL_INTERVAL  = 5;      // Control loop interval (ms)
const int STEPPER_INTERVAL_US = 20;   // Stepper update interval (microseconds)

// PID constants - optimized for stable balancing
float KP = 6.0;    // Proportional gain
float KI = 0.05;   // Integral gain - extremely low to prevent oscillation
float KD = 0.5;    // Derivative gain

// Balance setpoint - target value for balance
const float BALANCE_SETPOINT = 0.0;  // 0 degrees is balanced

// Motor control parameters
const float MAX_SPEED = 25.0;         // Maximum motor speed (rad/s)
const float MOTOR_DEADBAND = 1.0;     // Minimum motor speed

// Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;                 // Default pins for I2C are SCL: IO22, SDA: IO21

// Stepper motor objects
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Sensor readings
float currentAngle = 0;
float motorSpeed = 0;

// Calibration values - these will be set during calibration
float accelXZero = 0;  // X acceleration when vertical
float accelYZero = 0;  // Y acceleration when vertical
float accelZZero = 0;  // Z acceleration when vertical

// The axis to use for tilt detection (for vertical mounting)
// For vertical mounting, Z may be more reliable for forward/backward tilt
#define TILT_AXIS_Z 1  // Use Z axis as primary tilt axis (change to 0 to use X)

// Angle calculation constants
const float ANGLE_SCALE = 15.0;  // Scaling factor for angle calculation

// Debugging values
float rawAccelX = 0;
float rawAccelY = 0;
float rawAccelZ = 0;

// Interrupt Service Routine for motor update
bool TimerHandler(void * timerNo) {
  static bool toggle = false;

  // Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  // Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN, toggle);  
  toggle = !toggle;
  return true;
}

// Read sensor data and compute current angle for vertically mounted sensor
void readSensor() {
  // Get new sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Store raw values for debugging
  rawAccelX = a.acceleration.x;
  rawAccelY = a.acceleration.y;
  rawAccelZ = a.acceleration.z;
  
  // 添加中值滤波 - 减少异常值影响
  static float xBuffer[5] = {0};
  static float zBuffer[5] = {0};
  static int bufferIndex = 0;
  
  // 更新缓冲区
  xBuffer[bufferIndex] = rawAccelX;
  zBuffer[bufferIndex] = rawAccelZ;
  bufferIndex = (bufferIndex + 1) % 5;
  
  // 冒泡排序找出中值
  float xCopy[5], zCopy[5];
  for(int i=0; i<5; i++) {
    xCopy[i] = xBuffer[i];
    zCopy[i] = zBuffer[i];
  }
  
  for(int i=0; i<4; i++) {
    for(int j=0; j<4-i; j++) {
      if(xCopy[j] > xCopy[j+1]) {
        float temp = xCopy[j];
        xCopy[j] = xCopy[j+1];
        xCopy[j+1] = temp;
      }
      if(zCopy[j] > zCopy[j+1]) {
        float temp = zCopy[j];
        zCopy[j] = zCopy[j+1];
        zCopy[j+1] = temp;
      }
    }
  }
  
  // 使用中值
  float medianX = xCopy[2];
  float medianZ = zCopy[2];
  
  // 添加低通滤波 - 平滑数据
  static float filteredX = 0;
  static float filteredZ = 0;
  const float FILTER_ALPHA = 0.2; // 滤波系数，越小滤波越强
  
  filteredX = filteredX * (1 - FILTER_ALPHA) + medianX * FILTER_ALPHA;
  filteredZ = filteredZ * (1 - FILTER_ALPHA) + medianZ * FILTER_ALPHA;
  
  // For vertical mounting, we need to use Z axis for forward/backward tilt
  float primaryAxis, zeroPoint;
  
#if TILT_AXIS_Z
  // Use Z axis for tilt (may be more appropriate for vertical mounting)
  primaryAxis = filteredZ; // 使用滤波后的值
  zeroPoint = accelZZero;
#else
  // Use X axis for tilt
  primaryAxis = filteredX; // 使用滤波后的值
  zeroPoint = accelXZero;
#endif
  
  // Calculate tilt (deviation from zero position)
  float delta = primaryAxis - zeroPoint;
  
  // Convert to angle (simplified)
  // For vertical mounting, positive delta should be forward tilt, negative is backward
  // We invert the sign to make forward tilt positive and backward negative
  float newAngle = -delta * ANGLE_SCALE;
  
  // 对角度应用平滑滤波
  static float smoothedAngle = 0;
  const float ANGLE_SMOOTH = 0.3; // 角度平滑系数
  smoothedAngle = smoothedAngle * (1 - ANGLE_SMOOTH) + newAngle * ANGLE_SMOOTH;
  
  currentAngle = smoothedAngle;
}

// PID calculation function
float calculatePID() {
  // Calculate error
  error = BALANCE_SETPOINT - currentAngle;
  
  // 忽略极小误差以防止微小抖动
  if (abs(error) < 0.5) {
    error = 0;
    return 0; // 直接返回零输出
  }
  
  // Calculate integral term with anti-windup
  integral += error * CONTROL_INTERVAL / 1000.0;
  
  // Reset integral when crossing balance point to prevent overshoot
  if (error * lastError < 0) {
    integral = 0;
  }
  
  // 改进的积分限幅 - 动态值与KI匹配
  const float MAX_INTEGRAL = MAX_SPEED / (2.0 * KI);
  if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
  if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
  
  // Calculate derivative (change in error)
  derivative = (error - lastError) / (CONTROL_INTERVAL / 1000.0);
  lastError = error;
  
  // Apply simple low-pass filter to derivative to reduce noise
  static float filteredDerivative = 0;
  filteredDerivative = 0.8 * filteredDerivative + 0.2 * derivative;
  
  // Calculate PID output
  output = KP * error + KI * integral + KD * filteredDerivative;
  
  // 平滑输出变化
  static float lastOutput = 0;
  const float OUTPUT_SMOOTH = 0.3; // 平滑系数
  output = output * OUTPUT_SMOOTH + lastOutput * (1 - OUTPUT_SMOOTH);
  lastOutput = output;
  
  // 限制输出不超过最大速度
  if (output > MAX_SPEED) output = MAX_SPEED;
  if (output < -MAX_SPEED) output = -MAX_SPEED;
  
  // 定期打印调试信息
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) {
    lastDebugTime = millis();
    Serial.print("角度: ");
    Serial.print(error, 1);
    Serial.print("  PID输出: ");
    Serial.println(output, 1);
  }
  
  // Apply deadband - if output is very small, set to zero
  if (abs(output) < MOTOR_DEADBAND) output = 0;
  
  return output;
}

// Function to calibrate the accelerometer
void calibrateAccelerometer() {
  Serial.println("Calibrating accelerometer - keep robot upright and still");
  
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  const int samples = 100;
  
  // Collect multiple samples
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    
    delay(10);
  }
  
  // Calculate average values when upright
  accelXZero = sumX / samples;
  accelYZero = sumY / samples;
  accelZZero = sumZ / samples;
  
  Serial.print("Calibration complete - X zero: ");
  Serial.print(accelXZero);
  Serial.print(", Y zero: ");
  Serial.print(accelYZero);
  Serial.print(", Z zero: ");
  Serial.println(accelZZero);
}

// Test the direction response by tilting the robot
void testDirectionResponse() {
  Serial.println("Direction test - please tilt forward and backward");
  Serial.println("Forward tilt should give positive angles, backward tilt negative angles");
  
  // Read and display values for a few seconds
  unsigned long endTime = millis() + 10000;  // 10 seconds of testing
  while (millis() < endTime) {
    // Read sensors and calculate angle
    readSensor();
    
    // Display values
    Serial.print("Raw X: ");
    Serial.print(rawAccelX);
    Serial.print("\tRaw Z: ");
    Serial.print(rawAccelZ);
    
    // Show which axis we're using
#if TILT_AXIS_Z
    Serial.print("\tUsing Z-axis: ");
    Serial.print(rawAccelZ - accelZZero);
#else
    Serial.print("\tUsing X-axis: ");
    Serial.print(rawAccelX - accelXZero);
#endif
    
    Serial.print("\tAngle: ");
    Serial.println(currentAngle);
    
    delay(100);
  }
  
  Serial.println("Direction test complete");
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("Balance Robot - Vertical Mount PID Controller");
  
  // Configure the toggle pin for timing diagnostics
  pinMode(TOGGLE_PIN, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Initialize stepper motor parameters
  step1.setAccelerationRad(100.0);
  step2.setAccelerationRad(100.0);
  
  // Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);  // false = enabled for most drivers

  // Attach motor update ISR to timer
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }
  Serial.println("Stepper interrupt initialized");
  
  // Perform calibration
  calibrateAccelerometer();
  
  // Run the direction test to verify tilt detection
  testDirectionResponse();
  
  // Wait a moment before starting
  delay(500);
  
  Serial.println("Starting balance control");
}

void loop() {
  static uint32_t printTimer = 0;    // Next print time
  static uint32_t controlTimer = 0;  // Next control update time
  uint32_t now = millis();
  
  // Run the control loop at high frequency - 修正溢出问题
  if ((uint32_t)(now - controlTimer) >= CONTROL_INTERVAL) {
    controlTimer = now;
    
    // Read sensor data
    readSensor();
    
    // Calculate PID output
    motorSpeed = calculatePID();
    
    // Update motor speeds - one motor runs in reverse
    step1.setTargetSpeedRad(motorSpeed);
    step2.setTargetSpeedRad(-motorSpeed);
  }
  
  // Print debug information periodically - 修正溢出问题
  if ((uint32_t)(now - printTimer) >= PRINT_INTERVAL) {
    printTimer = now;
    
#if TILT_AXIS_Z
    Serial.print("Z-axis: ");
    Serial.print(rawAccelZ, 2);
#else
    Serial.print("X-axis: ");
    Serial.print(rawAccelX, 2);
#endif
    
    Serial.print("\tAngle: ");
    Serial.print(currentAngle, 1);
    Serial.print("\tError: ");
    Serial.print(error, 1);
    Serial.print("\tI: ");
    Serial.print(integral, 2);
    Serial.print("\tMotor: ");
    Serial.println(motorSpeed, 1);
  }
} 