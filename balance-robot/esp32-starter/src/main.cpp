#include <Arduino.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

#include "sensor_imu.h"
#include "pid_loop.h"


#define M1_DIR 16
#define M1_STP 17
#define M2_DIR 4
#define M2_STP 14
#define MOT_EN 15
#define SCOPE_PIN 32


TaskHandle_t stabilityHandle;
TaskHandle_t serialHandle;     


static constexpr int PRINT_MS          = 500;
static constexpr int LOOP_MS           = 10;
static constexpr int STEP_ISR_US       = 20;
static constexpr double ALPHA_COMP     = 0.98;//互补滤波系数
static constexpr double DEAD_BAND      = 7.0;
static constexpr float  WHEEL_DIA_CM   = 6.6;
static constexpr int    STEPS_REV      = 200*16;
static constexpr float  TRACK_CM       = 11.9;
static constexpr float  EMA_ALPHA      = 0.1;
static constexpr float TILT_OFFSET = 0.2;  // 开始设为0，然后根据需要调整(-0.043)


ESP32Timer hwTimer(3);
step motorL(STEP_ISR_US, M1_STP, M1_DIR);
step motorR(STEP_ISR_US, M2_STP, M2_DIR);
ImuSensor imu;

PIDLoop pidBal (800, 18, 85, 0); //800, 18, 78
PIDLoop pidSpd (1,   0.38, 0.23, 0);
PIDLoop pidYaw (3,     0,     0, 0);

bool  motorsActive = true;
bool  turning      = false;
double turnCmd     = 0;
double yawCorr     = 0;

double filtAngle = 0, prevAngle = 0;
float  emaSpdL   = 0, emaSpdR   = 0;


bool stepperIsr(void*) {
    static bool tgl = false;
    motorL.runStepper();
    motorR.runStepper();
    digitalWrite(SCOPE_PIN, tgl);
    tgl = !tgl;
    return true;
}

void enableMotors(bool on) {
    digitalWrite(MOT_EN, !on);           
    motorsActive = on;
    if (!on) {
        motorL.setTargetSpeedRad(0);
        motorR.setTargetSpeedRad(0);
        motorL.setAccelerationRad(0);
        motorR.setAccelerationRad(0);
    }
}

//调试命令
void serialCommandTask(void*) {
    while (true) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd == "forward")  { pidSpd.target(-20); turning=false; turnCmd=0; }
            else if (cmd == "back") { pidSpd.target( 20); turning=false; turnCmd=0; }
            else if (cmd == "left") { turnCmd= 0.4; turning=true;  }
            else if (cmd == "right"){ turnCmd=-0.4; turning=true;  }
            else if (cmd == "idle") { pidSpd.target(0); turning=false; turnCmd=0; }
        }
        delay(20);
    }
}

//主平衡任务
void stabilityTask(void*) {
    constexpr float distPerStep = (PI * WHEEL_DIA_CM) / STEPS_REV;
    static unsigned long nextLoop = 0, nextPrint = 0;
    static double lastBalOut = 0;

    for (;;) {
        const unsigned long now = millis();

        if (now >= nextLoop) {
            nextLoop = now + LOOP_MS;

            // 采样传感器
            ImuReading r; imu.fetch(r);

            const float dt = LOOP_MS / 1000.0;
            const float pitch = atan2(r.accZ, sqrt(r.accX*r.accX + r.accY*r.accY)) + 0.061664;
            const float gyroRate = r.gyroY;

            filtAngle = (1 - ALPHA_COMP) * pitch + ALPHA_COMP * (prevAngle + gyroRate*dt);
            prevAngle = filtAngle;

            // 倾角过大自动停电机
            if (fabs(filtAngle) > 0.7) enableMotors(false);
            else if (!motorsActive)    enableMotors(true);

            if (motorsActive) {
                // 速度测量.
                float rawL = motorL.getSpeed() / 2000.0;
                float rawR = motorR.getSpeed() / 2000.0;
                emaSpdL = EMA_ALPHA*rawL + (1-EMA_ALPHA)*emaSpdL;
                emaSpdR = EMA_ALPHA*rawR + (1-EMA_ALPHA)*emaSpdR;

                const float spdCmS_L = emaSpdL * distPerStep;
                const float spdCmS_R = emaSpdR * distPerStep;
                const float spdCmS   = (spdCmS_L - spdCmS_R) / 2.0;
                const float yawRate  = (spdCmS_L + spdCmS_R) / TRACK_CM;

                //  偏航外环
                if (!turning)  yawCorr = pidYaw.run(yawRate);
                else           yawCorr = 0;

                // 速度外环 目标倾角
                const double targetTilt = pidSpd.run(spdCmS) * 0.001 + TILT_OFFSET;

                // 平衡内环
                pidBal.target(targetTilt);
                double balOut = pidBal.run(filtAngle);
                balOut = balOut * 0.7 + lastBalOut * 0.3;  // 输出平滑
                lastBalOut = balOut;
                if (fabs(balOut) < DEAD_BAND) balOut = 0;

                motorL.setAccelerationRad(-balOut - turnCmd + yawCorr);
                motorR.setAccelerationRad( balOut - turnCmd + yawCorr);

                if (balOut > 0) { motorL.setTargetSpeedRad(-20); motorR.setTargetSpeedRad(20); }
                else if (balOut < 0) { motorL.setTargetSpeedRad(20); motorR.setTargetSpeedRad(-20); }
            }
        }

        if (now >= nextPrint) {
            nextPrint = now + PRINT_MS;
            Serial.print(F("Angle: ")); Serial.print(filtAngle, 4);
            Serial.print(F("  Speed(cm/s): "));
            Serial.println((emaSpdL - emaSpdR)*distPerStep/2.0, 4);
        }

        delay(1);       // give scheduler a breather
    }
}


void setup() {
    Serial.begin(115200);
    pinMode(SCOPE_PIN, OUTPUT);
    pinMode(MOT_EN, OUTPUT);
    enableMotors(true);       

    imu.begin();

    if (!hwTimer.attachInterruptInterval(STEP_ISR_US, stepperIsr)) {
        Serial.println(F("Timer attach failed")); while (true) {}
    }

    pidYaw.treatAsYaw(true);

    xTaskCreatePinnedToCore(stabilityTask, "stability", 10000, nullptr, 1, &stabilityHandle, 1);
    xTaskCreatePinnedToCore(serialCommandTask, "serialCtrl", 4096, nullptr, 1, &serialHandle, 0);
}

void loop() {
    
}
