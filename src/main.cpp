#include <Arduino.h>
#include "RobotConfig.h"
#include "SerialLink.h"
#include "RobotNav.h"
#include "CommandHandler.h"
#include "MainParams.h"


void setup() {
    delay(500);

    serialLink.begin();

    robotNav.init();

    robotNav.motorScaleL = 1.0f + MOTOR_TRIM_LEFT / 100.0f;
    robotNav.motorScaleR = 1.0f + MOTOR_TRIM_RIGHT / 100.0f;

    robotNav.turnMaxPwm = TURN_MAX_PWM;
    robotNav.turnMinPwm = TURN_MIN_PWM;
    robotNav.turnDecelZone = TURN_DECEL_ZONE;
    robotNav.turnTolerance = TURN_TOLERANCE;
    robotNav.turnLeftYawSign = TURN_LEFT_YAW_SIGN;
    robotNav.turnBrakeLead = TURN_BRAKE_LEAD;
    robotNav.cornerSpeed = CORNER_SPEED;
    robotNav.cornerInnerRatio = CORNER_INNER_RATIO;
    robotNav.cornerDecelZone = CORNER_DECEL_ZONE;
    robotNav.cornerTolerance = CORNER_TOLERANCE;
    robotNav.cornerLead = CORNER_LEAD;
    pinMode(BUTTON, INPUT_PULLUP);
    robotNav.encKp = ENC_STRAIGHT_KP;
    robotNav.encMaxCorr = ENC_MAX_CORR;
    robotNav.pulsesPerCell = CELL_PULSES;
    robotNav.hold.cells = TEST_CELLS;
    robotNav.hold.wallEnable = WALL_ENABLE;
    robotNav.hold.wallKp = WALL_KP;
    robotNav.hold.wallMaxCorr = WALL_MAX_CORR;
    robotNav.hold.wallBaseL = WALL_BASE_L;
    robotNav.hold.wallBaseR = WALL_BASE_R;
    robotNav.hold.wallValidMax = WALL_VALID_MAX;
    robotNav.hold.useFront = HOLD_USE_FRONT;
    robotNav.hold.targetF = HOLD_TARGET_FRONT;
    robotNav.hold.yawKp = HOLD_YAW_KP;

    Serial.println("==========================================");
    Serial.println(">> ROBOT MICROMOUSE READY!");
    Serial.println("==========================================");

#if TEST_MOTOR_DRIVE
    delay(HOLD_START_DELAY_MS);
    robotNav.resetEnc();
    robotNav.clearLog();
    // Test cân bằng motor: tắt PID/tường/gyro/trim, cùng 1 PWM cho 2 bánh
    robotNav.motorScaleL = 1.0f;
    robotNav.motorScaleR = 1.0f;
    robotNav.currentLeftSpeed = TEST_PWM;
    robotNav.currentRightSpeed = TEST_PWM;
    analogWrite(M1_IN1, robotNav.currentLeftSpeed);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, robotNav.currentRightSpeed);
    analogWrite(M2_IN2, 0);
#elif HOLD_AUTO_START
    // Chờ 3s (vẫn đọc cảm biến để giá trị lọc ổn định), động cơ đứng yên
    unsigned long t0 = millis();
    while (millis() - t0 < HOLD_START_DELAY_MS) {
        robotNav.updateSensors();
        commandHandler.update();
    }
    // Hết 3s: hiệu chuẩn lại gyro (xe phải đứng yên) và đưa yaw về 0 rồi mới chạy PID
    if (robotNav.mpuReady) {
        Serial.println(">> CALIB MPU6050... GIU XE DUNG YEN");
        robotNav.mpu6050.calibrate();
        robotNav.mpu6050.resetYaw();
        Serial.println(">> CALIB XONG, YAW = 0");
    }
    robotNav.startHold();
#endif
}

#if TEST_MOTOR_DRIVE
static bool testDone = false;
static bool testPrinted = false;
static long testRawL = 0, testRawR = 0, testDiff = 0;
static float testPct = 0.0f;
#endif

void loop() {
#if TEST_MOTOR_DRIVE
    if (!testDone) {
        robotNav.logSample(); // ghi log RAM ~30Hz (ENC, SPEED, PWM_L, PWM_R), không in Serial khi đang chạy
        long l = labs(robotNav.getLeftEncoder());
        long r = labs(robotNav.getRightEncoder());
        if ((l + r) / 2 >= TEST_PULSES) {
            robotNav.stopMotors();
            robotNav.logSample();
            testDone = true;
            testRawL = robotNav.getLeftEncoder();
            testRawR = robotNav.getRightEncoder();
            testDiff = l - r;
            testPct = 100.0f * testDiff / (float)((l + r) / 2);
        }
    } else if (!testPrinted && (bool)Serial) {
        // Xe đã dừng: khi cắm USB (mở Serial) in kết quả + log đúng 1 lần, sau đó im lặng
        testPrinted = true;
        delay(300);
        Serial.println("TEST MOTOR BALANCE (open-loop)");
        Serial.printf("PWM = %d (ca 2 banh), trim = 0\n", TEST_PWM);
        Serial.printf("ENC_L = %ld\n", testRawL);
        Serial.printf("ENC_R = %ld\n", testRawR);
        Serial.printf("DIFF = %ld (|L| - |R|)\n", testDiff);
        Serial.printf("DIFF_PERCENT = %.2f%%\n", testPct);
        Serial.printf("%s quay nhieu hon\n", testDiff > 0 ? "BANH TRAI" : "BANH PHAI");
        robotNav.printLog();
    }
    commandHandler.update();
    return;
#endif
    robotNav.update();
    commandHandler.update();

    // Nút BUTTON (nhấn = LOW): chờ TURN_DELAY_MS rồi quay trái TURN_ANGLE độ tại chỗ (chỉ khi xe không đang chạy thẳng)
    // Mỗi lần bấm = đúng 1 lần quay: phải nhả nút (HIGH) thì mới nhận lần bấm sau
    static unsigned long turnAt = 0;
    static bool armed = true;
    if (!armed && turnAt == 0 && digitalRead(BUTTON) == HIGH) {
        delay(30);
        if (digitalRead(BUTTON) == HIGH) armed = true;
    }
    if (armed && !robotNav.pidRunActive && turnAt == 0 && digitalRead(BUTTON) == LOW) {
        delay(30);
        if (digitalRead(BUTTON) == LOW) { turnAt = millis(); armed = false; }
    }
    if (turnAt != 0 && millis() - turnAt >= TURN_DELAY_MS) {
        turnAt = 0;
#if SEQ_MODE == 2
        robotNav.seqCellPulses = SEQ_CELL_PULSES;
        robotNav.calibRevPwm = CALIB_REV_PWM;
        robotNav.calibRevMs = CALIB_REV_MS;
        robotNav.calibFwdPulses = CALIB_FWD_PULSES;
        robotNav.stopLeadPulses = STOP_LEAD_PULSES;
        robotNav.runDeadEndTest();
        robotNav.stopLeadPulses = 0;
#elif SEQ_MODE == 1
        robotNav.seqCellPulses = SEQ_CELL_PULSES;
        robotNav.stopLeadPulses = STOP_LEAD_PULSES;
        robotNav.runCellToCellTest();
        robotNav.stopLeadPulses = 0;
#elif SEQ_MODE == 4
        robotNav.seqCellPulses = SEQ_CELL_PULSES;
        robotNav.runCornerNavTest();
#elif SEQ_MODE == 3
        robotNav.seqCellPulses = SEQ_CELL_PULSES;
        robotNav.runCalibTest(4);
#else
        robotNav.turnLeft90Test(TURN_RIGHT ? -TURN_ANGLE : TURN_ANGLE); // góc dương = trái, âm = phải
#endif
    }
    if (!robotNav.calibPrinted && (bool)Serial) {
        delay(300);
        Serial.println(robotNav.calibReport);
        robotNav.calibPrinted = true;
    }
    if (robotNav.turnLogPending() && (bool)Serial) {
        delay(300);
        robotNav.printTurnLog();
    }

    // Xe dừng xong: nếu cắm USB (mở Serial) thì in log đúng 1 lần, sau đó không in gì nữa
    static bool printedDone = false;
    if (robotNav.pidRunActive) printedDone = false;
    else if (!printedDone && (bool)Serial && robotNav.logCount() > 0) {
        printedDone = true;
        delay(300);
        robotNav.printLog();
    }
}
