#include "MPU6050.h"
#include "SerialLink.h"
#include <math.h>

// Các địa chỉ thanh ghi phần cứng MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

MPU6050::MPU6050(uint8_t addr) {
    _addr = addr;
    _gyroZOffset = 0.0;
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _lastTime = 0;
    _stillTime = 0.0;
}

bool MPU6050::begin() {
    // 1. Kiểm tra thiết bị phản hồi
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        return false; // Không tìm thấy MPU6050 ở địa chỉ _addr
    }

    // 2. Đánh thức MPU6050 (Power Management 1 = 0)
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    delay(10); // Đợi chip khởi động lại dao động

    // 3. Cấu hình dải đo Gyro: +/- 2000 deg/s (Hệ số nhạy: 16.4 LSB/(deg/s)) để đo chính xác tốc độ quay cao của xe
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_GYRO_CONFIG);
    Wire.write(0x18); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // 4. Cấu hình dải đo Accel: +/- 8g
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_CONFIG);
    Wire.write(0x10); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    _lastTime = micros();
    return true;
}

bool MPU6050::readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        return false; // I2C NACK hoặc bus bận, tránh gọi requestFrom gây Error 259
    }

    uint8_t bytesReceived = Wire.requestFrom(_addr, (uint8_t)14);
    if (bytesReceived != 14) {
        return false; // Không nhận đủ byte dữ liệu
    }

    *ax = (int16_t)((Wire.read() << 8) | Wire.read());
    *ay = (int16_t)((Wire.read() << 8) | Wire.read());
    *az = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // Bỏ qua giá trị nhiệt độ (Temperature)
    *gx = (int16_t)((Wire.read() << 8) | Wire.read());
    *gy = (int16_t)((Wire.read() << 8) | Wire.read());
    *gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

void MPU6050::calibrate(int samples) {
    float sumZ = 0;
    int validSamples = 0;
    int16_t ax, ay, az, gx, gy, gz;
    
    // Lưu ý: Khi hiệu chuẩn, robot phải đặt đứng yên hoàn toàn
    // Bỏ 30 mẫu đầu cho cảm biến ổn định rồi mới lấy trung bình
    for (int i = 0; i < samples + 30; i++) {
        if (readRawData(&ax, &ay, &az, &gx, &gy, &gz) && i >= 30) {
            sumZ += gz;
            validSamples++;
        }
        delay(3);
    }
    
    // Tính toán độ lệch tĩnh (offset) cho trục Z với dải 2000 deg/s (16.4 LSB/(deg/s))
    if (validSamples >= samples / 2) {
        _gyroZOffset = (sumZ / validSamples) / 16.4f;
    }
    _stillTime = 0.0;
    String calMsg = ">> GYRO CALIB: " + String(validSamples) + "/" + String(samples) +
                    " mau hop le | offset Z = " + String(_gyroZOffset, 3) + " do/s";
    Serial.println(calMsg);
    if (serialLink.isConnected()) {
        serialLink.println(calMsg);
    }
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _lastTime = micros();
}

void MPU6050::update() {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
        return; // Đọc I2C thất bại -> bỏ qua vòng này, không tính toán sai góc
    }

    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0; // Đổi từ micro giây sang giây
    if (_lastTime == 0 || dt > 1.0) {
        dt = 0.01;
    }
    _lastTime = now;

    // Góc nghiêng lấy từ gia tốc kế, tính theo độ.
    _roll = atan2((float)ay, (float)az) * 180.0 / PI;
    _pitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

    // Đổi giá trị thô sang đơn vị độ/giây (deg/s) với hệ số 16.4 (cho dải 2000 deg/s)
    // Cảm biến đặt úp nên đảo chiều quay quanh trục Z.
    float residual = (gz / 16.4f) - _gyroZOffset; // độ/giây sau khi trừ offset lúc calib

    // Tự bù trôi (drift): nếu gyro gần như đứng yên liên tục > 0.4s thì offset đang lệch
    // nhẹ do nhiệt độ -> kéo offset về giá trị đo được rất chậm.
    if (_autoBias && fabsf(residual) < 1.5f) {
        _stillTime += dt;
        if (_stillTime > 0.4f) {
            _gyroZOffset += residual * 0.02f;
            residual = (gz / 16.4f) - _gyroZOffset;
        }
    } else {
        _stillTime = 0.0f;
    }

    float gyroZRate = -residual;

    // Deadzone nhỏ còn lại để triệt nhiễu số
    if (fabsf(gyroZRate) < 0.15f) {
        gyroZRate = 0.0f;
    }

    // Tích phân vận tốc góc theo thời gian để ra góc Yaw
    _yaw += gyroZRate * dt;
}

bool MPU6050::rotateToAngle(float targetAngle, uint8_t m1In1, uint8_t m1In2,
                            uint8_t m2In1, uint8_t m2In2,
                            float compensation, uint8_t turnSpeed,
                            unsigned long timeoutMs) {
    if (targetAngle == 0.0f) {
        return true;
    }

    update();
    float startAngle = _yaw;
    float requestedAngle = fabs(targetAngle);
    float effectiveAngle = requestedAngle > compensation
                               ? requestedAngle - compensation
                               : 0.0f;

    String startMsg = ">> BAT DAU QUAY: Goc hien tai = " + String(startAngle, 2) +
                      " | Can quay = " + String(effectiveAngle, 2) +
                      " do | Toc do PWM = " + String(turnSpeed);
    Serial.println(startMsg);
    if (serialLink.isConnected()) {
        serialLink.println(startMsg);
        String jsonStart = "{\"type\":\"turn_start\",\"start\":" + String(startAngle, 2) +
                           ",\"effective\":" + String(effectiveAngle, 2) +
                           ",\"spd\":" + String(turnSpeed) + "}";
        serialLink.println(jsonStart);
    }

    // Điều khiển động cơ quay theo tốc độ PWM:
    // targetAngle > 0: Quay trái (Motor trái lùi, Motor phải tiến)
    // targetAngle < 0: Quay phải (Motor trái tiến, Motor phải lùi)
    unsigned long turnStart = millis();
    bool reachedTarget = false;

    // Giảm tốc tiếp cận (Proportional Deceleration)
    // Trong 35 độ cuối cùng, tốc độ PWM giảm dần từ turnSpeed về MIN_TURN_SPEED
    constexpr uint8_t MIN_TURN_SPEED = 42; 
    constexpr float DECEL_ZONE = 35.0f;

    while (millis() - turnStart <= timeoutMs) {
        update();
        float turnedAngle = fabs(_yaw - startAngle);
        float remainingAngle = effectiveAngle - turnedAngle;

        if (turnedAngle >= effectiveAngle || remainingAngle <= 0.0f) {
            reachedTarget = true;
            break;
        }

        // Tính tốc độ giảm dần theo góc còn lại
        uint8_t currentSpeed = turnSpeed;
        if (remainingAngle < DECEL_ZONE && turnSpeed > MIN_TURN_SPEED) {
            float ratio = remainingAngle / DECEL_ZONE; // 1.0 -> 0.0
            currentSpeed = MIN_TURN_SPEED + (uint8_t)((turnSpeed - MIN_TURN_SPEED) * ratio);
        }

        if (targetAngle > 0.0f) {
            // Quay trái: Motor trái lùi, Motor phải tiến
            analogWrite(m1In1, 0);
            analogWrite(m1In2, currentSpeed);
            analogWrite(m2In1, currentSpeed);
            analogWrite(m2In2, 0);
        } else {
            // Quay phải: Motor trái tiến, Motor phải lùi
            analogWrite(m1In1, currentSpeed);
            analogWrite(m1In2, 0);
            analogWrite(m2In1, 0);
            analogWrite(m2In2, currentSpeed);
        }

        delay(2);
    }

    // 1. Phanh ngắn mạch (Active Braking 100% PWM) trong 70ms để triệt tiêu hoàn toàn quán tính
    analogWrite(m1In1, 255);
    analogWrite(m1In2, 255);
    analogWrite(m2In1, 255);
    analogWrite(m2In2, 255);
    delay(70);

    // 2. Nhả motor về trạng thái thả tự do (Tắt dứt điểm)
    analogWrite(m1In1, 0);
    analogWrite(m1In2, 0);
    analogWrite(m2In1, 0);
    analogWrite(m2In2, 0);
    digitalWrite(m1In1, LOW);
    digitalWrite(m1In2, LOW);
    digitalWrite(m2In1, LOW);
    digitalWrite(m2In2, LOW);

    update();
    float turnedTotal = fabs(_yaw - startAngle);
    
    // Reset góc yaw về 0 sau khi quay xong
    resetYaw();
    
    String resMsg = ">> KET QUA: Goc da quay duoc = " + String(turnedTotal, 2) +
                    " do | Goc hien tai (da reset) = " + String(_yaw, 2) +
                    (reachedTarget ? " (THANH CONG)" : " (TIMEOUT - KET BANH HOAC LOI DOC GYRO)");
    Serial.println(resMsg);
    if (serialLink.isConnected()) {
        serialLink.println(resMsg);
        String jsonRes = "{\"type\":\"turn_res\",\"target\":" + String(fabs(targetAngle), 1) +
                         ",\"turned\":" + String(turnedTotal, 2) +
                         ",\"yaw\":" + String(_yaw, 2) +
                         ",\"success\":" + String(reachedTarget ? "true" : "false") + "}";
        serialLink.println(jsonRes);
    }

    return reachedTarget;
}

float MPU6050::getRoll() const {
    return _roll;
}

float MPU6050::getPitch() const {
    return _pitch;
}

float MPU6050::getYaw() const {
    return _yaw;
}

float MPU6050::getRelativeYaw() const {
    return _yaw - _yawSetpoint;
}

void MPU6050::setYawSetpoint() {
    _yawSetpoint = _yaw;
}

void MPU6050::resetYaw() {
    _yaw = 0.0;
    _yawSetpoint = 0.0;
}