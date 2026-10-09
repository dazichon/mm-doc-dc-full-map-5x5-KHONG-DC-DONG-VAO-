#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
    MPU6050(uint8_t addr = 0x68);
    bool begin();
    void calibrate(int samples = 300);
    void update();
    bool rotateToAngle(float targetAngle, uint8_t m1In1, uint8_t m1In2,
                       uint8_t m2In1, uint8_t m2In2,
                       float compensation = 18.0f,
                       uint8_t turnSpeed = 50,
                       unsigned long timeoutMs = 2500);
    
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getAngleZ() const { return getYaw(); }
    float getRelativeYaw() const;
    void setYawSetpoint();
    void resetYaw();
    void setAutoBias(bool on) { _autoBias = on; _stillTime = 0.0f; } // tắt khi xe đang chạy

private:
    uint8_t _addr;
    float _gyroZOffset;
    float _roll;
    float _pitch;
    float _yaw;
    float _yawSetpoint;
    bool _autoBias = true;
    float _stillTime;   // thời gian liên tục gần như đứng yên (s)
    unsigned long _lastTime;
    
    bool readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
};

#endif