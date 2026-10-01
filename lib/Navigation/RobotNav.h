#ifndef ROBOT_NAV_H
#define ROBOT_NAV_H

#include "BLEManager.h"
#include "PIDController.h"
#include "RobotConfig.h"
#include "encoder.h"
#include "maze_algorithm.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <Wire.h>

struct WallStatus {
  bool hasFront; // Có tường trước
  bool hasLeft;  // Có tường trái
  bool hasRight; // Có tường phải
  bool hasFrontNear = false; // Tường sát xe: vách trước của CHÍNH ô hiện tại
};

class RobotNav {
public:
  RobotNav();

  void init();
  void update();

  void stopMotors();
  void brakeMotors();

  void turnLeft(float angle = 90.0f);
  void turnRight(float angle = 90.0f);
  void runTurnTest();
  void updatePIDLoop();
  void startPID();
  void stopPID();
  void stepCell(int numCells = 1);

  // 4 Hàm Chuyển Động Nguyên Tử (Atomic Motion - Bước 3)
  // turn: 'F' thẳng, 'L' trái, 'R' phải, 'B' quay đầu (chỉ để báo cáo bản đồ)
  WallStatus moveOneCell(char turn = 'F');
  void reportCell(char act, const WallStatus &w, const WallStatus *pre = nullptr, bool blocked = false);
  WallStatus turnLeftAndStep();
  WallStatus turnRightAndStep();
  WallStatus turnAroundAndStep();
  WallStatus senseCurrentWalls();

  // Chế Độ 3: Bám Tường Tự Động (Autonomous Wall Follower)
  void startAutoWallFollow();
  void stopAutoWallFollow();
  void stepAutoWallFollow();
  bool autoWallFollowActive;
  // Cảm biến hông đặt xiên nên nhìn về PHÍA TRƯỚC ~1 ô: vách hông ô đích được lấy mẫu khi xe
  // đã chạy được sideSampleFrac (0..1) quãng đường của ô, không đọc lúc đã đứng ở tâm ô.
  float sideSampleFrac;
  bool lastMoveBlocked; // bước gần nhất bị tường chặn sớm, xe vẫn ở ô cũ
  bool startCellFresh; // xe còn ở ô xuất phát (hướng Bắc, có tường trái/phải/sau), chưa đi ô nào
  WallStatus preWalls; // vách đọc trước khi bước (cảm biến nhìn trước 1 ô) = vách ô sắp vào
  WallStatus curWalls; // vách của ô đang đứng (theo hướng xe hiện tại)
  bool patrolMode;      // true: đi thẳng, gặp tường thì quay đầu
  bool followRightHand; // true: Tay Phải, false: Tay Trái
  int autoCellCount;    // Đếm số ô đã đi
  int autoMaxCells;     // Giới hạn an toàn (mặc định 60 ô)
  String lastAutoDecision;

  // Chế Độ 4: Flood-fill (tự khám phá mê cung bằng thuật toán maze_algorithm.h)
  // Toạ độ tuyệt đối maze[x][y]: y tăng về Bắc, x tăng về Đông. Hướng: 0=N 1=E 2=S 3=W
  ParentMaze ffMaze;
  bool ffActive;
  uint8_t ffMode; // 0: đi tới đích (goal), 1: khám phá toàn bộ rồi quay về ô xuất phát
  int8_t ffX, ffY, ffH;
  Point ffStart, ffGoal;
  uint8_t ffCols, ffRows; // kích thước mê cung thật (<= MAZE_SIZE), ngoài vùng này coi như tường
  uint8_t ffGoalSize; // 1: đích 1 ô, 2: vùng đích 2x2 bắt đầu từ ffGoal (góc dưới trái)
  bool ffInGoal(int x, int y) const {
    return x >= ffGoal.x && x < ffGoal.x + ffGoalSize && y >= ffGoal.y && y < ffGoal.y + ffGoalSize;
  }
  void startFloodFill(uint8_t mode);
  void stopFloodFill(const char *reason = "ĐÃ DỪNG");
  void stepFloodFill();
  void resetFloodFill(int x = 0, int y = 0, int h = 0);

  // Encoder helper methods
  long getLeftEncoder() const;
  long getRightEncoder() const;
  void resetEnc();

  // Chạy từng ô theo Encoder (Cell Stepping)
  long pulsesPerCell;
  bool stepCellActive;
  long stepStartPulses;
  long stepStartL;
  long stepStartR;
  long stepTargetPulses;
  long stepTraveledPulses;
  unsigned long stepStartTime;

  // Các biến thông số có thể điều chỉnh
  float leftCompensation;
  float rightCompensation;
  uint8_t turnSpeed;
  uint8_t baseForwardSpeed;
  int currentLeftSpeed;
  int currentRightSpeed;
  bool autoTestMode;
  bool pidRunActive;

  // Cấu hình đồng bộ bánh bằng Encoder (Cascaded Inner Loop)
  bool encoderSyncActive;
  float encKp;
  bool invertEncLeft;
  bool invertEncRight;

  // Thông số hiệu chuẩn khoảng cách ô (VL53L0X)
  float targetLeftDist;
  float targetRightDist;
  float centerOffset;
  uint16_t wallThreshold;
  uint16_t frontStopDist;
  uint16_t frontWallDist; // dF <= ngưỡng này (mm) là có tường trước mặt

  // Cảm biến & PID
  VL53L0X sensorLeft;
  VL53L0X sensorFront;
  VL53L0X sensorRight;
  MPU6050 mpu6050;

  PIDController wallPID;
  PIDController gyroPID;

  bool leftReady;
  bool frontReady;
  bool rightReady;
  bool mpuReady;

  // Dữ liệu cảm biến sau lọc & Vùng chết tâm ô
  float smoothDL;
  float smoothDF;
  float smoothDR;
  float currentWallError;
  float wallDeadband; // Vùng chết khử nhiễu tâm ô (mm)

  void updateSensors();

private:
  float _targetYaw;
  unsigned long _lastPIDLoopTime;
  unsigned long _lastSensorReadTime;
  long _lastEncLeft;
  long _lastEncRight;
  long _startEncLeft;
  long _startEncRight;
  float _smoothError;

  bool initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address,
                const char *name);
};

extern RobotNav robotNav;

#endif
