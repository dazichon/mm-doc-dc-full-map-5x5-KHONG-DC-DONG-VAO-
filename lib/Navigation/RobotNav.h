#ifndef ROBOT_NAV_H
#define ROBOT_NAV_H

#include "SerialLink.h"
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

// Thông số chế độ GIỮ VỊ TRÍ TRONG Ô (đặt trong main.cpp)
struct HoldParams {
  float targetF = 97.0f;  // khoảng cách mong muốn tới tường trước (mm)
  float yawKp = 2.0f;      // độ mạnh giữ thẳng hướng bằng gyro (0 = tắt)
  // Wall correction (thành phần nhỏ cộng vào correction tổng, chỉ P)
  bool wallEnable = false;
  float wallKp = 0.3f;        // PWM / mm sai lệch
  float wallMaxCorr = 6.0f;   // giới hạn |wall correction| (PWM)
  float wallBaseL = 173.0f;   // VL53 trái khi xe ở giữa (mm)
  float wallBaseR = 168.0f;   // VL53 phải khi xe ở giữa (mm)
  float wallValidMax = 250.0f; // đọc xa hơn mức này = không có tường (hở/ô kế tiếp) -> bỏ qua
  float wallErrClamp = 30.0f; // kẹp sai lệch (mm)
  bool useFront = true;       // false: bỏ VL53 trước (không dùng để phanh)
  uint8_t cells = 5;       // số ô chạy trong bài test
};

enum CornerState : uint8_t { STRAIGHT, CORNER_LEFT, CORNER_RIGHT, EXIT_CORNER };

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
  long stopLeadPulses = 0; // xung: phanh sớm chừng này trước đích để bù quãng trôi sau phanh (0 = tắt)
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
  float yawMaxCorr = 20.0f; // giới hạn hiệu chỉnh yaw (PWM)
  float encMaxCorr = 10.0f; // giới hạn hiệu chỉnh encoder (PWM) mỗi bên: chỉ cân 2 bánh, không được bẻ lái mạnh
  bool invertEncLeft;
  bool invertEncRight;

  // Thông số hiệu chuẩn khoảng cách ô (VL53L0X)
  uint16_t wallThreshold;      // Tường TRÁI: dL < ngưỡng này là có tường (mm)
  uint16_t rightWallThreshold; // Tường PHẢI: dR < ngưỡng này là có tường (mm)
  uint16_t frontStopDist;
  uint16_t frontWallDist; // dF <= ngưỡng này (mm) là có tường trước mặt

  // Cảm biến & PID
  VL53L0X sensorLeft;
  VL53L0X sensorFront;
  VL53L0X sensorRight;
  MPU6050 mpu6050;

  bool leftReady;
  bool frontReady;
  bool rightReady;
  bool mpuReady;

  // Dữ liệu cảm biến sau lọc & Vùng chết tâm ô
  // Giá trị thô mới nhất của VL53L0X (mm), -1 = lỗi/timeout
  int16_t _rawL = -1, _rawF = -1, _rawR = -1;
  unsigned long _tL = 0, _tF = 0, _tR = 0;
  float smoothDL;
  float smoothDF;
  float smoothDR;

  void updateSensors();

  // Cân bằng 2 động cơ: hệ số nhân công suất, áp dụng chung cho MỌI tốc độ và mọi chế độ chạy
  uint16_t brakeMs = 50; // thời gian hãm chủ động (ms) khi dừng
  float motorScaleL = 1.0f;
  float motorScaleR = 1.0f;

  // Chế độ giữ vị trí trong ô: bám 3 tường (trái/trước/phải) theo HoldParams
  HoldParams hold;
  bool holdRun = false;   // true: chạy PID liên tục, tường trước <= frontStopDist thì đứng yên nhưng PID vẫn chạy
  void startHold(bool keepLog = false);

  // Log chạy lưu trong RAM (vòng 1000 mẫu / 50ms): in ra Serial khi cắm USB hoặc lệnh "LOG"
  // Test quay tại chỗ theo yaw MPU6050 (blocking). Log lưu RAM, in bằng printTurnLog()
  float turnMaxPwm = 70.0f;      // PWM lớn nhất khi quay
  float turnMinPwm = 42.0f;      // PWM nhỏ nhất (đủ thắng ma sát) khi gần target
  float turnDecelZone = 35.0f;   // trong vùng này (độ) PWM giảm tuyến tính về turnMinPwm
  float turnTolerance = 1.5f;    // dừng khi |error| <= giá trị này (độ)
  float turnLeftYawSign = -1.0f; // dấu thay đổi yaw khi xe quay TRÁI (đo từ log: lệch phải => yaw tăng, nên quay trái => yaw giảm)
  float turnBrakeLead = 0.05f;   // s: phanh sớm khi sai số <= dung sai + tốc độ quay * giá trị này
  float _turnHeading = 0.0f;
  bool _turnHeadingValid = false;
  bool turnLeft90Test(float angle = 90.0f);
  void printTurnLog();
  uint16_t seqCellPulses = 4200; // xung/ô cho sequence test (giá trị tạm, chưa calib)
  // Cua 90° khi đang chạy (không quay tại chỗ): 2 bánh PWM khác nhau, yaw MPU là feedback, giảm độ cong khi gần target
  float cornerSpeed = 60.0f;       // PWM bánh ngoài (cua chậm)
  float cornerInnerRatio = 0.45f;  // PWM bánh trong / bánh ngoài ở độ cong tối đa (nhỏ = cua gắt)
  float cornerDecelZone = 30.0f;   // độ: trong vùng này độ cong giảm dần về 0
  float cornerMinCurve = 0.2f;     // độ cong tối thiểu (0..1) để không đứng lại khi còn sai số
  float cornerTolerance = 2.0f;    // độ: coi là tới target
  float cornerLead = 0.05f;        // s: dự đoán sai số = err - tốc độ quay * lead (giảm overshoot)

  // Corner Controller (state machine không chặn): STRAIGHT -> CORNER_LEFT/RIGHT -> EXIT_CORNER -> STRAIGHT
  // Navigation gọi startCornerLeft()/startCornerRight(); RobotNav::update() tự chạy updateCorner().
  // Khi đang cua, PID đi thẳng (updatePIDLoop) tạm nghỉ; thoát cua thì reset PID rồi chạy tiếp.
  CornerState cornerState = STRAIGHT;
  uint16_t cornerEntryMs = 120;    // ms: giảm tốc từ tốc độ chạy thẳng về cornerSpeed khi vào cua
  uint16_t cornerExitMs = 150;     // ms: tăng tốc từ cornerSpeed về tốc độ chạy thẳng khi thoát cua
  uint16_t cornerTimeoutMs = 3500; // ms: quá thời gian này coi như tới target để không cua vô tận
  float cornerStartYaw = 0.0f;     // yaw lúc bắt đầu cua
  float cornerTargetYaw = 0.0f;    // yaw đích, chuẩn hóa về [-180, 180]
  bool startCornerLeft();
  bool startCornerRight();
  void updateCorner();
  bool cornerBusy() const { return cornerState != STRAIGHT; }
  // Calib lại 1 ô khi vào ngõ cụt: sau khi quay 180° lùi sát vách cụt (mốc đầu ô) rồi tiến calibFwdPulses xung tới tâm ô
  float calibRevPwm = 60.0f;            // PWM khi lùi sát vách
  uint16_t calibRevMs = 100;            // ms: thời gian lùi (cố định), sau đó dừng, calib gyro về 0 độ rồi tiến calibFwdPulses
  long calibFwdPulses = 1500;           // xung: từ vách tới tâm ô (không tính vào quãng 1 ô tiếp theo)
  bool calibCellFromBackWall();
  void runDeadEndTest();                // test: 1 ô -> dừng -> quay 180° -> calibCellFromBackWall()
  void runCellToCellTest();        // test: 1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng
  void runCornerNavTest();         // test: đi 1 ô -> cua trái (state machine) -> đi 1 ô -> dừng
  // Test calibration encoder: chạy thẳng 4 ô liên tục (seqCellPulses xung/ô), dừng rồi in ENC_L, ENC_R, AVG_TICKS, TICKS_PER_CELL
  void runCalibTest(uint8_t cells = 4);
  String calibReport;              // kết quả lần calib gần nhất (in lại khi cắm USB)
  bool calibPrinted = true;            // MOVE 1 ô (không dừng) -> CORNER trái 90 -> MOVE 1 ô -> STOP
  bool stepNoStop = false;         // true: đi hết quãng đường thì KHÔNG phanh/dừng, để chạy tiếp vào corner      // MOVE 1 ô -> TURN trái 90 -> MOVE -> TURN phải 90 -> MOVE -> TURN 180 -> MOVE -> STOP
  bool turnLogPending() const { return _turnLogN > 0 && !_turnLogPrinted; }
  void logSample();
  void printLog();
  uint16_t logCount() const { return _logCount; }
  void clearLog() { _logCount = 0; _logHead = 0; }

  // Lưu tốc độ chạy thẳng vào flash (NVS) -> giữ nguyên sau khi tắt/bật xe, dùng cho mọi chế độ
  void saveForwardSpeed();

private:
  struct LogRec { uint32_t t; int32_t eL, eR; int16_t dL, dF, dR, vL, vR, pL, pR, c, fL, fR; float yaw, yawT, yawE, yawC, wallE, wallC; };
  static const uint16_t LOG_SIZE = 1500;
  struct TurnRec { uint32_t t; float yaw, target, err; int16_t pl, pr; uint8_t st; }; // st: 0=TURN 1=CORNER 2=STRAIGHT
  static const uint16_t TURN_LOG_SIZE = 400;
  TurnRec _turnLog[TURN_LOG_SIZE];
  uint16_t _turnLogN = 0;
  bool _turnLogPrinted = true;
  LogRec _log[LOG_SIZE];
  uint16_t _logHead = 0, _logCount = 0;
  unsigned long _logLastT = 0;
  long _logLastL = 0, _logLastR = 0;
  bool _holdStopped = false;
  float _lastYawErr = 0.0f, _lastYawCorr = 0.0f;
  float _lastWallErr = 0.0f, _lastWallCorr = 0.0f;
  int16_t _wBufL[5] = {999, 999, 999, 999, 999}, _wBufR[5] = {999, 999, 999, 999, 999};
  uint8_t _wIdxL = 0, _wIdxR = 0;
  unsigned long _wLastTL = 0, _wLastTR = 0;
  float _wFiltL = 999.0f, _wFiltR = 999.0f; // trung vị 5 mẫu VL53 thô
  void updateWallFilter();
  float _lastCorr = 0.0f; // hiệu chỉnh PID lần gần nhất (>0: giảm bánh trái, tăng bánh phải)
  unsigned long _holdFreeSince = 0;
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

  // Corner Controller nội bộ
  float _cornerTarget = 0.0f;      // yaw đích cùng hệ góc tích lũy với MPU (không chuẩn hóa)
  unsigned long _cornerT0 = 0, _cornerStateT = 0, _cornerPrevT = 0;
  float _cornerPrevYaw = 0.0f, _cornerV0 = 0.0f;
  bool startCorner(CornerState dir);
  void finishCorner();
  void cornerPwm(float err, float errPred, float outer, int &pl, int &pr) const;
};

extern RobotNav robotNav;

#endif
