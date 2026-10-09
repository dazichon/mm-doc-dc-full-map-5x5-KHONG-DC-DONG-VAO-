#include "RobotNav.h"
#include <Preferences.h>

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR) {
  leftReady = false;
  frontReady = false;
  rightReady = false;
  mpuReady = false;
  autoTestMode = false;
  pidRunActive = false;

  // Thông số quay đã căn ở Chế độ 1 - dùng chung cho mọi lần quay (chế độ 1, 2, bám tường, flood-fill)
  leftCompensation = 5.0f;
  rightCompensation = 6.0f;
  turnSpeed = 65;
  baseForwardSpeed = 75;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  _targetYaw = 0.0f;
  _lastPIDLoopTime = 0;

  // Giá trị thực tế đo được tại tâm ô (Theo kết quả đo mới nhất trên sa hình)
  wallThreshold = 180;      // Tường TRÁI: dL < 180mm là có tường
  rightWallThreshold = 160; // Tường PHẢI: dR < 160mm là có tường
  frontStopDist = 123; // Phanh dừng khi cách tường trước <= 123mm (tâm ô thực tế là 121mm)
  frontWallDist = 360; // dF <= 360mm: có tường trước của Ô PHÍA TRƯỚC (~300mm)

  // Giá trị cảm biến lọc & Vùng chết tâm ô
  smoothDL = 171.0f;
  smoothDF = 999.0f;
  smoothDR = 146.0f;
  _lastSensorReadTime = 0;

  // Cấu hình đồng bộ bánh bằng Encoder (Cascaded Inner Loop)
  encoderSyncActive = true;
  encKp = 0.35f;
  invertEncLeft = false;
  invertEncRight = false;
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;
  _smoothError = 0.0f;

  // Chạy từng ô theo xung Encoder (Cell Stepping)
  pulsesPerCell = 4600; // Đo thực tế từ tâm ô 1 tới giữa ô 2 = 4400 xung
  stepCellActive = false;
  stepStartPulses = 0;
  stepStartL = 0;
  stepStartR = 0;
  stepTargetPulses = 0;
  stepTraveledPulses = 0;
  stepStartTime = 0;

  // Chế Độ 3: Bám Tường Tự Động (Autonomous Wall Follower)
  autoWallFollowActive = false;
  patrolMode = false;
  sideSampleFrac = 0.30f;
  curWalls = {false, false, false};
  followRightHand = true; // Mặc định ưu tiên luật bàn tay phải
  autoCellCount = 0;
  autoMaxCells = 60; // Tối đa 60 ô phòng lặp vô tận khi thử nghiệm
  lastAutoDecision = "SẴN SÀNG";

  // Chế Độ 4: Flood-fill
  ffActive = false;
  ffMode = 0;
  ffGoal = {7, 7};
  ffGoalSize = 2; // 4 ô trung tâm mê cung 16x16
  ffCols = ffRows = MAZE_SIZE;
  lastMoveBlocked = false;
  resetFloodFill(0, 0, 0);
}

long RobotNav::getLeftEncoder() const {
  return invertEncLeft ? -enc1A_count : enc1A_count;
}

long RobotNav::getRightEncoder() const {
  return invertEncRight ? -enc2A_count : enc2A_count;
}

void RobotNav::resetEnc() {
  resetEncoders();
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;
  _startEncRight = 0;
}

void RobotNav::init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotors();

  // Nạp tốc độ chạy thẳng đã lưu (chưa lưu lần nào -> giữ mặc định)
  {
    Preferences p;
    if (p.begin("mmparams", true)) {
      baseForwardSpeed = p.getUChar("fspd", baseForwardSpeed);
      p.end();
    }
  }

  // Khởi tạo phần cứng Encoder đọc xung bánh xe
  setupEncoders();
  resetEnc();
  Serial.println("ENCODERS INITIALIZED");

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  leftReady = initVL53(sensorLeft, XSHUT_LEFT, ADDRESS_LEFT, "LEFT");
  frontReady = initVL53(sensorFront, XSHUT_FRONT, ADDRESS_FRONT, "FRONT");
  rightReady = initVL53(sensorRight, XSHUT_RIGHT, ADDRESS_RIGHT, "RIGHT");

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 FOUND @ 0x68");
    mpuReady = mpu6050.begin();
    if (mpuReady) {
      mpu6050.calibrate();
      Serial.println("MPU6050 READY - Z AXIS ONLY");
      serialLink.println(">> MPU6050 & SENSORS READY!");
    } else {
      Serial.println("MPU6050 INIT FAIL");
    }
  } else {
    Serial.println("MPU6050 NOT FOUND @ 0x68");
  }
}

void RobotNav::saveForwardSpeed() {
  Preferences p;
  if (!p.begin("mmparams", false)) return;
  p.putUChar("fspd", baseForwardSpeed);
  p.end();
}

bool RobotNav::initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address,
                        const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50);
  sensor.setTimeout(50);

  if (!sensor.init()) {
    Serial.print(name);
    Serial.println(" VL53 FAIL");
    return false;
  }

  sensor.setAddress(address);
  sensor.startContinuous();
  Serial.print(name);
  Serial.print(" VL53 OK @ 0x");
  Serial.println(address, HEX);
  return true;
}

void RobotNav::stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

void RobotNav::brakeMotors() {
  analogWrite(M1_IN1, 255);
  analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255);
  analogWrite(M2_IN2, 255);
  delay(brakeMs);
  stopMotors();
}

void RobotNav::turnRight(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }
  bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         rightCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
  Serial.println(res);
  serialLink.println(res);
}

void RobotNav::turnLeft(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }

  bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         leftCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
  Serial.println(res);
  serialLink.println(res);
}

void RobotNav::runTurnTest() {
  if (!mpuReady)
    return;

  serialLink.println("==========================================");
  serialLink.println(">> AUTO TEST: CHUAN BI QUAY TRAI 90 DO...");
  delay(800);
  turnLeft(90.0f);
  stopMotors();
  delay(1500);

  serialLink.println(">> AUTO TEST: CHUAN BI QUAY PHAI 90 DO VE HUONG CU...");
  delay(800);
  turnRight(90.0f);
  stopMotors();
  delay(1500);
  serialLink.println(">> AUTO TEST: HOAN TAT 1 CHU KY.");
}

void RobotNav::startPID() {
  autoTestMode = false;
  if (mpuReady) {
    mpu6050.setAutoBias(false); // đang chạy: không để tự bù trôi "nuốt" chuyển động quay thật của xe
    mpu6050.update();
    _targetYaw = mpu6050.getYaw();
  }
  _startEncLeft = getLeftEncoder();
  _startEncRight = getRightEncoder();
  _lastEncLeft = _startEncLeft;
  _lastEncRight = _startEncRight;
  _smoothError = 0.0f;
  _lastPIDLoopTime = micros();
  pidRunActive = true;
}

void RobotNav::stopPID() {
  if (mpuReady) mpu6050.setAutoBias(true);
  holdRun = false;
  pidRunActive = false;
  autoTestMode = false;
  stepCellActive = false;
  autoWallFollowActive = false;
  stopMotors();
}

void RobotNav::stepCell(int numCells) {
  if (numCells <= 0) return;
  startPID(); // Kích hoạt PID bám tường và giữ hướng thẳng
  stepCellActive = true;
  stepStartTime = millis();
  stepStartL = getLeftEncoder();
  stepStartR = getRightEncoder();
  stepStartPulses = (stepStartL + stepStartR) / 2;
  stepTargetPulses = (long)numCells * pulsesPerCell;
  stepTraveledPulses = 0;

  String msg = ">> [CELL] BAT DAU TIEN " + String(numCells) + " O (" +
               String(stepTargetPulses) + " xung)...";
  Serial.println(msg);
  serialLink.println(msg);
}

// Trả về khoảng cách (mm) hoặc 999 nếu lỗi; cập nhật raw = -1 khi lỗi/timeout, không chặn vòng lặp
static uint16_t pollSensor(VL53L0X &sensor, bool ready, int16_t &raw,
                           unsigned long &lastNew, unsigned long now) {
  if (!ready) { raw = -1; return 999; }  // -1: init thất bại
  if (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    lastNew = now;
    raw = (r > 1200) ? -3 : (r < 15 ? 15 : (int16_t)r);  // -3: ngoài tầm đo; quá gần thì kẹp về 15 (tường sát cảm biến)
  } else if (now - lastNew > 200) {
    raw = -2;  // lâu không có mẫu mới => timeout
  }
  return raw < 0 ? 999 : (uint16_t)raw;
}

void RobotNav::updateSensors() {
  unsigned long now = millis();
  // Giới hạn chu kỳ đọc cảm biến khoảng 35ms một lần (tương thích timing budget 33ms của VL53L0X)
  if (now - _lastSensorReadTime < 35) {
    return;
  }
  _lastSensorReadTime = now;

  // Đọc không chặn: chỉ lấy mẫu khi cảm biến đã có dữ liệu mới, ngược lại giữ giá trị cũ
  uint16_t raw_dL = pollSensor(sensorLeft, leftReady, _rawL, _tL, now);
  uint16_t raw_dF = pollSensor(sensorFront, frontReady, _rawF, _tF, now);
  uint16_t raw_dR = pollSensor(sensorRight, rightReady, _rawR, _tR, now);

  // Lọc EMA cho cảm biến trái
  if (raw_dL < 800) {
    smoothDL = (0.55f * (float)raw_dL) + (0.45f * smoothDL);
  } else {
    smoothDL = (0.2f * 999.0f) + (0.8f * smoothDL);
  }

  // Lọc EMA cho cảm biến phải
  if (raw_dR < 800) {
    smoothDR = (0.55f * (float)raw_dR) + (0.45f * smoothDR);
  } else {
    smoothDR = (0.2f * 999.0f) + (0.8f * smoothDR);
  }

  // Lọc EMA cho cảm biến trước (phản ứng nhanh để dừng kịp thời)
  if (raw_dF < 800) {
    smoothDF = (0.6f * (float)raw_dF) + (0.4f * smoothDF);
  } else {
    smoothDF = (float)raw_dF;
  }

}

// Trung vị 5 mẫu VL53 thô (mẫu lỗi/ngoài tầm tính là 999): chống gai nhiễu mà không trễ như EMA nặng
static int16_t median5(const int16_t *b) {
  int16_t t[5];
  memcpy(t, b, sizeof(t));
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 5; j++)
      if (t[j] < t[i]) { int16_t x = t[i]; t[i] = t[j]; t[j] = x; }
  return t[2];
}

void RobotNav::updateWallFilter() {
  if (_tL != _wLastTL) { _wLastTL = _tL; _wBufL[_wIdxL] = (_rawL > 0) ? _rawL : 999; _wIdxL = (_wIdxL + 1) % 5; _wFiltL = median5(_wBufL); }
  if (_tR != _wLastTR) { _wLastTR = _tR; _wBufR[_wIdxR] = (_rawR > 0) ? _rawR : 999; _wIdxR = (_wIdxR + 1) % 5; _wFiltR = median5(_wBufR); }
}

void RobotNav::updatePIDLoop() {
  if (!pidRunActive)
    return;

  _lastPIDLoopTime = micros();

  uint16_t dF = (uint16_t)smoothDF;

  updateWallFilter();
  logSample();

  // Chế độ chạy liên tục: gặp tường trước (<= frontStopDist) thì đứng yên, PID vẫn chạy; tường xa ra > +8mm thì đi tiếp
  if (holdRun) {
    if (hold.useFront && frontReady && dF > 20 && dF <= frontStopDist) {
      if (!_holdStopped) brakeMotors(); // hãm chủ động, tránh trôi quán tính qua tường
      _holdStopped = true;
      _holdFreeSince = 0;
    } else if (_holdStopped) {
      // Chỉ đi tiếp khi tường trước xa ra liên tục >= 500ms (chống gai đọc 999 làm xe lao vào tường)
      if (dF > frontStopDist + 8) {
        if (_holdFreeSince == 0) _holdFreeSince = millis();
        if (millis() - _holdFreeSince >= 500) _holdStopped = false;
      } else {
        _holdFreeSince = 0;
      }
    }
    if (_holdStopped) {
      stopMotors();
      if (mpuReady) _targetYaw = mpu6050.getYaw();
      return;
    }
  }

  // 1. Phanh dừng an toàn khi gặp vách tường trước
  if (!holdRun && frontReady && dF > 20 && dF <= frontStopDist) {
    brakeMotors();
    stopPID();
    String msg =
        ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }

  // 2. Kiểm tra cự ly chạy theo số ô Encoder (Cell Stepping)
  if (stepCellActive) {
    long curL = getLeftEncoder();
    long curR = getRightEncoder();
    long distL = labs(curL - stepStartL);
    long distR = labs(curR - stepStartR);
    long distTraveled = (distL + distR) / 2;
    stepTraveledPulses = distTraveled;

    // Đạt điều kiện dừng khi:
    // a) Quãng đường trung bình 2 bánh đạt đủ targetPulses (loại bỏ trường hợp 1 bánh quay nhanh khi bẻ lái làm dừng sớm)
    // b) HOẶC timeout an toàn 4.5 giây phòng ngừa xe chạy vô tận
    bool pulseReached = (distTraveled >= stepTargetPulses - stopLeadPulses); // phanh sớm stopLeadPulses để bù quãng trôi sau phanh
    bool timeoutSafe = (millis() - stepStartTime > 4500UL * (unsigned long)max(1L, stepTargetPulses / pulsesPerCell));

    if ((pulseReached || timeoutSafe) && stepNoStop) {
      // Hết quãng đường nhưng giữ nguyên PWM hiện tại để vào corner đang chạy (không phanh, không stopMotors)
      stepNoStop = false;
      stepCellActive = false;
      pidRunActive = false;
      holdRun = false;
      return;
    }
    if (pulseReached || timeoutSafe) {
      bool quiet = holdRun; // chế độ test: dừng xong không in gì, log in riêng
      brakeMotors();
      stopPID();
      if (quiet) return;
      String msg = ">> [CELL] DA HOAN THANH TIEN O! Xung: " + String(distTraveled) +
                   "/" + String(stepTargetPulses) + " xung (L:" + String(distL) +
                   ", R:" + String(distR) + "). Phanh dung.";
      if (timeoutSafe && !pulseReached) {
        msg = ">> [CELL] TIMEOUT AN TOAN 4.5S! Xung: " + String(distTraveled) +
              "/" + String(stepTargetPulses) + " xung. Phanh dung.";
      }
      Serial.println(msg);
      serialLink.println(msg);
      return;
    }
  }

  long curL = getLeftEncoder();
  long curR = getRightEncoder();
  _lastEncLeft = curL;
  _lastEncRight = curR;

  float pidOut = 0.0f;
  float yawCorr = 0.0f;
  float wallErr = 0.0f, wallCorr = 0.0f;

  {
    // ĐƯỜNG PID DUY NHẤT: Encoder P + giữ hướng gyro + Wall P (VL53 trái/phải)
    // Encoder: sai số = chênh quãng đường TÍCH LŨY 2 bánh từ lúc xuất phát
    // (|L| - |R| > 0: bánh trái đi trước -> giảm bánh trái, tăng bánh phải). P trên vị trí nên không bị mất do làm tròn
    float travL = (float)labs(curL - _startEncLeft);
    float travR = (float)labs(curR - _startEncRight);
    float encCorr = constrain(encKp * (travL - travR), -encMaxCorr, encMaxCorr);
    pidOut = encCorr;
    _lastCorr = encCorr;
    // Cộng thêm giữ hướng bằng gyro. yawTarget = yaw lúc bắt đầu lệnh (startPID).
    // Đo từ log thực tế: xe lệch PHẢI thì YAW TĂNG (+), tức yawError = target - yaw < 0.
    // pidOut > 0 => giảm bánh trái, tăng bánh phải => xe quay TRÁI. Nên yawCorr = -Kp * yawError (phản hồi âm)
    yawCorr = 0.0f;
    if (mpuReady && hold.yawKp > 0.0f) {
      float e = _targetYaw - mpu6050.getYaw();
      while (e > 180.0f) e -= 360.0f;
      while (e < -180.0f) e += 360.0f;
      _lastYawErr = e;
      yawCorr = constrain(-hold.yawKp * e, -yawMaxCorr, yawMaxCorr);
      pidOut += yawCorr;
    }
    // Wall correction (P, nhỏ): sai lệch theo baseline từng bên, không dùng trực tiếp L-R.
    // eL = L - baseL > 0: xa tường trái hơn mức giữa (xe lệch phải); eR = R - baseR > 0: xa tường phải (xe lệch trái)
    // wallErr > 0 -> cần quay TRÁI -> cùng chiều pidOut > 0
    wallErr = 0.0f;
    wallCorr = 0.0f;
    if (hold.wallEnable) {
      bool okL = leftReady && _wFiltL > 20.0f && _wFiltL < hold.wallValidMax;
      bool okR = rightReady && _wFiltR > 20.0f && _wFiltR < hold.wallValidMax;
      float eL = _wFiltL - hold.wallBaseL;
      float eR = _wFiltR - hold.wallBaseR;
      if (okL && okR) wallErr = 0.5f * (eL - eR);
      else if (okL)   wallErr = eL;
      else if (okR)   wallErr = -eR;
      wallErr = constrain(wallErr, -hold.wallErrClamp, hold.wallErrClamp);
      wallCorr = constrain(hold.wallKp * wallErr, -hold.wallMaxCorr, hold.wallMaxCorr);
      pidOut += wallCorr;
    }
  }

  // 4. Xuất PWM cho 2 bánh có giảm tốc 2 giai đoạn (Profile Deceleration):
  // - 0% -> 70% quãng đường ô: chạy tốc độ baseForwardSpeed tiêu chuẩn
  // - 70% -> 100% quãng đường ô: hãm về tốc độ bò (crawl speed ~46 PWM) để khi ngắt động cơ xe dừng dứt điểm, không trượt quán tính
  int activeBaseSpeed = baseForwardSpeed;
  if (!holdRun && stepCellActive && !stepNoStop && stepTraveledPulses >= (long)(0.70f * stepTargetPulses)) {
    activeBaseSpeed = 46;
  }

  _lastYawCorr = yawCorr;
  _lastWallErr = wallErr;
  _lastWallCorr = wallCorr;
  int leftSpeed = activeBaseSpeed - (int)lroundf(pidOut);
  int rightSpeed = activeBaseSpeed + (int)lroundf(pidOut);

  leftSpeed = constrain((int)(leftSpeed * motorScaleL), 35, 255);
  rightSpeed = constrain((int)(rightSpeed * motorScaleR), 35, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  analogWrite(M1_IN1, leftSpeed);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, rightSpeed);
  analogWrite(M2_IN2, 0);
}

void RobotNav::update() {
  updateSensors();

  if (cornerState != STRAIGHT) {
    if (mpuReady) mpu6050.update();
    updateCorner();
    return;
  }

  if (ffActive) {
    stepFloodFill();
    return;
  }

  if (autoWallFollowActive) {
    stepAutoWallFollow();
    return;
  }

  if (pidRunActive) {
    updatePIDLoop();
  }

  if (mpuReady) {
    mpu6050.update();
  }

  if (autoTestMode && mpuReady) {
    runTurnTest();
  }
}

// =========================================================================
// 4 HÀM CHUYỂN ĐỘNG NGUYÊN TỬ (ATOMIC MOTION - BƯỚC 3)
// =========================================================================

WallStatus RobotNav::senseCurrentWalls() {
  updateSensors();
  WallStatus walls;
  // Tường sát xe (smoothDF <= 180mm): vách trước của ô hiện tại
  walls.hasFrontNear = frontReady && smoothDF > 20.0f && smoothDF <= 180.0f;
  // Tường trước của ô tiếp theo (180mm < smoothDF <= frontWallDist): vách trước của Ô PHÍA TRƯỚC
  walls.hasFront = !walls.hasFrontNear && frontReady && smoothDF > 180.0f && smoothDF <= (float)frontWallDist;

  if (walls.hasFrontNear) {
    // KHI BỊ CHẮN TƯỜNG TRƯỚC SÁT XE:
    // Cảm biến 45° chĩa ra trước sẽ phản xạ từ vách trước ở cự ly ~ smoothDF * 1.414.
    // Cần loại trừ phản xạ này để không nhận diện nhầm cửa mở/khoảng trống thành tường!
    float frontDiag = smoothDF * 1.414f;
    bool leftIsFrontReflection = (fabs(smoothDL - frontDiag) < 32.0f && smoothDL > 125.0f);
    bool rightIsFrontReflection = (fabs(smoothDR - frontDiag) < 32.0f && smoothDR > 125.0f);

    walls.hasLeft  = leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold && !leftIsFrontReflection;
    walls.hasRight = rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold && !rightIsFrontReflection;
  } else {
    // 2 Cảm biến 45° chĩa ra trước nhìn 2 bên sườn ô phía trước:
    walls.hasLeft  = (leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold);
    walls.hasRight = (rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold);
  }
  return walls;
}

void RobotNav::reportCell(char act, const WallStatus &w, const WallStatus *pre, bool blocked) {
  // Gói JSON cho bản đồ trên Dashboard: act = F/L/R/B (đã di chuyển) hoặc S (chỉ đọc vách)
  // Cảm biến nhìn trước 1 ô: wf/wl/wr = vách của Ô PHÍA TRƯỚC xe;
  // pf/pl/pr (nếu có) = vách của chính ô vừa bước vào
  String j = "{\"type\":\"cell\",\"act\":\"" + String(act) + "\"" +
             ",\"wf\":" + String(w.hasFront ? 1 : 0) +
             ",\"wl\":" + String(w.hasLeft ? 1 : 0) +
             ",\"wr\":" + String(w.hasRight ? 1 : 0) +
             ",\"wn\":" + String(w.hasFrontNear ? 1 : 0);
  if (pre) {
    j += ",\"pf\":" + String(pre->hasFront ? 1 : 0) +
         ",\"pl\":" + String(pre->hasLeft ? 1 : 0) +
         ",\"pr\":" + String(pre->hasRight ? 1 : 0);
  }
  if (blocked) j += ",\"blk\":1"; // chỉ xoay, không tiến ô
  j += "}";
  serialLink.println(j);
}

WallStatus RobotNav::moveOneCell(char turn) {
  String startMsg = ">> [ATOMIC] BAT DAU TIEN 1 O (" + String(pulsesPerCell) + " xung)...";
  Serial.println(startMsg);
  serialLink.println(startMsg);

  preWalls = senseCurrentWalls(); // cảm biến nhìn trước 1 ô: đây là vách của ô sắp bước vào
  startCellFresh = false;
  bool keepAuto = autoWallFollowActive; // lưu trạng thái tự động trước khi PID tự dừng cuối ô

  stepCell(1); // Kích hoạt PID bám tường và đặt mục tiêu đúng 1 ô (pulsesPerCell xung)

  unsigned long startTime = millis();

  // Thu thập mẫu vách hông liên tục trong cửa sổ [28% .. 75%] của ô
  int leftOpenCount = 0;
  int leftWallCount = 0;
  int rightOpenCount = 0;
  int rightWallCount = 0;

  while (stepCellActive && (millis() - startTime < 6000)) {
    updateSensors();
    if (mpuReady) {
      mpu6050.update();
    }

    // Cửa sổ nhận diện vách hông [28% .. 75%]:
    // - Đã vượt qua trụ góc và vách của ô cũ (>28%)
    // - Chưa bị ảnh hưởng bởi phản xạ tường trước (<75%)
    if (stepTraveledPulses >= (long)(sideSampleFrac * pulsesPerCell) &&
        stepTraveledPulses <= (long)(0.75f * pulsesPerCell)) {
      if (leftReady) {
        if (smoothDL >= (float)wallThreshold) {
          leftOpenCount++;
        } else if (smoothDL > 20.0f && smoothDL < (float)wallThreshold) {
          leftWallCount++;
        }
      }
      if (rightReady) {
        if (smoothDR >= (float)rightWallThreshold) {
          rightOpenCount++;
        } else if (smoothDR > 20.0f && smoothDR < (float)rightWallThreshold) {
          rightWallCount++;
        }
      }
    }

    // updatePIDLoop() tự động kiểm tra:
    // a) Chạm vách trước (dF <= frontStopDist)
    // b) Đạt đủ pulsesPerCell (pulsesPerCell xung)
    // và sẽ tự phanh dừng, tắt stepCellActive!
    updatePIDLoop();
    delay(2);
  }

  // Đảm bảo xe đã phanh dừng hoàn toàn dứt điểm tại tâm ô
  brakeMotors();
  stopPID();
  autoWallFollowActive = keepAuto; // stopPID() (kể cả trong updatePIDLoop) tắt cờ tự động -> khôi phục

  // Đã dừng hẳn ở ô mới: chờ ~250ms cho cảm biến đọc lại vài lần (bộ lọc EMA ổn định)
  // rồi mới đọc vách -> cập nhật lại tường của ô vừa tới bằng số đo lúc đứng yên.
  unsigned long settle = millis();
  while (millis() - settle < 250) {
    updateSensors();
    if (mpuReady) mpu6050.update();
    delay(5);
  }

  // Đã dừng ở tâm ô mới -> đọc vách của ô kế tiếp phía trước (look-ahead)
  WallStatus status = senseCurrentWalls();

  // Vách của chính ô hiện tại:
  // Ưu tiên kết quả thống kê liên tục trong cửa sổ [28% .. 75%] lúc xe đang lướt qua thân ô
  WallStatus curCellWalls;
  if (leftOpenCount + leftWallCount >= 2) {
    // Có khoảng trống nếu số lần đọc cửa mở >= 2 hoặc tỷ lệ mở >= tường
    curCellWalls.hasLeft = (leftOpenCount >= 2 && leftOpenCount >= leftWallCount) ? false : (leftWallCount > leftOpenCount);
  } else {
    curCellWalls.hasLeft = status.hasLeft;
  }

  if (rightOpenCount + rightWallCount >= 2) {
    curCellWalls.hasRight = (rightOpenCount >= 2 && rightOpenCount >= rightWallCount) ? false : (rightWallCount > rightOpenCount);
  } else {
    curCellWalls.hasRight = status.hasRight;
  }

  curCellWalls.hasFront = status.hasFrontNear;
  curCellWalls.hasFrontNear = status.hasFrontNear;
  curWalls = curCellWalls;

  // Bị tường chặn khi chưa đi được nửa ô -> xe vẫn ở ô cũ (thường do đọc nhầm cửa mở)
  lastMoveBlocked = stepTraveledPulses < pulsesPerCell / 2;
  if (lastMoveBlocked) {
    status.hasFrontNear = true;
    status.hasFront = false;
    serialLink.println(">> [ATOMIC] BI TUONG CHAN! Xe van o o cu (" + String(stepTraveledPulses) + " xung)");
  }

  String resMsg = ">> [ATOMIC] DA DEN TAM O MOI! O ke tiep: Truoc=" + String(status.hasFront ? "CO" : "TRONG") +
                  " | Trai=" + String(status.hasLeft ? "CO" : "TRONG") +
                  " | Phai=" + String(status.hasRight ? "CO" : "TRONG") +
                  " | Vach O nay: Trai=" + String(curCellWalls.hasLeft ? "CO" : "TRONG") +
                  " (Open:" + String(leftOpenCount) + "/Wall:" + String(leftWallCount) + ")" +
                  ", Phai=" + String(curCellWalls.hasRight ? "CO" : "TRONG") +
                  " (Open:" + String(rightOpenCount) + "/Wall:" + String(rightWallCount) + ")" +
                  ", Truoc=" + String(curCellWalls.hasFront ? "CO" : "TRONG") +
                  " | Xung: " + String(stepTraveledPulses);
  Serial.println(resMsg);
  serialLink.println(resMsg);
  reportCell(turn, status, lastMoveBlocked ? nullptr : &curCellWalls, lastMoveBlocked);

  return status;
}

WallStatus RobotNav::turnLeftAndStep() {
  serialLink.println(">> [ATOMIC] LENH: RE TRAI 90 DO & TIEN 1 O");
  turnLeft(90.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('L');
}

WallStatus RobotNav::turnRightAndStep() {
  serialLink.println(">> [ATOMIC] LENH: RE PHAI 90 DO & TIEN 1 O");
  turnRight(90.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('R');
}

WallStatus RobotNav::turnAroundAndStep() {
  serialLink.println(">> [ATOMIC] LENH: QUAY DAU 180 DO & TIEN 1 O");
  turnRight(180.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('B');
}

// =========================================================================
// CHẾ ĐỘ 3: BÁM TƯỜNG TỰ ĐỘNG (AUTONOMOUS WALL FOLLOWER)
// =========================================================================

void RobotNav::startAutoWallFollow() {
  autoTestMode = false;
  pidRunActive = false;
  stepCellActive = false;
  autoCellCount = 0;
  curWalls = senseCurrentWalls();
  if (startCellFresh) { // ô xuất phát luôn có tường hai bên
    curWalls.hasLeft = true;
    curWalls.hasRight = true;
  }
  lastAutoDecision = "KÍCH HOẠT";
  autoWallFollowActive = true;

  String msg = ">> [CHẾ ĐỘ 3] BẮT ĐẦU BÁM TƯỜNG TỰ ĐỘNG (" +
               String(followRightHand ? "LUẬT TAY PHẢI" : "LUẬT TAY TRÁI") +
               ") - GIỚI HẠN AN TOÀN " + String(autoMaxCells) + " Ô";
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stopAutoWallFollow() {
  autoWallFollowActive = false;
  cornerState = STRAIGHT;
  stopPID();
  brakeMotors();
  lastAutoDecision = "ĐÃ DỪNG";
  String msg = ">> [CHẾ ĐỘ 3] ĐÃ DỪNG TỰ ĐỘNG! Tổng số ô đã chạy: " + String(autoCellCount);
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stepAutoWallFollow() {
  if (!autoWallFollowActive) return;

  // 1. Kiểm tra an toàn giới hạn số ô để phòng chạy vòng tròn lặp vô tận
  if (autoCellCount >= autoMaxCells) {
    stopAutoWallFollow();
    String limMsg = ">> [CHẾ ĐỘ 3] ĐẠT MỐC AN TOÀN " + String(autoMaxCells) + " Ô! Tự động phanh dừng.";
    Serial.println(limMsg);
    serialLink.println(limMsg);
    return;
  }

  // 2. Vách ô hiện tại: hông = mẫu đã lưu khi vào ô, trước = đọc trực tiếp
  WallStatus walls = curWalls;
  walls.hasFront = senseCurrentWalls().hasFrontNear; // chỉ bị chắn khi tường sát xe

  // Chế độ đi thẳng: gặp tường trước thì quay đầu, còn lại đi thẳng
  if (patrolMode) {
    if (walls.hasFront) {
      lastAutoDecision = "GẶP TƯỜNG: QUAY ĐẦU 180°";
      turnAroundAndStep();
    } else {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    }
    autoCellCount++;
    delay(80);
    return;
  }

  // 3. Ra quyết định điều hướng tự động
  if (followRightHand) {
    // === QUY TẮC BÀN TAY PHẢI (RIGHT-HAND RULE) ===
    // Ưu tiên: 1. Rẽ Phải -> 2. Đi Thẳng -> 3. Rẽ Trái -> 4. Quay Đầu 180°
    if (!walls.hasRight) {
      lastAutoDecision = "RẼ PHẢI & TIẾN 1 Ô";
      turnRightAndStep();
    } else if (!walls.hasFront) {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    } else if (!walls.hasLeft) {
      lastAutoDecision = "RẼ TRÁI & TIẾN 1 Ô";
      turnLeftAndStep();
    } else {
      lastAutoDecision = "ĐƯỜNG CỤT: QUAY ĐẦU 180°";
      turnAroundAndStep();
    }
  } else {
    // === QUY TẮC BÀN TAY TRÁI (LEFT-HAND RULE) ===
    // Ưu tiên: 1. Rẽ Trái -> 2. Đi Thẳng -> 3. Rẽ Phải -> 4. Quay Đầu 180°
    if (!walls.hasLeft) {
      lastAutoDecision = "RẼ TRÁI & TIẾN 1 Ô";
      turnLeftAndStep();
    } else if (!walls.hasFront) {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    } else if (!walls.hasRight) {
      lastAutoDecision = "RẼ PHẢI & TIẾN 1 Ô";
      turnRightAndStep();
    } else {
      lastAutoDecision = "ĐƯỜNG CỤT: QUAY ĐẦU 180°";
      turnAroundAndStep();
    }
  }

  autoCellCount++;
  String statusMsg = ">> [CHẾ ĐỘ 3] Ô #" + String(autoCellCount) + ": " + lastAutoDecision;
  Serial.println(statusMsg);
  serialLink.println(statusMsg);

  // Cho xe nghỉ ổn định 80ms tại tâm ô trước khi sang ô tiếp theo
  delay(80);
}

// =========================================================================
// CHẾ ĐỘ 4: FLOOD-FILL (dùng maze_algorithm.h)
// =========================================================================

// sure = true: chắc chắn (xe vừa đi qua cửa này) -> ghi đè.
// sure = false: đọc cảm biến -> chỉ được THÊM tường, không xoá tường đã thấy
// (tránh đọc nhiễu 1 lần làm "phá tường" rồi xe lao vào tường).
static void ffSetWall(ParentMaze &m, int x, int y, int dir, bool wall, bool sure = false) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  int nx = x + DX[dir], ny = y + DY[dir];
  if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE)
    return; // tường biên luôn có sẵn
  Cell &c = m.cell(x, y);
  bool *self[4] = {&c.north_wall, &c.east_wall, &c.south_wall, &c.west_wall};
  if (!wall && !sure && *self[dir]) return;
  *self[dir] = wall;
  Cell &n = m.cell(nx, ny);
  bool *other[4] = {&n.north_wall, &n.east_wall, &n.south_wall, &n.west_wall};
  *other[(dir + 2) % 4] = wall; // đồng bộ tường với ô kề
}

// Cảm biến nhìn trước 1 ô: ws (trước/trái/phải) là vách của ô PHÍA TRƯỚC (x,y) theo hướng h
static void ffRecordAhead(ParentMaze &m, int x, int y, int h, const WallStatus &ws) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  ffSetWall(m, x, y, h, ws.hasFrontNear, true); // vách trước của ô đang đứng: đo lúc đứng yên -> ghi đè
  if (ws.hasFrontNear) return;            // bị chắn: không nhìn được ô phía trước
  int ax = x + DX[h], ay = y + DY[h];
  if (ax < 0 || ax >= MAZE_SIZE || ay < 0 || ay >= MAZE_SIZE) return;
  ffSetWall(m, ax, ay, (h + 3) % 4, ws.hasLeft);
  ffSetWall(m, ax, ay, (h + 1) % 4, ws.hasRight);
  ffSetWall(m, ax, ay, h, ws.hasFront);
  m.cell(ax, ay).known = true;
}

void RobotNav::resetFloodFill(int x, int y, int h) {
  ffMaze.clear_mem();
  startCellFresh = true; // ô xuất phát (bất kể toạ độ) luôn có tường trái, phải, sau theo hướng ban đầu
  x = constrain(x, 0, MAZE_SIZE - 1);
  y = constrain(y, 0, MAZE_SIZE - 1);
  ffX = x;
  ffY = y;
  ffH = ((h % 4) + 4) % 4;
  ffStart = {(int8_t)x, (int8_t)y};
  // Tường bao quanh mê cung thật ffCols x ffRows
  for (int i = 0; i < ffRows; i++) ffSetWall(ffMaze, ffCols - 1, i, 1, true, true);
  for (int i = 0; i < ffCols; i++) ffSetWall(ffMaze, i, ffRows - 1, 0, true, true);
}

void RobotNav::startFloodFill(uint8_t mode) {
  autoTestMode = false;
  pidRunActive = false;
  stepCellActive = false;
  autoWallFollowActive = false;
  autoCellCount = 0;
  ffMode = mode;

  if (!ffMaze.cell(ffX, ffY).run_visited) {
    WallStatus ws = senseCurrentWalls(); // vách của ô phía trước
    if (startCellFresh) { // ô xuất phát: mặc định có tường trái, phải và SAU (theo hướng ban đầu ffH)
      ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, true);
      ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, true);
      ffSetWall(ffMaze, ffX, ffY, (ffH + 2) % 4, true);
      ffMaze.cell(ffX, ffY).known = true;
    }
    ffMaze.cell(ffX, ffY).run_visited = true;
    ffRecordAhead(ffMaze, ffX, ffY, ffH, ws);
    reportCell('S', ws);
  }

  lastAutoDecision = "FF KÍCH HOẠT";
  ffActive = true;
  String msg = ">> [CHẾ ĐỘ 4] FLOOD-FILL " +
               String(mode == 0 ? "TỚI ĐÍCH" : "KHÁM PHÁ TOÀN BỘ") + " | Xe (" +
               String(ffX) + "," + String(ffY) + ") hướng " + String(ffH) +
               " | Đích (" + String(ffGoal.x) + "," + String(ffGoal.y) + ") " +
               String(ffGoalSize) + "x" + String(ffGoalSize);
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stopFloodFill(const char *reason) {
  ffActive = false;
  stopPID();
  brakeMotors();
  lastAutoDecision = reason;
  String msg = ">> [CHẾ ĐỘ 4] " + String(reason) + " | Số ô đã đi: " + String(autoCellCount);
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stepFloodFill() {
  if (!ffActive) return;

  if (autoCellCount >= autoMaxCells) {
    stopFloodFill("DỪNG AN TOÀN (ĐỦ SỐ Ô TỐI ĐA)");
    return;
  }

  Point cur = {ffX, ffY};
  Point target = ffGoal;
  Point next = cur;

  if (ffMode == 1) {
    // KHÁM PHÁ HẾT: ưu tiên ô kề CHƯA KHÁM PHÁ theo thứ tự thẳng -> trái -> phải
    static const int DX[4] = {0, 1, 0, -1};
    static const int DY[4] = {1, 0, -1, 0};
    static const int PRIORITY[3] = {0, 3, 1}; // lệch so với hướng xe: thẳng, trái, phải
    Cell &c = ffMaze.cell(cur.x, cur.y);
    bool wall[4] = {c.north_wall, c.east_wall, c.south_wall, c.west_wall};
    bool picked = false;
    for (int k = 0; k < 3 && !picked; k++) {
      int d = (ffH + PRIORITY[k]) % 4;
      int nx = cur.x + DX[d], ny = cur.y + DY[d];
      if (wall[d] || nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (ffMaze.cell(nx, ny).run_visited || ffMaze.cell(nx, ny).known) continue; // đã biết đủ vách
      next = {(int8_t)nx, (int8_t)ny};
      picked = true;
    }
    if (!picked) {
      // Hết ô kề mới: quay lại (kể cả quay đầu) tới ô chưa đi gần nhất
      Point n = ffMaze.find_nearest_unvisited(cur);
      if (n.x < 0) {
        stopFloodFill("ĐÃ ĐI HẾT TẤT CẢ CÁC Ô");
        return;
      }
      target = n;
    } else {
      target = next;
    }
  }

  bool arrived = (ffMode == 0) ? ffInGoal(cur.x, cur.y)
                                : (cur.x == target.x && cur.y == target.y);
  if (arrived) {
    stopFloodFill("ĐÃ TỚI ĐÍCH");
    return;
  }

  if (ffMode == 0 || (next.x == cur.x && next.y == cur.y)) {
    ffMaze.floodfill_update(target.x, target.y, false, false, ffMode == 0 ? ffGoalSize : 1);
    next = ffMaze.get_next_move(cur.x, cur.y);
    if ((next.x == cur.x && next.y == cur.y) || ffMaze.cell(cur.x, cur.y).step >= 65535) {
      stopFloodFill("KHÔNG CÒN ĐƯỜNG ĐI");
      return;
    }
  }

  int dir;
  if (next.y > cur.y) dir = 0;
  else if (next.x > cur.x) dir = 1;
  else if (next.y < cur.y) dir = 2;
  else dir = 3;

  int diff = (dir - ffH + 4) % 4;
  WallStatus ws;
  if (diff == 0) {
    lastAutoDecision = "TIẾN THẲNG 1 Ô";
    ws = moveOneCell('F');
  } else if (diff == 1) {
    lastAutoDecision = "RẼ PHẢI & TIẾN 1 Ô";
    ws = turnRightAndStep();
  } else if (diff == 3) {
    lastAutoDecision = "RẼ TRÁI & TIẾN 1 Ô";
    ws = turnLeftAndStep();
  } else {
    lastAutoDecision = "QUAY ĐẦU 180°";
    ws = turnAroundAndStep();
  }

  ffH = dir;
  if (lastMoveBlocked) {
    // Không đi được: xe vẫn ở ô cũ, hướng mới -> ghi chắc chắn tường trước rồi tính lại đường
    ffSetWall(ffMaze, ffX, ffY, ffH, true, true);
    String m = ">> [CHẾ ĐỘ 4] Bị tường chặn tại (" + String(ffX) + "," + String(ffY) + "), tính lại đường";
    Serial.println(m);
    serialLink.println(m);
    delay(80);
    return;
  }
  ffX = next.x;
  ffY = next.y;
  // Cửa xe vừa bước qua luôn thông
  ffSetWall(ffMaze, ffX, ffY, (ffH + 2) % 4, false, true);
  // Đã tới ô này: CẬP NHẬT LẠI vách hông bằng số đo khi đi qua thân ô (ghi đè kết quả
  // nhìn trước 1 ô lúc còn ở ô cũ), không còn chỉ ghi khi ô chưa biết.
  ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, curWalls.hasLeft, true);
  ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, curWalls.hasRight, true);
  ffMaze.cell(ffX, ffY).known = true;
  ffMaze.cell(ffX, ffY).run_visited = true;
  ffRecordAhead(ffMaze, ffX, ffY, ffH, ws);
  autoCellCount++;

  String m = ">> [CHẾ ĐỘ 4] Ô #" + String(autoCellCount) + " -> (" + String(ffX) + "," +
             String(ffY) + ") " + lastAutoDecision;
  Serial.println(m);
  serialLink.println(m);
  delay(80);
}

// ================= GIỮ VỊ TRÍ TRONG Ô =================

void RobotNav::startHold(bool keepLog) {
  // Chạy thẳng bằng đường PID duy nhất của updatePIDLoop: Encoder P + giữ hướng gyro + Wall P (VL53 trái/phải)
  stepCellActive = false;
  autoWallFollowActive = false;
  holdRun = true;
  _holdStopped = false;
  if (!keepLog) clearLog();
  resetEnc();
  frontStopDist = (uint16_t)hold.targetF;
  stepCell(hold.cells); // test: hold.cells ô x pulsesPerCell xung, PID liên tục rồi dừng
}

// ================= LOG CHẠY =================

void RobotNav::logSample() {
  unsigned long now = millis();
  if (now - _logLastT < 33) return; // ~30Hz
  float dt = (now - _logLastT) / 1000.0f;
  _logLastT = now;
  long l = getLeftEncoder(), r = getRightEncoder();
  LogRec &e = _log[_logHead];
  e.t = now;
  e.eL = l;
  e.eR = r;
  e.dL = _rawL; // VL53 thô (mm), -1 = lỗi/ngoài tầm
  e.dF = _rawF;
  e.dR = _rawR;
  e.vL = (_logCount == 0 || dt > 1.0f) ? 0 : (int16_t)((l - _logLastL) / dt);
  e.vR = (_logCount == 0 || dt > 1.0f) ? 0 : (int16_t)((r - _logLastR) / dt);
  e.pL = currentLeftSpeed;
  e.pR = currentRightSpeed;
  e.c = (int16_t)lroundf(_lastCorr);
  e.yaw = mpuReady ? mpu6050.getYaw() : 0.0f;
  e.yawT = _targetYaw;
  e.yawE = _lastYawErr;
  e.yawC = _lastYawCorr;
  e.wallE = _lastWallErr;
  e.wallC = _lastWallCorr;
  e.fL = (int16_t)min(_wFiltL, 999.0f);
  e.fR = (int16_t)min(_wFiltR, 999.0f);
  _logLastL = l;
  _logLastR = r;
  _logHead = (_logHead + 1) % LOG_SIZE;
  if (_logCount < LOG_SIZE) _logCount++;
}

void RobotNav::printLog() {
  Serial.println("==== LOG: TIME,ENC_L,ENC_R,YAW,VL53_L,VL53_F,VL53_R,VL53_L_MED,VL53_R_MED,WALL_ERROR,WALL_CORRECTION,SPEED_L,SPEED_R,ERROR,PID_CORRECTION,YAW_TARGET,YAW_ERROR,YAW_CORRECTION,PWM_L,PWM_R ====");
  uint16_t start = (_logCount < LOG_SIZE) ? 0 : _logHead;
  for (uint16_t i = 0; i < _logCount; i++) {
    const LogRec &e = _log[(start + i) % LOG_SIZE];
    Serial.printf("%lu,%ld,%ld,%.2f,%d,%d,%d,%d,%d,%.1f,%.2f,%d,%d,%ld,%d,%.2f,%.2f,%.2f,%d,%d\n", (unsigned long)e.t, (long)e.eL, (long)e.eR, e.yaw, e.dL, e.dF, e.dR, e.fL, e.fR, e.wallE, e.wallC, e.vL, e.vR, (long)(e.eL - e.eR), e.c, e.yawT, e.yawE, e.yawC, e.pL, e.pR);
  }
  Serial.printf("==== HET LOG (%u mau) ====\n", _logCount);
}

// ================= TEST QUAY TRÁI TẠI CHỖ (MPU6050 feedback) =================

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool RobotNav::turnLeft90Test(float angle) {
  if (!mpuReady) return false;
  stopPID();
  mpu6050.setAutoBias(false);
  mpu6050.update();
  // Target cộng dồn từ target lần trước (không lấy từ yaw đo được) -> sai số dư của lần quay trước được sửa ở lần sau,
  // không cộng dồn. Nếu xe bị xoay tay/lệch > 20° so với target cũ thì lấy lại từ yaw hiện tại.
  float base = mpu6050.getYaw();
  if (_turnHeadingValid && fabsf(_turnHeading - base) < 20.0f) base = _turnHeading;
  // Target KHÔNG chuẩn hóa: yaw của MPU là góc tích lũy (không quấn vòng) nên sai số = target - yaw luôn đúng chiều,
  // kể cả quay 180° (chuẩn hóa về [-180,180] sẽ làm xe quay ngược chiều khi target rơi sát ±180)
  float target = base + turnLeftYawSign * angle; // angle > 0: quay TRÁI, angle < 0: quay PHẢI
  _turnHeading = target;
  _turnHeadingValid = true;
  float prevYaw = mpu6050.getYaw();
  unsigned long prevT = millis();
  _turnLogN = 0;
  _turnLogPrinted = false;

  unsigned long t0 = millis(), lastLog = 0;
  bool done = false;
  while (!done && millis() - t0 < 6000) {
    mpu6050.update();
    float yaw = mpu6050.getYaw();
    float err = target - yaw;  // err > 0: yaw cần tăng
    int pl = 0, pr = 0;
    // Tốc độ quay (độ/s) để dự đoán quãng trôi sau khi phanh
    unsigned long nowT = millis();
    float rate = (nowT > prevT) ? (yaw - prevYaw) * 1000.0f / (float)(nowT - prevT) : 0.0f;
    prevYaw = yaw;
    prevT = nowT;
    // Phanh sớm: sai số còn lại <= dung sai + (tốc độ * turnBrakeLead). Quãng trôi sau phanh ~ rate * lead
    float lead = fabsf(rate) * turnBrakeLead;
    bool approaching = (rate * err) > 0.0f;  // đang quay về phía target

    if (fabsf(err) <= turnTolerance || (approaching && fabsf(err) <= turnTolerance + lead)) {
      // Tới target: phanh ngắn mạch. PHẢI cập nhật gyro liên tục trong lúc phanh (không delay mù),
      // nếu không phần góc xe còn quay khi phanh không được tích phân -> yaw báo thiếu, xe quay quá
      analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
      analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
      unsigned long s = millis();
      while (millis() - s < 80) { mpu6050.update(); delay(1); }
      stopMotors();
      s = millis();
      while (millis() - s < 150) { mpu6050.update(); delay(1); }
      yaw = mpu6050.getYaw();
      err = target - yaw;
      prevYaw = yaw;
      prevT = millis();
      if (fabsf(err) <= turnTolerance) done = true;
    } else {
      // Giảm tốc tuyến tính trong turnDecelZone cuối
      float k = constrain(fabsf(err) / turnDecelZone, 0.0f, 1.0f);
      int pwm = (int)lroundf(turnMinPwm + (turnMaxPwm - turnMinPwm) * k);
      // Cần quay theo chiều làm yaw thay đổi cùng dấu với err.
      // turnLeftYawSign * err > 0 => cần quay TRÁI (bánh trái lùi, bánh phải tiến), ngược lại quay PHẢI
      bool goLeft = (turnLeftYawSign * err) > 0.0f;
      if (goLeft) { pl = -pwm; pr = pwm; } else { pl = pwm; pr = -pwm; }
      analogWrite(M1_IN1, pl > 0 ? pl : 0); analogWrite(M1_IN2, pl < 0 ? -pl : 0);
      analogWrite(M2_IN1, pr > 0 ? pr : 0); analogWrite(M2_IN2, pr < 0 ? -pr : 0);
    }

    if (millis() - lastLog >= 20 && _turnLogN < TURN_LOG_SIZE) {
      lastLog = millis();
      _turnLog[_turnLogN++] = {(uint32_t)millis(), yaw, wrap180(target), err, (int16_t)pl, (int16_t)pr};
    }
    delay(2);
  }

  stopMotors();  // motor = 0
  mpu6050.setAutoBias(true);
  currentLeftSpeed = currentRightSpeed = 0;
  if (_turnLogN < TURN_LOG_SIZE) {
    mpu6050.update();
    float yaw = mpu6050.getYaw();
    _turnLog[_turnLogN++] = {(uint32_t)millis(), yaw, wrap180(target), target - yaw, 0, 0};
  }
  return done;
}

void RobotNav::printTurnLog() {
  Serial.println("==== TURN/CORNER LOG: TIME,STATE,YAW,TARGET,ERROR,PWM_L,PWM_R (PWM am = lui) ====");
  for (uint16_t i = 0; i < _turnLogN; i++) {
    const TurnRec &e = _turnLog[i];
    static const char *ST[] = {"TURN", "CORNER", "STRAIGHT"};
    Serial.printf("%lu,%s,%.2f,%.2f,%.2f,%d,%d\n", (unsigned long)e.t, ST[e.st > 2 ? 0 : e.st], e.yaw, e.target, e.err, e.pl, e.pr);
  }
  Serial.printf("==== HET (%u mau) ====\n", _turnLogN);
  _turnLogPrinted = true;
}

// ================= MOTION PRIMITIVE: CORNER 90° (cua khi đang chạy) =================

static void driveFwd(int l, int r) {
  analogWrite(M1_IN1, constrain(l, 0, 255)); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, constrain(r, 0, 255)); analogWrite(M2_IN2, 0);
}

// PWM 2 bánh khi cua: bánh ngoài = outer, bánh trong giảm theo độ cong (0..1, tỉ lệ sai số trong vùng giảm tốc)
void RobotNav::cornerPwm(float err, float errPred, float outer, int &pl, int &pr) const {
  float u = constrain(fabsf(errPred) / cornerDecelZone, cornerMinCurve, 1.0f);
  float inner = outer * (1.0f - (1.0f - cornerInnerRatio) * u);
  bool goLeft = (turnLeftYawSign * err) > 0.0f;  // cần làm yaw đổi theo chiều quay TRÁI
  pl = (int)lroundf((goLeft ? inner : outer) * motorScaleL);
  pr = (int)lroundf((goLeft ? outer : inner) * motorScaleR);
}

// ================= CORNER CONTROLLER (state machine không chặn) =================

bool RobotNav::startCornerLeft()  { return startCorner(CORNER_LEFT); }
bool RobotNav::startCornerRight() { return startCorner(CORNER_RIGHT); }

bool RobotNav::startCorner(CornerState dir) {
  if (!mpuReady || cornerState != STRAIGHT) return false;
  mpu6050.setAutoBias(false);
  mpu6050.update();
  cornerStartYaw = mpu6050.getYaw();
  // Trái: yaw đổi theo turnLeftYawSign (mặc định -1 => startYaw - 90). Phải: ngược lại
  float delta = (dir == CORNER_LEFT ? 1.0f : -1.0f) * turnLeftYawSign * 90.0f;
  _cornerTarget = cornerStartYaw + delta;
  cornerTargetYaw = wrap180(_cornerTarget);
  _cornerV0 = (currentLeftSpeed + currentRightSpeed) * 0.5f; // tốc độ lúc vào cua, giảm dần về cornerSpeed
  if (_cornerV0 < cornerSpeed) _cornerV0 = cornerSpeed;
  _cornerT0 = _cornerStateT = _cornerPrevT = millis();
  _cornerPrevYaw = cornerStartYaw;
  cornerState = dir;
  Serial.printf(">> [CORNERSM] START %s | yaw=%.2f target=%.2f\n",
                dir == CORNER_LEFT ? "TRAI" : "PHAI", cornerStartYaw, cornerTargetYaw);
  return true;
}

void RobotNav::updateCorner() {
  if (cornerState == STRAIGHT) return;
  if (!mpuReady) { finishCorner(); return; }
  unsigned long nowT = millis();
  float yaw = mpu6050.getYaw();
  int pl, pr;

  if (cornerState == EXIT_CORNER) {
    // Hai bánh về lại tốc độ chạy thẳng, không còn lệch trái/phải
    float t = cornerExitMs ? constrain((nowT - _cornerStateT) / (float)cornerExitMs, 0.0f, 1.0f) : 1.0f;
    float v = cornerSpeed + ((float)baseForwardSpeed - cornerSpeed) * t;
    pl = (int)lroundf(v * motorScaleL);
    pr = (int)lroundf(v * motorScaleR);
    driveFwd(pl, pr);
    currentLeftSpeed = pl;
    currentRightSpeed = pr;
    if (t >= 1.0f) finishCorner();
    return;
  }

  float err = wrap180(_cornerTarget - yaw);
  float rate = (nowT > _cornerPrevT) ? wrap180(yaw - _cornerPrevYaw) * 1000.0f / (float)(nowT - _cornerPrevT) : 0.0f;
  _cornerPrevYaw = yaw;
  _cornerPrevT = nowT;
  float errPred = err - rate * cornerLead;
  bool timeout = nowT - _cornerT0 > cornerTimeoutMs;
  if (fabsf(err) <= cornerTolerance || errPred * err <= 0.0f || timeout) {
    Serial.printf(">> [CORNERSM] %s | yaw=%.2f target=%.2f err=%.2f -> EXIT\n",
                  timeout ? "TIMEOUT" : "REACHED", yaw, cornerTargetYaw, err);
    cornerState = EXIT_CORNER;
    _cornerStateT = nowT;
    return;
  }

  // Vào cua: bánh ngoài giảm tốc từ tốc độ lúc vào về cornerSpeed, rồi cua với độ cong theo sai số yaw
  float t = cornerEntryMs ? constrain((nowT - _cornerT0) / (float)cornerEntryMs, 0.0f, 1.0f) : 1.0f;
  float outer = _cornerV0 + (cornerSpeed - _cornerV0) * t;
  cornerPwm(err, errPred, outer, pl, pr);
  driveFwd(pl, pr);
  currentLeftSpeed = pl;
  currentRightSpeed = pr;
}

void RobotNav::finishCorner() {
  cornerState = STRAIGHT;
  // Giữ hướng mới cho đoạn thẳng kế tiếp, xóa trạng thái PID cũ để không giật
  _targetYaw = _cornerTarget;
  _turnHeading = _cornerTarget;
  _turnHeadingValid = true;
  long curL = getLeftEncoder(), curR = getRightEncoder();
  _lastEncLeft = _startEncLeft = curL;   // enc PID tính chênh lệch từ đầu đoạn thẳng mới
  _lastEncRight = _startEncRight = curR;
  if (stepCellActive) { stepStartL = curL; stepStartR = curR; } // đoạn thẳng sau cua đếm quãng đường lại từ đầu
  _lastPIDLoopTime = micros();
  Serial.printf(">> [CORNERSM] STRAIGHT | yaw=%.2f heading=%.2f PWM %d/%d\n",
                mpu6050.getYaw(), cornerTargetYaw, currentLeftSpeed, currentRightSpeed);
}

// Test: đi 1 ô (không dừng) -> cua trái 90° bằng state machine -> đi 1 ô -> dừng
void RobotNav::runCornerNavTest() {
  if (!mpuReady) { Serial.println(">> [CORNERNAV] MPU6050 chua san sang, huy"); return; }
  stopPID();
  unsigned long s = millis();
  while (millis() - s < 200) { mpu6050.update(); delay(2); }

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = seqCellPulses;
  mpu6050.update();
  _turnHeading = mpu6050.getYaw();
  _turnHeadingValid = true;
  clearLog();

  auto move = [&](bool noStop) {
    startHold(true);
    _targetYaw = _turnHeading;
    stepNoStop = noStop;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    stepNoStop = false;
  };

  move(true);
  if (startCornerLeft()) {
    unsigned long t0 = millis();
    while (cornerBusy() && millis() - t0 < 6000) update();
  }
  move(false);
  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [CORNERNAV] DONE - PWM = 0");
}

// ================= TEST CELL-TO-CELL: 1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng =================

void RobotNav::runCellToCellTest() {
  if (!mpuReady) { Serial.println(">> [C2C] MPU6050 chua san sang, huy"); return; }
  stopPID();

  auto settle = [&](unsigned long ms) {
    unsigned long s = millis();
    while (millis() - s < ms) { mpu6050.update(); delay(2); }
  };

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = seqCellPulses;

  settle(200);
  mpu6050.update();
  _turnHeading = mpu6050.getYaw(); // hướng chuẩn: mỗi đoạn thẳng bám hướng này, lỗi quay không cộng dồn
  _turnHeadingValid = true;
  clearLog();
  Serial.printf(">> [C2C] BAT DAU | %u xung/o | yaw0=%.2f\n", (unsigned)seqCellPulses, _turnHeading);

  int step = 0;
  auto move = [&]() {
    step++;
    Serial.printf(">> [C2C] %d/5 MOVE 1 O\n", step);
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    settle(300); // dừng hẳn trước khi quay
    Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f (chuan %.2f)\n", stepTraveledPulses, mpu6050.getYaw(), _turnHeading);
  };
  auto turn = [&](float angle, const char *name) {
    step++;
    Serial.printf(">> [C2C] %d/5 TURN %s\n", step, name);
    bool ok = turnLeft90Test(angle); // góc dương = trái, âm = phải
    settle(200);
    Serial.printf(">>        TURN xong (%s) | yaw=%.2f (chuan %.2f)\n", ok ? "OK" : "TIMEOUT", mpu6050.getYaw(), _turnHeading);
  };

  move();
  turn(90.0f, "TRAI 90");
  move();
  turn(-90.0f, "PHAI 90");
  move();

  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [C2C] DONE - PWM = 0");
}


// ================= CALIB LẠI 1 Ô: lùi ngắn -> calib gyro về 0° -> tiến calibFwdPulses xung tới tâm ô =================
// Dùng khi vào đường cụt, sau khi quay 180°: lùi calibRevMs (mặc định ~0.1s) cho lưng xe sát vách, đứng yên,
// calib lại gyro và đặt yaw = 0, rồi tiến calibFwdPulses xung. Từ điểm dừng này coi như xe đang ở GIỮA Ô hiện tại;
// quãng calibFwdPulses không tính vào quãng 1 ô kế tiếp (ô kế tiếp đếm lại từ 0).
bool RobotNav::calibCellFromBackWall() {
  if (!mpuReady) { Serial.println(">> [CALIBCELL] MPU6050 chua san sang, huy"); return false; }
  stopPID();
  resetEnc();
  mpu6050.update();
  float target = _turnHeadingValid ? _turnHeading : mpu6050.getYaw(); // giữ hướng sau lần quay 180° trong lúc lùi ngắn

  Serial.printf(">> [CALIBCELL] LUI %u ms | PWM %.0f | yaw=%.2f\n", (unsigned)calibRevMs, calibRevPwm, mpu6050.getYaw());
  unsigned long t0 = millis();
  while (millis() - t0 < calibRevMs) {
    mpu6050.update();
    float e = target - mpu6050.getYaw();
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    // Lùi: bánh phải lùi nhanh hơn thì xe xoay phải (yaw tăng), ngược chiều với khi tiến -> đảo dấu so với khi chạy thẳng
    float corr = constrain(hold.yawKp * e, -10.0f, 10.0f);
    int pl = constrain((int)lroundf((calibRevPwm - corr) * motorScaleL), 0, 255);
    int pr = constrain((int)lroundf((calibRevPwm + corr) * motorScaleR), 0, 255);
    analogWrite(M1_IN1, 0); analogWrite(M1_IN2, pl);
    analogWrite(M2_IN1, 0); analogWrite(M2_IN2, pr);
    currentLeftSpeed = -pl;
    currentRightSpeed = -pr;
    delay(2);
  }
  stopMotors();
  long reversed = (labs(getLeftEncoder()) + labs(getRightEncoder())) / 2;
  Serial.printf(">> [CALIBCELL] DA LUI %ld xung | yaw=%.2f -> calib gyro, giu xe dung yen\n", reversed, mpu6050.getYaw());

  unsigned long s = millis();
  while (millis() - s < 300) { mpu6050.update(); delay(2); } // đứng yên hẳn trước khi calib gyro
  mpu6050.calibrate();
  mpu6050.resetYaw();                                          // yaw = 0 là hướng mới
  _turnHeading = 0.0f;
  _turnHeadingValid = true;
  Serial.printf(">> [CALIBCELL] GYRO CALIB XONG, YAW = %.2f | tien %ld xung toi giua o\n", mpu6050.getYaw(), calibFwdPulses);

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = calibFwdPulses;
  startHold(true);              // resetEnc() trong startHold: đếm lại từ 0
  _targetYaw = _turnHeading;
  unsigned long t1 = millis();
  while (pidRunActive && millis() - t1 < 15000) update();
  if (pidRunActive) stopPID();
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  s = millis();
  while (millis() - s < 300) { mpu6050.update(); delay(2); }
  Serial.printf(">> [CALIBCELL] XONG, DANG O GIUA O | tien %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  return true;
}

// ================= TEST NGÕ CỤT: 1 ô -> dừng -> quay 180° -> calib lại ô =================

void RobotNav::runDeadEndTest() {
  if (!mpuReady) { Serial.println(">> [DEADEND] MPU6050 chua san sang, huy"); return; }
  stopPID();

  auto settle = [&](unsigned long ms) {
    unsigned long s = millis();
    while (millis() - s < ms) { mpu6050.update(); delay(2); }
  };

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = seqCellPulses;

  settle(200);
  mpu6050.update();
  _turnHeading = mpu6050.getYaw();
  _turnHeadingValid = true;
  clearLog();
  Serial.printf(">> [DEADEND] BAT DAU | %u xung/o | yaw0=%.2f\n", (unsigned)seqCellPulses, _turnHeading);

  Serial.println(">> [DEADEND] 1/3 MOVE 1 O (coi nhu vao ngo cut)");
  startHold(true);
  _targetYaw = _turnHeading;
  unsigned long t0 = millis();
  while (pidRunActive && millis() - t0 < 15000) update();
  if (pidRunActive) stopPID();
  settle(300);
  Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());

  Serial.println(">> [DEADEND] 2/3 QUAY 180");
  bool ok = turnLeft90Test(180.0f);
  settle(300);
  Serial.printf(">>        QUAY xong (%s) | yaw=%.2f (chuan %.2f)\n", ok ? "OK" : "TIMEOUT", mpu6050.getYaw(), _turnHeading);

  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [DEADEND] 3/3 CALIB LAI O");
  if (calibCellFromBackWall()) {
    // Đang ở giữa ô sau calib: đi thẳng thêm 2 ô (hướng yaw = 0 vừa calib)
    Serial.println(">> [DEADEND] 4/4 MOVE 2 O");
    hold.cells = 2;
    pulsesPerCell = seqCellPulses;
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t2 = millis();
    while (pidRunActive && millis() - t2 < 20000) update();
    if (pidRunActive) stopPID();
    hold.cells = savedCells;
    pulsesPerCell = savedPulses;
    unsigned long s2 = millis();
    while (millis() - s2 < 300) { mpu6050.update(); delay(2); }
    Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  }

  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  Serial.println(">> [DEADEND] DONE - PWM = 0");
}

// ================= TEST CALIBRATION ENCODER (chạy thẳng N ô liên tục) =================

void RobotNav::runCalibTest(uint8_t cells) {
  if (cells == 0) cells = 1;
  stopPID();
  unsigned long s = millis();
  if (mpuReady) { while (millis() - s < 200) { mpu6050.update(); delay(2); } }

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = cells;
  pulsesPerCell = seqCellPulses;  // 4200 xung/ô: giá trị TẠM để test, không phải calibration chính thức
  long target = (long)cells * pulsesPerCell;
  Serial.printf(">> [CALIB] BAT DAU: %u o x %ld xung/o (tam) = %ld xung\n", (unsigned)cells, pulsesPerCell, target);

  startHold();  // Encoder PID + yaw correction + VL53 wall correction như đường chạy thẳng hiện tại
  unsigned long t0 = millis();
  while (pidRunActive && millis() - t0 < 30000) update();
  if (pidRunActive) stopPID();
  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;

  // Chờ xe trôi hết quán tính rồi mới đọc encoder
  s = millis();
  while (millis() - s < 500) { if (mpuReady) mpu6050.update(); delay(2); }

  long l = getLeftEncoder(), r = getRightEncoder();
  float avg = (labs(l) + labs(r)) / 2.0f;
  float perCell = avg / (float)cells;
  calibReport = "==== CALIB " + String((unsigned)cells) + " O (target " + String(target) + " xung) ====\n" +
                "ENC_L = " + String(l) + "\n" +
                "ENC_R = " + String(r) + "\n" +
                "AVG_TICKS = " + String(avg, 1) + "\n" +
                "TICKS_PER_CELL = " + String(perCell, 1) + "\n" +
                "==== HET CALIB ====";
  calibPrinted = false;
  Serial.println(calibReport);
  Serial.println(">> [CALIB] DONE - PWM = 0");
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
}
