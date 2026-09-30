#include "RobotNav.h"

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR),
      wallPID(1.0f, 0.0f, 0.08f, -35.0f, 35.0f),
      gyroPID(1.2f, 0.0f, 0.05f, -40.0f, 40.0f) {
  leftReady = false;
  frontReady = false;
  rightReady = false;
  mpuReady = false;
  autoTestMode = false;
  pidRunActive = false;

  leftCompensation = 22.0f;
  rightCompensation = 22.0f;
  turnSpeed = 70;
  baseForwardSpeed = 75;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  _targetYaw = 0.0f;
  _lastPIDLoopTime = 0;

  // Giá trị thực tế đo được tại tâm ô (Theo kết quả đo mới nhất trên sa hình)
  targetLeftDist = 171.0f;
  targetRightDist = 146.0f;
  centerOffset = 25.0f; // 171.0f - 146.0f = 25.0f
  wallThreshold = 230; // Khoảng cách < 230mm là có tường, > 230mm là cửa trống
  frontStopDist = 123; // Phanh dừng khi cách tường trước <= 123mm (tâm ô thực tế là 121mm)
  frontWallDist = 360; // dF <= 360mm: có tường trước của Ô PHÍA TRƯỚC (~300mm)

  // Giá trị cảm biến lọc & Vùng chết tâm ô
  smoothDL = 171.0f;
  smoothDF = 999.0f;
  smoothDR = 146.0f;
  currentWallError = 0.0f;
  wallDeadband = 3.0f; // Vùng chết 3mm khử nhiễu dao động ở tâm ô mà không làm trễ phản xạ bẻ lái
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
  pulsesPerCell = 4400; // Đo thực tế từ tâm ô 1 tới giữa ô 2 = 4400 xung
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
      bleManager.println(">> MPU6050 & SENSORS READY!");
    } else {
      Serial.println("MPU6050 INIT FAIL");
    }
  } else {
    Serial.println("MPU6050 NOT FOUND @ 0x68");
  }
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
  delay(50);
  stopMotors();
}

void RobotNav::turnRight(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    bleManager.println(msg);
    return;
  }
  bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         rightCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
  Serial.println(res);
  bleManager.println(res);
}

void RobotNav::turnLeft(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    bleManager.println(msg);
    return;
  }

  bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         leftCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
  Serial.println(res);
  bleManager.println(res);
}

void RobotNav::runTurnTest() {
  if (!mpuReady)
    return;

  bleManager.println("==========================================");
  bleManager.println(">> AUTO TEST: CHUAN BI QUAY TRAI 90 DO...");
  delay(800);
  turnLeft(90.0f);
  stopMotors();
  delay(1500);

  bleManager.println(">> AUTO TEST: CHUAN BI QUAY PHAI 90 DO VE HUONG CU...");
  delay(800);
  turnRight(90.0f);
  stopMotors();
  delay(1500);
  bleManager.println(">> AUTO TEST: HOAN TAT 1 CHU KY.");
}

void RobotNav::startPID() {
  autoTestMode = false;
  if (mpuReady) {
    mpu6050.update();
    _targetYaw = mpu6050.getYaw();
  }
  wallPID.reset();
  gyroPID.reset();
  _startEncLeft = getLeftEncoder();
  _startEncRight = getRightEncoder();
  _lastEncLeft = _startEncLeft;
  _lastEncRight = _startEncRight;
  _smoothError = 0.0f;
  _lastPIDLoopTime = micros();
  pidRunActive = true;
}

void RobotNav::stopPID() {
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
  bleManager.println(msg);
}

void RobotNav::updateSensors() {
  unsigned long now = millis();
  // Giới hạn chu kỳ đọc cảm biến khoảng 35ms một lần (tương thích timing budget 33ms của VL53L0X)
  if (now - _lastSensorReadTime < 35) {
    return;
  }
  _lastSensorReadTime = now;

  uint16_t raw_dL = 999;
  if (leftReady) {
    raw_dL = sensorLeft.readRangeContinuousMillimeters();
    if (sensorLeft.timeoutOccurred() || raw_dL > 1200 || raw_dL < 15) {
      raw_dL = 999;
    }
  }

  uint16_t raw_dF = 999;
  if (frontReady) {
    raw_dF = sensorFront.readRangeContinuousMillimeters();
    if (sensorFront.timeoutOccurred() || raw_dF > 1200 || raw_dF < 15) {
      raw_dF = 999;
    }
  }

  uint16_t raw_dR = 999;
  if (rightReady) {
    raw_dR = sensorRight.readRangeContinuousMillimeters();
    if (sensorRight.timeoutOccurred() || raw_dR > 1200 || raw_dR < 15) {
      raw_dR = 999;
    }
  }

  // Lọc EMA cho cảm biến trái
  if (raw_dL < 800) {
    smoothDL = (0.4f * (float)raw_dL) + (0.6f * smoothDL);
  } else {
    smoothDL = (0.2f * 999.0f) + (0.8f * smoothDL);
  }

  // Lọc EMA cho cảm biến phải
  if (raw_dR < 800) {
    smoothDR = (0.4f * (float)raw_dR) + (0.6f * smoothDR);
  } else {
    smoothDR = (0.2f * 999.0f) + (0.8f * smoothDR);
  }

  // Lọc EMA cho cảm biến trước (phản ứng nhanh để dừng kịp thời)
  if (raw_dF < 800) {
    smoothDF = (0.6f * (float)raw_dF) + (0.4f * smoothDF);
  } else {
    smoothDF = (float)raw_dF;
  }

  // Nhận diện tường bên
  bool hasLeftWall = (leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold);
  bool hasRightWall = (rightReady && smoothDR > 20.0f && smoothDR < (float)wallThreshold);

  float raw_wall_error = 0.0f;
  if (hasLeftWall && hasRightWall) {
    // Có cả 2 tường: tính độ lệch tâm (nhân 0.5f để độ nhạy đồng nhất với khi chỉ có 1 tường)
    raw_wall_error = 0.5f * ((smoothDL - smoothDR) - centerOffset);
  } else if (hasLeftWall) {
    // Chỉ có tường trái
    raw_wall_error = smoothDL - targetLeftDist;
  } else if (hasRightWall) {
    // Chỉ có tường phải
    raw_wall_error = targetRightDist - smoothDR;
  } else {
    // Không có tường
    raw_wall_error = 0.0f;
  }

  // Áp dụng Soft Deadband (Vùng chết mượt khử hoàn toàn nhiễu dao động ở tâm ô):
  if (fabs(raw_wall_error) <= wallDeadband) {
    currentWallError = 0.0f;
  } else if (raw_wall_error > wallDeadband) {
    currentWallError = raw_wall_error - wallDeadband;
  } else {
    currentWallError = raw_wall_error + wallDeadband;
  }
}

void RobotNav::updatePIDLoop() {
  if (!pidRunActive)
    return;

  unsigned long now = micros();
  float dt = (now - _lastPIDLoopTime) / 1000000.0f;
  if (_lastPIDLoopTime == 0 || dt > 0.5f || dt <= 0.0f)
    dt = 0.01f;
  _lastPIDLoopTime = now;

  uint16_t dL = (uint16_t)smoothDL;
  uint16_t dR = (uint16_t)smoothDR;
  uint16_t dF = (uint16_t)smoothDF;

  // 1. Phanh dừng an toàn khi gặp vách tường trước
  if (frontReady && dF > 20 && dF <= frontStopDist) {
    brakeMotors();
    stopPID();
    String msg =
        ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
    Serial.println(msg);
    bleManager.println(msg);
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
    bool pulseReached = (distTraveled >= stepTargetPulses);
    bool timeoutSafe = (millis() - stepStartTime > 4500);

    if (pulseReached || timeoutSafe) {
      brakeMotors();
      stopPID();
      String msg = ">> [CELL] DA HOAN THANH TIEN O! Xung: " + String(distTraveled) +
                   "/" + String(stepTargetPulses) + " xung (L:" + String(distL) +
                   ", R:" + String(distR) + "). Phanh dung.";
      if (timeoutSafe && !pulseReached) {
        msg = ">> [CELL] TIMEOUT AN TOAN 4.5S! Xung: " + String(distTraveled) +
              "/" + String(stepTargetPulses) + " xung. Phanh dung.";
      }
      Serial.println(msg);
      bleManager.println(msg);
      return;
    }
  }

  bool hasLeftWall = (leftReady && dL > 20 && dL < wallThreshold);
  bool hasRightWall = (rightReady && dR > 20 && dR < wallThreshold);

  // 1. Tính độ chênh lệch xung tức thời giữa 2 bánh trong chu kỳ này
  long curL = getLeftEncoder();
  long curR = getRightEncoder();
  long deltaL = curL - _lastEncLeft;
  long deltaR = curR - _lastEncRight;
  _lastEncLeft = curL;
  _lastEncRight = curR;

  // Sai số xung Encoder: bánh trái quay nhiều hơn bánh phải -> enc_error > 0
  float enc_error = (float)(deltaL - deltaR);

  float pidOut = 0.0f;

  if (hasLeftWall || hasRightWall) {
    // 2. KHI CÓ TƯỜNG: Bám tường là ưu tiên cao nhất!
    // Tuyệt đối KHÔNG cộng enc_error vào vì chênh lệch xung giữa 2 bánh khi bẻ lái
    // sẽ chống lại lực bẻ lái của xe!
    if (mpuReady) {
      _targetYaw = mpu6050.getYaw(); // Cập nhật hướng góc để khi mất tường sẵn sàng giữ hướng thẳng
    }
    // Giới hạn sai số tường tối đa [-35, 35] mm để chống sốc khi qua ngã rẽ
    float boundedWallError = constrain(currentWallError, -35.0f, 35.0f);
    pidOut = wallPID.computeError(boundedWallError, dt);
  } else {
    // 3. KHI KHÔNG CÓ TƯỜNG (Ngã tư / Cửa trống): Giữ thẳng tuyệt đối bằng Gyro + Encoder
    float gyro_error = 0.0f;
    if (mpuReady) {
      gyro_error = _targetYaw - mpu6050.getYaw();
    }
    float straightError = gyro_error + (0.05f * enc_error);
    pidOut = gyroPID.computeError(straightError, dt);
  }

  // 4. Xuất PWM cho 2 bánh có giảm tốc 2 giai đoạn (Profile Deceleration):
  // - 0% -> 70% quãng đường ô: chạy tốc độ baseForwardSpeed tiêu chuẩn
  // - 70% -> 100% quãng đường ô: hãm về tốc độ bò (crawl speed ~46 PWM) để khi ngắt động cơ xe dừng dứt điểm, không trượt quán tính
  int activeBaseSpeed = baseForwardSpeed;
  if (stepCellActive && stepTraveledPulses >= (long)(0.70f * stepTargetPulses)) {
    activeBaseSpeed = 46;
  }

  int leftSpeed = activeBaseSpeed - (int)pidOut;
  int rightSpeed = activeBaseSpeed + (int)pidOut;

  leftSpeed = constrain(leftSpeed, 35, 255);
  rightSpeed = constrain(rightSpeed, 35, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  analogWrite(M1_IN1, leftSpeed);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, rightSpeed);
  analogWrite(M2_IN2, 0);
}

void RobotNav::update() {
  updateSensors();

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
    walls.hasRight = rightReady && smoothDR > 20.0f && smoothDR < (float)wallThreshold && !rightIsFrontReflection;
  } else {
    // 2 Cảm biến 45° chĩa ra trước nhìn 2 bên sườn ô phía trước:
    walls.hasLeft  = (leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold);
    walls.hasRight = (rightReady && smoothDR > 20.0f && smoothDR < (float)wallThreshold);
  }
  return walls;
}

void RobotNav::reportCell(char act, const WallStatus &w, const WallStatus *pre) {
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
  j += "}";
  bleManager.println(j);
}

WallStatus RobotNav::moveOneCell(char turn) {
  String startMsg = ">> [ATOMIC] BAT DAU TIEN 1 O (" + String(pulsesPerCell) + " xung)...";
  Serial.println(startMsg);
  bleManager.println(startMsg);

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
        if (smoothDL > 215.0f) {
          leftOpenCount++;
        } else if (smoothDL > 20.0f && smoothDL < (float)wallThreshold) {
          leftWallCount++;
        }
      }
      if (rightReady) {
        if (smoothDR > 215.0f) {
          rightOpenCount++;
        } else if (smoothDR > 20.0f && smoothDR < (float)wallThreshold) {
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
  delay(50);

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
  bleManager.println(resMsg);
  reportCell(turn, status, &curCellWalls);

  return status;
}

WallStatus RobotNav::turnLeftAndStep() {
  bleManager.println(">> [ATOMIC] LENH: RE TRAI 90 DO & TIEN 1 O");
  turnLeft(90.0f);
  delay(60);
  resetEnc();
  wallPID.reset();
  gyroPID.reset();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('L');
}

WallStatus RobotNav::turnRightAndStep() {
  bleManager.println(">> [ATOMIC] LENH: RE PHAI 90 DO & TIEN 1 O");
  turnRight(90.0f);
  delay(60);
  resetEnc();
  wallPID.reset();
  gyroPID.reset();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('R');
}

WallStatus RobotNav::turnAroundAndStep() {
  bleManager.println(">> [ATOMIC] LENH: QUAY DAU 180 DO & TIEN 1 O");
  turnRight(180.0f);
  delay(60);
  resetEnc();
  wallPID.reset();
  gyroPID.reset();
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
  bleManager.println(msg);
}

void RobotNav::stopAutoWallFollow() {
  autoWallFollowActive = false;
  stopPID();
  brakeMotors();
  lastAutoDecision = "ĐÃ DỪNG";
  String msg = ">> [CHẾ ĐỘ 3] ĐÃ DỪNG TỰ ĐỘNG! Tổng số ô đã chạy: " + String(autoCellCount);
  Serial.println(msg);
  bleManager.println(msg);
}

void RobotNav::stepAutoWallFollow() {
  if (!autoWallFollowActive) return;

  // 1. Kiểm tra an toàn giới hạn số ô để phòng chạy vòng tròn lặp vô tận
  if (autoCellCount >= autoMaxCells) {
    stopAutoWallFollow();
    String limMsg = ">> [CHẾ ĐỘ 3] ĐẠT MỐC AN TOÀN " + String(autoMaxCells) + " Ô! Tự động phanh dừng.";
    Serial.println(limMsg);
    bleManager.println(limMsg);
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
  bleManager.println(statusMsg);

  // Cho xe nghỉ ổn định 80ms tại tâm ô trước khi sang ô tiếp theo
  delay(80);
}

// =========================================================================
// CHẾ ĐỘ 4: FLOOD-FILL (dùng maze_algorithm.h)
// =========================================================================

static void ffSetWall(ParentMaze &m, int x, int y, int dir, bool wall) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  int nx = x + DX[dir], ny = y + DY[dir];
  if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE)
    return; // tường biên luôn có sẵn
  Cell &c = m.cell(x, y);
  bool *self[4] = {&c.north_wall, &c.east_wall, &c.south_wall, &c.west_wall};
  *self[dir] = wall;
  Cell &n = m.cell(nx, ny);
  bool *other[4] = {&n.north_wall, &n.east_wall, &n.south_wall, &n.west_wall};
  *other[(dir + 2) % 4] = wall; // đồng bộ tường với ô kề
}

// Cảm biến nhìn trước 1 ô: ws (trước/trái/phải) là vách của ô PHÍA TRƯỚC (x,y) theo hướng h
static void ffRecordAhead(ParentMaze &m, int x, int y, int h, const WallStatus &ws) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  ffSetWall(m, x, y, h, ws.hasFrontNear); // vách trước của ô hiện tại
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
  startCellFresh = (x == 0 && y == 0); // ô (0,0) luôn có tường trái, phải, sau
  x = constrain(x, 0, MAZE_SIZE - 1);
  y = constrain(y, 0, MAZE_SIZE - 1);
  ffX = x;
  ffY = y;
  ffH = ((h % 4) + 4) % 4;
  ffStart = {(int8_t)x, (int8_t)y};
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
    if (startCellFresh) { // ô xuất phát: mặc định chỉ có 2 tường bên
      ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, true);
      ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, true);
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
               " | Đích (" + String(ffGoal.x) + "," + String(ffGoal.y) + ")";
  Serial.println(msg);
  bleManager.println(msg);
}

void RobotNav::stopFloodFill(const char *reason) {
  ffActive = false;
  stopPID();
  brakeMotors();
  lastAutoDecision = reason;
  String msg = ">> [CHẾ ĐỘ 4] " + String(reason) + " | Số ô đã đi: " + String(autoCellCount);
  Serial.println(msg);
  bleManager.println(msg);
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
    // KHÁM PHÁ HẾT: ưu tiên ô kề CHƯA ĐI theo thứ tự thẳng -> trái -> phải
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
      if (ffMaze.cell(nx, ny).run_visited) continue;
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

  if (cur.x == target.x && cur.y == target.y) {
    stopFloodFill("ĐÃ TỚI ĐÍCH");
    return;
  }

  if (ffMode == 0 || (next.x == cur.x && next.y == cur.y)) {
    ffMaze.floodfill_update(target.x, target.y, false, false);
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
  ffX = next.x;
  ffY = next.y;
  // Cửa xe vừa bước qua luôn thông
  ffSetWall(ffMaze, ffX, ffY, (ffH + 2) % 4, false);
  if (!ffMaze.cell(ffX, ffY).known) {
    ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, curWalls.hasLeft);
    ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, curWalls.hasRight);
    ffMaze.cell(ffX, ffY).known = true;
  }
  ffMaze.cell(ffX, ffY).run_visited = true;
  ffRecordAhead(ffMaze, ffX, ffY, ffH, ws);
  autoCellCount++;

  String m = ">> [CHẾ ĐỘ 4] Ô #" + String(autoCellCount) + " -> (" + String(ffX) + "," +
             String(ffY) + ") " + lastAutoDecision;
  Serial.println(m);
  bleManager.println(m);
  delay(80);
}
