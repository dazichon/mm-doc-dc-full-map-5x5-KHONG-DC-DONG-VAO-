#include "CommandHandler.h"

CommandHandler commandHandler;

CommandHandler::CommandHandler() { _lastTelemetryTime = 0; }

void CommandHandler::update() {
  processBLECommands();
  sendTelemetry();
}

void CommandHandler::processBLECommands() {
  if (!bleManager.hasCommand())
    return;

  String cmd = bleManager.readCommand();
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "0") {
    // Lệnh nhanh: DỪNG mọi chế độ tự động
    if (robotNav.ffActive) robotNav.stopFloodFill();
    robotNav.patrolMode = false;
    robotNav.stopAutoWallFollow();
  } else if (cmd == "1") {
    // Lệnh nhanh: CHẠY ưu tiên rẽ TRÁI (trái -> thẳng -> phải -> quay đầu)
    robotNav.patrolMode = false;
    robotNav.followRightHand = false;
    robotNav.startAutoWallFollow();
  } else if (cmd == "2") {
    // Lệnh nhanh: đi thẳng, gặp tường thì quay đầu
    robotNav.patrolMode = true;
    robotNav.startAutoWallFollow();
  } else if (cmd == "TL" || cmd == "TEST_LEFT") {
    robotNav.pidRunActive = false;
    bleManager.println(">> [BLE] LENH: RE TRAI 90 DO");
    robotNav.turnLeft(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "TR" || cmd == "TEST_RIGHT") {
    robotNav.pidRunActive = false;
    bleManager.println(">> [BLE] LENH: RE PHAI 90 DO");
    robotNav.turnRight(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "PID_ON" || cmd == "FORWARD" || cmd == "FWD") {
    robotNav.startPID();
    bleManager.println(
        ">> [BLE] KICH HOAT PID BAM TUONG & GIU HUONG TIEN THANG!");
  } else if (cmd == "PID_OFF" || cmd == "STOP" || cmd == "ST") {
    robotNav.stopPID();
    bleManager.println(">> [BLE] DA DUNG XE (PID OFF)");
  } else if (cmd == "RESET_YAW" || cmd == "RST") {
    robotNav.mpu6050.resetYaw();
    bleManager.println(">> [BLE] DA RESET GOC YAW VE 0.0 DO!");
  } else if (cmd == "CALIB" || cmd == "CALIBRATE") {
    robotNav.stopMotors();
    bleManager.println(
        ">> [BLE] DANG HIEU CHUAN GYRO... VUI LONG DE YEN ROBOT 1.5S!");
    robotNav.mpu6050.calibrate(400);
    bleManager.println(">> [BLE] HIEU CHUAN GYRO HOAN TAT! GOC YAW VE 0.0 DO.");
  } else if (cmd.startsWith("SET_LC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.leftCompensation = val;
      bleManager.println(">> [BLE] CAP NHAT LEFT_COMPENSATION = " +
                         String(robotNav.leftCompensation, 1));
    }
  } else if (cmd.startsWith("SET_RC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.rightCompensation = val;
      bleManager.println(">> [BLE] CAP NHAT RIGHT_COMPENSATION = " +
                         String(robotNav.rightCompensation, 1));
    }
  } else if (cmd.startsWith("SET_SPD=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.turnSpeed = (uint8_t)val;
      robotNav.baseForwardSpeed = (uint8_t)val;
      robotNav.saveForwardSpeed();
      bleManager.println(">> [BLE] CAP NHAT SPEED = " +
                         String(robotNav.turnSpeed));
    }
  } else if (cmd.startsWith("SET_FSPD=")) {
    int val = cmd.substring(9).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.baseForwardSpeed = (uint8_t)val;
      robotNav.saveForwardSpeed();
      bleManager.println(">> [BLE] CAP NHAT BASE_FORWARD_SPEED (DA LUU) = " +
                         String(robotNav.baseForwardSpeed));
    }
  } else if (cmd.startsWith("SET_KP=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(val, robotNav.wallPID.getKi(),
                              robotNav.wallPID.getKd());
    robotNav.gyroPID.setGains(val * 1.5f, robotNav.gyroPID.getKi(),
                              robotNav.gyroPID.getKd());
    bleManager.println(">> [BLE] CAP NHAT PID Kp = " + String(val, 2));
  } else if (cmd.startsWith("SET_KI=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(robotNav.wallPID.getKp(), val,
                              robotNav.wallPID.getKd());
    robotNav.gyroPID.setGains(robotNav.gyroPID.getKp(), val,
                              robotNav.gyroPID.getKd());
    bleManager.println(">> [BLE] CAP NHAT PID Ki = " + String(val, 3));
  } else if (cmd.startsWith("SET_KD=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(robotNav.wallPID.getKp(),
                              robotNav.wallPID.getKi(), val);
    robotNav.gyroPID.setGains(robotNav.gyroPID.getKp(),
                              robotNav.gyroPID.getKi(), val * 1.5f);
    bleManager.println(">> [BLE] CAP NHAT PID Kd = " + String(val, 2));
  } else if (cmd == "AUTO_3_ON" || cmd == "AUTO_ON" || cmd == "AUTO_START") {
    robotNav.patrolMode = false;
    robotNav.startAutoWallFollow();
  } else if (cmd == "AUTO_3_OFF" || cmd == "AUTO_OFF" || cmd == "AUTO_STOP") {
    if (robotNav.ffActive) robotNav.stopFloodFill();
    robotNav.stopAutoWallFollow();
  } else if (cmd == "FF_ON" || cmd == "FF_GOAL") {
    robotNav.startFloodFill(0);
  } else if (cmd == "FF_EXPLORE") {
    robotNav.startFloodFill(1);
  } else if (cmd == "FF_OFF") {
    robotNav.stopFloodFill();
  } else if (cmd.startsWith("SET_GOAL=")) {
    // SET_GOAL=x,y[,size] -> (x,y) là góc dưới trái vùng đích, size = 1 hoặc 2 (2x2)
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    int gx = cmd.substring(9, c1).toInt();
    int gy = cmd.substring(c1 + 1, c2 > 0 ? c2 : cmd.length()).toInt();
    int gs = c2 > 0 ? constrain(cmd.substring(c2 + 1).toInt(), 1, 2) : 1;
    if (c1 > 0 && gx >= 0 && gx + gs <= MAZE_SIZE && gy >= 0 && gy + gs <= MAZE_SIZE) {
      robotNav.ffGoal = {(int8_t)gx, (int8_t)gy};
      robotNav.ffGoalSize = gs;
      bleManager.println(">> [BLE] DICH FLOOD-FILL = (" + String(gx) + "," + String(gy) + ") " +
                         String(gs) + "x" + String(gs));
    }
  } else if (cmd.startsWith("SET_SIZE=")) {
    // SET_SIZE=cols,rows -> kích thước mê cung thật; áp dụng ở lần SET_POS/xoá map kế tiếp
    int c1 = cmd.indexOf(',');
    int cs = cmd.substring(9, c1).toInt(), rs = cmd.substring(c1 + 1).toInt();
    if (c1 > 0 && cs >= 1 && cs <= MAZE_SIZE && rs >= 1 && rs <= MAZE_SIZE) {
      robotNav.ffCols = cs;
      robotNav.ffRows = rs;
      bleManager.println(">> [BLE] KICH THUOC ME CUNG = " + String(cs) + "x" + String(rs));
    }
  } else if (cmd.startsWith("SET_POS=")) {
    // SET_POS=x,y,h -> xoá bản đồ flood-fill, đặt xe tại (x,y) hướng h (0=N 1=E 2=S 3=W)
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      robotNav.resetFloodFill(cmd.substring(8, c1).toInt(),
                              cmd.substring(c1 + 1, c2).toInt(),
                              cmd.substring(c2 + 1).toInt());
      bleManager.println(">> [BLE] DAT LAI XE FLOOD-FILL (" + String(robotNav.ffX) + "," +
                         String(robotNav.ffY) + ") HUONG " + String(robotNav.ffH));
    }
  } else if (cmd == "FF_RESET") {
    robotNav.resetFloodFill(robotNav.ffStart.x, robotNav.ffStart.y, 0);
    bleManager.println(">> [BLE] DA XOA BAN DO FLOOD-FILL");
  } else if (cmd == "SET_RULE_R" || cmd == "RULE_R") {
    robotNav.followRightHand = true;
    bleManager.println(">> [BLE] DA CHON QUY TAC BAN TAY PHAI (RIGHT-HAND RULE)");
  } else if (cmd == "SET_RULE_L" || cmd == "RULE_L") {
    robotNav.followRightHand = false;
    bleManager.println(">> [BLE] DA CHON QUY TAC BAN TAY TRAI (LEFT-HAND RULE)");
  } else if (cmd.startsWith("SET_MAX_CELLS=")) {
    int val = cmd.substring(14).toInt();
    if (val > 0 && val <= 300) {
      robotNav.autoMaxCells = val;
      bleManager.println(">> [BLE] CAP NHAT GIOI HAN SO O AN TOAN = " + String(val) + " o");
    }
  } else if (cmd.startsWith("SET_TLD=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 50.0f && val <= 300.0f) {
      robotNav.targetLeftDist = val;
      robotNav.centerOffset =
          robotNav.targetLeftDist - robotNav.targetRightDist;
      bleManager.println(">> [BLE] CAP NHAT TARGET_LEFT_DIST = " +
                         String(val, 1));
    }
  } else if (cmd.startsWith("SET_TRD=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 50.0f && val <= 300.0f) {
      robotNav.targetRightDist = val;
      robotNav.centerOffset =
          robotNav.targetLeftDist - robotNav.targetRightDist;
      bleManager.println(">> [BLE] CAP NHAT TARGET_RIGHT_DIST = " +
                         String(val, 1));
    }
  } else if (cmd.startsWith("SET_WTHR=")) {
    int val = cmd.substring(9).toInt();
    if (val >= 100 && val <= 400) {
      robotNav.rightWallThreshold = (uint16_t)val;
      bleManager.println(">> [BLE] CAP NHAT RIGHT_WALL_THRESHOLD = " + String(val));
    }
  } else if (cmd.startsWith("SET_WTH=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 100 && val <= 400) {
      robotNav.wallThreshold = (uint16_t)val;
      bleManager.println(">> [BLE] CAP NHAT WALL_THRESHOLD = " + String(val));
    }
  } else if (cmd.startsWith("SET_FSTOP=")) {
    int val = cmd.substring(10).toInt();
    if (val >= 30 && val <= 150) {
      robotNav.frontStopDist = (uint16_t)val;
      bleManager.println(">> [BLE] CAP NHAT FRONT_STOP_DIST = " + String(val));
    }
  } else if (cmd.startsWith("SET_DB=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 30.0f) {
      robotNav.wallDeadband = val;
      bleManager.println(">> [BLE] CAP NHAT WALL_DEADBAND = " +
                         String(robotNav.wallDeadband, 1) + " mm");
    }
  } else if (cmd.startsWith("SET_SSF=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 0.05f && val <= 0.9f) {
      robotNav.sideSampleFrac = val;
      bleManager.println(">> [BLE] CAP NHAT SIDE_SAMPLE_FRAC = " + String(val, 2));
    }
  } else if (cmd == "RESET_ENC" || cmd == "RST_ENC") {
    robotNav.resetEnc();
    bleManager.println(">> [BLE] DA RESET XUNG ENCODER VE 0");
  } else if (cmd == "INV_ENCL") {
    robotNav.invertEncLeft = !robotNav.invertEncLeft;
    bleManager.println(">> [BLE] DAO CHIEU ENCODER TRAI: " +
                       String(robotNav.invertEncLeft ? "INVERTED (-)" : "NORMAL (+)"));
  } else if (cmd == "INV_ENCR") {
    robotNav.invertEncRight = !robotNav.invertEncRight;
    bleManager.println(">> [BLE] DAO CHIEU ENCODER PHAI: " +
                       String(robotNav.invertEncRight ? "INVERTED (-)" : "NORMAL (+)"));
  } else if (cmd == "STEP_1" || cmd == "STEP" || cmd == "CELL_1" || cmd == "M1") {
    robotNav.moveOneCell();
  } else if (cmd == "TL_STEP" || cmd == "TLS") {
    robotNav.turnLeftAndStep();
  } else if (cmd == "TR_STEP" || cmd == "TRS") {
    robotNav.turnRightAndStep();
  } else if (cmd == "TA_STEP" || cmd == "TAS") {
    robotNav.turnAroundAndStep();
  } else if (cmd == "SENSE") {
    WallStatus ws = robotNav.senseCurrentWalls();
    String senseMsg = ">> [VÁCH Ô PHÍA TRƯỚC] Trước=" + String(ws.hasFront ? "CÓ" : "TRỐNG") +
                      " | Trái=" + String(ws.hasLeft ? "CÓ" : "TRỐNG") +
                      " | Phải=" + String(ws.hasRight ? "CÓ" : "TRỐNG");
    bleManager.println(senseMsg);
    Serial.println(senseMsg);
    robotNav.reportCell('S', ws);
  } else if (cmd.startsWith("STEP=")) {
    int num = cmd.substring(5).toInt();
    if (num > 0 && num <= 20) {
      robotNav.stepCell(num);
    }
  } else if (cmd.startsWith("SET_PPC=")) {
    long val = cmd.substring(8).toInt();
    if (val >= 100 && val <= 50000) {
      robotNav.pulsesPerCell = val;
      bleManager.println(">> [BLE] CAP NHAT PULSES_PER_CELL = " + String(val) + " xung/o (180mm)");
    }
  } else if (cmd == "STATUS" || cmd == "GET") {
    printStatus();
  }
}

void CommandHandler::sendTelemetry() {
  if (bleManager.isConnected() && millis() - _lastTelemetryTime >= 200) {
    _lastTelemetryTime = millis();
    float yaw = robotNav.mpu6050.getYaw();
    uint16_t dLeft = (uint16_t)robotNav.smoothDL;
    uint16_t dFront = (uint16_t)robotNav.smoothDF;
    uint16_t dRight = (uint16_t)robotNav.smoothDR;
    long encL = robotNav.getLeftEncoder();
    long encR = robotNav.getRightEncoder();

    String jsonMsg =
        "{\"type\":\"telemetry\",\"yaw\":" + String(yaw, 1) +
        ",\"l\":" + String(dLeft) + ",\"f\":" + String(dFront) +
        ",\"r\":" + String(dRight) +
        ",\"err\":" + String(robotNav.currentWallError, 1) +
        ",\"db\":" + String(robotNav.wallDeadband, 1) +
        ",\"el\":" + String(encL) + ",\"er\":" + String(encR) +
        ",\"ppc\":" + String(robotNav.pulsesPerCell) +
        ",\"step\":" + String(robotNav.stepCellActive ? "true" : "false") +
        ",\"sstart\":" + String(robotNav.stepStartPulses) +
        ",\"starg\":" + String(robotNav.stepTargetPulses) +
        ",\"stravel\":" + String(robotNav.stepTraveledPulses) + "}";
    bleManager.println(jsonMsg);
    delay(6);

    jsonMsg = String("{\"type\":\"telemetry\"") +
        ",\"lc\":" + String(robotNav.leftCompensation, 1) +
        ",\"rc\":" + String(robotNav.rightCompensation, 1) +
        ",\"spd\":" + String(robotNav.turnSpeed) +
        ",\"kp\":" + String(robotNav.wallPID.getKp(), 2) +
        ",\"ki\":" + String(robotNav.wallPID.getKi(), 3) +
        ",\"kd\":" + String(robotNav.wallPID.getKd(), 2) +
        ",\"pid\":" + String(robotNav.pidRunActive ? "true" : "false") +
        ",\"auto\":" + String(robotNav.autoTestMode ? "true" : "false") +
        ",\"lspd\":" + String(robotNav.currentLeftSpeed) +
        ",\"rspd\":" + String(robotNav.currentRightSpeed) +
        ",\"fspd\":" + String(robotNav.baseForwardSpeed) +
        ",\"tld\":" + String(robotNav.targetLeftDist, 1) +
        ",\"trd\":" + String(robotNav.targetRightDist, 1) +
        ",\"wth\":" + String(robotNav.wallThreshold) +
        ",\"wthr\":" + String(robotNav.rightWallThreshold) +
        ",\"fstop\":" + String(robotNav.frontStopDist) +
        "}";
    bleManager.println(jsonMsg);
    delay(6);

    jsonMsg = String("{\"type\":\"telemetry\"") +
        ",\"auto3\":" + String(robotNav.autoWallFollowActive ? "true" : "false") +
        ",\"acnt\":" + String(robotNav.autoCellCount) +
        ",\"arule\":\"" + String(robotNav.followRightHand ? "R" : "L") + "\"" +
        ",\"adec\":\"" + robotNav.lastAutoDecision + "\"" +
        ",\"ffon\":" + String(robotNav.ffActive ? "true" : "false") +
        ",\"ffx\":" + String(robotNav.ffX) + ",\"ffy\":" + String(robotNav.ffY) +
        ",\"ffh\":" + String(robotNav.ffH) +
        ",\"fgx\":" + String(robotNav.ffGoal.x) + ",\"fgy\":" + String(robotNav.ffGoal.y) +
        ",\"fgs\":" + String(robotNav.ffGoalSize) + "}";
    bleManager.println(jsonMsg);
  }
}

void CommandHandler::printStatus() {
  uint16_t dLeft = (uint16_t)robotNav.smoothDL;
  uint16_t dFront = (uint16_t)robotNav.smoothDF;
  uint16_t dRight = (uint16_t)robotNav.smoothDR;

  robotNav.mpu6050.update();
  float yaw = robotNav.mpu6050.getYaw();

  String statusMsg = "--- STATS ---\n";
  statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
  statusMsg += "TOF (L/F/R): " + String(dLeft) + " / " + String(dFront) +
               " / " + String(dRight) + " mm\n";
  statusMsg += "DEADBAND: " + String(robotNav.wallDeadband, 1) +
               " mm | ERROR: " + String(robotNav.currentWallError, 1) + " mm\n";
  statusMsg += "PULSES/CELL: " + String(robotNav.pulsesPerCell) + " pulses (180mm)\n";
  statusMsg += "PARAMS -> LEFT_COMP: " + String(robotNav.leftCompensation, 1) +
               " | RIGHT_COMP: " + String(robotNav.rightCompensation, 1) +
               " | SPEED: " + String(robotNav.turnSpeed) + "\n";
  statusMsg += "PID GAINS -> Kp: " + String(robotNav.wallPID.getKp(), 2) +
               " | Ki: " + String(robotNav.wallPID.getKi(), 3) +
               " | Kd: " + String(robotNav.wallPID.getKd(), 2) + "\n";
  statusMsg += "PID ACTIVE: " + String(robotNav.pidRunActive ? "YES" : "NO");

  Serial.println(statusMsg);
  bleManager.println(statusMsg);
}
