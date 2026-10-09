#include "CommandHandler.h"

CommandHandler commandHandler;

CommandHandler::CommandHandler() { _lastTelemetryTime = 0; }

void CommandHandler::update() {
  processSerialCommands();
  sendTelemetry();
}

void CommandHandler::processSerialCommands() {
  if (!serialLink.hasCommand())
    return;

  String cmd = serialLink.readCommand();
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
    serialLink.println(">> [SERIAL] LENH: RE TRAI 90 DO");
    robotNav.turnLeft(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "TR" || cmd == "TEST_RIGHT") {
    robotNav.pidRunActive = false;
    serialLink.println(">> [SERIAL] LENH: RE PHAI 90 DO");
    robotNav.turnRight(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "LOG") {
    robotNav.printLog();
  } else if (cmd == "HOLD") {
    robotNav.startHold();
    serialLink.println(">> [SERIAL] BAT CHE DO GIU VI TRI TRONG O (STOP de dung)");
  } else if (cmd == "PID_ON" || cmd == "FORWARD" || cmd == "FWD") {
    robotNav.startPID();
    serialLink.println(
        ">> [SERIAL] KICH HOAT PID BAM TUONG & GIU HUONG TIEN THANG!");
  } else if (cmd == "PID_OFF" || cmd == "STOP" || cmd == "ST") {
    robotNav.stopPID();
    serialLink.println(">> [SERIAL] DA DUNG XE (PID OFF)");
  } else if (cmd == "RESET_YAW" || cmd == "RST") {
    robotNav.mpu6050.resetYaw();
    serialLink.println(">> [SERIAL] DA RESET GOC YAW VE 0.0 DO!");
  } else if (cmd == "CALIB" || cmd == "CALIBRATE") {
    robotNav.stopMotors();
    serialLink.println(
        ">> [SERIAL] DANG HIEU CHUAN GYRO... VUI LONG DE YEN ROBOT 1.5S!");
    robotNav.mpu6050.calibrate(400);
    serialLink.println(">> [SERIAL] HIEU CHUAN GYRO HOAN TAT! GOC YAW VE 0.0 DO.");
  } else if (cmd.startsWith("SET_LC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.leftCompensation = val;
      serialLink.println(">> [SERIAL] CAP NHAT LEFT_COMPENSATION = " +
                         String(robotNav.leftCompensation, 1));
    }
  } else if (cmd.startsWith("SET_RC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.rightCompensation = val;
      serialLink.println(">> [SERIAL] CAP NHAT RIGHT_COMPENSATION = " +
                         String(robotNav.rightCompensation, 1));
    }
  } else if (cmd.startsWith("SET_SPD=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.turnSpeed = (uint8_t)val;
      robotNav.baseForwardSpeed = (uint8_t)val;
      robotNav.saveForwardSpeed();
      serialLink.println(">> [SERIAL] CAP NHAT SPEED = " +
                         String(robotNav.turnSpeed));
    }
  } else if (cmd.startsWith("SET_FSPD=")) {
    int val = cmd.substring(9).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.baseForwardSpeed = (uint8_t)val;
      robotNav.saveForwardSpeed();
      serialLink.println(">> [SERIAL] CAP NHAT BASE_FORWARD_SPEED (DA LUU) = " +
                         String(robotNav.baseForwardSpeed));
    }
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
      serialLink.println(">> [SERIAL] DICH FLOOD-FILL = (" + String(gx) + "," + String(gy) + ") " +
                         String(gs) + "x" + String(gs));
    }
  } else if (cmd.startsWith("SET_SIZE=")) {
    // SET_SIZE=cols,rows -> kích thước mê cung thật; áp dụng ở lần SET_POS/xoá map kế tiếp
    int c1 = cmd.indexOf(',');
    int cs = cmd.substring(9, c1).toInt(), rs = cmd.substring(c1 + 1).toInt();
    if (c1 > 0 && cs >= 1 && cs <= MAZE_SIZE && rs >= 1 && rs <= MAZE_SIZE) {
      robotNav.ffCols = cs;
      robotNav.ffRows = rs;
      serialLink.println(">> [SERIAL] KICH THUOC ME CUNG = " + String(cs) + "x" + String(rs));
    }
  } else if (cmd.startsWith("SET_POS=")) {
    // SET_POS=x,y,h -> xoá bản đồ flood-fill, đặt xe tại (x,y) hướng h (0=N 1=E 2=S 3=W)
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      robotNav.resetFloodFill(cmd.substring(8, c1).toInt(),
                              cmd.substring(c1 + 1, c2).toInt(),
                              cmd.substring(c2 + 1).toInt());
      serialLink.println(">> [SERIAL] DAT LAI XE FLOOD-FILL (" + String(robotNav.ffX) + "," +
                         String(robotNav.ffY) + ") HUONG " + String(robotNav.ffH));
    }
  } else if (cmd == "FF_RESET") {
    robotNav.resetFloodFill(robotNav.ffStart.x, robotNav.ffStart.y, 0);
    serialLink.println(">> [SERIAL] DA XOA BAN DO FLOOD-FILL");
  } else if (cmd == "SET_RULE_R" || cmd == "RULE_R") {
    robotNav.followRightHand = true;
    serialLink.println(">> [SERIAL] DA CHON QUY TAC BAN TAY PHAI (RIGHT-HAND RULE)");
  } else if (cmd == "SET_RULE_L" || cmd == "RULE_L") {
    robotNav.followRightHand = false;
    serialLink.println(">> [SERIAL] DA CHON QUY TAC BAN TAY TRAI (LEFT-HAND RULE)");
  } else if (cmd.startsWith("SET_MAX_CELLS=")) {
    int val = cmd.substring(14).toInt();
    if (val > 0 && val <= 300) {
      robotNav.autoMaxCells = val;
      serialLink.println(">> [SERIAL] CAP NHAT GIOI HAN SO O AN TOAN = " + String(val) + " o");
    }
  } else if (cmd.startsWith("SET_WTHR=")) {
    int val = cmd.substring(9).toInt();
    if (val >= 100 && val <= 400) {
      robotNav.rightWallThreshold = (uint16_t)val;
      serialLink.println(">> [SERIAL] CAP NHAT RIGHT_WALL_THRESHOLD = " + String(val));
    }
  } else if (cmd.startsWith("SET_WTH=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 100 && val <= 400) {
      robotNav.wallThreshold = (uint16_t)val;
      serialLink.println(">> [SERIAL] CAP NHAT WALL_THRESHOLD = " + String(val));
    }
  } else if (cmd.startsWith("SET_FSTOP=")) {
    int val = cmd.substring(10).toInt();
    if (val >= 30 && val <= 150) {
      robotNav.frontStopDist = (uint16_t)val;
      serialLink.println(">> [SERIAL] CAP NHAT FRONT_STOP_DIST = " + String(val));
    }
  } else if (cmd.startsWith("SET_SSF=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 0.05f && val <= 0.9f) {
      robotNav.sideSampleFrac = val;
      serialLink.println(">> [SERIAL] CAP NHAT SIDE_SAMPLE_FRAC = " + String(val, 2));
    }
  } else if (cmd == "RESET_ENC" || cmd == "RST_ENC") {
    robotNav.resetEnc();
    serialLink.println(">> [SERIAL] DA RESET XUNG ENCODER VE 0");
  } else if (cmd == "INV_ENCL") {
    robotNav.invertEncLeft = !robotNav.invertEncLeft;
    serialLink.println(">> [SERIAL] DAO CHIEU ENCODER TRAI: " +
                       String(robotNav.invertEncLeft ? "INVERTED (-)" : "NORMAL (+)"));
  } else if (cmd == "INV_ENCR") {
    robotNav.invertEncRight = !robotNav.invertEncRight;
    serialLink.println(">> [SERIAL] DAO CHIEU ENCODER PHAI: " +
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
    serialLink.println(senseMsg);
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
      serialLink.println(">> [SERIAL] CAP NHAT PULSES_PER_CELL = " + String(val) + " xung/o (180mm)");
    }
  } else if (cmd == "STATUS" || cmd == "GET") {
    printStatus();
  }
}

static void fmtDist(char *buf, size_t n, char tag, int16_t v) {
  if (v == -1) snprintf(buf, n, "%c:NOINIT", tag);
  else if (v == -2) snprintf(buf, n, "%c:TIMEOUT", tag);
  else if (v < 0) snprintf(buf, n, "%c:OOR", tag);
  else snprintf(buf, n, "%c:%d mm", tag, v);
}

// 1: in debug lệch 2 động cơ (ENC/SPEED/PWM mỗi 100ms) thay cho dòng 3 cảm biến; 0: in 3 cảm biến
#define DEBUG_MOTOR_BALANCE 1

void CommandHandler::sendTelemetry() {
  // Bài test chạy/giữ vị trí: không in telemetry (kể cả sau khi xe dừng), dữ liệu đã nằm trong log RAM
  if (robotNav.holdRun || robotNav.logCount() > 0) return;
#if DEBUG_MOTOR_BALANCE
  static unsigned long lastMs = 0;
  static long lastEncL = 0, lastEncR = 0;
  unsigned long now = millis();
  if (now - lastMs >= 100) {
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    long encL = robotNav.getLeftEncoder();
    long encR = robotNav.getRightEncoder();
    long dL = encL - lastEncL;
    long dR = encR - lastEncR;
    lastEncL = encL;
    lastEncR = encR;
    // Tốc độ thực tế (mm/s) = xung * (180mm / pulsesPerCell) / dt
    float mmPerPulse = 180.0f / (float)robotNav.pulsesPerCell;
    Serial.printf("ENC_L:%ld | ENC_R:%ld | SPEED_L:%.1f mm/s | SPEED_R:%.1f mm/s | PWM_L:%d | PWM_R:%d\n",
                  dL, dR, dL * mmPerPulse / dt, dR * mmPerPulse / dt,
                  robotNav.currentLeftSpeed, robotNav.currentRightSpeed);
  }
  return;
#endif
  // In 3 cảm biến trên 1 dòng, ~30 Hz, không chặn
  if (millis() - _lastTelemetryTime >= 33) {
    _lastTelemetryTime = millis();
    char l[20], f[20], r[20];
    fmtDist(l, sizeof(l), 'L', robotNav._rawL);
    fmtDist(f, sizeof(f), 'F', robotNav._rawF);
    fmtDist(r, sizeof(r), 'R', robotNav._rawR);
    Serial.printf("%s | %s | %s\n", l, f, r);
  }
}

void CommandHandler::printStatus() {
  uint16_t dLeft = (unsigned)robotNav.smoothDL;
  uint16_t dFront = (unsigned)robotNav.smoothDF;
  uint16_t dRight = (unsigned)robotNav.smoothDR;

  robotNav.mpu6050.update();
  float yaw = robotNav.mpu6050.getYaw();

  String statusMsg = "--- STATS ---\n";
  statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
  statusMsg += "TOF (L/F/R): " + String(dLeft) + " / " + String(dFront) +
               " / " + String(dRight) + " mm\n";
  statusMsg += "PULSES/CELL: " + String(robotNav.pulsesPerCell) + " pulses (180mm)\n";
  statusMsg += "PARAMS -> LEFT_COMP: " + String(robotNav.leftCompensation, 1) +
               " | RIGHT_COMP: " + String(robotNav.rightCompensation, 1) +
               " | SPEED: " + String(robotNav.turnSpeed) + "\n";
  statusMsg += "ENC_KP: " + String(robotNav.encKp, 3) +
               " | YAW_KP: " + String(robotNav.hold.yawKp, 2) +
               " | WALL_KP: " + String(robotNav.hold.wallKp, 2) + "\n";
  statusMsg += "PID ACTIVE: " + String(robotNav.pidRunActive ? "YES" : "NO");

  Serial.println(statusMsg);
  serialLink.println(statusMsg);
}
