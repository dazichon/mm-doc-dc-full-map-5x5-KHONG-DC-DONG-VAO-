#pragma once

// ###########################################################################################################
// #  1. PID ĐI THẲNG  (Encoder + Gyro + Wall). Cả ba chỉ có Kp, tổng correction = enc + yaw + wall           #
// ###########################################################################################################

// ----- 1a. ENCODER PID: cân 2 bánh -----
#define ENC_STRAIGHT_KP 0.13f // PWM hiệu chỉnh = KP * (|ENC_L| - |ENC_R|) [XUNG, tích lũy]. Đơn vị PWM/xung: 0.05 -> lệch 20 xung = 1 PWM. KHÔNG dùng số lớn (1.8 = 1.8 PWM cho mỗi xung lệch -> bão hòa)
#define ENC_MAX_CORR    10.0f // Hiệu chỉnh tối đa (PWM) cho mỗi bánh, để encoder chỉ cân tốc độ 2 bánh

// ----- 1b. GYRO PID: giữ hướng -----
#define HOLD_YAW_KP 0.65f      // Giữ thẳng hướng bằng gyro. Tăng: xe sửa lệch hướng mạnh hơn, quá cao thì ngoằn ngoèo. 0: tắt
                              // (giới hạn correction yawMaxCorr = 20 PWM nằm trong RobotNav.h)

// ----- 1c. WALL PID: sửa lệch ngang theo VL53 trái/phải -----
#define WALL_ENABLE    1       // 1: cộng wall correction nhỏ vào correction tổng (Encoder PID + yaw vẫn chạy). 0: tắt
#define WALL_KP        0.3f    // PWM cho mỗi mm lệch so với baseline. 10mm lệch -> 3 PWM
#define WALL_MAX_CORR  8.0f    // Giới hạn |wall correction| (PWM) - nhỏ hơn yaw (20) và encoder (10)
#define WALL_BASE_L    173.0f  // VL53 trái khi xe ở giữa (mm)
#define WALL_BASE_R    168.0f  // VL53 phải khi xe ở giữa (mm)
#define WALL_VALID_MAX 190.0f  // Đọc xa hơn mức này (mm) = hở tường / nhìn sang ô kế tiếp -> bỏ qua bên đó

// ----- 1d. VL53 trước -----
#define HOLD_USE_FRONT    0       // 0: không dùng VL53 trước (đang trả -3), không phanh theo tường trước
#define HOLD_TARGET_FRONT 150.0f  // Tăng: xe dừng xa tường trước hơn. Giảm: xe dừng sát tường trước hơn

// ###########################################################################################################
// #  2. CÂN BẰNG 2 ĐỘNG CƠ (áp dụng CHUNG cho mọi tốc độ & mọi chế độ)                                      #
// ###########################################################################################################
// Đơn vị %: công suất bánh = PWM tính ra * (1 + giá trị/100), nên bù cùng tỉ lệ ở mọi tốc độ.
// Xe lệch sang PHẢI (bánh trái mạnh hơn): giảm MOTOR_TRIM_LEFT (số âm) hoặc tăng MOTOR_TRIM_RIGHT.
// Xe lệch sang TRÁI (bánh phải mạnh hơn): ngược lại. Ví dụ -5 = bánh đó yếu đi 5%.
#define MOTOR_TRIM_LEFT   -6.0f   // % bù bánh trái. Tăng: bánh trái mạnh hơn. Giảm (âm): bánh trái yếu đi
#define MOTOR_TRIM_RIGHT  0.0f   // % bù bánh phải. Tăng: bánh phải mạnh hơn. Giảm (âm): bánh phải yếu đi

// ###########################################################################################################
// #  3. CHIỀU DÀI Ô & SỐ Ô CHẠY TEST                                                                        #
// ###########################################################################################################
#define CELL_PULSES      4300   // Số xung encoder của 1 ô (đổi số này khi đo lại quãng đường 1 ô)
#define TEST_CELLS       1      // Số ô chạy trong bài test (tổng xung = CELL_PULSES * TEST_CELLS)
#define SEQ_CELL_PULSES  4300   // Xung/ô trong sequence - giá trị TẠM để test, chưa calib
#define CALIB_REV_PWM        80     // Calib ô (chế độ 2): PWM khi lùi sát vách cụt. Tăng: chạm vách mạnh hơn
#define CALIB_REV_MS         400    // Calib ô: thời gian lùi cố định (ms), rồi dừng, calib gyro về 0 độ
#define CALIB_FWD_PULSES     1500   // Calib ô: xung tiến từ vách tới tâm ô (không tính vào ô kế tiếp)
#define STOP_LEAD_PULSES 280   // Chỉ chế độ 1 (cell-to-cell): phanh sớm chừng này xung trước đích để bù quãng xe trôi sau phanh (đo ~280). 0: tắt

// ###########################################################################################################
// #  4. CHỌN CHẾ ĐỘ TEST (bấm nút BUTTON, chờ TURN_DELAY_MS rồi chạy)                                       #
// ###########################################################################################################
#define TURN_DELAY_MS    2000   // Chờ sau khi bấm nút
#define SEQ_MODE         1      // 2: ngõ cụt (1 ô -> dừng -> quay 180 -> lùi sát vách -> tiến CALIB_FWD_PULSES tới tâm ô). 1: cell-to-cell (1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng). 4: test CORNER state machine (1 ô -> cua trái -> 1 ô -> STOP). 3: calibration encoder (chạy thẳng 4 ô liên tục, in ENC_L/ENC_R/AVG_TICKS/TICKS_PER_CELL). 0: chỉ quay tại chỗ TURN_ANGLE

// ###########################################################################################################
// #  5. CUA KHI ĐANG CHẠY (Corner Controller)                                                               #
// ###########################################################################################################
#define CORNER_SPEED       60.0f // PWM bánh ngoài khi cua (cua chậm)
#define CORNER_INNER_RATIO 0.45f // PWM bánh trong / bánh ngoài ở độ cong tối đa. Nhỏ hơn = cua gắt hơn
#define CORNER_DECEL_ZONE  30.0f // độ cuối: độ cong giảm dần về 0
#define CORNER_TOLERANCE   2.0f  // coi là tới target khi sai số yaw <= độ này
#define CORNER_LEAD        0.05f // s: giảm cong sớm theo tốc độ quay, chống overshoot

// ###########################################################################################################
// #  6. QUAY TẠI CHỖ                                                                                        #
// ###########################################################################################################
#define TURN_ANGLE       90.0f  // Góc quay (độ): 90 hoặc 180
#define TURN_RIGHT       1      // 1: bấm nút quay PHẢI. 0: quay TRÁI
#define TURN_MAX_PWM     70.0f  // PWM lớn nhất khi quay
#define TURN_MIN_PWM     42.0f  // PWM nhỏ nhất khi gần target (đủ thắng ma sát)
#define TURN_DECEL_ZONE  35.0f  // Giảm tốc tuyến tính trong ngần này độ cuối
#define TURN_TOLERANCE   0.8f   // Dừng khi sai số yaw <= độ này
#define TURN_BRAKE_LEAD  0.04f  // s: phanh sớm = tốc độ quay (độ/s) * giá trị này. Tăng nếu vẫn quay quá, giảm nếu quay thiếu
#define TURN_LEFT_YAW_SIGN -1.0f // Dấu thay đổi yaw khi quay TRÁI. Log đi thẳng: lệch phải => yaw tăng => quay trái làm yaw giảm (-1). Nếu quay sai chiều, đổi thành +1

// ###########################################################################################################
// #  7. KHỞI ĐỘNG                                                                                           #
// ###########################################################################################################
#define HOLD_AUTO_START     0     // 1: tự chạy giữ thẳng sau HOLD_START_DELAY_MS khi bật nguồn; 0: chờ lệnh "HOLD" qua Serial ("STOP" để dừng)
#define HOLD_START_DELAY_MS 3000  // Thời gian chờ sau khi bật nguồn trước khi PID chạy (ms). Tăng: có nhiều thời gian đặt xe hơn

// ###########################################################################################################
// #  8. TEST LỆCH 2 ĐỘNG CƠ (open-loop, tắt PID)                                                            #
// ###########################################################################################################
// 1: TẮT PID, cấp cùng 1 PWM cho 2 bánh chạy thẳng TEST_PULSES xung rồi dừng, in chênh lệch encoder. 0: chạy bình thường
#define TEST_MOTOR_DRIVE 0
#define TEST_PWM    100    // PWM cấp cho cả 2 bánh khi test (đã nhân MOTOR_TRIM). Đổi số này để test ở tốc độ khác
#define TEST_PULSES 9000   // Dừng khi trung bình xung 2 bánh đạt giá trị này
