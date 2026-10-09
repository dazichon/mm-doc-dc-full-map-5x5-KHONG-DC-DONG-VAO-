#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H

#include <Arduino.h>

// Thay thế BLEManager: toàn bộ log/telemetry in ra Serial, lệnh nhận từ Serial (kết thúc bằng '\n')
class SerialLink {
public:
    void begin(unsigned long baud = 115200);
    bool isConnected() const { return true; }

    void print(const String &msg);
    void println(const String &msg);

    bool hasCommand();
    String readCommand();

private:
    String _rxBuffer;
    String _lastCommand;
    bool _commandReady = false;
};

extern SerialLink serialLink;

#endif
