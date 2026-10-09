#include "SerialLink.h"

SerialLink serialLink;

void SerialLink::begin(unsigned long baud) {
    Serial.begin(baud);
}

void SerialLink::print(const String &msg) {
    Serial.print(msg);
}

void SerialLink::println(const String &msg) {
    Serial.println(msg);
}

bool SerialLink::hasCommand() {
    while (!_commandReady && Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            _rxBuffer.trim();
            if (_rxBuffer.length() > 0) {
                _lastCommand = _rxBuffer;
                _commandReady = true;
            }
            _rxBuffer = "";
        } else {
            _rxBuffer += c;
        }
    }
    return _commandReady;
}

String SerialLink::readCommand() {
    _commandReady = false;
    String cmd = _lastCommand;
    _lastCommand = "";
    return cmd;
}
