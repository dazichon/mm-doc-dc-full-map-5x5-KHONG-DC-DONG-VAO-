#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "SerialLink.h"
#include "RobotNav.h"

class CommandHandler {
public:
    CommandHandler();

    void update();
    void processSerialCommands();
    void sendTelemetry();
    void printStatus();

private:
    unsigned long _lastTelemetryTime;
};

extern CommandHandler commandHandler;

#endif
