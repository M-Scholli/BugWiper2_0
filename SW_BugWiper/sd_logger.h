#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include "BugWiper.h"

// Initialisierung
void sdLoggerInit();

// Card Detect / State Handling
void sdLoggerHandleCard();

// Logging
void sdLoggerLog(unsigned long t, BW_MODE mode, int32_t pos, int32_t speed, double current, double voltage);

// Status
bool sdLoggerAvailable();

#endif