#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include "BugWiper.h"

// Initialisation
void sdLoggerInit();

bool sdLoggerOpenLogLater();

// Card Detect / State Handling
void sdLoggerHandleCard();

// Logging
void sdLoggerLog(unsigned long t, BW_MODE mode, int32_t pos, int32_t speed, uint32_t current, float voltage, float ntcTemp, const char* event);
// Status
bool sdLoggerAvailable();

#endif