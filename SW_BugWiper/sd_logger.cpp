#include "sd_logger.h"
#include "my_debug.h"
#include <SPI.h>
#include <SD.h>

// ================= SPI PINS =================
static const int PIN_SCK = 41;
static const int PIN_MISO = 42;
static const int PIN_MOSI = 40;
static const int PIN_CS = 39;

// ================= CONFIG =================
static const char* BASE_DIR = "/BW_logs";

// ================= STATE =================
enum SDStatus {
  SD_UNKNOWN,
  SD_OK,
  SD_NOT_PRESENT,
  SD_INIT_FAILED,
  SD_FILE_ERROR
};

static SDStatus sdStatus = SD_UNKNOWN;
static File logFile;
static bool lastCardState = false;

// ================= CARD DETECT =================
#define SD_DETECT_PIN 2

static bool isCardInserted() {
  return digitalRead(SD_DETECT_PIN) == LOW;
}

// ================= STATUS HANDLING =================
static void setSDStatus(SDStatus newStatus) {
  if (sdStatus == newStatus) return;

  sdStatus = newStatus;

  switch (sdStatus) {
    case SD_OK:
      DEBUG_INFO("[SD] OK");
      break;

    case SD_NOT_PRESENT:
      DEBUG_INFO("[SD] NOT_PRESENT");
      break;

    case SD_INIT_FAILED:
      DEBUG_ERROR("[SD] INIT_FAILED");
      break;

    case SD_FILE_ERROR:
      DEBUG_ERROR("[SD] FILE_ERROR");
      break;

    default:
      DEBUG_WARNING("[SD] State unknown");
      break;
  }
}

// ================= FILE NUMBER FINDER =================
static int findNextFileNumber() {
  char filepath[64];

  for (int i = 0; i <= 9999; i++) {
    snprintf(filepath, sizeof(filepath), "%s/log_%04d.csv", BASE_DIR, i);

    if (!SD.exists(filepath)) {
      return i;
    }
  }
  return -1;
}

// ================= INIT =================
static void initSD() {
  DEBUG_INFO("[SD] Init start");
  if (!isCardInserted()) {
    setSDStatus(SD_NOT_PRESENT);
    return;
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!SD.begin(PIN_CS)) {
    setSDStatus(SD_INIT_FAILED);
    return;
  }

  if (!SD.exists(BASE_DIR)) {
    SD.mkdir(BASE_DIR);
  }

  int fileNumber = findNextFileNumber();
  if (fileNumber < 0) {
    setSDStatus(SD_FILE_ERROR);
    return;
  }

  char filepath[64];
  snprintf(filepath, sizeof(filepath), "%s/log_%04d.csv", BASE_DIR, fileNumber);

  logFile = SD.open(filepath, FILE_WRITE);
  DEBUG_INFO("[SD] Log-Datei Nummer: " + String(fileNumber));
  if (!logFile) {
    setSDStatus(SD_FILE_ERROR);
    return;
  }

  logFile.println("time;state;position;speed;motor_current;battery_voltage");
  logFile.flush();

  setSDStatus(SD_OK);
}

// ================= PUBLIC API =================
void sdLoggerInit() {
  pinMode(SD_DETECT_PIN, INPUT_PULLUP);
  delay(50);
  lastCardState = isCardInserted();
  if (lastCardState) {
    DEBUG_INFO("[SD] PRESENT");
    initSD();
  } else {
    setSDStatus(SD_NOT_PRESENT);
  }
}

void sdLoggerHandleCard() {
  bool current = isCardInserted();

  if (current && sdStatus != SD_OK) {
    initSD();
  }


  if (!current && sdStatus == SD_OK) {
    if (logFile) logFile.close();
    setSDStatus(SD_NOT_PRESENT);
  }

  lastCardState = current;
}

void sdLoggerLog(unsigned long t, BW_MODE mode, int32_t pos, int32_t speed, double current, double voltage) {

  if (sdStatus != SD_OK) return;
  if (!isCardInserted()) return;

  const char* modeStr = bwModeToString(mode);

  if (!logFile.printf("%lu;%s;%d;%d;%.2f;%.1f\n",
                      t, modeStr, pos, speed, current, voltage)) {
    setSDStatus(SD_FILE_ERROR);
    logFile.close();
  } else {
    logFile.flush();
  }
}

bool sdLoggerAvailable() {
  return sdStatus == SD_OK;
}