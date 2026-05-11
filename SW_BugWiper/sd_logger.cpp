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
  SD_FILE_ERROR,
  SD_MOUNTED
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
    
    case SD_MOUNTED:
      DEBUG_INFO("[SD] MOUNTED");
      break;

    default:
      DEBUG_WARNING("[SD] State unknown");
      break;
  }
}

// ================= DIR SCAN (maxNum + count) =================
struct DirStats {
  int maxNum;
  int count;
};

static DirStats scanLogDir() {
  DirStats s{ .maxNum = -1, .count = 0 };

  File dir = SD.open(BASE_DIR);
  if (!dir) return s;

  while (true) {
    File e = dir.openNextFile();
    if (!e) break;

    if (!e.isDirectory()) {
      s.count++;

      int num;
      // SD-Lib liefert meist nur den Dateinamen (8.3), passt hier: "log_0000.csv"
      if (sscanf(e.name(), "log_%4d.csv", &num) == 1) {
        if (num > s.maxNum) s.maxNum = num;
      }
    }
    e.close();
  }

  dir.close();
  return s;
}

// ================= LAST FILE VALIDATION =================
static bool fileHasLessThanNLines(const char* path, int minLines) {
  File f = SD.open(path, FILE_READ);
  if (!f) return true;              // wenn nicht lesbar, behandeln wie "ungültig"

  int lines = 0;
  bool anyData = false;

  while (f.available()) {
    int c = f.read();
    anyData = true;
    if (c == '\n') {
      lines++;
      if (lines >= minLines) {      // früh abbrechen -> schnell
        f.close();
        return false;               // hat genug Zeilen
      }
    }
  }
  f.close();

  if (!anyData) return true;        // 0 Bytes
  return (lines < minLines);
}

static bool overwriteFile(const char* path) {
  if (SD.exists(path)) SD.remove(path);
  logFile = SD.open(path, FILE_WRITE);   // neu erstellen
  return (bool)logFile;
}

// ================= INIT =================

static bool sdMountOnly() {
  DEBUG_INFO("[SD] mount start");

  if (!isCardInserted()) {
    setSDStatus(SD_NOT_PRESENT);
    return false;
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!SD.begin(PIN_CS)) {
    setSDStatus(SD_INIT_FAILED);
    return false;
  }

  if (!SD.exists(BASE_DIR)) {
    SD.mkdir(BASE_DIR);
  }

  setSDStatus(SD_MOUNTED);
  return true;
}

static bool sdFindAndOpenLogFile() {
  if (!isCardInserted()) {
    setSDStatus(SD_NOT_PRESENT);
    return false;
  }

  if (sdStatus != SD_MOUNTED) {
    setSDStatus(SD_INIT_FAILED);
    return false;
  }

  // 1) Scan: count + max
  DirStats st = scanLogDir();
  DEBUG_INFO("[SD] files inside folder: " + String(st.count) + " maxNum: " + String(st.maxNum));

  int fileNumber = 0;

  if (st.maxNum >= 0) {
    char lastPath[64];
    snprintf(lastPath, sizeof(lastPath), "%s/log_%04d.csv", BASE_DIR, st.maxNum);

    bool tooShort = fileHasLessThanNLines(lastPath, 20);
    if (tooShort) {
      fileNumber = st.maxNum;
      if (!overwriteFile(lastPath)) {
        setSDStatus(SD_FILE_ERROR);
        return false;
      }
      DEBUG_INFO("[SD] overwrite last file: " + String(fileNumber));
    } else {
      fileNumber = st.maxNum + 1;
    }
  } else {
    fileNumber = 0;
  }

  if (!logFile) {
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s/log_%04d.csv", BASE_DIR, fileNumber);

    logFile = SD.open(filepath, FILE_WRITE);
    if (!logFile) {
      setSDStatus(SD_FILE_ERROR);
      return false;
    }
  }

  DEBUG_INFO("[SD] log-file number: " + String(fileNumber));
  logFile.println("time,state,position,speed,motor_current,BatteryVoltage_filtered");

  setSDStatus(SD_OK);
  return true;
}

static void initSD() {
  if (!sdMountOnly()) return;
  sdFindAndOpenLogFile();
}

// ================= PUBLIC API =================
void sdLoggerInit() {
  pinMode(SD_DETECT_PIN, INPUT_PULLUP);
  delay(50);

  lastCardState = isCardInserted();
  if (lastCardState) {
    DEBUG_INFO("[SD] PRESENT");
    sdMountOnly();   // <-- nur Phase 1
  } else {
    setSDStatus(SD_NOT_PRESENT);
  }
}

bool sdLoggerOpenLogLater() {
  return sdFindAndOpenLogFile();
}


void sdLoggerHandleCard() {
  const bool current = isCardInserted();

  if (current && !lastCardState) {
    DEBUG_INFO("[SD] Card inserted");
    if (sdMountOnly()) {
      sdFindAndOpenLogFile();   // Phase 2 immediately
    }
  }

  if (!current && lastCardState) {
    DEBUG_INFO("[SD] Card removed");

    if (logFile) {
      logFile.close();
      logFile = File();
    }

    SD.end();
    setSDStatus(SD_NOT_PRESENT);
  }

  lastCardState = current;
}

static bool writeCsvEscaped(File& f, const char* s) {
  if (!s) s = "";

  if (f.write('"') != 1) return false;

  for (const char* p = s; *p; ++p) {
    if (*p == '"') {
      // Quote verdoppeln: ""
      if (f.write('"') != 1) return false;
      if (f.write('"') != 1) return false;
    } else if (*p == '\r') {
      // optional ignorieren
    } else {
      if (f.write((uint8_t)*p) != 1) return false;
    }
  }

  if (f.write('"') != 1) return false;
  return true;
}

void sdLoggerLog(unsigned long t, BW_MODE mode, int32_t pos, int32_t speed, uint32_t current, float voltage, float ntcTemp, const char* event) {


  if (sdStatus != SD_OK) return;
  if (!isCardInserted()) return;

  const char* modeStr = bw_ModeToString(mode);

  if (!logFile.printf("%lu,%s,%d,%d,%d,%.1f,%.1f,",
                      t, modeStr, pos, speed, current, voltage, ntcTemp)) {
    setSDStatus(SD_FILE_ERROR);
    logFile.close();
    return;
  }

  if (!writeCsvEscaped(logFile, event)) {
      setSDStatus(SD_FILE_ERROR);
      logFile.close();
      return;
    }

  if (logFile.write('\n') != 1) {
    setSDStatus(SD_FILE_ERROR);
    logFile.close();
    return;
  }

  logFile.flush();
}

bool sdLoggerAvailable() {
  return sdStatus == SD_OK;
}