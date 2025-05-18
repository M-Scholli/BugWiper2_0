
#define DEBUG_LEVEL_INFO      3
#define DEBUG_LEVEL_WARNING   2
#define DEBUG_LEVEL_ERROR     1
#define DEBUG_LEVEL_NO_TRACE  0

#define NO_DEBUG  0
#define DEBUG_LEVEL DEBUG_LEVEL_INFO
#define DEBUG_HW  USBSerial
#if defined(NOTRACE)
// Empty macro
#define DEBUG_INIT(...)    { }
#define DEBUG_INFO(...)    { }
#define DEBUG_WARNING(...) { }               
#define DEBUG_ERROR(...)   { }

#else
#define DEBUG_INIT(...)   { Serial.begin(__VA_ARGS__); }
// Trace compilation depends on TRACE_LEVEL value
#if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
#define DEBUG_INFO(...)   { Serial.print("-I- "); Serial.print(__VA_ARGS__ ); Serial.println("");}
#else
#define DEBUG_INFO(...)   { }
#endif

#if (DEBUG_LEVEL >= DEBUG_LEVEL_WARNING)
#define DEBUG_WARNING(...) { Serial.print("-W- "); Serial.print(__VA_ARGS__ ); Serial.println("");}
#else
#define DEBUG_WARNING(...) { }
#endif

#if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR)
#define DEBUG_ERROR(...)   { Serial.printf("-E- "); Serial.print(__VA_ARGS__ ); Serial.println("");}
#else
#define DEBUG_ERROR(...)   { }
#endif

#endif