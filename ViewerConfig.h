// Viewer function (DAFFODIL_VIEWER, config switch 1001) - added 2026-11-06.
//
// A Viewer Daffodil shows data it hears over LoRa from other devices (today: ChinampaData and the
// weather forecast) on its 15 LEDs, using the same glyphs as the normal functions. Which data
// items are shown, in what order, is the user's configuration - stored as JSON in Preferences
// ("Viewer"/"cfg"), set from the web page (formName=SetViewerConfig) or the serial command
// SetViewerConfig#<json>.
//
// Config JSON:
//   {"src":"",            ChinampaData devicename to accept ("" = any Chinampa)
//    "stale":600,         seconds after which a source's data counts as stale (shown as "no data")
//    "frost":0,           forecast temperatures at or below this are drawn blue (frost)
//    "pages":[{"f":"outdoorTemp"},{"f":"sumpLevel","s":1},...]}
//                         f = field id from VIEWER_FIELDS, s = slot dot (0 none, 1 = LED 4,
//                         2 = LED 14; ignored on temperature pages, whose digits use those LEDs)
//
// The glyph is fixed by the field, not configured: temperature digits, the level tower, or the
// flow F. Forecast pages blink so a prediction is never mistaken for a current reading.
#ifndef DAFFODIL_VIEWER_CONFIG_H
#define DAFFODIL_VIEWER_CONFIG_H
#include <Arduino.h>

#define VIEWER_MAX_PAGES 8

#define VIEWER_GLYPH_TEMPERATURE 1
#define VIEWER_GLYPH_LEVEL 2
#define VIEWER_GLYPH_FLOW 3

#define VIEWER_SOURCE_CHINAMPA 1
#define VIEWER_SOURCE_FORECAST 2

// Field indexes - must match the order of VIEWER_FIELDS below.
#define VF_OUTDOOR_TEMP 0
#define VF_SUMP_LEVEL 1
#define VF_FISH_LEVEL 2
#define VF_FISH_OUTFLOW 3
#define VF_PUMP_FLOW 4
#define VF_FROST_MORNING 5
#define VF_FORECAST_NEXT 6

struct ViewerField {
  const char *id;
  const char *label;
  uint8_t source;
  uint8_t glyph;
};

static const ViewerField VIEWER_FIELDS[] = {
  { "outdoorTemp", "Chinampa outdoor temperature", VIEWER_SOURCE_CHINAMPA, VIEWER_GLYPH_TEMPERATURE },
  { "sumpLevel", "Chinampa sump trough level", VIEWER_SOURCE_CHINAMPA, VIEWER_GLYPH_LEVEL },
  { "fishLevel", "Chinampa fish tank level", VIEWER_SOURCE_CHINAMPA, VIEWER_GLYPH_LEVEL },
  { "fishOutflow", "Chinampa fish tank outflow", VIEWER_SOURCE_CHINAMPA, VIEWER_GLYPH_FLOW },
  { "pumpFlow", "Chinampa pump flow", VIEWER_SOURCE_CHINAMPA, VIEWER_GLYPH_FLOW },
  { "frostMorning", "Forecast: lowest temperature next morning (03-09h)", VIEWER_SOURCE_FORECAST, VIEWER_GLYPH_TEMPERATURE },
  { "forecastNext", "Forecast: temperature in the next 3 hours", VIEWER_SOURCE_FORECAST, VIEWER_GLYPH_TEMPERATURE },
};
#define VIEWER_FIELD_COUNT (sizeof(VIEWER_FIELDS) / sizeof(VIEWER_FIELDS[0]))

struct ViewerConfig {
  uint8_t count = 0;
  uint8_t field[VIEWER_MAX_PAGES] = { 0 };
  uint8_t slot[VIEWER_MAX_PAGES] = { 0 };
  char source[17] = "";
  uint16_t staleSec = 600;
  float frost = 0;
};

#define VIEWER_DEFAULT_CONFIG "{\"src\":\"\",\"stale\":600,\"frost\":0,\"pages\":[{\"f\":\"outdoorTemp\"},{\"f\":\"sumpLevel\",\"s\":1},{\"f\":\"fishLevel\",\"s\":2},{\"f\":\"frostMorning\"}]}"

#endif
