// Viewer function (DAFFODIL_VIEWER, config switch 1001) - added 2026-11-06, devices dynamic 2026-11-07.
//
// A Viewer Daffodil shows data it hears over LoRa from other devices on its 15 LEDs, using the same
// glyphs as the normal functions. It keeps a table of every device it has heard (name, packet type,
// last packet - see viewerHeard in Daffodil.ino); the web page offers those devices, and for each one
// the variables of its packet type. Which pages are shown, in what order, is the user's configuration
// - stored as JSON in Preferences ("Viewer"/"cfg"), set from the web page (formName=SetViewerConfig)
// or the serial command SetViewerConfig#<json>.
//
// Config JSON:
//   {"stale":600,         seconds after which a device's data counts as stale (shown as "no data")
//    "frost":0,           forecast temperatures at or below this are drawn blue (frost)
//    "pages":[{"t":"chinampa","d":"Chinampa1","f":"outdoorTemp"},{"t":"forecast","f":"frostMorning"},...]}
//                         t = device type (VIEWER_TYPES), d = device name ("" or missing = the most
//                         recently heard device of that type), f = field id of that type
//                         (VIEWER_FIELDS), s = slot dot (0 none, 1 = LED 4, 2 = LED 14; ignored on
//                         temperature pages, whose digits use those LEDs)
//   Configs saved before 2026-11-07 ({"src":..,"pages":[{"f":"outdoorTemp"}]}) still load: a page
//   without "t" takes the first field with that id, and "src" becomes the Chinampa pages' device.
//
// The glyph is fixed by the field, not configured: temperature digits, the level tower, or the
// flow F. Forecast pages blink so a prediction is never mistaken for a current reading.
//
// Adding a device type: a struct with a LoRa size no other packet uses, a VIEWER_TYPE_ entry, its
// fields below, a branch in processLora() that calls viewerRecord(), and its values in
// viewerFieldValue().
#ifndef DAFFODIL_VIEWER_CONFIG_H
#define DAFFODIL_VIEWER_CONFIG_H
#include <Arduino.h>

#define VIEWER_MAX_PAGES 8
#define VIEWER_MAX_DEVICES 6  // heard-device table, in RTC memory (~270 B each)
#define VIEWER_NAME_LEN 17    // device names: 16 chars + terminator (ChinampaData/LangleyData are 16)

#define VIEWER_GLYPH_TEMPERATURE 1
#define VIEWER_GLYPH_LEVEL 2
#define VIEWER_GLYPH_FLOW 3

// Device types = what kind of packet the device sends. Index into VIEWER_TYPES.
#define VIEWER_TYPE_FORECAST 0  // Annabelle's WeatherForecastUpdate - one "device", no name
#define VIEWER_TYPE_CHINAMPA 1  // ChinampaData
#define VIEWER_TYPE_DAFFODIL 2  // DigitalStablesData (every Daffodil, e.g. TopTank)
#define VIEWER_TYPE_LANGLEY 3   // LangleyData

struct ViewerType {
  const char *id;    // in the config JSON
  const char *name;  // shown on the web page
};

static const ViewerType VIEWER_TYPES[] = {
  { "forecast", "Forecast" },
  { "chinampa", "Chinampa" },
  { "daffodil", "Daffodil" },
  { "langley", "Langley" },
};
#define VIEWER_TYPE_COUNT (sizeof(VIEWER_TYPES) / sizeof(VIEWER_TYPES[0]))

// Field indexes - must match the order of VIEWER_FIELDS below.
enum {
  VF_FROST_MORNING,
  VF_FORECAST_NEXT,
  VF_CH_OUTDOOR_TEMP,
  VF_CH_SUMP_LEVEL,
  VF_CH_FISH_LEVEL,
  VF_CH_FISH_OUTFLOW,
  VF_CH_PUMP_FLOW,
  VF_DA_OUTDOOR_TEMP,
  VF_DA_WATER_TEMP,
  VF_DA_TROUGH_LEVEL,
  VF_DA_FLOW1,
  VF_DA_FLOW2,
  VF_LA_TEMP,
};

struct ViewerField {
  uint8_t type;
  const char *id;     // unique within its type
  const char *label;  // without the device - the page picks the device first
  uint8_t glyph;
};

// Forecast and Chinampa come first: a pre-2026-11-07 page has no type and takes the first match.
static const ViewerField VIEWER_FIELDS[] = {
  { VIEWER_TYPE_FORECAST, "frostMorning", "Morning low (03-09h)", VIEWER_GLYPH_TEMPERATURE },
  { VIEWER_TYPE_FORECAST, "forecastNext", "Next 3 hours", VIEWER_GLYPH_TEMPERATURE },
  { VIEWER_TYPE_CHINAMPA, "outdoorTemp", "Outdoor temperature", VIEWER_GLYPH_TEMPERATURE },
  { VIEWER_TYPE_CHINAMPA, "sumpLevel", "Sump trough level", VIEWER_GLYPH_LEVEL },
  { VIEWER_TYPE_CHINAMPA, "fishLevel", "Fish tank level", VIEWER_GLYPH_LEVEL },
  { VIEWER_TYPE_CHINAMPA, "fishOutflow", "Fish tank outflow", VIEWER_GLYPH_FLOW },
  { VIEWER_TYPE_CHINAMPA, "pumpFlow", "Pump flow", VIEWER_GLYPH_FLOW },
  { VIEWER_TYPE_DAFFODIL, "outdoorTemp", "Outdoor temperature", VIEWER_GLYPH_TEMPERATURE },
  { VIEWER_TYPE_DAFFODIL, "waterTemp", "Water temperature", VIEWER_GLYPH_TEMPERATURE },
  { VIEWER_TYPE_DAFFODIL, "troughLevel", "Trough level", VIEWER_GLYPH_LEVEL },
  { VIEWER_TYPE_DAFFODIL, "flow1", "Flow 1", VIEWER_GLYPH_FLOW },
  { VIEWER_TYPE_DAFFODIL, "flow2", "Flow 2", VIEWER_GLYPH_FLOW },
  { VIEWER_TYPE_LANGLEY, "temperature", "Temperature", VIEWER_GLYPH_TEMPERATURE },
};
#define VIEWER_FIELD_COUNT (sizeof(VIEWER_FIELDS) / sizeof(VIEWER_FIELDS[0]))

struct ViewerConfig {
  uint8_t count = 0;
  uint8_t field[VIEWER_MAX_PAGES] = { 0 };
  uint8_t slot[VIEWER_MAX_PAGES] = { 0 };
  char device[VIEWER_MAX_PAGES][VIEWER_NAME_LEN] = {};  // "" = most recently heard of the type
  uint16_t staleSec = 600;
  float frost = 0;
};

// No pages: the user adds them from the devices the Viewer hears (nothing is assumed to exist).
#define VIEWER_DEFAULT_CONFIG "{\"stale\":600,\"frost\":0,\"pages\":[]}"

#endif
