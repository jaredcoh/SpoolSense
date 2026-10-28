#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <Preferences.h>
#include <stdarg.h>
#include "HX711.h"
#include "LCD_Test.h"

#define DEG2RAD 0.0174532925f

UWORD Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
UWORD *BlackImage;

CST816S touch(6, 7, 13, 5); // sda, scl, rst, irq
Preferences prefs;

// --------------------------------------------------
// HX711 load-cell wiring
// --------------------------------------------------
// Waveshare ESP32-S3 Touch LCD 1.28 breakout connector wiring:
// HX711 DT / DOUT -> GPIO18
// HX711 SCK / CLK -> GPIO21
// HX711 VCC       -> 3V3
// HX711 GND       -> GND
#define HX711_DOUT 18
#define HX711_SCK  21

unsigned long lastHx711DebugPrint = 0;

HX711 spoolScale;

// This value will need final calibration with your 1000g calibration weight.
// If weight goes negative when you add weight, change this to positive or swap A+ / A-.
float hx711CalibrationFactor = -350.0f;
bool hx711Ready = false;
unsigned long lastHx711StatusPrintMs = 0;
const unsigned long HX711_READ_INTERVAL_MS = 250;
const int HX711_AVERAGE_SAMPLES = 5;

// Redraw live weight screens twice per second.
// This is intentionally independent from the user-facing WT. REFRESH setting
// so the HX711 proof-of-concept feels live while testing the load cell.
const unsigned long LIVE_WEIGHT_DISPLAY_UPDATE_MS = 100;

// Print HX711 status continuously during bench testing.
// This prevents the Serial Monitor from only showing Waveshare display messages.
const unsigned long HX711_DEBUG_PRINT_INTERVAL_MS = 1000;
const unsigned long HX711_GPIO_DIAG_PRINT_INTERVAL_MS = 2000;
unsigned long lastHx711GpioDiagPrint = 0;

// HX711 DOUT naturally goes HIGH between samples. Do not treat one or two
// not-ready checks as wiring failure. Only declare the scale lost after
// it has stayed not-ready for several seconds.
const unsigned long HX711_NOT_READY_GRACE_MS = 3000;
unsigned long hx711NotReadyStartedMs = 0;
bool hx711StartupTareDone = false;
bool hx711EverReady = false;
int lastHx711MeasuredGrams = 0;
bool lastHx711MeasurementValid = false;

// HX711 calibration/debug values.
// Raw/tared counts are independent of hx711CalibrationFactor and are the
// safest way to confirm whether the load cell is physically responding.
long prevHx711RawCounts = 0;
long currentHx711RawCounts = 0;
long prevHx711TaredCounts = 0;
long currentHx711TaredCounts = 0;
float prevHx711CalculatedGrams = 0.0f;
float currentHx711CalculatedGrams = 0.0f;
bool hx711RawDebugValid = false;

// Put your known calibration mass here. The serial monitor will print a
// suggested factor using this value, so calibration is based on the known
// weight rather than guesswork.
const float HX711_KNOWN_CAL_WEIGHT_GRAMS = 1000.0f;


// Use printf-style debug because the Waveshare driver messages are visible
// in your Serial Monitor while Serial.println() messages are not always routed
// to the same USB console on ESP32-S3 boards.
void debugPrintf(const char* fmt, ...) {
  char buffer[256];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  // On this Waveshare ESP32-S3 board profile, Serial0 is the route that
  // appears in Arduino Serial Monitor. Do not also print through printf here,
  // because that duplicates every SpoolSense debug line.
  Serial0.print(buffer);
  Serial0.flush();
}


void saveSettings();
void loadSettings();
void noteUserInteraction();
void wakeDisplay();
void updateDimAndSleepIfNeeded();
void updateDimLevelPreviewIfNeeded();
void startDimLevelPreview();
void stopDimLevelPreview(bool redraw);
void drawBootScreen(const char* statusText);
UWORD applyUiBrightness(UWORD c);
void initHx711();
void updateHx711WeightInternal();
void printHx711BenchDebug(bool forcePrint);
void printHx711GpioDiagnostics(bool forcePrint);
void runHx711PinStartupDiagnostic();

// --------------------------------------------------
// UI Colors
// --------------------------------------------------
#define UI_BG        BLACK
#define UI_WHITE     WHITE
#define UI_SUBTEXT   0xBDF7
#define UI_DARKGREY  0x4208
#define UI_MIDGREY   0x8410
#define UI_CYAN      CYAN
#define UI_YELLOW    YELLOW
#define UI_ORANGE    0xFD20
#define UI_RED       RED

void drawCurrentPage();

// --------------------------------------------------
// Normal pages
// --------------------------------------------------
enum UiPage {
  PAGE_WEIGHT = 0,
  PAGE_TOOL = 1,
  PAGE_MATERIAL = 2,
  PAGE_COLOR = 3
};

const int NORMAL_PAGE_COUNT = 4;
int currentPage = PAGE_WEIGHT;

// --------------------------------------------------
// Toolheads
// Always assume up to 16 toolheads.
// --------------------------------------------------
const int MAX_TOOLS = 16;
int selectedTool = 0;       // Starts as T1
int activeToolCount = MAX_TOOLS;    // Always assume 16 toolheads

void getToolName(int tool, char* buffer) {
  sprintf(buffer, "T%d", tool + 1);
}

// --------------------------------------------------
// Materials
// --------------------------------------------------
const char* filamentTypes[] = {
  "PLA",
  "PLA+",
  "PLA SILK",
  "PLA MATTE",
  "PLA-CF",
  "PETG",
  "PETG-CF",
  "TPU",
  "ABS",
  "ASA",
  "PA",
  "PA-CF",
  "PC",
  "PC-CF",
  "PVA",
  "HIPS",
  "SUPPORT",
  "WOOD",
  "METAL",
  "CUSTOM"
};

const int FILAMENT_COUNT = 20;

bool filamentEnabled[FILAMENT_COUNT] = {
  true,   // PLA
  true,   // PLA+
  true,   // PLA SILK
  true,   // PLA MATTE
  true,   // PLA-CF
  true,   // PETG
  true,   // PETG-CF
  true,   // TPU
  true,   // ABS
  true,   // ASA
  true,   // PA
  true,   // PA-CF
  false,  // PC
  false,  // PC-CF
  false,  // PVA
  false,  // HIPS
  true,   // SUPPORT
  false,  // WOOD
  false,  // METAL
  false   // CUSTOM
};

int filamentIndex[MAX_TOOLS] = {
  0, 1, 2, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

int nextEnabledFilamentIndex(int currentIndex) {
  for (int step = 1; step <= FILAMENT_COUNT; step++) {
    int next = (currentIndex + step) % FILAMENT_COUNT;

    if (filamentEnabled[next]) {
      return next;
    }
  }

  // Safety fallback: keep PLA enabled if everything was disabled.
  filamentEnabled[0] = true;
  return 0;
}

// --------------------------------------------------
// Global material color
// This color is NOT tied to T1/T2/T3/T4.
// Toggling toolheads will not affect color.
// --------------------------------------------------
uint8_t colorHue = 0;
uint8_t colorSat = 0;
uint8_t colorBrightness = 255;

// 0 = HEX, 1 = RGB, 2 = HSV
int colorReadoutMode = 0;

// --------------------------------------------------
// Global settings
// --------------------------------------------------
int spoolSizeOptions[] = {250, 500, 750, 1000, 2000};
const int SPOOL_SIZE_OPTION_COUNT = 5;
int spoolSizeIndex = 3; // 1000g

unsigned long displayUpdateOptions[] = {2000, 5000, 10000, 20000, 30000};
const int DISPLAY_UPDATE_OPTION_COUNT = 5;
int displayUpdateIndex = 1; // 5s default

int filamentResolutionOptions[] = {1, 5, 10, 50};
const int FILAMENT_RESOLUTION_OPTION_COUNT = 4;
int filamentResolutionIndex = 1; // 5g default

int brightnessOptions[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const int BRIGHTNESS_OPTION_COUNT = 10;
int brightnessIndex = 9; // 100%

int dimOptionsSeconds[] = {0, 30, 120, 300};
const int DIM_OPTION_COUNT = 4;
int dimIndex = 0; // never

int dimPercentOptions[] = {10, 20, 30, 40, 50};
const int DIM_PERCENT_OPTION_COUNT = 5;
int dimPercentIndex = 1; // 20%

int sleepOptionsSeconds[] = {0, 300, 900, 1800};
const int SLEEP_OPTION_COUNT = 4;
int sleepIndex = 0; // never

// Grams-only UI. Percent display was intentionally removed.
const int unitsMode = 0;

int startupPagePreference = PAGE_WEIGHT;

// Low-material warning thresholds.
// OFF = disabled. Otherwise, threshold is percent remaining, in 5% increments.
// Priority order when drawing: red, orange, yellow.
const int LOW_COLOR_OFF = 0;
int yellowThresholdPct = 20;
int orangeThresholdPct = 10;
int redThresholdPct = 5;

// Calibration action display only for now.
// Real HX711 tare/calibration should hook into this later.
int calibrationMode = 0; // 0 TARE, 1 CAL 100G, 2 RESET

int spoolMaxGrams() {
  return spoolSizeOptions[spoolSizeIndex];
}

void formatSpoolSizeLabel(char* buffer) {
  int grams = spoolMaxGrams();

  if (grams >= 1000 && grams % 1000 == 0) {
    sprintf(buffer, "%dkg", grams / 1000);
  } else {
    sprintf(buffer, "%dg", grams);
  }
}

unsigned long currentDisplayUpdateMs() {
  return displayUpdateOptions[displayUpdateIndex];
}

int currentFilamentResolutionGrams() {
  return filamentResolutionOptions[filamentResolutionIndex];
}

int clampRawWeight(int grams) {
  if (grams < 0) {
    grams = 0;
  }

  return grams;
}

int roundWeightToResolution(int grams) {
  grams = clampRawWeight(grams);

  int res = currentFilamentResolutionGrams();

  if (res <= 1) {
    return grams;
  }

  int rounded = ((grams + res / 2) / res) * res;

  return clampRawWeight(rounded);
}

bool shouldShowLessThanStep(int rawGrams) {
  rawGrams = clampRawWeight(rawGrams);

  int res = currentFilamentResolutionGrams();

  if (res <= 1) {
    return false;
  }

  return rawGrams > 0 && roundWeightToResolution(rawGrams) == 0;
}

int getDisplayWeightGrams(int rawGrams) {
  return roundWeightToResolution(rawGrams);
}

// --------------------------------------------------
// Settings screen state
// --------------------------------------------------
bool settingsOpen = false;
int previousNormalPage = PAGE_WEIGHT;

enum SettingsView {
  SETTINGS_MAIN = 0,
  SETTINGS_FILAMENTS = 1
};

SettingsView settingsView = SETTINGS_MAIN;

enum SettingItem {
  SETTING_SPOOL = 0,
  SETTING_CALIBRATION = 1,
  SETTING_UPDATE = 2,
  SETTING_RESOLUTION = 3,
  SETTING_BRIGHTNESS = 4,
  SETTING_DIM = 5,
  SETTING_DIM_PERCENT = 6,
  SETTING_SLEEP = 7,
  SETTING_TOOL_SELECT = 8,
  SETTING_FILAMENTS = 9,
  SETTING_STARTUP = 10,
  SETTING_YELLOW_ALERT = 11,
  SETTING_ORANGE_ALERT = 12,
  SETTING_RED_ALERT = 13,
  SETTING_HELP = 14
};

const int SETTINGS_COUNT = 15;
int settingsIndex = SETTING_SPOOL;
int filamentListOffset = 0;
unsigned long lastSettingsTickerRedraw = 0;
const unsigned long SETTINGS_TICKER_MS = 1000;

// Top access zone to open settings.
// I kept it as a top strip so it does not fight the color picker or normal controls.
const int SETTINGS_HOTSPOT_Y = 36;

// Show this hint only briefly after boot.
unsigned long uiStartMs = 0;
bool advHintCleared = false;
const unsigned long ADV_SETTINGS_HINT_MS = 5000;

unsigned long lastUserInteractionMs = 0;
bool uiDimmed = false;
bool uiSleeping = false;

// Temporary dim preview used when the user changes DIM LEVEL.
// This lets the user see the selected dim percentage immediately, then
// automatically returns to normal brightness without waiting for idle dimming.
bool dimLevelPreviewActive = false;
unsigned long dimLevelPreviewStartedMs = 0;
unsigned long lastDimLevelPreviewRedraw = 0;
const unsigned long DIM_LEVEL_PREVIEW_MS = 5000;

// Two-step tare confirmation. First tap arms it; second tap confirms.
bool tareConfirmArmed = false;
unsigned long tareConfirmStartedMs = 0;
const unsigned long TARE_CONFIRM_TIMEOUT_MS = 5000;

bool pointInSettingsHotspot(int x, int y) {
  return y <= SETTINGS_HOTSPOT_Y;
}

bool showAdvancedSettingsHint() {
  // Do not show or allow this hint on the color screen.
  // Screen 4 uses the top arc for brightness/color selection.
  return !settingsOpen &&
         currentPage != PAGE_COLOR &&
         millis() - uiStartMs <= ADV_SETTINGS_HINT_MS;
}

void drawAdvancedSettingsHint() {
  if (!showAdvancedSettingsHint()) {
    return;
  }

  drawCenteredTextOnPage(
    0,
    6,
    "tap at top",
    &Font12,
    UI_BG,
    UI_SUBTEXT
  );

  drawCenteredTextOnPage(
    0,
    20,
    "for more settings",
    &Font12,
    UI_BG,
    UI_SUBTEXT
  );
}

bool pointInBackButton(int x, int y) {
  return x >= 60 && x <= 180 && y >= 190;
}

const int SETTINGS_RIGHT_NEXT_HIT_X = 160;
const int SETTINGS_SIDE_HIT_Y_TOP = 60;
const int SETTINGS_SIDE_HIT_Y_BOTTOM = 180;

// Visual chevron placement only. Touch zones stay generous so the arrows
// look closer to the bezel without becoming harder to tap.
const int SETTINGS_LEFT_ARROW_X = 12;
const int SETTINGS_RIGHT_ARROW_X = 212;
const int SETTINGS_ARROW_Y = 118;

bool pointInSettingsRightNextButton(int x, int y) {
  return x > SETTINGS_RIGHT_NEXT_HIT_X &&
         y > SETTINGS_SIDE_HIT_Y_TOP &&
         y < SETTINGS_SIDE_HIT_Y_BOTTOM;
}

// --------------------------------------------------
// Current displayed weights
// For this first proof-of-concept, one HX711/load cell feeds the selected tool display.
// --------------------------------------------------
int fakeWeightGrams[MAX_TOOLS] = {
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

unsigned long lastWeightInternalUpdate = 0;
unsigned long lastDisplayWeightUpdate = 0;

const unsigned long WEIGHT_INTERNAL_UPDATE_MS = 5000;

// --------------------------------------------------
// Touch settings
// --------------------------------------------------
unsigned long lastTouchAction = 0;

const unsigned long TOUCH_DEBOUNCE_MS = 250;
const unsigned long NO_REDRAW_AFTER_TOUCH_MS = 450;

bool touchTracking = false;
bool touchMoved = false;

int touchStartX = 0;
int touchStartY = 0;
int touchLastX = 0;
int touchLastY = 0;

const int SWIPE_THRESHOLD_X = 45;
const int SWIPE_MAX_VERTICAL_DRIFT = 85;
const int TAP_MAX_MOVE = 25;

// --------------------------------------------------
// Button geometry
// --------------------------------------------------
const int TOGGLE_X = 63;
const int TOGGLE_Y = 146;
const int TOGGLE_W = 114;
const int TOGGLE_H = 34;

const int MINUS_X = 47;
const int PLUS_X = 127;
const int PM_Y = 142;
const int PM_W = 66;
const int PM_H = 34;

bool pointInToggleButton(int x, int y) {
  return x >= TOGGLE_X &&
         x <= TOGGLE_X + TOGGLE_W &&
         y >= TOGGLE_Y &&
         y <= TOGGLE_Y + TOGGLE_H;
}

bool pointInMinusButton(int x, int y) {
  return x >= MINUS_X &&
         x <= MINUS_X + PM_W &&
         y >= PM_Y &&
         y <= PM_Y + PM_H;
}

bool pointInPlusButton(int x, int y) {
  return x >= PLUS_X &&
         x <= PLUS_X + PM_W &&
         y >= PM_Y &&
         y <= PM_Y + PM_H;
}

// Screen 2: tare / define-weight buttons
const int TARE_SET_BTN_W = 56;
const int TARE_SET_BTN_H = 22;
const int TARE_SET_X0 = 18;
const int TARE_SET_X1 = 92;
const int TARE_SET_X2 = 166;
const int TARE_SET_MINUS_Y = 100;
const int TARE_SET_PLUS_Y = 130;
const int TARE_BTN_X = 80;
const int TARE_BTN_Y = 162;
const int TARE_BTN_W = 80;
const int TARE_BTN_H = 24;

bool pointInRect(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x <= rx + rw && y >= ry && y <= ry + rh;
}

bool pointInTareButton(int x, int y) {
  return pointInRect(x, y, TARE_BTN_X, TARE_BTN_Y, TARE_BTN_W, TARE_BTN_H);
}

// --------------------------------------------------
// Color helpers
// --------------------------------------------------
UWORD rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void rgb565ToRgb(UWORD c, uint8_t* r, uint8_t* g, uint8_t* b) {
  *r = ((c >> 11) & 0x1F) << 3;
  *g = ((c >> 5) & 0x3F) << 2;
  *b = (c & 0x1F) << 3;
}

void hsvToRgb(uint8_t hue, uint8_t sat, uint8_t val, uint8_t* outR, uint8_t* outG, uint8_t* outB) {
  float h = hue * 360.0f / 255.0f;
  float s = sat / 255.0f;
  float v = val / 255.0f;

  float c = v * s;
  float x = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
  float m = v - c;

  float rf = 0;
  float gf = 0;
  float bf = 0;

  if (h < 60) {
    rf = c; gf = x; bf = 0;
  } else if (h < 120) {
    rf = x; gf = c; bf = 0;
  } else if (h < 180) {
    rf = 0; gf = c; bf = x;
  } else if (h < 240) {
    rf = 0; gf = x; bf = c;
  } else if (h < 300) {
    rf = x; gf = 0; bf = c;
  } else {
    rf = c; gf = 0; bf = x;
  }

  *outR = (uint8_t)((rf + m) * 255.0f);
  *outG = (uint8_t)((gf + m) * 255.0f);
  *outB = (uint8_t)((bf + m) * 255.0f);
}

UWORD hsvTo565(uint8_t hue, uint8_t sat, uint8_t val) {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  hsvToRgb(hue, sat, val, &r, &g, &b);

  return rgb565(r, g, b);
}

UWORD getActiveColor() {
  return applyUiBrightness(hsvTo565(colorHue, colorSat, colorBrightness));
}

void getActiveRgb(uint8_t* r, uint8_t* g, uint8_t* b) {
  hsvToRgb(colorHue, colorSat, colorBrightness, r, g, b);
}

bool isDarkColor(UWORD c) {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  rgb565ToRgb(c, &r, &g, &b);

  int brightness = (r * 299 + g * 587 + b * 114) / 1000;

  return brightness < 95;
}

UWORD applyUiBrightness(UWORD c) {
  if (c == BLACK) {
    return BLACK;
  }

  uint8_t r;
  uint8_t g;
  uint8_t b;

  rgb565ToRgb(c, &r, &g, &b);

  int percent = brightnessOptions[brightnessIndex];

  if (uiDimmed || dimLevelPreviewActive) {
    percent = min(percent, dimPercentOptions[dimPercentIndex]);
  }

  r = (uint8_t)((int)r * percent / 100);
  g = (uint8_t)((int)g * percent / 100);
  b = (uint8_t)((int)b * percent / 100);

  return rgb565(r, g, b);
}

bool recentlyTouched() {
  return millis() - lastTouchAction < NO_REDRAW_AFTER_TOUCH_MS;
}

int getRemainingPercentForTool(int tool) {
  int displayGrams = getDisplayWeightGrams(fakeWeightGrams[tool]);

  return map(displayGrams, 0, spoolMaxGrams(), 0, 100);
}

int getArcFillPercentForTool(int tool) {
  return getRemainingPercentForTool(tool);
}

int thresholdToGrams(int pct) {
  return (spoolMaxGrams() * pct) / 100;
}

void formatWeightForCurrentUnits(int grams, char* buffer) {
  int raw = clampRawWeight(grams);
  int displayGrams = getDisplayWeightGrams(raw);

  if (shouldShowLessThanStep(raw)) {
    sprintf(buffer, "<%dg", currentFilamentResolutionGrams());
  } else {
    sprintf(buffer, "%dg", displayGrams);
  }
}

UWORD getLowMaterialColorForTool(int tool) {
  int pct = getRemainingPercentForTool(tool);

  if (redThresholdPct > 0 && pct <= redThresholdPct) {
    return UI_RED;
  }

  if (orangeThresholdPct > 0 && pct <= orangeThresholdPct) {
    return UI_ORANGE;
  }

  if (yellowThresholdPct > 0 && pct <= yellowThresholdPct) {
    return UI_YELLOW;
  }

  return getActiveColor();
}

UWORD getWeightNumberColorForTool(int tool) {
  int pct = getRemainingPercentForTool(tool);

  if (redThresholdPct > 0 && pct <= redThresholdPct) {
    return UI_RED;
  }

  if (orangeThresholdPct > 0 && pct <= orangeThresholdPct) {
    return UI_ORANGE;
  }

  if (yellowThresholdPct > 0 && pct <= yellowThresholdPct) {
    return UI_YELLOW;
  }

  // Normal grams/readout should stay white.
  // The selected filament color should affect the arc/accent, not the grams.
  return UI_WHITE;
}

int* thresholdPointerForSetting(int setting) {
  if (setting == SETTING_YELLOW_ALERT) return &yellowThresholdPct;
  if (setting == SETTING_ORANGE_ALERT) return &orangeThresholdPct;
  if (setting == SETTING_RED_ALERT) return &redThresholdPct;
  return NULL;
}

void cycleThresholdPercent(int* pct) {
  if (*pct == LOW_COLOR_OFF) {
    *pct = 5;
    return;
  }

  *pct += 5;

  if (*pct > 95) {
    *pct = LOW_COLOR_OFF;
  }
}

// --------------------------------------------------
// Text helpers
// --------------------------------------------------
void drawCenteredTextOnPage(int pageOffsetX, int y, const char* text, sFONT* font, UWORD bg, UWORD fg) {
  int textWidth = strlen(text) * font->Width;
  int x = pageOffsetX + (240 - textWidth) / 2;

  Paint_DrawString_EN(x, y, text, font, bg, applyUiBrightness(fg));
}

void drawOutlinedCenteredTextOnPage(int pageOffsetX, int y, const char* text, sFONT* font, UWORD bg, UWORD fg, UWORD outline) {
  int textWidth = strlen(text) * font->Width;
  int x = pageOffsetX + (240 - textWidth) / 2;

  Paint_DrawString_EN(x - 1, y, text, font, bg, applyUiBrightness(outline));
  Paint_DrawString_EN(x + 1, y, text, font, bg, applyUiBrightness(outline));
  Paint_DrawString_EN(x, y - 1, text, font, bg, applyUiBrightness(outline));
  Paint_DrawString_EN(x, y + 1, text, font, bg, applyUiBrightness(outline));

  Paint_DrawString_EN(x, y, text, font, bg, applyUiBrightness(fg));
}

void drawSmartCenteredTextOnPage(int pageOffsetX, int y, const char* text, sFONT* font, UWORD bg, UWORD fg) {
  if (isDarkColor(fg)) {
    drawOutlinedCenteredTextOnPage(pageOffsetX, y, text, font, bg, fg, WHITE);
  } else {
    drawCenteredTextOnPage(pageOffsetX, y, text, font, bg, fg);
  }
}


// --------------------------------------------------
// Settings arc geometry
// The title/header live on the top bezel.
// The settings progress indicator lives on one shared bottom arc for every
// settings item so it never shifts from screen to screen.
// --------------------------------------------------
const int SETTINGS_ARC_CX = 120;
const int SETTINGS_ARC_CY = 120;
const int SETTINGS_PROGRESS_RADIUS = 117;
const int SETTINGS_PROGRESS_TICK_HALF_DEG = 5;

// Bottom progress arc. This is intentionally one shared global geometry.
// It is wide, but stops before the side controls and BACK label get crowded.
// The order runs visually left-to-right as the user advances settings.
const float SETTINGS_PROGRESS_START_DEG = 160.0f;
const float SETTINGS_PROGRESS_END_DEG = 20.0f;

// Top curved-text geometry.
// On this display, 270 degrees is top-center.
const int SETTINGS_HEADER_RADIUS = 108;
const int SETTINGS_TITLE_RADIUS_SHORT = 108;
const int SETTINGS_TITLE_RADIUS_LONG = 108;
const float SETTINGS_HEADER_SPAN_DEG = 82.0f;
const float SETTINGS_TITLE_SPAN_SHORT_DEG = 108.0f;
const float SETTINGS_TITLE_SPAN_LONG_DEG = 124.0f;

// Forward declaration. The actual glyph table is below.
const uint8_t* getBlockGlyph(char c);

// Invisible spacing character used by the block-font renderer.
// This gives labels like "1. PLA" a tighter gap without changing normal spaces elsewhere.
const char BLOCK_HALF_SPACE = '~';

// --------------------------------------------------
// Rotated curved settings text helpers
// Draws small bitmap letters along the upper bezel and rotates each character
// to follow the arc tangent. This avoids relying on rotated Waveshare fonts.
// --------------------------------------------------
void drawRotatedBlockChar(int centerX, int centerY, char c, int scale, float rotationDeg, UWORD color) {
  const uint8_t* glyph = getBlockGlyph(c);

  float rot = rotationDeg * DEG2RAD;
  float cosA = cos(rot);
  float sinA = sin(rot);

  float glyphW = 5.0f * scale;
  float glyphH = 7.0f * scale;

  UWORD drawColor = applyUiBrightness(color);

  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 5; col++) {
      if (!(glyph[row] & (1 << (4 - col)))) {
        continue;
      }

      for (int sy = 0; sy < scale; sy++) {
        for (int sx = 0; sx < scale; sx++) {
          float localX = col * scale + sx - glyphW / 2.0f;
          float localY = row * scale + sy - glyphH / 2.0f;

          int px = centerX + (int)(localX * cosA - localY * sinA);
          int py = centerY + (int)(localX * sinA + localY * cosA);

          if (px >= 0 && px < 240 && py >= 0 && py < 240) {
            Paint_DrawPoint(px, py, drawColor, DOT_PIXEL_1X1, DOT_FILL_RIGHTUP);
          }
        }
      }
    }
  }
}

float curvedBlockSpacingDeg(int scale, int radius) {
  float spacingPx = 6.0f * scale;
  return spacingPx * 180.0f / (PI * radius);
}

void drawCurvedRotatedTextOnPage(
  int pageOffsetX,
  const char* text,
  int scale,
  int radius,
  float centerAngleDeg,
  float maxSpanDeg,
  UWORD color
) {
  int len = strlen(text);

  if (len <= 0) {
    return;
  }

  int cx = pageOffsetX + SETTINGS_ARC_CX;
  int cy = SETTINGS_ARC_CY;

  float spacingDeg = curvedBlockSpacingDeg(scale, radius);

  if (len > 1) {
    float naturalSpan = spacingDeg * (len - 1);

    if (naturalSpan > maxSpanDeg) {
      spacingDeg = maxSpanDeg / (len - 1);
    }
  }

  float totalSpan = spacingDeg * (len - 1);
  float firstAngle = centerAngleDeg - totalSpan / 2.0f;

  for (int i = 0; i < len; i++) {
    float angle = firstAngle + spacingDeg * i;
    float rad = angle * DEG2RAD;

    int x = cx + (int)(cos(rad) * radius);
    int y = cy + (int)(sin(rad) * radius);

    // Tangent rotation. At top-center this is horizontal/upright.
    // Characters lean naturally as they move toward the left/right bezel.
    float rotationDeg = angle + 90.0f;

    drawRotatedBlockChar(x, y, text[i], scale, rotationDeg, color);
  }
}

void drawSettingsCurvedHeader() {
  // Deprecated visually: settings and title now draw on one shared curved line.
}

void drawSettingsCurvedTitle(const char* title, UWORD color) {
  // Deprecated visually: settings and title now draw on one shared curved line.
}

void drawSettingsCurvedCombinedTitle(const char* title, UWORD color) {
  char combined[48];
  snprintf(combined, sizeof(combined), "SETTINGS - %s", title);

  int len = strlen(combined);
  int scale = 2;
  int radius = SETTINGS_HEADER_RADIUS;
  float maxSpan = 166.0f;

  // Keep labels large enough to read, including SETTINGS-QUESTIONS/ISSUES?.
  // If a future title is extremely long, shrink only that title text, but
  // keep the same radius so it still sits close to the bezel.
  if (len > 30) {
    scale = 1;
    maxSpan = 168.0f;
  }

  drawCurvedRotatedTextOnPage(0, combined, scale, radius, 270.0f, maxSpan, color);
}

// --------------------------------------------------
// Oversized block text renderer
// Used where Waveshare Font24 still looks too small.
// Supports A-Z, 0-9, hyphen, percent, and space.
// --------------------------------------------------
const uint8_t* getBlockGlyph(char c) {
  static const uint8_t GLYPH_SPACE[7] = {0,0,0,0,0,0,0};
  // Narrow dash for material names like PA-CF and PETG-CF.
  // Uses the center 3 columns instead of all 5 so hyphenated names feel tighter.
  static const uint8_t GLYPH_DASH[7]  = {0,0,0,14,0,0,0};
  static const uint8_t GLYPH_PLUS[7]  = {0,4,4,31,4,4,0};
  static const uint8_t GLYPH_DOT[7]   = {0,0,0,0,0,12,12};
  static const uint8_t GLYPH_SLASH[7] = {1,2,2,4,8,8,16};
  static const uint8_t GLYPH_QUESTION[7] = {14,17,1,2,4,0,4};
  static const uint8_t GLYPH_PERCENT[7]= {17,2,4,8,16,17,0};

  // Lowercase unit glyphs used by settings values like 500g, 1kg, and 5s.
  // These are checked before the generic a-z to A-Z conversion below.
  static const uint8_t GLYPH_LOWER_G[7] = {0,0,14,17,15,1,14};
  static const uint8_t GLYPH_LOWER_S[7] = {0,0,15,16,14,1,30};
  static const uint8_t GLYPH_LOWER_K[7] = {16,16,18,20,24,20,18};

  static const uint8_t GLYPH_0[7] = {14,17,19,21,25,17,14};
  static const uint8_t GLYPH_1[7] = {4,12,4,4,4,4,14};
  static const uint8_t GLYPH_2[7] = {14,17,1,2,4,8,31};
  static const uint8_t GLYPH_3[7] = {30,1,1,14,1,1,30};
  static const uint8_t GLYPH_4[7] = {2,6,10,18,31,2,2};
  static const uint8_t GLYPH_5[7] = {31,16,16,30,1,1,30};
  static const uint8_t GLYPH_6[7] = {14,16,16,30,17,17,14};
  static const uint8_t GLYPH_7[7] = {31,1,2,4,8,8,8};
  static const uint8_t GLYPH_8[7] = {14,17,17,14,17,17,14};
  static const uint8_t GLYPH_9[7] = {14,17,17,15,1,1,14};

  static const uint8_t GLYPH_A[7] = {14,17,17,31,17,17,17};
  static const uint8_t GLYPH_B[7] = {30,17,17,30,17,17,30};
  static const uint8_t GLYPH_C[7] = {14,17,16,16,16,17,14};
  static const uint8_t GLYPH_D[7] = {30,17,17,17,17,17,30};
  static const uint8_t GLYPH_E[7] = {31,16,16,30,16,16,31};
  static const uint8_t GLYPH_F[7] = {31,16,16,30,16,16,16};
  static const uint8_t GLYPH_G[7] = {14,17,16,23,17,17,14};
  static const uint8_t GLYPH_H[7] = {17,17,17,31,17,17,17};
  static const uint8_t GLYPH_I[7] = {14,4,4,4,4,4,14};
  static const uint8_t GLYPH_J[7] = {7,2,2,2,18,18,12};
  static const uint8_t GLYPH_K[7] = {17,18,20,24,20,18,17};
  static const uint8_t GLYPH_L[7] = {16,16,16,16,16,16,31};
  static const uint8_t GLYPH_M[7] = {17,27,21,21,17,17,17};
  static const uint8_t GLYPH_N[7] = {17,25,21,19,17,17,17};
  static const uint8_t GLYPH_O[7] = {14,17,17,17,17,17,14};
  static const uint8_t GLYPH_P[7] = {30,17,17,30,16,16,16};
  static const uint8_t GLYPH_Q[7] = {14,17,17,17,21,18,13};
  static const uint8_t GLYPH_R[7] = {30,17,17,30,20,18,17};
  static const uint8_t GLYPH_S[7] = {15,16,16,14,1,1,30};
  static const uint8_t GLYPH_T[7] = {31,4,4,4,4,4,4};
  static const uint8_t GLYPH_U[7] = {17,17,17,17,17,17,14};
  static const uint8_t GLYPH_V[7] = {17,17,17,17,17,10,4};
  static const uint8_t GLYPH_W[7] = {17,17,17,21,21,21,10};
  static const uint8_t GLYPH_X[7] = {17,17,10,4,10,17,17};
  static const uint8_t GLYPH_Y[7] = {17,17,10,4,4,4,4};
  static const uint8_t GLYPH_Z[7] = {31,1,2,4,8,16,31};

  if (c == 'g') return GLYPH_LOWER_G;
  if (c == 's') return GLYPH_LOWER_S;
  if (c == 'k') return GLYPH_LOWER_K;

  if (c >= 'a' && c <= 'z') {
    c -= 32;
  }

  switch (c) {
    case '0': return GLYPH_0;
    case '1': return GLYPH_1;
    case '2': return GLYPH_2;
    case '3': return GLYPH_3;
    case '4': return GLYPH_4;
    case '5': return GLYPH_5;
    case '6': return GLYPH_6;
    case '7': return GLYPH_7;
    case '8': return GLYPH_8;
    case '9': return GLYPH_9;

    case 'A': return GLYPH_A;
    case 'B': return GLYPH_B;
    case 'C': return GLYPH_C;
    case 'D': return GLYPH_D;
    case 'E': return GLYPH_E;
    case 'F': return GLYPH_F;
    case 'G': return GLYPH_G;
    case 'H': return GLYPH_H;
    case 'I': return GLYPH_I;
    case 'J': return GLYPH_J;
    case 'K': return GLYPH_K;
    case 'L': return GLYPH_L;
    case 'M': return GLYPH_M;
    case 'N': return GLYPH_N;
    case 'O': return GLYPH_O;
    case 'P': return GLYPH_P;
    case 'Q': return GLYPH_Q;
    case 'R': return GLYPH_R;
    case 'S': return GLYPH_S;
    case 'T': return GLYPH_T;
    case 'U': return GLYPH_U;
    case 'V': return GLYPH_V;
    case 'W': return GLYPH_W;
    case 'X': return GLYPH_X;
    case 'Y': return GLYPH_Y;
    case 'Z': return GLYPH_Z;

    case '-': return GLYPH_DASH;
    case '+': return GLYPH_PLUS;
    case '.': return GLYPH_DOT;
    case '/': return GLYPH_SLASH;
    case '?': return GLYPH_QUESTION;
    case '%': return GLYPH_PERCENT;
    case ' ': return GLYPH_SPACE;
  }

  return GLYPH_SPACE;
}

int blockCharAdvance(char c, int scale) {
  if (c == BLOCK_HALF_SPACE) {
    return 3 * scale;
  }

  // Hyphens looked too wide in names like PA-CF. Give them a compact advance
  // so the dash is about half the normal character width.
  if (c == '-') {
    return 3 * scale;
  }

  return 6 * scale;
}

int blockGlyphVisibleWidth(char c, int scale) {
  if (c == '-') {
    return 3 * scale;
  }

  if (c == BLOCK_HALF_SPACE) {
    return 0;
  }

  return 5 * scale;
}

int blockTextWidth(const char* text, int scale) {
  int cursorX = 0;
  int maxX = 0;

  for (int i = 0; i < (int)strlen(text); i++) {
    if (text[i] == BLOCK_HALF_SPACE) {
      cursorX += blockCharAdvance(text[i], scale);
      continue;
    }

    maxX = cursorX + blockGlyphVisibleWidth(text[i], scale);
    cursorX += blockCharAdvance(text[i], scale);
  }

  return maxX;
}

void drawBlockChar(int x, int y, char c, int scale, UWORD color) {
  const uint8_t* glyph = getBlockGlyph(c);

  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 5; col++) {
      if (glyph[row] & (1 << (4 - col))) {
        Paint_DrawRectangle(
          x + col * scale,
          y + row * scale,
          x + col * scale + scale - 1,
          y + row * scale + scale - 1,
          applyUiBrightness(color),
          DOT_PIXEL_1X1,
          DRAW_FILL_FULL
        );
      }
    }
  }
}

void drawBlockText(int x, int y, const char* text, int scale, UWORD color) {
  int cursorX = x;

  for (int i = 0; i < (int)strlen(text); i++) {
    if (text[i] == BLOCK_HALF_SPACE) {
      cursorX += blockCharAdvance(text[i], scale);
      continue;
    }

    drawBlockChar(cursorX, y, text[i], scale, color);
    cursorX += blockCharAdvance(text[i], scale);
  }
}

void drawCenteredBlockTextOnPage(int pageOffsetX, int y, const char* text, int scale, UWORD color) {
  int w = blockTextWidth(text, scale);
  int x = pageOffsetX + (240 - w) / 2;

  drawBlockText(x, y, text, scale, color);
}

int chooseBlockScaleForText(const char* text, int preferredScale, int maxWidth) {
  int scale = preferredScale;

  while (scale > 2 && blockTextWidth(text, scale) > maxWidth) {
    scale--;
  }

  return scale;
}

// --------------------------------------------------
// Arc helpers
// --------------------------------------------------
void drawArc(int cx, int cy, int r, int startDeg, int endDeg, UWORD color, int thickness) {
  for (int angle = startDeg; angle <= endDeg; angle++) {
    float rad = angle * DEG2RAD;

    for (int t = 0; t < thickness; t++) {
      int rr = r - t;
      int x = cx + (int)(cos(rad) * rr);
      int y = cy + (int)(sin(rad) * rr);

      Paint_DrawPoint(x, y, applyUiBrightness(color), DOT_PIXEL_1X1, DOT_FILL_RIGHTUP);
    }
  }
}

void drawTopProgressArc(int pageOffsetX, int percent, UWORD activeColor) {
  int startAngle = 170;
  int endAngle = 370;
  int span = endAngle - startAngle;

  int cx = pageOffsetX + 120;
  int cy = 120;
  int radius = 109;
  int thickness = 4;

  drawArc(cx, cy, radius, startAngle, endAngle, UI_DARKGREY, thickness);

  int activeEnd = startAngle + map(percent, 0, 100, 0, span);

  if (isDarkColor(activeColor)) {
    drawArc(cx, cy, radius, startAngle, activeEnd, WHITE, thickness + 2);
  }

  drawArc(cx, cy, radius, startAngle, activeEnd, activeColor, thickness);

  float endRad = activeEnd * DEG2RAD;

  int markerRadius = radius - 2;
  int ex = cx + (int)(cos(endRad) * markerRadius);
  int ey = cy + (int)(sin(endRad) * markerRadius);

  Paint_DrawCircle(ex, ey, 9, applyUiBrightness(WHITE), DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(ex, ey, 6, applyUiBrightness(activeColor), DOT_PIXEL_1X1, DRAW_FILL_FULL);

  if (isDarkColor(activeColor)) {
    Paint_DrawCircle(ex, ey, 6, applyUiBrightness(WHITE), DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(ex, ey, 4, applyUiBrightness(activeColor), DOT_PIXEL_1X1, DRAW_FILL_FULL);
  }
}

// --------------------------------------------------
// Bottom navigation arcs
// --------------------------------------------------
int navArcAngles[4] = {
  132, 104, 76, 48
};

void drawNavigationArcs(int activePage, int pageOffsetX) {
  int cx = pageOffsetX + 120;
  int cy = 120;
  int radius = 109;

  for (int i = 0; i < 4; i++) {
    int centerAngle = navArcAngles[i];
    int startAngle = centerAngle - 7;
    int endAngle = centerAngle + 7;

    UWORD c = (i == activePage) ? UI_WHITE : UI_SUBTEXT;
    int thickness = (i == activePage) ? 5 : 3;

    drawArc(cx, cy, radius, startAngle, endAngle, c, thickness);
  }
}

int getTappedNavigationArc(int x, int y) {
  int cx = 120;
  int cy = 120;

  int dx = x - cx;
  int dy = y - cy;

  float dist = sqrt((float)(dx * dx + dy * dy));

  if (dist < 88 || dist > 124) {
    return -1;
  }

  float angle = atan2((float)dy, (float)dx) / DEG2RAD;

  if (angle < 0) {
    angle += 360;
  }

  for (int i = 0; i < 4; i++) {
    int centerAngle = navArcAngles[i];

    if (angle >= centerAngle - 13 && angle <= centerAngle + 13) {
      return i;
    }
  }

  return -1;
}

// --------------------------------------------------
// Buttons
// --------------------------------------------------
void drawButton(int pageOffsetX, int x, int y, int w, int h, const char* label) {
  int realX = pageOffsetX + x;

  Paint_DrawRectangle(realX, y, realX + w, y + h, applyUiBrightness(UI_DARKGREY), DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawRectangle(realX, y, realX + w, y + h, applyUiBrightness(UI_WHITE), DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

  sFONT* buttonFont = &Font16;
  int textWidth = strlen(label) * buttonFont->Width;
  int tx = realX + (w - textWidth) / 2;
  int ty = y + (h - buttonFont->Height) / 2;

  if (tx < realX + 2) {
    tx = realX + 2;
  }

  Paint_DrawString_EN(tx, ty, label, buttonFont, applyUiBrightness(UI_DARKGREY), applyUiBrightness(UI_WHITE));
}

void drawToggleButton(int pageOffsetX) {
  drawButton(pageOffsetX, TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H, "TOGGLE");
}

int getTareIncrementCount() {
  int res = currentFilamentResolutionGrams();

  if (res == 50) {
    return 2;
  }

  return 3;
}

void getTareIncrements(int* values, int* count) {
  int res = currentFilamentResolutionGrams();

  if (res == 1) {
    values[0] = 1;
    values[1] = 10;
    values[2] = 100;
    *count = 3;
  }
  else if (res == 5) {
    values[0] = 5;
    values[1] = 50;
    values[2] = 250;
    *count = 3;
  }
  else if (res == 10) {
    values[0] = 10;
    values[1] = 100;
    values[2] = 500;
    *count = 3;
  }
  else {
    values[0] = 50;
    values[1] = 250;
    *count = 2;
  }
}

int getTareButtonXForIndex(int index, int count) {
  if (count == 2) {
    return index == 0 ? 52 : 132;
  }

  if (index == 0) return TARE_SET_X0;
  if (index == 1) return TARE_SET_X1;
  return TARE_SET_X2;
}

void makeSignedButtonLabel(char* label, int value, bool positive) {
  if (positive) {
    sprintf(label, "+%d", value);
  } else {
    sprintf(label, "-%d", value);
  }
}

void drawTareSetButtons(int pageOffsetX) {
  int values[3];
  int count = 0;
  char label[8];

  getTareIncrements(values, &count);

  for (int i = 0; i < count; i++) {
    int x = getTareButtonXForIndex(i, count);

    makeSignedButtonLabel(label, values[i], false);
    drawButton(pageOffsetX, x, TARE_SET_MINUS_Y, TARE_SET_BTN_W, TARE_SET_BTN_H, label);

    makeSignedButtonLabel(label, values[i], true);
    drawButton(pageOffsetX, x, TARE_SET_PLUS_Y, TARE_SET_BTN_W, TARE_SET_BTN_H, label);
  }

  drawButton(pageOffsetX, TARE_BTN_X, TARE_BTN_Y, TARE_BTN_W, TARE_BTN_H, tareConfirmArmed ? "CONFIRM" : "TARE");
}

void drawPlusMinusButtons(const char* minusLabel = "-", const char* plusLabel = "+") {
  drawButton(0, MINUS_X, PM_Y, PM_W, PM_H, minusLabel);
  drawButton(0, PLUS_X, PM_Y, PM_W, PM_H, plusLabel);
}

void drawBackButton() {
  drawCenteredTextOnPage(0, 204, "BACK", &Font16, UI_BG, UI_WHITE);
}

void drawSettingsRightNextChevron() {
  // Same minimal right-side affordance used on the main settings pages.
  // This replaces the older large filament side tab so "next" feels implied
  // without taking over the checklist layout.
  Paint_DrawString_EN(SETTINGS_RIGHT_ARROW_X, SETTINGS_ARROW_Y, ">", &Font24, UI_BG, applyUiBrightness(UI_SUBTEXT));
}


void drawSmallPageHint(const char* text) {
  drawCenteredTextOnPage(0, 36, text, &Font12, UI_BG, UI_SUBTEXT);
}

// --------------------------------------------------
// Big number renderer
// --------------------------------------------------
void drawSegment(int x, int y, int w, int h, UWORD color) {
  Paint_DrawRectangle(x, y, x + w, y + h, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

void drawBigDigit(int x, int y, int digit, UWORD color) {
  int t = 5;

  bool a = false;
  bool b = false;
  bool c = false;
  bool d = false;
  bool e = false;
  bool f = false;
  bool g = false;

  switch (digit) {
    case 0: a = b = c = d = e = f = true; break;
    case 1: b = c = true; break;
    case 2: a = b = g = e = d = true; break;
    case 3: a = b = c = d = g = true; break;
    case 4: f = g = b = c = true; break;
    case 5: a = f = g = c = d = true; break;
    case 6: a = f = e = d = c = g = true; break;
    case 7: a = b = c = true; break;
    case 8: a = b = c = d = e = f = g = true; break;
    case 9: a = b = c = d = f = g = true; break;
  }

  if (a) drawSegment(x + 5, y, 18, t, color);
  if (b) drawSegment(x + 23, y + 5, t, 17, color);
  if (c) drawSegment(x + 23, y + 26, t, 17, color);
  if (d) drawSegment(x + 5, y + 43, 18, t, color);
  if (e) drawSegment(x, y + 26, t, 17, color);
  if (f) drawSegment(x, y + 5, t, 17, color);
  if (g) drawSegment(x + 5, y + 22, 18, t, color);
}

void drawBigNumberCenteredWithSuffix(int pageOffsetX, int y, int number, const char* suffix, UWORD color) {
  char buffer[16];
  sprintf(buffer, "%d", number);

  int digitW = 28;
  int gap = 5;
  int count = strlen(buffer);

  int numberWidth = count * digitW + (count - 1) * gap;
  int startX = pageOffsetX + (240 - numberWidth) / 2;

  for (int i = 0; i < count; i++) {
    int d = buffer[i] - '0';
    drawBigDigit(startX + i * (digitW + gap), y, d, color);
  }

  Paint_DrawString_EN(
    startX + numberWidth + 8,
    y + 24,
    suffix,
    &Font16,
    UI_BG,
    UI_SUBTEXT
  );
}

void drawLessThanWeightCentered(int pageOffsetX, int y, int step, const char* suffix, UWORD color) {
  char buffer[16];
  sprintf(buffer, "%d", step);

  int ltWidth = Font24.Width;
  int digitW = 28;
  int gap = 5;
  int count = strlen(buffer);

  int numberWidth = count * digitW + (count - 1) * gap;
  int totalWidth = ltWidth + 6 + numberWidth;

  int startX = pageOffsetX + (240 - totalWidth) / 2;

  Paint_DrawString_EN(startX, y + 10, "<", &Font24, UI_BG, applyUiBrightness(color));

  int digitStartX = startX + ltWidth + 6;

  for (int i = 0; i < count; i++) {
    int d = buffer[i] - '0';
    drawBigDigit(digitStartX + i * (digitW + gap), y, d, color);
  }

  Paint_DrawString_EN(
    digitStartX + numberWidth + 8,
    y + 24,
    suffix,
    &Font16,
    UI_BG,
    UI_SUBTEXT
  );
}

void drawDisplayWeightCentered(int pageOffsetX, int y, int grams, UWORD numberColor) {
  int raw = clampRawWeight(grams);
  int displayGrams = getDisplayWeightGrams(raw);

  if (shouldShowLessThanStep(raw)) {
    drawLessThanWeightCentered(pageOffsetX, y, currentFilamentResolutionGrams(), "g", numberColor);
  } else {
    drawBigNumberCenteredWithSuffix(pageOffsetX, y, displayGrams, "g", numberColor);
  }
}

void drawLargeToolValue(int pageOffsetX, int y) {
  char tool[8];
  getToolName(selectedTool, tool);

  int scale = 9;

  // T1-T9 fit huge. T10-T16 auto-shrink a little.
  if (selectedTool >= 9) {
    scale = 7;
  }

  drawCenteredBlockTextOnPage(pageOffsetX, y, tool, scale, UI_WHITE);
}

void drawLargeMaterialValue(int pageOffsetX, int y) {
  const char* material = filamentTypes[filamentIndex[selectedTool]];

  // Screen 3 has the top gram arc and bezel nearby, so long names must stay
  // inside a safer center box. This prevents PLA SILK / PETG-CF / SUPPORT
  // from crowding the arc near the circular edge.
  int maxWidth = 178;
  int scale = chooseBlockScaleForText(material, 7, maxWidth);

  if (scale < 3) {
    scale = 3;
  }

  int textHeight = 7 * scale;
  int safeTop = 78;
  int safeBottom = 138;
  int centeredY = safeTop + ((safeBottom - safeTop) - textHeight) / 2;

  if (centeredY < safeTop) {
    centeredY = safeTop;
  }

  drawCenteredBlockTextOnPage(pageOffsetX, centeredY, material, scale, UI_WHITE);
}

// --------------------------------------------------
// Color readout
// --------------------------------------------------
void makeHexString(char* buffer, uint8_t r, uint8_t g, uint8_t b) {
  sprintf(buffer, "#%02X%02X%02X", r, g, b);
}

void drawColorReadout(int pageOffsetX) {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  getActiveRgb(&r, &g, &b);

  char buffer[32];

  if (colorReadoutMode == 0) {
    makeHexString(buffer, r, g, b);

    drawCenteredTextOnPage(
      pageOffsetX,
      184,
      buffer,
      &Font16,
      UI_BG,
      UI_WHITE
    );
  }
  else if (colorReadoutMode == 1) {
    sprintf(buffer, "%d %d %d", r, g, b);

    drawCenteredTextOnPage(
      pageOffsetX,
      178,
      buffer,
      &Font16,
      UI_BG,
      UI_WHITE
    );

    drawCenteredTextOnPage(
      pageOffsetX,
      198,
      "R  G  B",
      &Font12,
      UI_BG,
      UI_SUBTEXT
    );
  }
  else {
    sprintf(buffer, "%d %d %d", colorHue, colorSat, colorBrightness);

    drawCenteredTextOnPage(
      pageOffsetX,
      178,
      buffer,
      &Font16,
      UI_BG,
      UI_WHITE
    );

    drawCenteredTextOnPage(
      pageOffsetX,
      198,
      "H  S  V",
      &Font12,
      UI_BG,
      UI_SUBTEXT
    );
  }
}

bool pointInColorReadout(int x, int y) {
  return y >= 174 && y <= 218;
}

void toggleColorReadoutMode() {
  colorReadoutMode++;

  if (colorReadoutMode > 2) {
    colorReadoutMode = 0;
  }

  saveSettings();
  noteUserInteraction();
  drawCurrentPage();
}

// --------------------------------------------------
// Color picker
// --------------------------------------------------
void drawBrightnessSlider(int pageOffsetX) {
  int startAngle = 170;
  int endAngle = 370;
  int span = endAngle - startAngle;

  int cx = pageOffsetX + 120;
  int cy = 120;
  int radius = 109;
  int thickness = 5;

  for (int angle = startAngle; angle <= endAngle; angle++) {
    int brightness = map(angle, startAngle, endAngle, 0, 255);

    UWORD c = hsvTo565(
      colorHue,
      colorSat,
      brightness
    );

    float rad = angle * DEG2RAD;

    for (int t = 0; t < thickness; t++) {
      int rr = radius - t;
      int x = cx + (int)(cos(rad) * rr);
      int y = cy + (int)(sin(rad) * rr);

      Paint_DrawPoint(x, y, c, DOT_PIXEL_1X1, DOT_FILL_RIGHTUP);
    }
  }

  int markerAngle = startAngle + map(colorBrightness, 0, 255, 0, span);
  float markerRad = markerAngle * DEG2RAD;

  int markerRadius = radius - 2;
  int mx = cx + (int)(cos(markerRad) * markerRadius);
  int my = cy + (int)(sin(markerRad) * markerRadius);

  Paint_DrawCircle(mx, my, 9, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(mx, my, 6, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

void drawRGBCircle(int pageOffsetX) {
  int cx = pageOffsetX + 120;
  int cy = 106;
  int radius = 62;
  int block = 5;

  for (int y = -radius; y <= radius; y += block) {
    for (int x = -radius; x <= radius; x += block) {
      float dist = sqrt((float)(x * x + y * y));

      if (dist <= radius) {
        float angle = atan2((float)y, (float)x);

        if (angle < 0) {
          angle += 2.0f * PI;
        }

        uint8_t hue = (uint8_t)(angle * 255.0f / (2.0f * PI));
        uint8_t sat = (uint8_t)(dist * 255.0f / radius);

        UWORD c = hsvTo565(hue, sat, colorBrightness);

        Paint_DrawRectangle(
          cx + x,
          cy + y,
          cx + x + block,
          cy + y + block,
          c,
          DOT_PIXEL_1X1,
          DRAW_FILL_FULL
        );
      }
    }
  }

  float hueRad = colorHue * 2.0f * PI / 255.0f;
  float satRadius = colorSat * radius / 255.0f;

  int sx = cx + (int)(cos(hueRad) * satRadius);
  int sy = cy + (int)(sin(hueRad) * satRadius);

  Paint_DrawCircle(sx, sy, 8, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(sx, sy, 5, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

bool pointInRGBCircle(int x, int y) {
  int cx = 120;
  int cy = 106;
  int radius = 66;

  int dx = x - cx;
  int dy = y - cy;

  return dx * dx + dy * dy <= radius * radius;
}

bool pointNearBrightnessArc(int x, int y) {
  int cx = 120;
  int cy = 120;

  int dx = x - cx;
  int dy = y - cy;

  float dist = sqrt((float)(dx * dx + dy * dy));

  if (dist < 92 || dist > 120) {
    return false;
  }

  float angle = atan2((float)dy, (float)dx) / DEG2RAD;

  if (angle < 0) {
    angle += 360;
  }

  return angle >= 170 || angle <= 10;
}

void handleColorPickerTouch(int x, int y) {
  if (pointInColorReadout(x, y)) {
    toggleColorReadoutMode();
    return;
  }

  if (pointNearBrightnessArc(x, y)) {
    int cx = 120;
    int cy = 120;

    int dx = x - cx;
    int dy = y - cy;

    float angle = atan2((float)dy, (float)dx) / DEG2RAD;

    if (angle < 0) {
      angle += 360;
    }

    if (angle <= 10) {
      angle += 360;
    }

    angle = constrain(angle, 170, 370);

    colorBrightness = map((int)angle, 170, 370, 0, 255);
    saveSettings();

    noteUserInteraction();
    drawCurrentPage();
    return;
  }

  if (pointInRGBCircle(x, y)) {
    int cx = 120;
    int cy = 106;
    int radius = 62;

    int dx = x - cx;
    int dy = y - cy;

    float dist = sqrt((float)(dx * dx + dy * dy));
    dist = constrain(dist, 0, radius);

    float angle = atan2((float)dy, (float)dx);

    if (angle < 0) {
      angle += 2.0f * PI;
    }

    colorHue = (uint8_t)(angle * 255.0f / (2.0f * PI));
    colorSat = (uint8_t)(dist * 255.0f / radius);
    saveSettings();

    noteUserInteraction();
    drawCurrentPage();
    return;
  }
}

// --------------------------------------------------
// Normal page center content
// --------------------------------------------------
void drawWeightCenterOnly() {
  UWORD activeColor = getActiveColor();

  char tool[8];
  getToolName(selectedTool, tool);

  drawSmartCenteredTextOnPage(
    0,
    62,
    tool,
    &Font24,
    UI_BG,
    activeColor
  );

  drawDisplayWeightCentered(
    0,
    96,
    fakeWeightGrams[selectedTool],
    getWeightNumberColorForTool(selectedTool)
  );

  drawSmartCenteredTextOnPage(
    0,
    156,
    filamentTypes[filamentIndex[selectedTool]],
    &Font24,
    UI_BG,
    activeColor
  );
}

// --------------------------------------------------
// Questions / Issues QR code
// URL: www.jaredscottcohen.com
// --------------------------------------------------
const int SUPPORT_QR_SIZE = 29;

const char* supportQr[SUPPORT_QR_SIZE] = {
  "00000000000000000000000000000",
  "00000000000000000000000000000",
  "00111111101011011000111111100",
  "00100000101111010000100000100",
  "00101110101100011110101110100",
  "00101110100110010110101110100",
  "00101110101000011010101110100",
  "00100000100101011000100000100",
  "00111111101010101010111111100",
  "00000000000100011110000000000",
  "00100111111111100011001011100",
  "00100010000101111101011111000",
  "00101110100101110111101010100",
  "00000110010011100110010010000",
  "00101110111110100110110100100",
  "00100101001010011110001111000",
  "00111110100011100110100101100",
  "00101010000101000001011111000",
  "00101111100010010111111110100",
  "00000000001001110110001000000",
  "00111111101001000010101100100",
  "00100000101010110110001101000",
  "00101110101010101111111101100",
  "00101110101010011011100011100",
  "00101110100110100001011001100",
  "00100000100000001100001111100",
  "00111111101110111010011000100",
  "00000000000000000000000000000",
  "00000000000000000000000000000"
};

void drawSupportQrCode() {
  int scale = 4;
  int qrPixelSize = SUPPORT_QR_SIZE * scale;
  int startX = (240 - qrPixelSize) / 2;
  int startY = 70; // Lowered so the QR does not crowd the QUESTIONS/ISSUES text.

  // Tight white backing for scan contrast without wasting space.
  Paint_DrawRectangle(
    startX - 1,
    startY - 1,
    startX + qrPixelSize + 1,
    startY + qrPixelSize + 1,
    UI_WHITE,
    DOT_PIXEL_1X1,
    DRAW_FILL_FULL
  );

  for (int y = 0; y < SUPPORT_QR_SIZE; y++) {
    for (int x = 0; x < SUPPORT_QR_SIZE; x++) {
      if (supportQr[y][x] == '1') {
        Paint_DrawRectangle(
          startX + x * scale,
          startY + y * scale,
          startX + x * scale + scale - 1,
          startY + y * scale + scale - 1,
          BLACK,
          DOT_PIXEL_1X1,
          DRAW_FILL_FULL
        );
      }
    }
  }
}


// --------------------------------------------------
// Boot screen
// Shows only during real startup/init. It does not show during normal redraws,
// page changes, settings changes, dim wake, or sleep wake.
// --------------------------------------------------
void drawBootScreen(const char* statusText) {
  Paint_Clear(UI_BG);

  // Brand lockup. Keep this simple so it draws quickly during startup.
  drawCenteredBlockTextOnPage(0, 62, "SPOOLSENSE", 4, UI_WHITE);

  drawCenteredTextOnPage(
    0,
    116,
    "U1 SPOOL SCALE",
    &Font16,
    UI_BG,
    UI_CYAN
  );

  drawCenteredTextOnPage(
    0,
    144,
    statusText,
    &Font16,
    UI_BG,
    UI_SUBTEXT
  );

  // Small bottom hint so a user knows the device is not stuck.
  drawCenteredTextOnPage(
    0,
    190,
    "checking settings...",
    &Font12,
    UI_BG,
    UI_SUBTEXT
  );

  LCD_1IN28_Display(BlackImage);
}


// --------------------------------------------------
// Persistent settings
// --------------------------------------------------
void normalizeAlertThresholds() {
  if (redThresholdPct < 0) redThresholdPct = 0;
  if (orangeThresholdPct < 0) orangeThresholdPct = 0;
  if (yellowThresholdPct < 0) yellowThresholdPct = 0;

  if (redThresholdPct > 95) redThresholdPct = 95;
  if (orangeThresholdPct > 95) orangeThresholdPct = 95;
  if (yellowThresholdPct > 95) yellowThresholdPct = 95;

  // Enforce severity order: red must trigger at the lowest remaining weight,
  // then orange, then yellow. OFF remains allowed for any level.
  if (yellowThresholdPct > 0 && orangeThresholdPct > 0 && orangeThresholdPct >= yellowThresholdPct) {
    orangeThresholdPct = yellowThresholdPct - 5;

    if (orangeThresholdPct < 0) {
      orangeThresholdPct = 0;
    }
  }

  if (orangeThresholdPct > 0 && redThresholdPct > 0 && redThresholdPct >= orangeThresholdPct) {
    redThresholdPct = orangeThresholdPct - 5;

    if (redThresholdPct < 0) {
      redThresholdPct = 0;
    }
  }

  if (yellowThresholdPct > 0 && orangeThresholdPct == 0 && redThresholdPct > 0 && redThresholdPct >= yellowThresholdPct) {
    redThresholdPct = yellowThresholdPct - 5;

    if (redThresholdPct < 0) {
      redThresholdPct = 0;
    }
  }
}

void saveSettings() {
  prefs.begin("u1ui", false);

  prefs.putInt("spool", spoolSizeIndex);
  prefs.putInt("refresh", displayUpdateIndex);
  prefs.putInt("step", filamentResolutionIndex);
  prefs.putInt("bright", brightnessIndex);
  prefs.putInt("dim", dimIndex);
  prefs.putInt("dimPct", dimPercentIndex);
  prefs.putInt("sleep", sleepIndex);
  prefs.putInt("startup", startupPagePreference);
  prefs.putInt("tool", selectedTool);
  prefs.putUChar("hue", colorHue);
  prefs.putUChar("sat", colorSat);
  prefs.putUChar("val", colorBrightness);
  prefs.putInt("readout", colorReadoutMode);
  prefs.putInt("yellow", yellowThresholdPct);
  prefs.putInt("orange", orangeThresholdPct);
  prefs.putInt("red", redThresholdPct);

  for (int i = 0; i < MAX_TOOLS; i++) {
    char key[12];
    sprintf(key, "mat%d", i);
    prefs.putInt(key, filamentIndex[i]);
  }

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    char key[12];
    sprintf(key, "fil%d", i);
    prefs.putBool(key, filamentEnabled[i]);
  }

  prefs.end();
}

void loadSettings() {
  prefs.begin("u1ui", true);

  spoolSizeIndex = constrain(prefs.getInt("spool", spoolSizeIndex), 0, SPOOL_SIZE_OPTION_COUNT - 1);
  displayUpdateIndex = constrain(prefs.getInt("refresh", displayUpdateIndex), 0, DISPLAY_UPDATE_OPTION_COUNT - 1);
  filamentResolutionIndex = constrain(prefs.getInt("step", filamentResolutionIndex), 0, FILAMENT_RESOLUTION_OPTION_COUNT - 1);
  brightnessIndex = constrain(prefs.getInt("bright", brightnessIndex), 0, BRIGHTNESS_OPTION_COUNT - 1);
  dimIndex = constrain(prefs.getInt("dim", dimIndex), 0, DIM_OPTION_COUNT - 1);
  dimPercentIndex = constrain(prefs.getInt("dimPct", dimPercentIndex), 0, DIM_PERCENT_OPTION_COUNT - 1);
  sleepIndex = constrain(prefs.getInt("sleep", sleepIndex), 0, SLEEP_OPTION_COUNT - 1);
  startupPagePreference = constrain(prefs.getInt("startup", startupPagePreference), PAGE_WEIGHT, PAGE_COLOR);
  selectedTool = constrain(prefs.getInt("tool", selectedTool), 0, MAX_TOOLS - 1);
  colorHue = prefs.getUChar("hue", colorHue);
  colorSat = prefs.getUChar("sat", colorSat);
  colorBrightness = prefs.getUChar("val", colorBrightness);
  colorReadoutMode = constrain(prefs.getInt("readout", colorReadoutMode), 0, 2);
  yellowThresholdPct = prefs.getInt("yellow", yellowThresholdPct);
  orangeThresholdPct = prefs.getInt("orange", orangeThresholdPct);
  redThresholdPct = prefs.getInt("red", redThresholdPct);

  for (int i = 0; i < MAX_TOOLS; i++) {
    char key[12];
    sprintf(key, "mat%d", i);
    filamentIndex[i] = constrain(prefs.getInt(key, filamentIndex[i]), 0, FILAMENT_COUNT - 1);
  }

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    char key[12];
    sprintf(key, "fil%d", i);
    filamentEnabled[i] = prefs.getBool(key, filamentEnabled[i]);
  }

  prefs.end();

  bool anyEnabled = false;

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    if (filamentEnabled[i]) {
      anyEnabled = true;
      break;
    }
  }

  if (!anyEnabled) {
    filamentEnabled[0] = true;
  }

  normalizeAlertThresholds();
}

void noteUserInteraction() {
  lastUserInteractionMs = millis();
  lastTouchAction = millis();
  tareConfirmArmed = false;

  if (uiDimmed || uiSleeping) {
    wakeDisplay();
  }
}

void wakeDisplay() {
  uiSleeping = false;
  uiDimmed = false;
  lastUserInteractionMs = millis();
  drawCurrentPage();
}

void startDimLevelPreview() {
  dimLevelPreviewActive = true;
  dimLevelPreviewStartedMs = millis();
  lastDimLevelPreviewRedraw = 0;
}

void stopDimLevelPreview(bool redraw) {
  if (!dimLevelPreviewActive) {
    return;
  }

  dimLevelPreviewActive = false;

  if (redraw && !uiSleeping) {
    drawCurrentPage();
  }
}

int dimLevelPreviewSecondsRemaining() {
  if (!dimLevelPreviewActive) {
    return 0;
  }

  unsigned long elapsed = millis() - dimLevelPreviewStartedMs;

  if (elapsed >= DIM_LEVEL_PREVIEW_MS) {
    return 0;
  }

  return (int)((DIM_LEVEL_PREVIEW_MS - elapsed + 999) / 1000);
}

void updateDimLevelPreviewIfNeeded() {
  if (!dimLevelPreviewActive) {
    return;
  }

  bool stillOnDimLevelScreen = settingsOpen &&
                                settingsView == SETTINGS_MAIN &&
                                settingsIndex == SETTING_DIM_PERCENT &&
                                !uiSleeping;

  if (!stillOnDimLevelScreen) {
    stopDimLevelPreview(false);
    return;
  }

  if (millis() - dimLevelPreviewStartedMs >= DIM_LEVEL_PREVIEW_MS) {
    stopDimLevelPreview(true);
    return;
  }

  if (millis() - lastDimLevelPreviewRedraw >= 1000) {
    lastDimLevelPreviewRedraw = millis();
    drawCurrentPage();
  }
}

void drawSleepScreen() {
  Paint_Clear(UI_BG);
  LCD_1IN28_Display(BlackImage);
}

void updateDimAndSleepIfNeeded() {
  if (settingsOpen) {
    return;
  }

  unsigned long idleMs = millis() - lastUserInteractionMs;

  if (sleepOptionsSeconds[sleepIndex] > 0 && idleMs >= (unsigned long)sleepOptionsSeconds[sleepIndex] * 1000UL) {
    if (!uiSleeping) {
      uiSleeping = true;
      uiDimmed = false;
      drawSleepScreen();
    }

    return;
  }

  if (dimOptionsSeconds[dimIndex] > 0 && idleMs >= (unsigned long)dimOptionsSeconds[dimIndex] * 1000UL) {
    if (!uiDimmed) {
      uiDimmed = true;
      drawCurrentPage();
    }
  } else if (uiDimmed) {
    uiDimmed = false;
    drawCurrentPage();
  }
}

// --------------------------------------------------
// Settings helpers
// --------------------------------------------------
void openSettings() {
  if (!settingsOpen) {
    previousNormalPage = currentPage;
    settingsOpen = true;
    settingsView = SETTINGS_MAIN;
    lastTouchAction = millis();
    drawCurrentPage();
  }
}

void closeSettings() {
  stopDimLevelPreview(false);
  settingsOpen = false;
  currentPage = previousNormalPage;
  lastTouchAction = millis();
  drawCurrentPage();
}

void nextSetting() {
  stopDimLevelPreview(false);
  settingsIndex++;

  if (settingsIndex >= SETTINGS_COUNT) {
    settingsIndex = 0;
  }

  lastTouchAction = millis();
  drawCurrentPage();
}

void previousSetting() {
  stopDimLevelPreview(false);
  settingsIndex--;

  if (settingsIndex < 0) {
    settingsIndex = SETTINGS_COUNT - 1;
  }

  lastTouchAction = millis();
  drawCurrentPage();
}

void applyCalibrationAction() {
  if (calibrationMode == 0) {
    // TARE: zero the real HX711/load-cell reading.
    if (hx711Ready && spoolScale.is_ready()) {
      spoolScale.tare(15);
    }

    fakeWeightGrams[selectedTool] = 0;
  }
  else if (calibrationMode == 1) {
    // CAL 100G is still a UI placeholder. Final calibration factor tuning is
    // done by changing hx711CalibrationFactor after testing with a known weight.
    debugPrintf("CAL 100G selected. Place a known weight and tune hx711CalibrationFactor if needed.\n");
  }
  else {
    // RESET: restore the configured calibration factor and tare again.
    if (hx711Ready) {
      spoolScale.set_scale(hx711CalibrationFactor);
      if (spoolScale.is_ready()) {
        spoolScale.tare(15);
      }
    }

    fakeWeightGrams[selectedTool] = 0;
  }
}

void toggleCurrentSetting() {
  if (settingsIndex == SETTING_SPOOL) {
    spoolSizeIndex++;

    if (spoolSizeIndex >= SPOOL_SIZE_OPTION_COUNT) {
      spoolSizeIndex = 0;
    }

    for (int i = 0; i < MAX_TOOLS; i++) {
      if (fakeWeightGrams[i] > spoolMaxGrams()) {
        fakeWeightGrams[i] = spoolMaxGrams();
      }
    }
  }
  else if (settingsIndex == SETTING_CALIBRATION) {
    applyCalibrationAction();

    calibrationMode++;

    if (calibrationMode > 2) {
      calibrationMode = 0;
    }
  }
  else if (settingsIndex == SETTING_UPDATE) {
    displayUpdateIndex++;

    if (displayUpdateIndex >= DISPLAY_UPDATE_OPTION_COUNT) {
      displayUpdateIndex = 0;
    }
  }
  else if (settingsIndex == SETTING_RESOLUTION) {
    filamentResolutionIndex++;

    if (filamentResolutionIndex >= FILAMENT_RESOLUTION_OPTION_COUNT) {
      filamentResolutionIndex = 0;
    }
  }
  else if (settingsIndex == SETTING_BRIGHTNESS) {
    // Brightness is controlled with the -10 / +10 buttons on this setting.
  }
  else if (settingsIndex == SETTING_DIM) {
    dimIndex++;

    if (dimIndex >= DIM_OPTION_COUNT) {
      dimIndex = 0;
    }
  }
  else if (settingsIndex == SETTING_DIM_PERCENT) {
    // Dim level is controlled with the -10 / +10 buttons on this setting.
  }
  else if (settingsIndex == SETTING_SLEEP) {
    sleepIndex++;

    if (sleepIndex >= SLEEP_OPTION_COUNT) {
      sleepIndex = 0;
    }
  }
  else if (settingsIndex == SETTING_TOOL_SELECT) {
    // Selected toolhead is controlled with the + / - buttons on this setting.
    // The center toggle button is intentionally unused here.
  }
  else if (settingsIndex == SETTING_FILAMENTS) {
    stopDimLevelPreview(false);
    settingsView = SETTINGS_FILAMENTS;
    filamentListOffset = 0;
  }
  else if (settingsIndex == SETTING_STARTUP) {
    startupPagePreference++;

    if (startupPagePreference >= NORMAL_PAGE_COUNT) {
      startupPagePreference = PAGE_WEIGHT;
    }
  }
  else if (settingsIndex == SETTING_YELLOW_ALERT ||
           settingsIndex == SETTING_ORANGE_ALERT ||
           settingsIndex == SETTING_RED_ALERT) {
    int* pct = thresholdPointerForSetting(settingsIndex);

    if (pct != NULL) {
      cycleThresholdPercent(pct);
    }
  }
  else if (settingsIndex == SETTING_HELP) {
    // QR-only section. No toggle action.
  }

  normalizeAlertThresholds();
  saveSettings();
  lastTouchAction = millis();
  lastUserInteractionMs = millis();
  drawCurrentPage();
}

const char* startupPageName() {
  if (startupPagePreference == PAGE_WEIGHT) return "WT.";
  if (startupPagePreference == PAGE_TOOL) return "TARE";
  if (startupPagePreference == PAGE_MATERIAL) return "MATERIAL";
  return "COLOR";
}

const char* unitsName() {
  if (unitsMode == 0) return "GRAMS";
  return "PERCENT";
}

const char* dimName() {
  if (dimOptionsSeconds[dimIndex] == 0) return "NEVER";
  if (dimOptionsSeconds[dimIndex] == 30) return "30s";
  if (dimOptionsSeconds[dimIndex] == 120) return "2m";
  return "5m";
}

const char* dimPercentName() {
  static char value[8];
  sprintf(value, "%d%%", dimPercentOptions[dimPercentIndex]);
  return value;
}

void adjustBrightnessByStep(int step) {
  brightnessIndex += step;
  brightnessIndex = constrain(brightnessIndex, 0, BRIGHTNESS_OPTION_COUNT - 1);
  lastTouchAction = millis();
  lastUserInteractionMs = millis();
  saveSettings();
  drawCurrentPage();
}

void adjustDimPercentByStep(int step) {
  dimPercentIndex += step;
  dimPercentIndex = constrain(dimPercentIndex, 0, DIM_PERCENT_OPTION_COUNT - 1);
  lastTouchAction = millis();
  lastUserInteractionMs = millis();
  saveSettings();
  startDimLevelPreview();
  drawCurrentPage();
}

const char* sleepName() {
  if (sleepOptionsSeconds[sleepIndex] == 0) return "NEVER";
  if (sleepOptionsSeconds[sleepIndex] == 300) return "5m";
  if (sleepOptionsSeconds[sleepIndex] == 900) return "15m";
  return "30m";
}

void getThresholdValueText(int pct, char* value) {
  if (pct <= 0) {
    strcpy(value, "OFF");
  } else {
    sprintf(value, "%d%%", pct);
  }
}

int getEnabledFilamentCount() {
  int count = 0;

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    if (filamentEnabled[i]) {
      count++;
    }
  }

  return count;
}

const char* getEnabledFilamentByDisplayIndex(int displayIndex) {
  int count = getEnabledFilamentCount();

  if (count <= 0) {
    filamentEnabled[0] = true;
    return filamentTypes[0];
  }

  int target = displayIndex % count;
  int seen = 0;

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    if (filamentEnabled[i]) {
      if (seen == target) {
        return filamentTypes[i];
      }

      seen++;
    }
  }

  return filamentTypes[0];
}

void getEnabledFilamentTickerText(int displayIndex, char* buffer, int bufferSize) {
  int count = getEnabledFilamentCount();

  if (count <= 0) {
    filamentEnabled[0] = true;
    snprintf(buffer, bufferSize, "1.%c%s", BLOCK_HALF_SPACE, filamentTypes[0]);
    return;
  }

  int target = displayIndex % count;
  int seen = 0;

  for (int i = 0; i < FILAMENT_COUNT; i++) {
    if (filamentEnabled[i]) {
      if (seen == target) {
        // Number the cycling list by enabled-filament order, not by the raw
        // internal material index, so the user sees 1, 2, 3... as it cycles.
        snprintf(buffer, bufferSize, "%d.%c%s", seen + 1, BLOCK_HALF_SPACE, filamentTypes[i]);
        return;
      }

      seen++;
    }
  }

  snprintf(buffer, bufferSize, "1.%c%s", BLOCK_HALF_SPACE, filamentTypes[0]);
}

UWORD getCurrentSettingAccentColor() {
  if (settingsIndex == SETTING_YELLOW_ALERT) return UI_YELLOW;
  if (settingsIndex == SETTING_ORANGE_ALERT) return UI_ORANGE;
  if (settingsIndex == SETTING_RED_ALERT) return UI_RED;

  return UI_SUBTEXT;
}

void getSettingSubtitle(char* subtitle) {
  strcpy(subtitle, "");

  if (settingsIndex == SETTING_FILAMENTS) {
    int count = getEnabledFilamentCount();
    sprintf(subtitle, "%d FILAMENTS", count);
    return;
  }

  if (settingsIndex == SETTING_RESOLUTION) {
    return;
  }

  int* pct = thresholdPointerForSetting(settingsIndex);

  if (pct == NULL || *pct <= 0) {
    return;
  }

  char weightText[16];
  formatWeightForCurrentUnits(thresholdToGrams(*pct), weightText);

  sprintf(subtitle, "(%s)", weightText);
}


void getSettingTitleAndValue(char* title, char* value, char* buttonLabel) {
  strcpy(buttonLabel, "TOGGLE");

  if (settingsIndex == SETTING_SPOOL) {
    strcpy(title, "SPOOL SIZE");
    formatSpoolSizeLabel(value);
  }
  else if (settingsIndex == SETTING_CALIBRATION) {
    strcpy(title, "CALIBRATION");

    if (calibrationMode == 0) strcpy(value, "TARE");
    else if (calibrationMode == 1) strcpy(value, "CAL 100G");
    else strcpy(value, "RESET");

    strcpy(buttonLabel, "RUN");
  }
  else if (settingsIndex == SETTING_UPDATE) {
    strcpy(title, "WT. REFRESH");
    sprintf(value, "%lus", currentDisplayUpdateMs() / 1000);
  }
  else if (settingsIndex == SETTING_RESOLUTION) {
    strcpy(title, "WT. STEP");
    sprintf(value, "%dg", currentFilamentResolutionGrams());
  }
  else if (settingsIndex == SETTING_BRIGHTNESS) {
    strcpy(title, "BRIGHTNESS");
    sprintf(value, "%d%%", brightnessOptions[brightnessIndex]);
  }
  else if (settingsIndex == SETTING_DIM) {
    strcpy(title, "DIM AFTER");
    strcpy(value, dimName());
  }
  else if (settingsIndex == SETTING_DIM_PERCENT) {
    strcpy(title, "DIM LEVEL");
    strcpy(value, dimPercentName());
    strcpy(buttonLabel, "");
  }
  else if (settingsIndex == SETTING_SLEEP) {
    strcpy(title, "SLEEP AFTER");
    strcpy(value, sleepName());
  }
  else if (settingsIndex == SETTING_TOOL_SELECT) {
    strcpy(title, "TOOLHEAD");
    getToolName(selectedTool, value);
    strcpy(buttonLabel, "");
  }
  else if (settingsIndex == SETTING_FILAMENTS) {
    int enabledCount = getEnabledFilamentCount();
    int tickerIndex = enabledCount > 0 ? (millis() / 1000) % enabledCount : 0;

    strcpy(title, "FILAMENTS");
    getEnabledFilamentTickerText(tickerIndex, value, 24);
    strcpy(buttonLabel, "OPEN");
  }
  else if (settingsIndex == SETTING_STARTUP) {
    strcpy(title, "STARTUP");
    strcpy(value, startupPageName());
  }
  else if (settingsIndex == SETTING_YELLOW_ALERT) {
    strcpy(title, "YELLOW AT");
    getThresholdValueText(yellowThresholdPct, value);
  }
  else if (settingsIndex == SETTING_ORANGE_ALERT) {
    strcpy(title, "ORANGE AT");
    getThresholdValueText(orangeThresholdPct, value);
  }
  else if (settingsIndex == SETTING_RED_ALERT) {
    strcpy(title, "RED AT");
    getThresholdValueText(redThresholdPct, value);
  }
  else if (settingsIndex == SETTING_HELP) {
    strcpy(title, "QUESTIONS/ISSUES?");
    strcpy(value, "SCAN QR");
    strcpy(buttonLabel, "");
  }
}

void drawSettingsProgressArcs() {
  int cx = SETTINGS_ARC_CX;
  int cy = SETTINGS_ARC_CY;
  int radius = SETTINGS_PROGRESS_RADIUS;

  // One consistent bottom progress geometry across every settings screen.
  // If it ever needs to be shortened for one screen, change these shared
  // SETTINGS_PROGRESS_* constants so all screens adjust together.
  float startAngle = SETTINGS_PROGRESS_START_DEG;
  float endAngle = SETTINGS_PROGRESS_END_DEG;
  float step = 0.0f;

  if (SETTINGS_COUNT > 1) {
    step = (endAngle - startAngle) / (SETTINGS_COUNT - 1);
  }

  for (int i = 0; i < SETTINGS_COUNT; i++) {
    int centerAngle = (int)(startAngle + i * step + 0.5f);
    int arcStart = centerAngle - SETTINGS_PROGRESS_TICK_HALF_DEG;
    int arcEnd = centerAngle + SETTINGS_PROGRESS_TICK_HALF_DEG;

    UWORD c = (i == settingsIndex) ? UI_WHITE : UI_SUBTEXT;
    int thickness = (i == settingsIndex) ? 5 : 3;

    drawArc(cx, cy, radius, arcStart, arcEnd, c, thickness);
  }
}


void incrementSelectedTool() {
  if (selectedTool < MAX_TOOLS - 1) {
    selectedTool++;
  }

  saveSettings();
  lastTouchAction = millis();
  lastUserInteractionMs = millis();
  drawCurrentPage();
}

void decrementSelectedTool() {
  if (selectedTool > 0) {
    selectedTool--;
  }

  saveSettings();
  lastTouchAction = millis();
  lastUserInteractionMs = millis();
  drawCurrentPage();
}

void drawSettingsMain() {
  char title[24];
  char value[24];
  char buttonLabel[12];
  char subtitle[24];

  getSettingTitleAndValue(title, value, buttonLabel);
  getSettingSubtitle(subtitle);

  drawSettingsProgressArcs();

  UWORD settingAccent = getCurrentSettingAccentColor();

  if (settingsIndex == SETTING_HELP) {
    drawSettingsCurvedCombinedTitle(title, UI_SUBTEXT);
    drawSupportQrCode();

    Paint_DrawString_EN(SETTINGS_LEFT_ARROW_X, SETTINGS_ARROW_Y, "<", &Font24, UI_BG, applyUiBrightness(UI_SUBTEXT));
    Paint_DrawString_EN(SETTINGS_RIGHT_ARROW_X, SETTINGS_ARROW_Y, ">", &Font24, UI_BG, applyUiBrightness(UI_SUBTEXT));

    drawBackButton();
    return;
  }

  drawSettingsCurvedCombinedTitle(title, settingAccent);

  UWORD valueColor = settingsIndex == SETTING_YELLOW_ALERT ||
                     settingsIndex == SETTING_ORANGE_ALERT ||
                     settingsIndex == SETTING_RED_ALERT ? settingAccent : UI_WHITE;

  int valueScale = chooseBlockScaleForText(value, 6, 176);
  int minValueScale = settingsIndex == SETTING_FILAMENTS ? 3 : 4;

  if (valueScale < minValueScale) {
    valueScale = minValueScale;
  }

  int valueY = 82;
  drawCenteredBlockTextOnPage(0, valueY, value, valueScale, valueColor);

  if (settingsIndex == SETTING_DIM_PERCENT && dimLevelPreviewActive) {
    char cue[28];
    sprintf(cue, "back to nondim (%ds)", dimLevelPreviewSecondsRemaining());
    drawCenteredTextOnPage(0, 124, cue, &Font12, UI_BG, UI_SUBTEXT);
  }
  else if (strlen(subtitle) > 0) {
    drawCenteredTextOnPage(0, 126, subtitle, &Font16, UI_BG, settingsIndex == SETTING_FILAMENTS ? UI_SUBTEXT : settingAccent);
  }

  if (settingsIndex == SETTING_TOOL_SELECT) {
    drawPlusMinusButtons("-", "+");
  } else if (settingsIndex == SETTING_BRIGHTNESS || settingsIndex == SETTING_DIM_PERCENT) {
    drawPlusMinusButtons("-10", "+10");
  } else {
    drawButton(0, TOGGLE_X, TOGGLE_Y, TOGGLE_W, TOGGLE_H, buttonLabel);
  }

  Paint_DrawString_EN(SETTINGS_LEFT_ARROW_X, SETTINGS_ARROW_Y, "<", &Font24, UI_BG, applyUiBrightness(UI_SUBTEXT));
  Paint_DrawString_EN(SETTINGS_RIGHT_ARROW_X, SETTINGS_ARROW_Y, ">", &Font24, UI_BG, applyUiBrightness(UI_SUBTEXT));

  drawBackButton();
}

void drawCheckmark(int x, int y, bool checked) {
  Paint_DrawRectangle(x, y, x + 16, y + 16, applyUiBrightness(UI_WHITE), DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

  if (!checked) {
    return;
  }

  // Simple checkmark drawn manually so it is clear and not font-dependent.
  Paint_DrawLine(x + 3, y + 8, x + 7, y + 12, applyUiBrightness(UI_WHITE), DOT_PIXEL_2X2, LINE_STYLE_SOLID);
  Paint_DrawLine(x + 7, y + 12, x + 14, y + 4, applyUiBrightness(UI_WHITE), DOT_PIXEL_2X2, LINE_STYLE_SOLID);
}

void drawFilamentChecklist() {
  drawSettingsProgressArcs();
  drawSettingsCurvedCombinedTitle("FILAMENTS", UI_SUBTEXT);

  char rangeText[20];
  int endItem = filamentListOffset + 5;

  if (endItem > FILAMENT_COUNT) {
    endItem = FILAMENT_COUNT;
  }

  sprintf(rangeText, "%d-%d/%d", filamentListOffset + 1, endItem, FILAMENT_COUNT);
  drawCenteredTextOnPage(0, 66, rangeText, &Font12, UI_BG, UI_SUBTEXT);

  int rows = 5;

  for (int i = 0; i < rows; i++) {
    int idx = filamentListOffset + i;

    if (idx >= FILAMENT_COUNT) {
      break;
    }

    int y = 84 + i * 21;

    char itemNumber[8];
    sprintf(itemNumber, "%d.", idx + 1);

    Paint_DrawString_EN(12, y + 2, itemNumber, &Font12, UI_BG, applyUiBrightness(UI_SUBTEXT));
    drawCheckmark(48, y, filamentEnabled[idx]);
    Paint_DrawString_EN(74, y, filamentTypes[idx], &Font16, UI_BG, applyUiBrightness(UI_WHITE));
  }

  drawSettingsRightNextChevron();
  drawBackButton();
}


void drawSettingsScreen() {
  Paint_Clear(UI_BG);

  if (settingsView == SETTINGS_MAIN) {
    drawSettingsMain();
  } else {
    drawFilamentChecklist();
  }

  LCD_1IN28_Display(BlackImage);
}

void handleSettingsTap(int x, int y) {
  if (pointInBackButton(x, y)) {
    if (settingsView == SETTINGS_FILAMENTS) {
      settingsView = SETTINGS_MAIN;
      lastTouchAction = millis();
      drawCurrentPage();
    } else {
      closeSettings();
    }

    return;
  }

  if (settingsView == SETTINGS_MAIN) {
    if (settingsIndex == SETTING_TOOL_SELECT) {
      if (pointInMinusButton(x, y)) {
        decrementSelectedTool();
        return;
      }

      if (pointInPlusButton(x, y)) {
        incrementSelectedTool();
        return;
      }
    }

    if (settingsIndex == SETTING_BRIGHTNESS) {
      if (pointInMinusButton(x, y)) {
        adjustBrightnessByStep(-1);
        return;
      }

      if (pointInPlusButton(x, y)) {
        adjustBrightnessByStep(1);
        return;
      }
    }

    if (settingsIndex == SETTING_DIM_PERCENT) {
      if (pointInMinusButton(x, y)) {
        adjustDimPercentByStep(-1);
        return;
      }

      if (pointInPlusButton(x, y)) {
        adjustDimPercentByStep(1);
        return;
      }
    }

    if (pointInToggleButton(x, y)) {
      toggleCurrentSetting();
      return;
    }

    if (x < 80 && y > 60 && y < 180) {
      previousSetting();
      return;
    }

    if (pointInSettingsRightNextButton(x, y)) {
      nextSetting();
      return;
    }

    return;
  }

  if (settingsView == SETTINGS_FILAMENTS) {
    if (pointInSettingsRightNextButton(x, y)) {
      filamentListOffset += 5;

      if (filamentListOffset >= FILAMENT_COUNT) {
        filamentListOffset = 0;
      }

      lastTouchAction = millis();
      drawCurrentPage();
      return;
    }

    for (int i = 0; i < 5; i++) {
      int idx = filamentListOffset + i;

      if (idx >= FILAMENT_COUNT) {
        continue;
      }

      int rowTop = 54 + i * 24;
      int rowBottom = rowTop + 22;

      if (y >= rowTop && y <= rowBottom) {
        filamentEnabled[idx] = !filamentEnabled[idx];

        bool anyEnabled = false;

        for (int j = 0; j < FILAMENT_COUNT; j++) {
          if (filamentEnabled[j]) {
            anyEnabled = true;
            break;
          }
        }

        if (!anyEnabled) {
          filamentEnabled[idx] = true;
        }

        if (!filamentEnabled[filamentIndex[selectedTool]]) {
          filamentIndex[selectedTool] = nextEnabledFilamentIndex(filamentIndex[selectedTool]);
        }

        saveSettings();
        lastUserInteractionMs = millis();
        lastTouchAction = millis();
        drawCurrentPage();
        return;
      }
    }
  }
}


// --------------------------------------------------
// Screen 2 tare / defined-weight helpers
// --------------------------------------------------
void setSelectedToolWeight(int grams) {
  fakeWeightGrams[selectedTool] = clampRawWeight(grams);
}

void adjustSelectedToolWeight(int delta) {
  tareConfirmArmed = false;
  int shownWeight = roundWeightToResolution(fakeWeightGrams[selectedTool]);
  setSelectedToolWeight(shownWeight + delta);
  lastTouchAction = millis();
  drawCurrentPage();
}

void tareSelectedToolWeight() {
  unsigned long now = millis();

  if (!tareConfirmArmed || now - tareConfirmStartedMs > TARE_CONFIRM_TIMEOUT_MS) {
    tareConfirmArmed = true;
    tareConfirmStartedMs = now;
    lastTouchAction = now;
    drawCurrentPage();
    return;
  }

  // Confirmed tare zeros the real HX711/load-cell reading.
  // Wait briefly because HX711 can be "not ready" between samples.
  bool tareDone = false;
  unsigned long waitStartMs = millis();

  while (millis() - waitStartMs < 1500) {
    if (spoolScale.is_ready()) {
      spoolScale.set_scale(hx711CalibrationFactor);
      spoolScale.tare(15);
      tareDone = true;
      hx711Ready = true;
      hx711EverReady = true;
      hx711StartupTareDone = true;
      hx711NotReadyStartedMs = 0;
      break;
    }

    delay(10);
  }

  // Reset all local displayed/debug values so the screen does not bounce back.
  fakeWeightGrams[selectedTool] = 0;
  lastHx711MeasuredGrams = 0;
  lastHx711MeasurementValid = tareDone;

  prevHx711TaredCounts = 0;
  currentHx711TaredCounts = 0;
  prevHx711CalculatedGrams = 0.0f;
  currentHx711CalculatedGrams = 0.0f;
  hx711RawDebugValid = false;

  tareConfirmArmed = false;
  lastTouchAction = now;
  lastUserInteractionMs = now;

  debugPrintf(tareDone ? "[TARE] Confirmed tare complete.\n" : "[TARE] Confirmed, but HX711 was not ready.\n");

  drawCurrentPage();
}

void handleTareSetTap(int x, int y) {
  if (pointInTareButton(x, y)) {
    tareSelectedToolWeight();
    return;
  }

  int values[3];
  int count = 0;

  getTareIncrements(values, &count);

  for (int i = 0; i < count; i++) {
    int bx = getTareButtonXForIndex(i, count);

    if (pointInRect(x, y, bx, TARE_SET_MINUS_Y, TARE_SET_BTN_W, TARE_SET_BTN_H)) {
      adjustSelectedToolWeight(-values[i]);
      return;
    }

    if (pointInRect(x, y, bx, TARE_SET_PLUS_Y, TARE_SET_BTN_W, TARE_SET_BTN_H)) {
      adjustSelectedToolWeight(values[i]);
      return;
    }
  }
}

void drawTareSetPageValue(int pageOffsetX) {
  char buffer[18];

  formatWeightForCurrentUnits(fakeWeightGrams[selectedTool], buffer);

  drawCenteredTextOnPage(
    pageOffsetX,
    68,
    buffer,
    &Font24,
    UI_BG,
    getWeightNumberColorForTool(selectedTool)
  );

  if (tareConfirmArmed) {
    drawCenteredTextOnPage(pageOffsetX, 190, "tap CONFIRM to zero", &Font12, UI_BG, UI_YELLOW);
  }
}


// --------------------------------------------------
// Page renderers
// --------------------------------------------------
void drawSharedWeightArc(int pageOffsetX) {
  UWORD activeColor = getLowMaterialColorForTool(selectedTool);
  int arcPercent = getArcFillPercentForTool(selectedTool);

  drawTopProgressArc(pageOffsetX, arcPercent, activeColor);
}

void renderWeightPage(int pageOffsetX) {
  drawSharedWeightArc(pageOffsetX);

  if (pageOffsetX == 0) {
    drawWeightCenterOnly();
  } else {
    UWORD activeColor = getActiveColor();
    char tool[8];

    getToolName(selectedTool, tool);

    drawSmartCenteredTextOnPage(pageOffsetX, 62, tool, &Font24, UI_BG, activeColor);
    drawDisplayWeightCentered(pageOffsetX, 96, fakeWeightGrams[selectedTool], getWeightNumberColorForTool(selectedTool));
    drawSmartCenteredTextOnPage(pageOffsetX, 156, filamentTypes[filamentIndex[selectedTool]], &Font24, UI_BG, activeColor);
  }

  drawNavigationArcs(PAGE_WEIGHT, pageOffsetX);
}

void renderToolPage(int pageOffsetX) {
  drawSharedWeightArc(pageOffsetX);

  drawCenteredTextOnPage(
    pageOffsetX,
    42,
    "TARE / SET",
    &Font16,
    UI_BG,
    UI_SUBTEXT
  );

  drawTareSetPageValue(pageOffsetX);
  drawTareSetButtons(pageOffsetX);

  drawNavigationArcs(PAGE_TOOL, pageOffsetX);
}

void renderMaterialPage(int pageOffsetX) {
  drawSharedWeightArc(pageOffsetX);

  drawLargeMaterialValue(pageOffsetX, 0);

  drawToggleButton(pageOffsetX);

  drawNavigationArcs(PAGE_MATERIAL, pageOffsetX);
}

void renderColorPage(int pageOffsetX) {
  drawBrightnessSlider(pageOffsetX);
  drawRGBCircle(pageOffsetX);
  drawColorReadout(pageOffsetX);
  drawNavigationArcs(PAGE_COLOR, pageOffsetX);
}

void renderPage(int page, int pageOffsetX) {
  if (page == PAGE_WEIGHT) {
    renderWeightPage(pageOffsetX);
  } else if (page == PAGE_TOOL) {
    renderToolPage(pageOffsetX);
  } else if (page == PAGE_MATERIAL) {
    renderMaterialPage(pageOffsetX);
  } else if (page == PAGE_COLOR) {
    renderColorPage(pageOffsetX);
  }
}

void drawCurrentPage() {
  if (uiSleeping) {
    drawSleepScreen();
    return;
  }

  if (settingsOpen) {
    drawSettingsScreen();
    return;
  }

  Paint_Clear(UI_BG);
  renderPage(currentPage, 0);
  drawAdvancedSettingsHint();
  LCD_1IN28_Display(BlackImage);
}

// --------------------------------------------------
// Page navigation
// --------------------------------------------------
void goToPage(int newPage) {
  currentPage = newPage;
  tareConfirmArmed = false;
  noteUserInteraction();
  drawCurrentPage();
}

void nextPage() {
  int next = currentPage + 1;

  if (next >= NORMAL_PAGE_COUNT) {
    next = PAGE_WEIGHT;
  }

  goToPage(next);
}

void previousPage() {
  int prev = currentPage - 1;

  if (prev < PAGE_WEIGHT) {
    prev = PAGE_COLOR;
  }

  goToPage(prev);
}

// --------------------------------------------------
// Button-only page actions
// --------------------------------------------------
void handleButtonTap(int x, int y) {
  if (currentPage == PAGE_TOOL) {
    handleTareSetTap(x, y);
    return;
  }

  if (currentPage == PAGE_MATERIAL) {
    if (pointInToggleButton(x, y)) {
      filamentIndex[selectedTool] = nextEnabledFilamentIndex(filamentIndex[selectedTool]);
      saveSettings();

      noteUserInteraction();
      drawCurrentPage();
    }

    return;
  }

  if (currentPage == PAGE_COLOR) {
    if (pointInColorReadout(x, y)) {
      toggleColorReadoutMode();
    }

    return;
  }
}

void handleTapZone(int x, int y) {
  noteUserInteraction();

  if (settingsOpen) {
    handleSettingsTap(x, y);
    return;
  }

  if (currentPage != PAGE_COLOR && pointInSettingsHotspot(x, y)) {
    openSettings();
    return;
  }

  int tappedArc = getTappedNavigationArc(x, y);

  if (tappedArc >= 0) {
    debugPrintf("Navigation arc tapped. Going to page %d\n", tappedArc);
    goToPage(tappedArc);
    return;
  }

  handleButtonTap(x, y);
}

// --------------------------------------------------
// Fast swipe + tap handling
// --------------------------------------------------
void checkTouch() {
  bool touching = touch.available();

  if (!touching) {
    if (touchTracking && !touchMoved) {
      int dx = touchLastX - touchStartX;
      int dy = touchLastY - touchStartY;

      if (abs(dx) <= TAP_MAX_MOVE && abs(dy) <= TAP_MAX_MOVE) {
        if (millis() - lastTouchAction > TOUCH_DEBOUNCE_MS) {
          handleTapZone(touchStartX, touchStartY);
        }
      }
    }

    touchTracking = false;
    touchMoved = false;
    return;
  }

  int x = touch.data.x;
  int y = touch.data.y;

  if (uiSleeping) {
    wakeDisplay();
    touchTracking = false;
    touchMoved = false;
    return;
  }

  if (uiDimmed) {
    wakeDisplay();
    touchTracking = false;
    touchMoved = false;
    return;
  }

  if (!touchTracking) {
    touchTracking = true;
    touchMoved = false;

    touchStartX = x;
    touchStartY = y;
    touchLastX = x;
    touchLastY = y;
    return;
  }

  touchLastX = x;
  touchLastY = y;

  int dx = touchLastX - touchStartX;
  int dy = touchLastY - touchStartY;

  if (millis() - lastTouchAction < TOUCH_DEBOUNCE_MS) {
    return;
  }

  if (settingsOpen) {
    return;
  }

  if (abs(dx) >= SWIPE_THRESHOLD_X && abs(dy) <= SWIPE_MAX_VERTICAL_DRIFT) {
    touchMoved = true;
    touchTracking = false;

    if (dx < 0) {
      debugPrintf("Swipe left: next page\n");
      nextPage();
    } else {
      debugPrintf("Swipe right: previous page\n");
      previousPage();
    }

    return;
  }

  if (currentPage == PAGE_COLOR) {
    if (pointInRGBCircle(x, y) || pointNearBrightnessArc(x, y)) {
      touchMoved = true;
      handleColorPickerTouch(x, y);
    }
  }
}

// --------------------------------------------------
// HX711 weight update
// --------------------------------------------------

void printHx711GpioDiagnostics(bool forcePrint) {
  if (!forcePrint && millis() - lastHx711GpioDiagPrint < HX711_GPIO_DIAG_PRINT_INTERVAL_MS) {
    return;
  }

  lastHx711GpioDiagPrint = millis();

  int doutState = digitalRead(HX711_DOUT);
  int sckState = digitalRead(HX711_SCK);
  bool readyNow = spoolScale.is_ready();

  debugPrintf(
    "[GPIO] DOUT GPIO%d=%d | SCK GPIO%d=%d | is_ready=%s | last=%dg | valid=%s\n",
    HX711_DOUT,
    doutState,
    HX711_SCK,
    sckState,
    readyNow ? "YES" : "NO",
    lastHx711MeasuredGrams,
    lastHx711MeasurementValid ? "YES" : "NO"
  );

  debugPrintf("[GPIO] Note: DOUT HIGH / ready=NO can be normal between HX711 samples. Only a sustained not-ready state is a problem.\n");
}

void runHx711PinStartupDiagnostic() {
  debugPrintf("\n========== GPIO PIN DIAGNOSTIC ==========" "\n");
  debugPrintf("Configured HX711_DOUT = GPIO%d\n", HX711_DOUT);
  debugPrintf("Configured HX711_SCK  = GPIO%d\n", HX711_SCK);
  debugPrintf("Expected connector pins: DT -> GPIO18, SCK -> GPIO21, VCC -> 3V3, GND -> GND\n");

  pinMode(HX711_DOUT, INPUT);
  pinMode(HX711_SCK, OUTPUT);
  digitalWrite(HX711_SCK, LOW);
  delay(20);

  int doutInitial = digitalRead(HX711_DOUT);
  int sckLowReadback = digitalRead(HX711_SCK);

  debugPrintf("After pinMode: DOUT read=%d, SCK commanded LOW/readback=%d\n", doutInitial, sckLowReadback);

  digitalWrite(HX711_SCK, HIGH);
  delayMicroseconds(10);
  int sckHighReadback = digitalRead(HX711_SCK);

  digitalWrite(HX711_SCK, LOW);
  delayMicroseconds(10);
  int sckLowAgainReadback = digitalRead(HX711_SCK);

  debugPrintf("SCK output test: HIGH readback=%d, LOW readback=%d\n", sckHighReadback, sckLowAgainReadback);
  debugPrintf("If SCK readback follows HIGH/LOW, the selected SCK GPIO is controllable by this sketch.\n");
  debugPrintf("If DOUT is always HIGH and HX711 is never ready, check HX711 VCC/GND/DT wiring first.\n");
  debugPrintf("======== END GPIO PIN DIAGNOSTIC ========\n\n");
}

void printHx711BenchDebug(bool forcePrint) {
  if (!forcePrint && millis() - lastHx711DebugPrint < HX711_DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  lastHx711DebugPrint = millis();

  bool readyNow = spoolScale.is_ready();

  debugPrintf(
    "[HX711] DT=%d SCK=%d ready=%s hx711Ready=%s valid=%s factor=%.6f\n",
    HX711_DOUT,
    HX711_SCK,
    readyNow ? "YES" : "NO",
    hx711Ready ? "YES" : "NO",
    lastHx711MeasurementValid ? "YES" : "NO",
    hx711CalibrationFactor
  );

  if (hx711RawDebugValid) {
    debugPrintf(
      "[HX711 LIVE] prev_raw=%ld now_raw=%ld | prev_delta=%ld now_delta=%ld | prev_calc=%.2fg now_calc=%.2fg | UI=%dg\n",
      prevHx711RawCounts,
      currentHx711RawCounts,
      prevHx711TaredCounts,
      currentHx711TaredCounts,
      prevHx711CalculatedGrams,
      currentHx711CalculatedGrams,
      lastHx711MeasuredGrams
    );

    if (labs(currentHx711TaredCounts) > 10) {
      debugPrintf("[CAL TABLE] Suggested hx711CalibrationFactor values if current load equals:\n");

      for (int knownGrams = 100; knownGrams <= 1200; knownGrams += 100) {
        float suggestedFactor = currentHx711TaredCounts / (float)knownGrams;
        debugPrintf("  %4dg -> %.6f\n", knownGrams, suggestedFactor);
      }

      debugPrintf(
        "[CAL] Example: if the current weight is actually %.0fg, use factor %.6f\n",
        HX711_KNOWN_CAL_WEIGHT_GRAMS,
        currentHx711TaredCounts / HX711_KNOWN_CAL_WEIGHT_GRAMS
      );
    } else {
      debugPrintf("[CAL] Add a known weight to calculate suggested calibration factors.\n");
    }
  } else {
    debugPrintf("[HX711 LIVE] No raw/tared sample captured yet.\n");
  }
}

void initHx711() {
  debugPrintf("\n========== HX711 INIT ==========" "\n");
  debugPrintf("Expected wiring:\n");
  debugPrintf("HX711 VCC  -> ESP32 3V3\n");
  debugPrintf("HX711 GND  -> ESP32 GND\n");
  debugPrintf("HX711 DT   -> ESP32 GPIO%d\n", HX711_DOUT);
  debugPrintf("HX711 SCK  -> ESP32 GPIO%d\n", HX711_SCK);
  debugPrintf("Using stable wiring target: DT=GPIO18, SCK=GPIO21.\n");
  debugPrintf("Calibration helper known weight = %.0fg. Change HX711_KNOWN_CAL_WEIGHT_GRAMS if using a different weight.\n", HX711_KNOWN_CAL_WEIGHT_GRAMS);

  spoolScale.begin(HX711_DOUT, HX711_SCK);
  spoolScale.set_scale(hx711CalibrationFactor);
  hx711Ready = false;
  hx711StartupTareDone = false;
  hx711EverReady = false;
  hx711NotReadyStartedMs = millis();
  lastHx711MeasurementValid = false;
  delay(500);

  printHx711BenchDebug(true);

  if (spoolScale.is_ready()) {
    hx711Ready = true;
    hx711EverReady = true;
    hx711NotReadyStartedMs = 0;

    debugPrintf("HX711 ready at startup. Taring ONCE with no load...\n");
    spoolScale.tare(15);
    hx711StartupTareDone = true;
    debugPrintf("HX711 startup tare complete.\n");
    printHx711BenchDebug(true);
  } else {
    debugPrintf("HX711 not ready at startup. Will wait without repeatedly taring.\n");
    debugPrintf("If it becomes ready later, the code will do one startup tare only.\n");
  }

  debugPrintf("======== END HX711 INIT ========\n\n");
}
void updateHx711WeightInternal() {
  printHx711BenchDebug(false);

  if (millis() - lastWeightInternalUpdate < HX711_READ_INTERVAL_MS) {
    return;
  }

  lastWeightInternalUpdate = millis();

  bool readyNow = spoolScale.is_ready();

  if (!readyNow) {
    if (hx711NotReadyStartedMs == 0) {
      hx711NotReadyStartedMs = millis();
    }

    unsigned long notReadyMs = millis() - hx711NotReadyStartedMs;

    // DOUT/ready can briefly go HIGH/NO between conversions. Only call it
    // lost if it remains not-ready for a sustained period.
    if (hx711Ready && notReadyMs >= HX711_NOT_READY_GRACE_MS) {
      hx711Ready = false;
      lastHx711MeasurementValid = false;
      debugPrintf("HX711 has been not-ready for %lu ms. Check DT/SCK/VCC/GND if this persists.\n", notReadyMs);
    }

    // Keep the last displayed weight instead of bouncing the UI to 0g during
    // normal between-sample not-ready windows.
    if (!hx711EverReady && notReadyMs >= HX711_NOT_READY_GRACE_MS) {
      fakeWeightGrams[selectedTool] = 0;
    }

    return;
  }

  // Ready now.
  hx711NotReadyStartedMs = 0;

  if (!hx711Ready) {
    hx711Ready = true;
    hx711EverReady = true;
    spoolScale.set_scale(hx711CalibrationFactor);
    debugPrintf("HX711 ready/recovered.\n");

    if (!hx711StartupTareDone) {
      debugPrintf("Performing one startup tare with no load...\n");
      spoolScale.tare(15);
      hx711StartupTareDone = true;
      debugPrintf("Startup tare complete.\n");
      return;
    }
  }

  // Capture raw and tared counts first. These values are independent of the
  // current calibration factor, so they are the best debug/calibration source.
  prevHx711RawCounts = currentHx711RawCounts;
  prevHx711TaredCounts = currentHx711TaredCounts;
  prevHx711CalculatedGrams = currentHx711CalculatedGrams;

  currentHx711RawCounts = spoolScale.read_average(1);
  currentHx711TaredCounts = spoolScale.get_value(1);

  if (fabs(hx711CalibrationFactor) > 0.000001f) {
    currentHx711CalculatedGrams = currentHx711TaredCounts / hx711CalibrationFactor;
  } else {
    currentHx711CalculatedGrams = 0.0f;
  }

  hx711RawDebugValid = true;

  // Use the same calculated value for the UI, but do not let the UI hide the
  // raw/tared values. Serial Monitor will still show negative/current values.
  float measuredGramsFloat = fabs(currentHx711CalculatedGrams);
  int measuredGrams = (int)lround(measuredGramsFloat);

  // Suppress tiny negative drift around zero after tare.
  if (measuredGrams < 0 && measuredGrams > -5) {
    measuredGrams = 0;
  }

  measuredGrams = clampRawWeight(measuredGrams);

  lastHx711MeasuredGrams = measuredGrams;
  lastHx711MeasurementValid = true;

  // First POC has one physical load cell. The selected tool screen shows that
  // live reading while the tool label/material UI can still be exercised.
  fakeWeightGrams[selectedTool] = measuredGrams;
}

void updateWeightDisplayIfNeeded() {
  if (uiSleeping) {
    return;
  }

  if (settingsOpen) {
    return;
  }

  if (currentPage != PAGE_WEIGHT &&
      currentPage != PAGE_TOOL &&
      currentPage != PAGE_MATERIAL) {
    return;
  }

  if (recentlyTouched()) {
    return;
  }

  if (millis() - lastDisplayWeightUpdate >= LIVE_WEIGHT_DISPLAY_UPDATE_MS) {
    lastDisplayWeightUpdate = millis();
    drawCurrentPage();
  }
}

void updateSettingsTickerIfNeeded() {
  if (!settingsOpen) {
    return;
  }

  if (settingsView != SETTINGS_MAIN || settingsIndex != SETTING_FILAMENTS) {
    return;
  }

  if (millis() - lastSettingsTickerRedraw > SETTINGS_TICKER_MS) {
    lastSettingsTickerRedraw = millis();
    drawCurrentPage();
  }
}

void updateAdvancedSettingsHintIfNeeded() {
  if (settingsOpen || advHintCleared) {
    return;
  }

  if (millis() - uiStartMs > ADV_SETTINGS_HINT_MS) {
    advHintCleared = true;
    drawCurrentPage();
  }
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  delay(2500);
  setvbuf(stdout, NULL, _IONBF, 0);

  // This board profile shows Serial0 and printf in Serial Monitor.
  // Do not wait on normal Serial here, because normal Serial may never attach
  // on this Waveshare ESP32-S3 board profile.

  debugPrintf("\n!!!!!!!! SPOOLSENSE HX711 STABLE-READY DEBUG BUILD IS RUNNING !!!!!!!!\n");
  debugPrintf("If you see Waveshare messages but not this line, the debug route is still not attached.\n");
  debugPrintf("================================\n");
  debugPrintf("SPOOLSENSE STARTING\n");
  debugPrintf("Serial Monitor route: Serial0 + printf at 115200 baud\n");
  debugPrintf("HX711 pins: DOUT=GPIO%d, SCK=GPIO%d\n", HX711_DOUT, HX711_SCK);
  debugPrintf("================================\n");
  delay(1000);
  randomSeed((uint32_t)esp_random());

  debugPrintf("Starting SpoolSense U1 circular module UI...\n");

  touch.begin();

  bool hasPSRAM = psramInit();

  if (hasPSRAM) {
    debugPrintf("PSRAM is correctly initialized\n");
    BlackImage = (UWORD *)ps_malloc(Imagesize);
  } else {
    debugPrintf("PSRAM not available, using normal RAM\n");
    BlackImage = (UWORD *)malloc(Imagesize);
  }

  if (BlackImage == NULL) {
    debugPrintf("Failed to allocate screen memory.\n");

    while (true) {
      delay(1000);
    }
  }

  if (DEV_Module_Init() != 0) {
    debugPrintf("GPIO Init Fail!\n");
  } else {
    debugPrintf("GPIO Init successful!\n");
  }

  LCD_1IN28_Init(HORIZONTAL);
  LCD_1IN28_Clear(UI_BG);

  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, UI_BG);
  Paint_SetScale(65);
  Paint_SetRotate(ROTATE_0);
  Paint_Clear(UI_BG);

  // Only show SpoolSense startup UI while the device is actually initializing.
  // There is no forced delay, so it will not slow down normal boot.
  drawBootScreen("STARTING...");

  loadSettings();
  initHx711();

  currentPage = startupPagePreference;
  uiStartMs = millis();
  lastUserInteractionMs = millis();

  lastWeightInternalUpdate = millis();
  lastDisplayWeightUpdate = millis();

  drawCurrentPage();

  debugPrintf("SpoolSense U1 circular module UI running.\n");
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {
  checkTouch();

  updateHx711WeightInternal();
  printHx711GpioDiagnostics(false);
  updateWeightDisplayIfNeeded();
  updateSettingsTickerIfNeeded();
  updateDimLevelPreviewIfNeeded();
  updateAdvancedSettingsHintIfNeeded();
  updateDimAndSleepIfNeeded();

  delay(10);
}
