#include <Arduino.h>
#include <driver/twai.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

// ================== CAN B (TWAI) ==================
#define CAN_TX 7
#define CAN_RX 6

// ================== OLED 1.5" SSD1351 SPI (SOFTWARE SPI) ==================
#define OLED_MOSI 4
#define OLED_SCK  5
#define OLED_CS   16
#define OLED_DC   15
#define OLED_RST  17

Adafruit_SSD1351 display = Adafruit_SSD1351(128, 128, OLED_CS, OLED_DC, OLED_MOSI, OLED_SCK, OLED_RST);

// ===== kolory 565 =====
static const uint16_t C_BLACK  = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_CYAN   = 0x07FF;
static const uint16_t C_YELL   = 0xFFE0;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_ORANGE = 0xFD20;  // pomarańczowy w formacie 565

// ================== Przycisk ==================
#define PIN_PRZYCISKU 21

// ================== Marketing/logo 0x001 ==================
static const uint32_t MARKETING_INTERVAL = 200;
static uint32_t lastMarketingTime = 0;
static int marketingStep = 0;

const uint8_t adPart1[8] = {0x69, 0x6E, 0x73, 0x74, 0x61, 0x6C, 0x61, 0x63};
const uint8_t adPart2[8] = {0x6A, 0x65, 0x6F, 0x66, 0x66, 0x72, 0x6F, 0x61};
const uint8_t adPart3[8] = {0x64, 0x2E, 0x70, 0x6C, 0x03, 0x02, 0x00, 0x07};

// ================== AUTO-WYBÓR TRYBU CAN ==================
enum CanMode : uint8_t { CAN_UNKNOWN = 0, CAN_OLD = 1, CAN_NEW = 2 };
static CanMode canMode = CAN_UNKNOWN;

static uint32_t lastOldSeenMs = 0;
static uint32_t lastNewSeenMs = 0;
static const uint32_t CAN_MODE_TIMEOUT_MS = 800;

static inline void updateCanModeFromSeen() {
  const uint32_t now = millis();
  const bool newAlive = (now - lastNewSeenMs) < CAN_MODE_TIMEOUT_MS;
  const bool oldAlive = (now - lastOldSeenMs) < CAN_MODE_TIMEOUT_MS;
  if (newAlive) canMode = CAN_NEW;
  else if (oldAlive) canMode = CAN_OLD;
  else canMode = CAN_UNKNOWN;
}

// ================== Stan bieżący ==================
char bieg = -99;
char tryb = -99;
int8_t zmia = 0;

int  tSkrzyni = -99;
int  tSilnika = -99;

int stanPrzycisku = HIGH;

// ================== Stan poprzedni ==================
char poprzbieg = -99;
char poprztryb = -99;
int8_t poprzzmia = 0;
int poptSkrzyni = -99;
int poptSilnika = -99;

// ================== STRZAŁKA — latch ==================
static int8_t  zmiaUI = 0;
static int8_t  zmiaLatch = 0;
static uint32_t zmiaLatchMs = 0;
static const uint32_t ARROW_MIN_SHOW_MS = 180;
static char biegUI = -99;

static void updateArrowUI() {
  const uint32_t now = millis();
  if (zmia != 0) {
    zmiaLatch = zmia;
    zmiaLatchMs = now;
    zmiaUI = zmia;
    biegUI = bieg;
  } else if (zmiaLatch != 0) {
    if ((now - zmiaLatchMs) < ARROW_MIN_SHOW_MS) {
      zmiaUI = zmiaLatch;
    } else {
      zmiaUI = 0;
      zmiaLatch = 0;
      biegUI = bieg;
    }
  } else {
    zmiaUI = 0;
    biegUI = bieg;
  }
}

// ================== DEBOUNCE trybu P/R/N ==================
static char trybUI = -99;
static char poprzTrybUI = -99;

static bool prnDebounceActive = false;
static uint32_t prnDebounceStartMs = 0;
static const uint32_t PRN_DEBOUNCE_MS = 80;

static inline bool isDriveMode(char t) { return (t == 'D' || t == 'S' || t == 'M'); }
static inline bool isPRN(char t) { return (t == 'P' || t == 'R' || t == 'N'); }
static inline bool isPRN_ui() { return isPRN(trybUI); }

static void updateTrybUI() {
  const uint32_t now = millis();
  if (prnDebounceActive) {
    if (tryb != 'N') { prnDebounceActive = false; trybUI = tryb; return; }
    if ((now - prnDebounceStartMs) >= PRN_DEBOUNCE_MS) { prnDebounceActive = false; trybUI = 'N'; return; }
    return;
  }
  if (tryb == 'N' && isDriveMode(trybUI)) {
    prnDebounceActive = true;
    prnDebounceStartMs = now;
    return;
  }
  trybUI = tryb;
}

// ================== LAYOUT ==================
//
// size 5: znak = 30w × 40h
// size 3: znak = 18w × 24h  (dwukropek)
// size 2: znak = 12w × 16h  (strzałka)
//
// Górna kreska: y=25   (pod ENG)
// Dolna kreska: y=104  (nad GEAR)
// Strefa środka: y=27..102  (H=76)
// Margin góra: 2px,  Margin dół: 2px od kreski
//
// Centrowanie size5: 27 + (76-40)/2 = 27+18 = 45
//
// Layout "T : B ^"
//   T(30) + gap(4) + :(18) + gap(4) + B(30) + gap(4) + arrow(12) = 102
//   startX = (128 - 102) / 2 = 13
//
// Layout PRN solo:
//   (128 - 30) / 2 = 49
//

static const int Y_ENG_ROW   = 7;
static const int X_ENG_LABEL = 0;
static const int X_ENG_NUM   = 50;

static const int Y_GBX_ROW   = 108;
static const int X_GBX_LABEL = 0;
static const int X_GBX_NUM   = 60;

static const int MID_MARGIN  = 2;                                // margines góra/dół od kresek
static const int Y_MID_TOP   = 25 + MID_MARGIN;                  // 27  (2px pod górną kreską)
static const int Y_MID_BOT   = 104 - MID_MARGIN;                 // 102 (2px nad dolną kreską)
static const int H_MID       = Y_MID_BOT - Y_MID_TOP;            // 75

// size 5 wymiary
static const int S5_W = 30;
static const int S5_H = 40;

// size 3 wymiary (dwukropek)
static const int S3_W = 18;
static const int S3_H = 24;

// size 2 wymiary (strzałka)
static const int S2_W = 12;
static const int S2_H = 16;

// Centrowanie w pionie
static const int Y_MID_CY = Y_MID_TOP + (H_MID - S5_H) / 2;    // 27 + 17 = 44

// Layout DSM: "T : B ^"
static const int X_MODE_DSM  = 13;
static const int X_COLON     = X_MODE_DSM + S5_W + 4;            // 47
static const int X_GEAR_CHAR = X_COLON + S3_W + 4;               // 69
static const int X_ARROW     = X_GEAR_CHAR + S5_W + 4;           // 103

// Dwukropek: centrowany w pionie względem size5
static const int Y_COLON = Y_MID_CY + (S5_H - S3_H) / 2;       // 44+8 = 52

// Strzałka: góra/dół w strefie środka
static const int Y_ARROW_UP = Y_MID_CY;                          // 44
static const int Y_ARROW_DN = Y_MID_CY + S5_H - S2_H;           // 44+40-16 = 68

// Layout PRN solo
static const int X_MODE_SOLO = (128 - S5_W) / 2;                 // 49

// Temp: rozmiar znaku textSize=2
static const int CHAR_W = 12;
static const int CHAR_H = 16;

static const uint32_t UI_MIN_INTERVAL_MS = 20;
static uint32_t lastUiDraw = 0;

// ================== Cache — środek ==================
static char drawnLayout = 0;
static char drawnModeChar = 0;
static char drawnGearChar = 0;
static int8_t drawnArrow = 0;

// ================== Cache temp ==================
static char engPrevBuf[8] = "";
static uint8_t engPrevLen = 0;
static bool engSuffixDrawn = false;

static char gbxPrevBuf[8] = "";
static uint8_t gbxPrevLen = 0;
static bool gbxSuffixDrawn = false;

// ================== SHIFT watchdog ==================
static uint32_t oldShiftStartMs = 0;
static bool oldShiftActive = false;
static uint32_t newShiftStartMs = 0;
static bool newShiftActive = false;
static const uint32_t SHIFT_TIMEOUT_MS = 250;

// ================== Splash screen ==================
static const uint32_t SPLASH_DURATION_MS = 2000;  // czas wyświetlania splash screenu

// ================== Helpers ==================
static inline uint8_t bsfp1(uint8_t x) {
  uint8_t res = x != 0;
  while (res && (x & 1) == 0) { ++res; x >>= 1; }
  return res;
}
static inline int tempNew(uint8_t raw) { return (int)raw - 48; }
static inline int tempOld(uint8_t raw) { return (raw * 20 - 50 * 26 + 13) / 26; }
static inline bool isGearDigit(char g) { return (g >= '1' && g <= '6'); }
static inline bool isOneHot(uint8_t x) { return x && ((x & (x - 1)) == 0); }

// ================== TWAI send ==================
static void twaiSendStd(uint32_t id, const uint8_t *data) {
  twai_message_t tx = {};
  tx.identifier = id;
  tx.extd = 0; tx.rtr = 0; tx.data_length_code = 8;
  for (int i = 0; i < 8; i++) tx.data[i] = data[i];
  twai_transmit(&tx, pdMS_TO_TICKS(0));
}

static void handleMarketingSender() {
  uint32_t now = millis();
  if (now - lastMarketingTime < MARKETING_INTERVAL) return;
  lastMarketingTime = now;
  switch (marketingStep) {
    case 0: twaiSendStd(0x001, adPart1); marketingStep = 1; break;
    case 1: twaiSendStd(0x001, adPart2); marketingStep = 2; break;
    default: twaiSendStd(0x001, adPart3); marketingStep = 0; break;
  }
}

static void handleButton() {
  int tmp = digitalRead(PIN_PRZYCISKU);
  if (tmp != stanPrzycisku) stanPrzycisku = tmp;
}

// ================== Splash screen ==================

static void drawSplashScreen() {
  display.fillScreen(C_BLACK);

  // "Escape" — textSize 3: znak = 18w × 24h, 6 znaków = 108px
  // centrowanie X: (128 - 108) / 2 = 10
  // Dwa wiersze łącznie: 24 + 4(gap) + 24 = 52
  // centrowanie Y: (128 - 52) / 2 = 38
  const int splashGap = 4;
  const int line1W = 6 * 18;  // "Escape" = 6 znaków × 18px
  const int line2W = 3 * 18;  // "4x4"   = 3 znaki  × 18px
  const int totalH = 24 + splashGap + 24;
  const int startY = (128 - totalH) / 2;

  // Linia 1: "Escape" — biały
  display.setTextSize(3);
  display.setTextColor(C_WHITE);
  display.setCursor((128 - line1W) / 2, startY);
  display.print("Escape");

  // Linia 2: "4x4" — pomarańczowy
  display.setTextSize(3);
  display.setTextColor(C_ORANGE);
  display.setCursor((128 - line2W) / 2, startY + 24 + splashGap);
  display.print("4x4");
}

// ================== UI — temperatura ==================

static void drawSuffix(int16_t x, int16_t y, uint16_t color) {
  display.setTextSize(2);
  display.setTextColor(color);
  display.setCursor(x, y);
  display.write(248);
  display.print("C");
}

static void clearCharAt(int16_t baseX, int16_t baseY, uint8_t idx) {
  display.fillRect(baseX + idx * CHAR_W, baseY, CHAR_W, CHAR_H, C_BLACK);
}

static void drawTempSmart(int16_t baseX, int16_t baseY, uint16_t color,
                          int newVal,
                          char *prevBuf, uint8_t &prevLen, bool &suffixDrawn)
{
  char newBuf[8];
  if (newVal != -99) snprintf(newBuf, sizeof(newBuf), "%d", newVal);
  else strcpy(newBuf, "--");

  uint8_t newLen = (uint8_t)strlen(newBuf);
  uint8_t oldLen = prevLen;
  bool lenChanged = (newLen != oldLen);

  if (lenChanged || !suffixDrawn) {
    if (suffixDrawn && oldLen > 0) {
      int16_t oldSuffX = baseX + oldLen * CHAR_W + 2;
      display.fillRect(oldSuffX - 1, baseY - 1, 30, CHAR_H + 2, C_BLACK);
    }
    uint8_t maxLen = (oldLen > newLen) ? oldLen : newLen;
    if (maxLen > 0) display.fillRect(baseX, baseY, maxLen * CHAR_W, CHAR_H, C_BLACK);

    display.setTextSize(2);
    display.setTextColor(color);
    display.setCursor(baseX, baseY);
    display.print(newBuf);

    int16_t newSuffX = baseX + newLen * CHAR_W + 2;
    drawSuffix(newSuffX, baseY, color);
    suffixDrawn = true;
  } else {
    display.setTextSize(2);
    display.setTextColor(color);
    for (uint8_t i = 0; i < newLen; i++) {
      if (newBuf[i] != prevBuf[i]) {
        clearCharAt(baseX, baseY, i);
        display.setCursor(baseX + i * CHAR_W, baseY);
        display.print(newBuf[i]);
      }
    }
  }
  strcpy(prevBuf, newBuf);
  prevLen = newLen;
}

// ================== UI — środek ==================

static uint16_t modeColor(char t) {
  switch (t) {
    case 'P': return C_RED;
    case 'R': return C_YELL;
    case 'N': return C_WHITE;
    case 'D': return C_WHITE;
    case 'S': return C_CYAN;
    case 'M': return C_CYAN;
    case 'G': return C_RED;
    default:  return C_WHITE;
  }
}

static void clearMidAll() {
  display.fillRect(0, Y_MID_TOP, 128, H_MID, C_BLACK);
  drawnModeChar = 0; drawnGearChar = 0; drawnArrow = 0; drawnLayout = 0;
}

// Pole: tryb DSM
static void clearModeField() {
  display.fillRect(X_MODE_DSM, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawModeFieldDSM(char ch, uint16_t color) {
  display.setTextSize(5);
  display.setTextColor(color);
  display.setCursor(X_MODE_DSM, Y_MID_CY);
  display.print(ch);
}

// Pole: dwukropek
static void drawColonField() {
  display.setTextSize(3);
  display.setTextColor(C_WHITE);
  display.setCursor(X_COLON, Y_COLON);
  display.print(':');
}

// Pole: bieg
static void clearGearField() {
  display.fillRect(X_GEAR_CHAR, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawGearField(char ch) {
  display.setTextSize(5);
  display.setTextColor(C_WHITE);
  display.setCursor(X_GEAR_CHAR, Y_MID_CY);
  display.print(ch);
}

// Pole: strzałka (size 2, po prawej od biegu)
static void clearArrowField() {
  display.fillRect(X_ARROW, Y_MID_CY, S2_W, S5_H, C_BLACK);
}
static void drawArrowField(int8_t dir) {
  if (dir == 0) return;
  display.setTextSize(2);
  display.setTextColor(dir > 0 ? C_GREEN : C_RED);
  display.setCursor(X_ARROW, (dir > 0) ? Y_ARROW_UP : Y_ARROW_DN);
  display.print(dir > 0 ? '^' : 'v');
}

// Pole: tryb solo (PRN)
static void clearModeSoloField() {
  display.fillRect(X_MODE_SOLO, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawModeSoloField(char ch, uint16_t color) {
  display.setTextSize(5);
  display.setTextColor(color);
  display.setCursor(X_MODE_SOLO, Y_MID_CY);
  display.print(ch);
}

// ================== updateMidArea ==================

static void updateMidArea() {
  char wantLayout;
  if (trybUI == 'G')       wantLayout = 'G';
  else if (isPRN_ui())     wantLayout = 'P';
  else if (trybUI == -99)  wantLayout = 'P';
  else                     wantLayout = 'D';

  if (wantLayout != drawnLayout) {
    clearMidAll();
    drawnLayout = wantLayout;

    if (wantLayout == 'G') {
      display.setTextSize(3);
      display.setTextColor(C_RED);
      display.setCursor(19, Y_MID_TOP + (H_MID - S3_H) / 2);
      display.print("FAULT");
      drawnModeChar = 'G';
      return;
    }

    if (wantLayout == 'P') {
      char ch = (trybUI != -99) ? trybUI : '-';
      drawModeSoloField(ch, modeColor(trybUI));
      drawnModeChar = ch;
      drawnArrow = 0;
      return;
    }

    // 'D' layout
    char mch = (trybUI != -99) ? trybUI : '-';
    drawModeFieldDSM(mch, modeColor(trybUI));
    drawnModeChar = mch;

    drawColonField();

    char gch = isGearDigit(biegUI) ? biegUI : '-';
    drawGearField(gch);
    drawnGearChar = gch;

    drawArrowField(zmiaUI);
    drawnArrow = zmiaUI;
    return;
  }

  if (drawnLayout == 'G') return;

  if (drawnLayout == 'P') {
    char ch = (trybUI != -99) ? trybUI : '-';
    if (ch != drawnModeChar) {
      clearModeSoloField();
      drawModeSoloField(ch, modeColor(trybUI));
      drawnModeChar = ch;
    }
    return;
  }

  // 'D' — per-pole

  char mch = (trybUI != -99) ? trybUI : '-';
  if (mch != drawnModeChar) {
    clearModeField();
    drawModeFieldDSM(mch, modeColor(trybUI));
    drawnModeChar = mch;
  }

  char gch = isGearDigit(biegUI) ? biegUI : '-';
  if (gch != drawnGearChar) {
    clearGearField();
    drawGearField(gch);
    drawnGearChar = gch;
  }

  if (zmiaUI != drawnArrow) {
    clearArrowField();
    drawArrowField(zmiaUI);
    drawnArrow = zmiaUI;
  }
}

// ================== UI — main ==================

static void drawStaticUI() {
  display.fillScreen(C_BLACK);
  display.drawFastHLine(0, 25, 128, C_CYAN);    // górna kreska (pod ENG)
  display.drawFastHLine(0, 104, 128, C_CYAN);   // dolna kreska (nad GEAR)

  display.setTextSize(2);
  display.setTextColor(C_CYAN);
  display.setCursor(X_ENG_LABEL, Y_ENG_ROW);
  display.print("ENG:");

  display.setTextColor(C_CYAN);
  display.setCursor(X_GBX_LABEL, Y_GBX_ROW);
  display.print("GEAR:");

  engPrevBuf[0] = '\0'; engPrevLen = 0; engSuffixDrawn = false;
  gbxPrevBuf[0] = '\0'; gbxPrevLen = 0; gbxSuffixDrawn = false;
  drawnLayout = 0; drawnModeChar = 0; drawnGearChar = 0; drawnArrow = 0;
}

static void drawEngTemp() {
  drawTempSmart(X_ENG_NUM, Y_ENG_ROW, C_YELL,
                tSilnika, engPrevBuf, engPrevLen, engSuffixDrawn);
}

static void drawGbxTemp() {
  drawTempSmart(X_GBX_NUM, Y_GBX_ROW, C_WHITE,
                tSkrzyni, gbxPrevBuf, gbxPrevLen, gbxSuffixDrawn);
}

static void updateUI(bool forceAll) {
  updateTrybUI();
  updateArrowUI();

  uint32_t now = millis();
  if (!forceAll && (now - lastUiDraw) < UI_MIN_INTERVAL_MS) return;
  lastUiDraw = now;

  if (forceAll) {
    engPrevBuf[0] = '\0'; engPrevLen = 0; engSuffixDrawn = false;
    gbxPrevBuf[0] = '\0'; gbxPrevLen = 0; gbxSuffixDrawn = false;
    drawnLayout = 0; drawnModeChar = 0; drawnGearChar = 0; drawnArrow = 0;
    zmiaLatch = 0; zmiaUI = 0; biegUI = bieg;
  }

  bool midChanged = forceAll
                    || (poprzTrybUI != trybUI)
                    || (drawnGearChar != (isGearDigit(biegUI) ? biegUI : '-'))
                    || (drawnArrow != zmiaUI);
  bool engChanged = forceAll || (poptSilnika != tSilnika);
  bool gbxChanged = forceAll || (poptSkrzyni != tSkrzyni);

  if (midChanged) updateMidArea();
  if (engChanged) drawEngTemp();
  if (gbxChanged) drawGbxTemp();
}

static void commitPops() {
  poprzbieg   = bieg;
  poprzzmia   = zmia;
  poprztryb   = tryb;
  poprzTrybUI = trybUI;
  poptSkrzyni = tSkrzyni;
  poptSilnika = tSilnika;
}

// ================== CAN init ==================
static bool initCanB_500k() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  g.tx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_PRZYCISKU, INPUT_PULLUP);

  display.begin();
  display.cp437(true);
  display.setRotation(2);

  // ===== Splash screen przy starcie =====
  drawSplashScreen();
  delay(SPLASH_DURATION_MS);

  drawStaticUI();

  if (!initCanB_500k()) {
    display.setTextColor(C_RED);
    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print("CAN FAIL");
    while (1) delay(1000);
  }

  updateUI(true);
  commitPops();
}

void loop() {
  handleButton();
  handleMarketingSender();

  bool anyStateChanged = false;
  twai_message_t msg;

  while (twai_receive(&msg, pdMS_TO_TICKS(0)) == ESP_OK) {
    if (msg.extd) continue;

    const uint32_t rxId = msg.identifier;
    const uint8_t len = msg.data_length_code;

    uint8_t rxBuf[8] = {0};
    for (int i = 0; i < 8 && i < len; i++) rxBuf[i] = msg.data[i];

    if (rxId == 0x43F || rxId == 0x43B || rxId == 0x329) lastOldSeenMs = millis();
    if (rxId == 0x1D2 || rxId == 0x0BA || rxId == 0x0B5 || rxId == 0x1D0) lastNewSeenMs = millis();
    if (canMode == CAN_UNKNOWN) {
      if (rxId == 0x1D2 || rxId == 0x0BA || rxId == 0x0B5 || rxId == 0x1D0) canMode = CAN_NEW;
      else if (rxId == 0x43F || rxId == 0x43B || rxId == 0x329) canMode = CAN_OLD;
    }

    // ================== OLD CAN ==================
    if (canMode == CAN_OLD) {
      switch (rxId) {
        case 0x43F: {
          int8_t newZm = 0;
          char newTryb = tryb;
          char newBieg = bieg;

          const uint32_t nowMs = millis();
          const bool shiftFlag = (rxBuf[0] & 0x08);

          if (shiftFlag) {
            if (!oldShiftActive) { oldShiftActive = true; oldShiftStartMs = nowMs; }
          } else {
            oldShiftActive = false;
          }
          const bool shiftTimedOut = oldShiftActive && ((nowMs - oldShiftStartMs) > SHIFT_TIMEOUT_MS);

          uint8_t D2 = rxBuf[1] & 0x0F;
          if (D2 == 0) {
            newTryb = 'G'; newBieg = -99; newZm = 0;
            oldShiftActive = false;
          } else {
            const char* map = "N123456R";
            char b = map[rxBuf[0] & 0x07];

            if (b >= '1' && b <= '6') {
              uint8_t D3 = rxBuf[2] >> 5;
              const char* tmap = "EMS";
              newTryb = (D3 > 2) ? 'D' : tmap[D3];

              if (newTryb != 'M' && shiftFlag && !shiftTimedOut &&
                  poprzbieg >= '1' && poprzbieg <= '6') {
                newZm = (int8_t)((int)b - (int)poprzbieg);
                newBieg = poprzbieg;
              } else {
                newBieg = b; newZm = 0;
                if (!shiftFlag) oldShiftActive = false;
              }
            } else {
              newTryb = (D2 == 8) ? 'P' : b;
              newBieg = -99; newZm = 0;
              oldShiftActive = false;
            }
          }

          if (newTryb == 'P' || newTryb == 'R' || newTryb == 'N') {
            newBieg = -99; newZm = 0;
            oldShiftActive = false;
          }

          if (newTryb != tryb || newBieg != bieg || newZm != zmia) {
            tryb = newTryb; bieg = newBieg; zmia = newZm;
            anyStateChanged = true;
          }
        } break;

        case 0x43B:
          if (len >= 1) {
            int v = (int)rxBuf[0] - 55;
            if (v != tSkrzyni) { tSkrzyni = v; anyStateChanged = true; }
          }
          break;

        case 0x329:
          if (len >= 2) {
            int v = tempOld(rxBuf[1]);
            if (v != tSilnika) { tSilnika = v; anyStateChanged = true; }
          }
          break;

        default: break;
      }
    }

    // ================== NEW CAN ==================
    if (canMode == CAN_NEW) {
      switch (rxId) {
        case 0x1D2: {
          uint8_t pos = rxBuf[0] & 0x0F;
          uint8_t tmp = 0;
          if (pos == 0) tmp = 0;
          else if (isOneHot(pos)) tmp = bsfp1(pos);
          else tmp = pos;
          if (tmp == 4) tmp += (rxBuf[4] & 0x03);
          if (tmp > 7) tmp = 7;
          char newTryb = "0PRNDSM?"[tmp];
          if (newTryb != tryb) { tryb = newTryb; anyStateChanged = true; }
          if (tryb == 'P' || tryb == 'R' || tryb == 'N') {
            if (bieg != -99 || zmia != 0) { bieg = -99; zmia = 0; anyStateChanged = true; }
            newShiftActive = false;
          }
        } break;

        case 0x0BA: {
          if (isPRN(tryb) || tryb == 'G') {
            if (bieg != -99 || zmia != 0) { bieg = -99; zmia = 0; anyStateChanged = true; }
            newShiftActive = false;
            break;
          }
          const uint32_t nowMs = millis();
          const bool shiftFlag = (rxBuf[0] & 0x10);
          if (shiftFlag) {
            if (!newShiftActive) { newShiftActive = true; newShiftStartMs = nowMs; }
          } else {
            newShiftActive = false;
          }
          const bool shiftTimedOut = newShiftActive && ((nowMs - newShiftStartMs) > SHIFT_TIMEOUT_MS);

          char newBieg = (char)((rxBuf[0] & 0x0F) + ('0' - 4));
          int8_t newZm = 0;

          if (newBieg <= '0') {
            newBieg = -99; newZm = 0;
            newShiftActive = false;
          } else if (tryb != -99 && tryb != 'M' && shiftFlag && !shiftTimedOut &&
                     poprzbieg >= '1' && poprzbieg <= '6') {
            newZm = (int8_t)((int)newBieg - (int)poprzbieg);
            newBieg = poprzbieg;
          } else {
            newZm = 0;
            if (!shiftFlag) newShiftActive = false;
          }

          if (newBieg != bieg || newZm != zmia) { bieg = newBieg; zmia = newZm; anyStateChanged = true; }
        } break;

        case 0x0B5:
          if (len >= 8) {
            int v = (int)rxBuf[7] - 40;
            if (v != tSkrzyni) { tSkrzyni = v; anyStateChanged = true; }
          }
          break;

        case 0x1D0:
          if (len >= 1) {
            int v = tempNew(rxBuf[0]);
            if (v != tSilnika) { tSilnika = v; anyStateChanged = true; }
          }
          break;

        default: break;
      }
    }
  }

  CanMode prevMode = canMode;
  updateCanModeFromSeen();
  if (canMode != prevMode) anyStateChanged = true;

  {
    char prevTrybUI = trybUI;
    updateTrybUI();
    if (trybUI != prevTrybUI) anyStateChanged = true;
  }

  {
    int8_t prevZmiaUI = zmiaUI;
    char prevBiegUI = biegUI;
    updateArrowUI();
    if (zmiaUI != prevZmiaUI || biegUI != prevBiegUI) anyStateChanged = true;
  }

  if (anyStateChanged) {
    updateUI(false);
    commitPops();
  }
}