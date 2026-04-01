#include <Arduino.h>
#include <driver/twai.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

// ================== CAN B (TWAI) ==================
#define CAN_TX 7
#define CAN_RX 6

// ================== K-LINE ==================
#define K_LINE_RX 35
#define K_LINE_TX 38
#define K_LINE_BAUD 10400
#define TARGET_ADDR 0x12
#define TESTER_ADDR 0xF1

HardwareSerial kLineSerial(1);

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
static const uint16_t C_ORANGE = 0xFD20;

// ================== Przycisk — toggle góra/dół ==================
#define PIN_PRZYCISKU 21

// false = CAN (ENG/GEAR temp), true = K-Line (BOOST/IAT)
static bool showKLine = false;
static bool prevShowKLine = false;

// Debounce
static bool lastBtnReading = HIGH;
static bool btnState = HIGH;
static uint32_t btnDebounceMs = 0;
static const uint32_t BTN_DEBOUNCE = 50;

// ================== K-LINE — stan ==================
static uint8_t kResultBuffer[64] = {0};
static bool kLineConnected = false;
static String kConnectedProtocol = "";
static float kBoostBar = 0.0;
static float kBoostAbsMbar = 0.0;
static float kIAT = -99.0;
static float kATM = 985.0;
static bool kATMCalibrated = false;
static float kPeakBoost = 0.0;
static uint32_t kLastReadMs = 0;
static const uint32_t K_READ_INTERVAL = 30;
static uint8_t kCycleCount = 0;
static bool kPidDefined = false;
static uint32_t kLastConnectAttempt = 0;
static const uint32_t K_RECONNECT_INTERVAL = 3000;

static uint16_t _kByteWriteInterval = 5;
static uint16_t _kInterByteTimeout = 60;
static uint16_t _kReadTimeout = 200;

// ================== K-LINE — UI cache (góra/dół) ==================
static char kBoostPrevBuf[10] = "";
static uint8_t kBoostPrevLen = 0;
static bool kBoostSuffixDrawn = false;

static char kIATPrevBuf[10] = "";
static uint8_t kIATPrevLen = 0;
static bool kIATSuffixDrawn = false;

static bool kLabelsDrawn = false;

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
static const int Y_ENG_ROW   = 7;
static const int X_ENG_LABEL = 0;
static const int X_ENG_NUM   = 50;

static const int Y_GBX_ROW   = 108;
static const int X_GBX_LABEL = 0;
static const int X_GBX_NUM   = 60;

static const int MID_MARGIN  = 2;
static const int Y_MID_TOP   = 25 + MID_MARGIN;
static const int Y_MID_BOT   = 104 - MID_MARGIN;
static const int H_MID       = Y_MID_BOT - Y_MID_TOP;

static const int S5_W = 30;
static const int S5_H = 40;
static const int S3_W = 18;
static const int S3_H = 24;
static const int S2_W = 12;
static const int S2_H = 16;

static const int Y_MID_CY = Y_MID_TOP + (H_MID - S5_H) / 2;

static const int X_MODE_DSM  = 13;
static const int X_COLON     = X_MODE_DSM + S5_W + 4;
static const int X_GEAR_CHAR = X_COLON + S3_W + 4;
static const int X_ARROW     = X_GEAR_CHAR + S5_W + 4;

static const int Y_COLON = Y_MID_CY + (S5_H - S3_H) / 2;

static const int Y_ARROW_UP = Y_MID_CY;
static const int Y_ARROW_DN = Y_MID_CY + S5_H - S2_H;

static const int X_MODE_SOLO = (128 - S5_W) / 2;

static const int CHAR_W = 12;
static const int CHAR_H = 16;

static const uint32_t UI_MIN_INTERVAL_MS = 20;
static uint32_t lastUiDraw = 0;

// ================== Cache — środek ==================
static char drawnLayout = 0;
static char drawnModeChar = 0;
static char drawnGearChar = 0;
static int8_t drawnArrow = 0;

// ================== Cache — CAN temp (góra/dół) ==================
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
static const uint32_t SPLASH_DURATION_MS = 2000;

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

// ================== Przycisk — toggle góra/dół ==================
static void handleButton() {
  int reading = digitalRead(PIN_PRZYCISKU);
  if (reading != lastBtnReading) btnDebounceMs = millis();
  if ((millis() - btnDebounceMs) > BTN_DEBOUNCE) {
    if (reading != btnState) {
      btnState = reading;
      if (btnState == LOW) {           // zbocze opadające = naciśnięcie
        showKLine = !showKLine;
      }
    }
  }
  lastBtnReading = reading;
}

// ================================================================
//  K-LINE — KOMUNIKACJA
// ================================================================

static uint8_t kCalcChecksum(const uint8_t *data, uint8_t length) {
  uint8_t cs = 0;
  for (int i = 0; i < length; i++) cs += data[i];
  return cs;
}

static void kClearEcho() {
  kLineSerial.flush();
  delay(20);
  while (kLineSerial.available()) kLineSerial.read();
}

static uint8_t kReadData() {
  unsigned long startMs = millis();
  int bytesRead = 0;
  while (millis() - startMs < _kReadTimeout) {
    if (kLineSerial.available() > 0) {
      unsigned long lastByteTime = millis();
      memset(kResultBuffer, 0, sizeof(kResultBuffer));
      while (millis() - lastByteTime < _kInterByteTimeout) {
        if (kLineSerial.available() > 0) {
          if (bytesRead >= (int)sizeof(kResultBuffer)) return bytesRead;
          kResultBuffer[bytesRead++] = kLineSerial.read();
          lastByteTime = millis();
        }
      }
      return bytesRead;
    }
  }
  return 0;
}

static void kSendKWP(const uint8_t *data, uint8_t length) {
  uint8_t cs = kCalcChecksum(data, length);
  for (int i = 0; i < length; i++) {
    kLineSerial.write(data[i]);
    delay(_kByteWriteInterval);
  }
  kLineSerial.write(cs);
  delay(_kByteWriteInterval);
  kClearEcho();
}

static void kSend5baud(uint8_t data) {
  uint8_t even = 1;
  uint8_t bits[10];
  bits[0] = 0; bits[9] = 1;
  for (int i = 1; i <= 7; i++) {
    bits[i] = (data >> (i - 1)) & 1;
    even ^= bits[i];
  }
  bits[8] = (even == 0) ? 1 : 0;
  pinMode(K_LINE_TX, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(K_LINE_TX, bits[i] ? HIGH : LOW);
    delay(200);
  }
}

static void kEnableSerial() {
  kLineSerial.begin(K_LINE_BAUD, SERIAL_8N1, K_LINE_RX, K_LINE_TX);
}

static void kDisableSerial() {
  kLineSerial.end();
  pinMode(K_LINE_RX, INPUT_PULLUP);
  pinMode(K_LINE_TX, OUTPUT);
  digitalWrite(K_LINE_TX, HIGH);
  delay(5500);
}

// ================== K-LINE — inicjalizacja ==================

static bool kTryISO14230_Fast() {
  kLineSerial.updateBaudRate(360);
  kLineSerial.write(0x00);
  kLineSerial.flush();
  delay(22);
  kLineSerial.updateBaudRate(K_LINE_BAUD);
  while (kLineSerial.available()) kLineSerial.read();

  uint8_t startComm[] = {0x81, TARGET_ADDR, TESTER_ADDR, 0x81, 0x00};
  startComm[4] = kCalcChecksum(startComm, 4);
  for (int i = 0; i < 5; i++) kLineSerial.write(startComm[i]);
  kLineSerial.flush();
  delay(60);

  if (!kReadData()) return false;
  for (int i = 0; i < (int)sizeof(kResultBuffer); i++) {
    if (kResultBuffer[i] == 0xC1) {
      kLineConnected = true;
      kConnectedProtocol = "ISO14230_Fast";
      delay(50);
      return true;
    }
  }
  return false;
}

static bool kTryISO9141() {
  kDisableSerial();
  kSend5baud(0x33);
  kEnableSerial();

  uint16_t savedTimeout = _kInterByteTimeout;
  _kInterByteTimeout = 30;
  uint8_t len = kReadData();
  _kInterByteTimeout = savedTimeout;

  if (!len || kResultBuffer[0] != 0x55) return false;
  if (kResultBuffer[1] != kResultBuffer[2]) return false;

  kLineSerial.write(~kResultBuffer[2]);
  delay(_kByteWriteInterval);
  kClearEcho();

  if (!kReadData()) return false;
  if (kResultBuffer[0] == 0xCC) {
    kLineConnected = true;
    kConnectedProtocol = "ISO9141";
    return true;
  }
  return false;
}

static bool kTryISO14230_Slow() {
  kDisableSerial();
  kSend5baud(0x33);
  kEnableSerial();

  uint16_t savedTimeout = _kInterByteTimeout;
  _kInterByteTimeout = 30;
  uint8_t len = kReadData();
  _kInterByteTimeout = savedTimeout;

  if (!len || kResultBuffer[0] != 0x55) return false;
  if (kResultBuffer[1] == kResultBuffer[2]) return false;

  kLineSerial.write(~kResultBuffer[2]);
  delay(_kByteWriteInterval);
  kClearEcho();

  if (!kReadData()) return false;
  if (kResultBuffer[0] == 0xCC) {
    kLineConnected = true;
    kConnectedProtocol = "ISO14230_Slow";
    return true;
  }
  return false;
}

static bool kInitConnection() {
  if (kLineConnected) return true;
  if (kTryISO14230_Fast()) return true;
  if (kTryISO9141()) return true;
  if (kTryISO14230_Slow()) return true;
  return false;
}

// ================== K-LINE — KWP2000 Define + Read ==================

static bool kDefineLocalPID(uint8_t pid) {
  uint8_t cmd[] = {0x84, TARGET_ADDR, TESTER_ADDR, 0x2C, 0x10, 0x00, pid};
  kSendKWP(cmd, sizeof(cmd));
  uint8_t len = kReadData();
  if (len < 4) return false;
  if (kResultBuffer[3] == 0x7F) return false;
  return (kResultBuffer[3] == 0x6C);
}

static int32_t kReadLocalID() {
  uint8_t cmd[] = {0x82, TARGET_ADDR, TESTER_ADDR, 0x21, 0x10};
  kSendKWP(cmd, sizeof(cmd));
  uint8_t len = kReadData();
  if (len < 7) return -1;
  if (kResultBuffer[3] == 0x7F) return -1;
  if (kResultBuffer[3] != 0x61) return -1;
  return ((uint16_t)kResultBuffer[5] << 8) | kResultBuffer[6];
}

static int32_t kReadPID(uint8_t pid) {
  if (!kDefineLocalPID(pid)) return -1;
  return kReadLocalID();
}

// ================== K-LINE — cykliczny odczyt ==================

static void kLineReadCycle() {
  if (!kLineConnected) return;

  uint32_t now = millis();
  if ((now - kLastReadMs) < K_READ_INTERVAL) return;
  kLastReadMs = now;

  kCycleCount++;

  // Boost (PID 0x9E) — każdy cykl
  if (!kPidDefined) {
    if (!kDefineLocalPID(0x9E)) {
      kLineConnected = false;
      kPidDefined = false;
      return;
    }
    kPidDefined = true;
  }

  int32_t rawBoost = kReadLocalID();
  if (rawBoost >= 0) {
    kBoostAbsMbar = rawBoost * 0.3616;
    float boost = (kBoostAbsMbar - kATM) / 1000.0;
    kBoostBar = (boost > 0) ? boost : 0.0;
    if (kBoostBar > kPeakBoost) kPeakBoost = kBoostBar;

    if (!kATMCalibrated && kBoostAbsMbar > 800 && kBoostAbsMbar < 1100) {
      kATM = kBoostAbsMbar;
      kATMCalibrated = true;
    }
  } else {
    kPidDefined = false;
  }

  // IAT (PID 0x0F) — co 10 cykli
  if (kCycleCount % 10 == 0) {
    int32_t rawIAT = kReadPID(0x0F);
    if (rawIAT >= 0) {
      kIAT = (float)rawIAT - 40.0;
    }
    kPidDefined = false;  // po odczycie IAT trzeba ponownie define 0x9E
  }
}

// ================================================================
//  UI — WSPÓLNE HELPERS
// ================================================================

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

// ================================================================
//  UI — GÓRNA STREFA (y=0..24)
// ================================================================

// Pozycje wspólne dla labeli góra/dół — zmieniane przy toggle
static const int X_TOP_NUM = 62;   // boost wartość startX (po "BST: ")
static const int X_BOT_NUM = 54;   // IAT wartość startX (po "IAT:")

// --- CAN: rysuj label "ENG:" ---
static void drawTopLabel_CAN() {
  display.fillRect(0, Y_ENG_ROW, 128, CHAR_H, C_BLACK);
  display.setTextSize(2);
  display.setTextColor(C_CYAN);
  display.setCursor(X_ENG_LABEL, Y_ENG_ROW);
  display.print("ENG:");

  engPrevBuf[0] = '\0'; engPrevLen = 0; engSuffixDrawn = false;
}

// --- K-Line: rysuj label "BST:" ---
static void drawTopLabel_KLine() {
  display.fillRect(0, Y_ENG_ROW, 128, CHAR_H, C_BLACK);
  display.setTextSize(2);
  display.setTextColor(C_CYAN);
  display.setCursor(0, Y_ENG_ROW);
  display.print("BST:");

  kBoostPrevBuf[0] = '\0'; kBoostPrevLen = 0; kBoostSuffixDrawn = false;
}

// --- CAN: rysuj label "GEAR:" ---
static void drawBotLabel_CAN() {
  display.fillRect(0, Y_GBX_ROW, 128, CHAR_H, C_BLACK);
  display.setTextSize(2);
  display.setTextColor(C_CYAN);
  display.setCursor(X_GBX_LABEL, Y_GBX_ROW);
  display.print("GEAR:");

  gbxPrevBuf[0] = '\0'; gbxPrevLen = 0; gbxSuffixDrawn = false;
}

// --- K-Line: rysuj label "IAT:" ---
static void drawBotLabel_KLine() {
  display.fillRect(0, Y_GBX_ROW, 128, CHAR_H, C_BLACK);
  display.setTextSize(2);
  display.setTextColor(C_CYAN);
  display.setCursor(0, Y_GBX_ROW);
  display.print("IAT:");

  kIATPrevBuf[0] = '\0'; kIATPrevLen = 0; kIATSuffixDrawn = false;
}

// --- CAN: aktualizuj wartość ENG temp ---
static void drawTopValue_CAN() {
  drawTempSmart(X_ENG_NUM, Y_ENG_ROW, C_YELL,
                tSilnika, engPrevBuf, engPrevLen, engSuffixDrawn);
}

// --- K-Line: aktualizuj wartość BOOST ---
static void drawTopValue_KLine() {
  // Format: "X.XX" + "bar" zamiast "°C"
  char newBuf[8];
  if (kBoostBar >= 0) snprintf(newBuf, sizeof(newBuf), "%.2f", kBoostBar);
  else strcpy(newBuf, "--");

  uint8_t newLen = (uint8_t)strlen(newBuf);
  uint8_t oldLen = kBoostPrevLen;
  bool lenChanged = (newLen != oldLen);

  // Kolor zależny od wartości
  uint16_t color;
  if (kBoostBar < 0.8)       color = C_GREEN;
  else if (kBoostBar < 1.5)  color = C_YELL;
  else                        color = C_RED;

  if (lenChanged || !kBoostSuffixDrawn) {
    // Wyczyść starą wartość + stary suffix
    if (kBoostSuffixDrawn && oldLen > 0) {
      int16_t oldSuffX = X_TOP_NUM + oldLen * CHAR_W + 2;
      display.fillRect(oldSuffX - 1, Y_ENG_ROW - 1, 36, CHAR_H + 2, C_BLACK);
    }
    uint8_t maxLen = (oldLen > newLen) ? oldLen : newLen;
    if (maxLen > 0) display.fillRect(X_TOP_NUM, Y_ENG_ROW, maxLen * CHAR_W, CHAR_H, C_BLACK);

    display.setTextSize(2);
    display.setTextColor(color);
    display.setCursor(X_TOP_NUM, Y_ENG_ROW);
    display.print(newBuf);

    // Suffix "bar" zamiast "°C"
    int16_t suffX = X_TOP_NUM + newLen * CHAR_W + 2;
    display.fillRect(suffX, Y_ENG_ROW, 40, CHAR_H, C_BLACK);
    display.setTextSize(1);
    display.setTextColor(C_WHITE);
    display.setCursor(suffX, Y_ENG_ROW + 4);
    display.print("bar");

    kBoostSuffixDrawn = true;
  } else {
    display.setTextSize(2);
    display.setTextColor(color);
    for (uint8_t i = 0; i < newLen; i++) {
      if (newBuf[i] != kBoostPrevBuf[i]) {
        clearCharAt(X_TOP_NUM, Y_ENG_ROW, i);
        display.setCursor(X_TOP_NUM + i * CHAR_W, Y_ENG_ROW);
        display.print(newBuf[i]);
      }
    }
  }
  strcpy(kBoostPrevBuf, newBuf);
  kBoostPrevLen = newLen;
}

// --- CAN: aktualizuj wartość GEAR temp ---
static void drawBotValue_CAN() {
  drawTempSmart(X_GBX_NUM, Y_GBX_ROW, C_WHITE,
                tSkrzyni, gbxPrevBuf, gbxPrevLen, gbxSuffixDrawn);
}

// --- K-Line: aktualizuj wartość IAT ---
static void drawBotValue_KLine() {
  int iatInt = (kIAT > -90) ? (int)kIAT : -99;
  drawTempSmart(X_BOT_NUM, Y_GBX_ROW, C_GREEN,
                iatInt, kIATPrevBuf, kIATPrevLen, kIATSuffixDrawn);
}

// ================================================================
//  UI — ŚRODEK (bez zmian, zawsze CAN)
// ================================================================

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

static void clearModeField() {
  display.fillRect(X_MODE_DSM, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawModeFieldDSM(char ch, uint16_t color) {
  display.setTextSize(5);
  display.setTextColor(color);
  display.setCursor(X_MODE_DSM, Y_MID_CY);
  display.print(ch);
}

static void drawColonField() {
  display.setTextSize(3);
  display.setTextColor(C_WHITE);
  display.setCursor(X_COLON, Y_COLON);
  display.print(':');
}

static void clearGearField() {
  display.fillRect(X_GEAR_CHAR, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawGearField(char ch) {
  display.setTextSize(5);
  display.setTextColor(C_WHITE);
  display.setCursor(X_GEAR_CHAR, Y_MID_CY);
  display.print(ch);
}

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

static void clearModeSoloField() {
  display.fillRect(X_MODE_SOLO, Y_MID_CY, S5_W, S5_H, C_BLACK);
}
static void drawModeSoloField(char ch, uint16_t color) {
  display.setTextSize(5);
  display.setTextColor(color);
  display.setCursor(X_MODE_SOLO, Y_MID_CY);
  display.print(ch);
}

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

// ================================================================
//  SPLASH
// ================================================================

static void drawSplashScreen() {
  display.fillScreen(C_BLACK);
  const int splashGap = 4;
  const int line1W = 6 * 18;
  const int line2W = 3 * 18;
  const int totalH = 24 + splashGap + 24;
  const int startY = (128 - totalH) / 2;

  display.setTextSize(3);
  display.setTextColor(C_WHITE);
  display.setCursor((128 - line1W) / 2, startY);
  display.print("Escape");

  display.setTextSize(3);
  display.setTextColor(C_ORANGE);
  display.setCursor((128 - line2W) / 2, startY + 24 + splashGap);
  display.print("4x4");
}

// ================================================================
//  STATIC UI — kreska + środek (wspólne)
// ================================================================

static void drawStaticFrame() {
  display.fillScreen(C_BLACK);
  display.drawFastHLine(0, 25, 128, C_CYAN);
  display.drawFastHLine(0, 104, 128, C_CYAN);
  drawnLayout = 0; drawnModeChar = 0; drawnGearChar = 0; drawnArrow = 0;
}

// ================================================================
//  GŁÓWNA FUNKCJA UI
// ================================================================

static void updateUI(bool forceAll) {
  updateTrybUI();
  updateArrowUI();

  uint32_t now = millis();
  if (!forceAll && (now - lastUiDraw) < UI_MIN_INTERVAL_MS) return;
  lastUiDraw = now;

  // --- Przełączenie trybu góra/dół ---
  if (showKLine != prevShowKLine) {
    prevShowKLine = showKLine;
    if (showKLine) {
      drawTopLabel_KLine();
      drawBotLabel_KLine();
    } else {
      drawTopLabel_CAN();
      drawBotLabel_CAN();
    }
    forceAll = true;
  }

  if (forceAll) {
    zmiaLatch = 0; zmiaUI = 0; biegUI = bieg;
  }

  // --- ŚRODEK — zawsze CAN ---
  bool midChanged = forceAll
                    || (poprzTrybUI != trybUI)
                    || (drawnGearChar != (isGearDigit(biegUI) ? biegUI : '-'))
                    || (drawnArrow != zmiaUI);
  if (midChanged) updateMidArea();

  // --- GÓRA + DÓŁ — zależne od trybu ---
  if (showKLine) {
    // K-Line: boost góra, IAT dół
    drawTopValue_KLine();
    drawBotValue_KLine();
  } else {
    // CAN: ENG góra, GEAR dół
    bool engChanged = forceAll || (poptSilnika != tSilnika);
    bool gbxChanged = forceAll || (poptSkrzyni != tSkrzyni);
    if (engChanged) drawTopValue_CAN();
    if (gbxChanged) drawBotValue_CAN();
  }
}

static void commitPops() {
  poprzbieg   = bieg;
  poprzzmia   = zmia;
  poprztryb   = tryb;
  poprzTrybUI = trybUI;
  poptSkrzyni = tSkrzyni;
  poptSilnika = tSilnika;
}

// ================================================================
//  CAN INIT
// ================================================================

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

// ================================================================
//  SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_PRZYCISKU, INPUT_PULLUP);

  display.begin();
  display.cp437(true);
  display.setRotation(2);

  drawSplashScreen();
  delay(SPLASH_DURATION_MS);

  // Ramka statyczna + labele CAN
  drawStaticFrame();
  drawTopLabel_CAN();
  drawBotLabel_CAN();

  if (!initCanB_500k()) {
    display.setTextColor(C_RED);
    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print("CAN FAIL");
    while (1) delay(1000);
  }

  // K-Line UART — uruchom od razu, połączenie dopiero po toggle
  kEnableSerial();

  updateUI(true);
  commitPops();
}

// ================================================================
//  LOOP
// ================================================================

void loop() {
  handleButton();
  handleMarketingSender();

  // ===== CAN — przetwarzaj ZAWSZE =====
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
          } else { oldShiftActive = false; }
          const bool shiftTimedOut = oldShiftActive && ((nowMs - oldShiftStartMs) > SHIFT_TIMEOUT_MS);
          uint8_t D2 = rxBuf[1] & 0x0F;
          if (D2 == 0) {
            newTryb = 'G'; newBieg = -99; newZm = 0; oldShiftActive = false;
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
              newBieg = -99; newZm = 0; oldShiftActive = false;
            }
          }
          if (newTryb == 'P' || newTryb == 'R' || newTryb == 'N') {
            newBieg = -99; newZm = 0; oldShiftActive = false;
          }
          if (newTryb != tryb || newBieg != bieg || newZm != zmia) {
            tryb = newTryb; bieg = newBieg; zmia = newZm; anyStateChanged = true;
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
          } else { newShiftActive = false; }
          const bool shiftTimedOut = newShiftActive && ((nowMs - newShiftStartMs) > SHIFT_TIMEOUT_MS);
          char newBieg = (char)((rxBuf[0] & 0x0F) + ('0' - 4));
          int8_t newZm = 0;
          if (newBieg <= '0') {
            newBieg = -99; newZm = 0; newShiftActive = false;
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
    char pt = trybUI;
    updateTrybUI();
    if (trybUI != pt) anyStateChanged = true;
  }
  {
    int8_t pz = zmiaUI;
    char pb = biegUI;
    updateArrowUI();
    if (zmiaUI != pz || biegUI != pb) anyStateChanged = true;
  }

  // ===== K-LINE — odczyt (tylko gdy tryb K-Line aktywny) =====
  if (showKLine) {
    if (!kLineConnected) {
      uint32_t now = millis();
      if ((now - kLastConnectAttempt) > K_RECONNECT_INTERVAL) {
        kLastConnectAttempt = now;
        kInitConnection();
        if (kLineConnected) {
          kPidDefined = false;
          kATMCalibrated = false;
          kPeakBoost = 0.0;
        }
      }
    } else {
      kLineReadCycle();
      anyStateChanged = true;  // K-Line dane się zmieniają ciągle
    }
  }

  // ===== UI =====
  if (anyStateChanged) {
    updateUI(false);
    commitPops();
  }
}