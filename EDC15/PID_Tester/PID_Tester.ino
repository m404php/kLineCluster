// ============================================================
//  PID_Tester.ino – Narzędzie diagnostyczne BMW EDC16C31 (M57)
//  Protokół: KWP2000 po K-Line (10400 baud, ESP32)
//  Piny: RX=35, TX=38
// ============================================================

#define RX_PIN       35
#define TX_PIN       38
#define TARGET_ADDR  0x12   // Adres ECU (silnik BMW)
#define TESTER_ADDR  0xF1   // Adres testera

// ── Znane / potwierdzone adresy ──────────────────────────────
#define ADDR_RPM     0x0091
#define ADDR_VOLT    0x0093
#define ADDR_TEMP    0x0005
#define ADDR_BOOST   0x009E

// ── Pomocnicze makra do wydruku hex ─────────────────────────
#define PHEX(v)  do { if((v)<0x10) Serial.print('0'); Serial.print((v),HEX); } while(0)

// ============================================================
//  Tablica znanych adresów (opcja 2)
// ============================================================
struct ScanEntry {
  uint8_t     addrHi;
  uint8_t     addrLo;
  const char* possibleName;
};

static const ScanEntry knownAddrs[] = {
  {0x00, 0x01, "RPM / Injection Quantity"},
  {0x00, 0x02, "Coolant Temp"},
  {0x00, 0x03, "Air Mass / Engine Torque"},
  {0x00, 0x04, "unknown"},
  {0x00, 0x05, "MAP/Boost / Engine Speed (potwierdzone)"},
  {0x00, 0x06, "Vehicle Speed / Injection Qty"},
  {0x00, 0x07, "Vehicle Speed"},
  {0x00, 0x08, "unknown"},
  {0x00, 0x09, "Coolant Temp"},
  {0x00, 0x0A, "unknown"},
  {0x00, 0x0B, "Intake Air Temp (IAT)!!!"},
  {0x00, 0x0C, "unknown"},
  {0x00, 0x0D, "Rail Pressure"},
  {0x00, 0x0E, "unknown"},
  {0x00, 0x0F, "Rail Pressure target"},
  {0x00, 0x10, "unknown"},
  {0x00, 0x11, "Injection Quantity"},
  {0x00, 0x13, "Lambda"},
  {0x00, 0x15, "EGR position"},
  {0x00, 0x17, "Throttle valve"},
  {0x00, 0x19, "Accelerator pedal"},
  {0x00, 0x91, "MAF / RPM (potwierdzone)"},
  {0x00, 0x93, "Atmospheric pressure / Voltage (potwierdzone)"},
  {0x00, 0x95, "Fuel Temperature / Vehicle Speed"},
  {0x00, 0x96, "Turbo/Boost Pressure"},
  {0x00, 0x98, "EGR Position"},
  {0x00, 0x9A, "Battery Voltage"},
  {0x00, 0x9C, "unknown"},
  {0x00, 0x9E, "IAT / Boost (potwierdzone)"},
  {0x00, 0x9F, "unknown"},
  {0x00, 0xA0, "Rail Pressure"},
  {0x00, 0xA2, "Accelerator Pedal / Fuel Temp"},
  {0x00, 0xA3, "IAT?"},
  {0x00, 0xA5, "unknown"},
  {0x00, 0xAF, "Air Mass"},
  {0x00, 0xB0, "RPM"},
  {0x00, 0xC0, "Throttle"},
  {0x00, 0xC2, "Oil Temp"},
  {0x00, 0xD8, "Rail Pressure / Pedal"},
};
static const int KNOWN_COUNT = (int)(sizeof(knownAddrs) / sizeof(knownAddrs[0]));

// ============================================================
//  Funkcje komunikacyjne
// ============================================================

byte calcChecksum(byte* data, int len) {
  int sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return (byte)(sum & 0xFF);
}

bool fastInitECU() {
  Serial1.end();
  Serial1.begin(10400, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.updateBaudRate(360);
  Serial1.write(0x00);
  Serial1.flush();
  delay(22);
  Serial1.updateBaudRate(10400);
  while (Serial1.available()) Serial1.read();

  byte startComm[] = {0x81, TARGET_ADDR, TESTER_ADDR, 0x81, 0x05};
  Serial1.write(startComm, 5);
  Serial1.flush();

  unsigned long t = millis();
  int cnt = 0;
  byte rxBuf[64];
  while (millis() - t < 200) {
    if (Serial1.available()) {
      rxBuf[cnt++] = Serial1.read();
      t = millis();
    }
  }
  for (int i = 0; i < cnt; i++) {
    if (rxBuf[i] == 0xC1) {
      delay(50);
      return true;
    }
  }
  return false;
}

int sendAndReceive(byte* req, int reqLen, byte* resp) {
  while (Serial1.available()) Serial1.read();
  Serial1.write(req, reqLen);
  Serial1.flush();
  unsigned long t = millis();
  int cnt = 0;
  while (millis() - t < 300) {
    if (Serial1.available()) {
      resp[cnt++] = Serial1.read();
      t = millis();
    }
  }
  return cnt;
}

// ============================================================
//  Zbuduj ramkę Service 0x2C (DynamicallyDefineLocalIdentifier)
//  i wyślij; zwraca liczbę odebranych bajtów (0 = brak odpowiedzi)
// ============================================================
int query2C(uint8_t addrHi, uint8_t addrLo, byte* resp) {
  byte req[8];
  req[0] = 0x84;
  req[1] = TARGET_ADDR;
  req[2] = TESTER_ADDR;
  req[3] = 0x2C;
  req[4] = 0x10;
  req[5] = addrHi;
  req[6] = addrLo;
  req[7] = calcChecksum(req, 7);
  return sendAndReceive(req, 8, resp);
}

// ============================================================
//  Pomocnicze: wydobądź raw16 z ramki 0x2C  (offset 8+5, 8+6)
//  Zwraca true jeśli odpowiedź pozytywna (zawiera 0x6C)
// ============================================================
bool parseResponse2C(byte* resp, int len, uint16_t* raw16) {
  // Szukamy 0x6C gdziekolwiek w odpowiedzi (tolerancja na echo TX)
  for (int i = 0; i < len - 1; i++) {
    if (resp[i] == 0x6C) {
      // Dane są 2 bajty za 0x6C (po bajcie 0x4C/ADDR)
      if (i + 3 < len) {
        *raw16 = ((uint16_t)resp[i + 2] << 8) | resp[i + 3];
      } else if (i + 2 < len) {
        *raw16 = resp[i + 2];
      } else {
        *raw16 = 0;
      }
      return true;
    }
  }
  return false;
}

// ============================================================
//  Wydruk ramki hex (TX lub RX)
// ============================================================
void printFrame(const char* label, byte* buf, int len) {
  Serial.print(label);
  for (int i = 0; i < len; i++) {
    Serial.print(' ');
    PHEX(buf[i]);
  }
  Serial.println();
}

// ============================================================
//  Wydruk wszystkich interpretacji dla jednego raw16
// ============================================================
void printAllFormulas(uint16_t raw16) {
  uint8_t lo = raw16 & 0xFF;
  uint8_t hi = (raw16 >> 8) & 0xFF;

  Serial.println(F("  ── Interpretacje ──────────────────────────────"));
  Serial.print(F("  Raw uint16 (HI<<8|LO) : ")); Serial.println(raw16);
  Serial.print(F("  Raw HI byte           : 0x")); PHEX(hi); Serial.println();
  Serial.print(F("  Raw LO byte           : 0x")); PHEX(lo); Serial.print(F("  (")); Serial.print(lo); Serial.println(F(")"));

  Serial.println(F("  ── Temperatura ─────────────────────────────────"));
  Serial.print(F("  raw16*0.1-40          : ")); Serial.print(raw16 * 0.1f - 40.0f, 1); Serial.println(F(" °C"));
  Serial.print(F("  LO_byte - 40          : ")); Serial.print((int)lo - 40);             Serial.println(F(" °C"));
  Serial.print(F("  LO_byte - 48          : ")); Serial.print((int)lo - 48);             Serial.println(F(" °C"));
  Serial.print(F("  LO_byte - 50          : ")); Serial.print((int)lo - 50);             Serial.println(F(" °C"));
  Serial.print(F("  raw16 - 40            : ")); Serial.print((int)raw16 - 40);          Serial.println(F(" °C"));

  Serial.println(F("  ── Ciśnienie ───────────────────────────────────"));
  Serial.print(F("  raw16 * 0.136         : ")); Serial.print(raw16 * 0.136f, 1);  Serial.println(F(" hPa"));
  Serial.print(F("  raw16 * 0.0375        : ")); Serial.print(raw16 * 0.0375f, 2); Serial.println(F(" bar"));
  Serial.print(F("  raw16 * 0.1           : ")); Serial.print(raw16 * 0.1f, 1);    Serial.println(F(" kPa"));
  Serial.print(F("  raw16 (surowe kPa)    : ")); Serial.print(raw16);              Serial.println(F(" kPa"));

  Serial.println(F("  ── Napięcie ────────────────────────────────────"));
  Serial.print(F("  raw16 * 0.00268       : ")); Serial.print(raw16 * 0.00268f, 2); Serial.println(F(" V"));
  Serial.print(F("  raw16 * 0.001         : ")); Serial.print(raw16 * 0.001f, 3);   Serial.println(F(" V"));
  Serial.print(F("  LO * 0.1              : ")); Serial.print(lo * 0.1f, 1);        Serial.println(F(" V"));
  Serial.print(F("  LO * 0.08             : ")); Serial.print(lo * 0.08f, 2);       Serial.println(F(" V"));

  Serial.println(F("  ── RPM ─────────────────────────────────────────"));
  Serial.print(F("  raw16 / 4             : ")); Serial.print(raw16 / 4);   Serial.println(F(" rpm"));
  Serial.print(F("  raw16 / 8             : ")); Serial.print(raw16 / 8);   Serial.println(F(" rpm"));
}

// ============================================================
//  Sugestia co to może być na podstawie zakresu wartości
// ============================================================
void printValueHint(uint16_t raw16) {
  uint8_t lo = raw16 & 0xFF;
  uint8_t hi = (raw16 >> 8) & 0xFF;
  Serial.print(F("  💡 Sugestia: "));
  if (raw16 == 0) {
    Serial.println(F("brak danych / czujnik odłączony"));
  } else if (lo >= 30 && lo <= 200 && hi == 0) {
    Serial.println(F("prawdopodobnie TEMPERATURA (1 bajt, °C po odjęciu offsetu)"));
  } else if (raw16 >= 500 && raw16 <= 8000) {
    Serial.println(F("prawdopodobnie RPM (zakres 500–8000)"));
  } else if (raw16 >= 7000 && raw16 <= 8500) {
    Serial.println(F("prawdopodobnie ciśnienie atmosferyczne/boost (zakres hPa)"));
  } else if (raw16 >= 4000 && raw16 <= 6500) {
    Serial.println(F("prawdopodobnie NAPIĘCIE (raw ~5000 ≈ 13.4V przy ×0.00268)"));
  } else {
    Serial.println(F("nieznany zakres – sprawdź ręcznie"));
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PID_Tester – BMW EDC16C31 (M57) K-Line Tester  ║"));
  Serial.println(F("║  Protokol: KWP2000 / Service 0x2C & 0x21        ║"));
  Serial.println(F("╚══════════════════════════════════════════════════╝"));
}

// ============================================================
//  Wyświetl menu główne
// ============================================================
void printMenu() {
  Serial.println(F("\n┌──────────────────────────────────────────────────┐"));
  Serial.println(F("│  MENU GŁÓWNE                                     │"));
  Serial.println(F("├──────────────────────────────────────────────────┤"));
  Serial.println(F("│  1 - PEŁNY SKAN (0x0001–0x00FF, Service 0x2C)   │"));
  Serial.println(F("│  2 - SKAN ZNANYCH ADRESÓW (~39 wpisów)           │"));
  Serial.println(F("│  3 - LIVE MONITOR (RPM / Volt / Temp / Boost)    │"));
  Serial.println(F("│  4 - TEST POJEDYNCZEGO ADRESU                    │"));
  Serial.println(F("│  5 - PORÓWNANIE FORMUŁ (RPM + Temperatura)       │"));
  Serial.println(F("│  6 - TRYB IDENTYFIKACJI (20× odczyt, min/max/avg)│"));
  Serial.println(F("│  7 - SKAN Service 0x21 (Local ID 0x01–0x20)      │"));
  Serial.println(F("└──────────────────────────────────────────────────┘"));
  Serial.print(F("Wybierz opcję: "));
}

// ============================================================
//  Opcja 1 – Pełny skan 0x0001–0x00FF
// ============================================================
void fullScan() {
  Serial.println(F("\n📡 PEŁNY SKAN – adresy 0x0001–0x00FF (Service 0x2C)"));
  Serial.println(F("════════════════════════════════════════════════════"));
  Serial.println(F("Inicjalizcja ECU..."));
  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU. Przerywam."));
    return;
  }
  Serial.println(F("✅ ECU gotowe.\n"));

  // Bufory wyników
  uint8_t  foundAddrs[256];
  uint16_t foundRaw[256];
  int      foundCount = 0;

  byte resp[128];

  for (int addr = 0x01; addr <= 0xFF; addr++) {
    // Co 10 zapytań – re-init K-Line
    if (addr > 1 && ((addr - 1) % 10 == 0)) {
      Serial.println(F("  🔄 Re-init ECU..."));
      if (!fastInitECU()) {
        Serial.println(F("  ❌ Re-init nieudany – przerywam skan."));
        break;
      }
    }

    int len = query2C(0x00, (uint8_t)addr, resp);
    delay(50);

    Serial.print(F("  [0x00"));
    PHEX((uint8_t)addr);
    Serial.print(F("] "));

    bool negative = false;
    for (int i = 0; i < len; i++) {
      if (resp[i] == 0x7F) { negative = true; break; }
    }

    uint16_t raw16 = 0;
    bool positive = parseResponse2C(resp, len, &raw16);

    if (positive) {
      Serial.print(F("✅ RAW: "));
      PHEX((raw16 >> 8) & 0xFF);
      Serial.print(' ');
      PHEX(raw16 & 0xFF);
      Serial.print(F("  (uint16="));
      Serial.print(raw16);
      Serial.println(F(")"));
      foundAddrs[foundCount] = (uint8_t)addr;
      foundRaw[foundCount]   = raw16;
      foundCount++;
    } else if (len == 0) {
      Serial.println(F("⏱ TIMEOUT"));
    } else if (negative) {
      Serial.println(F("❌ negative response (0x7F)"));
    } else {
      Serial.print(F("? nieznana odpowiedź len="));
      Serial.println(len);
    }
  }

  // Podsumowanie
  Serial.println(F("\n════════════════════════════════════════════════════"));
  Serial.print(F("📊 PODSUMOWANIE: znaleziono "));
  Serial.print(foundCount);
  Serial.println(F(" adresów z odpowiedzią pozytywną:"));
  for (int i = 0; i < foundCount; i++) {
    Serial.print(F("  0x00"));
    PHEX(foundAddrs[i]);
    Serial.print(F("  raw16="));
    Serial.print(foundRaw[i]);
    Serial.print(F("  (0x"));
    PHEX((foundRaw[i] >> 8) & 0xFF);
    Serial.print(' ');
    PHEX(foundRaw[i] & 0xFF);
    Serial.println(F(")"));
  }
}

// ============================================================
//  Opcja 2 – Skan znanych adresów
// ============================================================
void knownScan() {
  Serial.println(F("\n🔍 SKAN ZNANYCH ADRESÓW (~39 wpisów)"));
  Serial.println(F("══════════════════════════════════════════════════"));
  Serial.println(F("Inicjalizacja ECU..."));
  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU. Przerywam."));
    return;
  }
  Serial.println(F("✅ ECU gotowe.\n"));

  byte resp[128];

  for (int i = 0; i < KNOWN_COUNT; i++) {
    // Co 10 zapytań – re-init
    if (i > 0 && (i % 10 == 0)) {
      Serial.println(F("  🔄 Re-init ECU..."));
      if (!fastInitECU()) {
        Serial.println(F("  ❌ Re-init nieudany – przerywam."));
        break;
      }
    }

    uint8_t hi = knownAddrs[i].addrHi;
    uint8_t lo = knownAddrs[i].addrLo;
    int len = query2C(hi, lo, resp);
    delay(50);

    Serial.print(F("  [0x"));
    PHEX(hi); PHEX(lo);
    Serial.print(F("] "));
    Serial.print(knownAddrs[i].possibleName);
    Serial.print(F(": "));

    uint16_t raw16 = 0;
    bool positive = parseResponse2C(resp, len, &raw16);

    if (positive) {
      uint8_t loB = raw16 & 0xFF;
      Serial.print(F("✅ RAW=0x"));
      PHEX((raw16 >> 8) & 0xFF); PHEX(loB);
      Serial.print(F(" ("));
      Serial.print(raw16);
      Serial.print(F(") | Temp(LO-40)="));
      Serial.print((int)loB - 40);
      Serial.print(F("°C | Boost×0.136="));
      Serial.print(raw16 * 0.136f, 0);
      Serial.print(F("hPa | V×0.00268="));
      Serial.print(raw16 * 0.00268f, 2);
      Serial.print(F("V | RPM/4="));
      Serial.print(raw16 / 4);
      Serial.print(F(" RPM/8="));
      Serial.println(raw16 / 8);
    } else if (len == 0) {
      Serial.println(F("⏱ TIMEOUT"));
    } else {
      bool neg = false;
      for (int j = 0; j < len; j++) if (resp[j] == 0x7F) neg = true;
      if (neg) Serial.println(F("❌ negative response"));
      else { Serial.print(F("? len=")); Serial.println(len); }
    }
  }
  Serial.println(F("\n══════════════════════════════════════════════════"));
  Serial.println(F("✅ Skan znanych adresów zakończony."));
}

// ============================================================
//  Opcja 3 – Live Monitor
// ============================================================
void liveMonitor() {
  Serial.println(F("\n📺 LIVE MONITOR – RPM / Napięcie / Temperatura / Boost"));
  Serial.println(F("Wyślij dowolny znak aby zatrzymać."));
  Serial.println(F("════════════════════════════════════════════════════"));

  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU."));
    return;
  }
  Serial.println(F("✅ ECU gotowe. Start monitoringu...\n"));

  static const uint8_t pids[4][2] = {
    {0x00, 0x91},  // RPM
    {0x00, 0x93},  // Napięcie
    {0x00, 0x05},  // Temperatura
    {0x00, 0x9E},  // Boost
  };

  byte resp[128];
  int iteration = 0;

  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("\n⏹ Zatrzymano przez użytkownika."));
      break;
    }

    // Co 20 iteracji – re-init
    if (iteration > 0 && (iteration % 20 == 0)) {
      if (!fastInitECU()) {
        Serial.println(F("❌ Re-init nieudany – zatrzymuję monitor."));
        break;
      }
    }

    uint16_t raw[4] = {0, 0, 0, 0};
    bool ok[4] = {false, false, false, false};

    for (int p = 0; p < 4; p++) {
      int len = query2C(pids[p][0], pids[p][1], resp);
      delay(40);
      ok[p] = parseResponse2C(resp, len, &raw[p]);
    }

    // Wydruk iteracji
    Serial.print(F("#"));
    Serial.print(iteration);
    Serial.print(F("  RPM(÷4)="));
    if (ok[0]) { Serial.print(raw[0] / 4); Serial.print(F("  (÷8)=")); Serial.print(raw[0] / 8); }
    else Serial.print(F("---"));

    Serial.print(F("  |  Volt="));
    if (ok[1]) Serial.print(raw[1] * 0.00268f, 2);
    else Serial.print(F("---"));

    Serial.print(F("V  |  Temp(LO-40)="));
    if (ok[2]) {
      uint8_t lo = raw[2] & 0xFF;
      Serial.print((int)lo - 40);
      Serial.print(F("°C  (raw16×0.1-40="));
      Serial.print(raw[2] * 0.1f - 40.0f, 1);
      Serial.print(F("°C  LO-48="));
      Serial.print((int)lo - 48);
      Serial.print(F("°C)"));
    } else {
      Serial.print(F("---"));
    }

    Serial.print(F("  |  Boost="));
    if (ok[3]) { Serial.print(raw[3] * 0.136f, 0); Serial.print(F("hPa")); }
    else Serial.print(F("---"));

    Serial.println();
    iteration++;
    delay(500);
  }
}

// ============================================================
//  Wczytaj adres hex od użytkownika (np. "0B" lub "009E")
//  Zwraca adres jako uint16_t lub 0xFFFF przy błędzie
// ============================================================
uint16_t readHexAddress() {
  Serial.print(F("\nPodaj adres hex (np. 0B lub 009E): "));
  while (Serial.available() == 0) delay(10);
  delay(100); // poczekaj na cały wpis

  char buf[16];
  int idx = 0;
  while (Serial.available() && idx < 15) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') break;
    buf[idx++] = c;
  }
  buf[idx] = '\0';
  while (Serial.available()) Serial.read(); // flush reszty

  if (idx == 0) return 0xFFFF;
  return (uint16_t)strtoul(buf, nullptr, 16);
}

// ============================================================
//  Opcja 4 – Test pojedynczego adresu
// ============================================================
void singleTest() {
  Serial.println(F("\n🔬 TEST POJEDYNCZEGO ADRESU"));

  uint16_t addr = readHexAddress();
  if (addr == 0xFFFF) {
    Serial.println(F("❌ Nieprawidłowy adres."));
    return;
  }

  uint8_t addrHi = (addr >> 8) & 0xFF;
  uint8_t addrLo = addr & 0xFF;

  Serial.print(F("Testuje adres: 0x"));
  PHEX(addrHi); PHEX(addrLo);
  Serial.println();

  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU."));
    return;
  }

  byte req[8];
  req[0] = 0x84;
  req[1] = TARGET_ADDR;
  req[2] = TESTER_ADDR;
  req[3] = 0x2C;
  req[4] = 0x10;
  req[5] = addrHi;
  req[6] = addrLo;
  req[7] = calcChecksum(req, 7);

  byte resp[128];
  int len = sendAndReceive(req, 8, resp);

  printFrame("  TX:", req, 8);
  printFrame("  RX:", resp, len);

  uint16_t raw16 = 0;
  bool positive = parseResponse2C(resp, len, &raw16);

  if (!positive) {
    bool neg = false;
    for (int i = 0; i < len; i++) if (resp[i] == 0x7F) neg = true;
    if (neg) {
      Serial.println(F("❌ Odpowiedź negatywna (0x7F) – adres nieobsługiwany."));
    } else if (len == 0) {
      Serial.println(F("⏱ Timeout – brak odpowiedzi."));
    } else {
      Serial.println(F("? Nierozpoznana odpowiedź."));
    }
    return;
  }

  Serial.print(F("✅ Odpowiedź pozytywna (0x6C). Raw uint16 = "));
  Serial.println(raw16);

  printAllFormulas(raw16);
  printValueHint(raw16);
}

// ============================================================
//  Opcja 5 – Porównanie formuł (RPM + Temperatura)
// ============================================================
void formulaComparison() {
  Serial.println(F("\n📐 PORÓWNANIE FORMUŁ – RPM (0x0091) i Temperatura (0x0005)"));
  Serial.println(F("══════════════════════════════════════════════════════════"));

  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU."));
    return;
  }

  byte resp[128];
  uint16_t rawRPM = 0, rawTemp = 0;

  int lenRPM  = query2C(0x00, 0x91, resp);
  delay(60);
  bool okRPM  = parseResponse2C(resp, lenRPM, &rawRPM);

  int lenTemp = query2C(0x00, 0x05, resp);
  delay(60);
  bool okTemp = parseResponse2C(resp, lenTemp, &rawTemp);

  Serial.println(F("\n── RPM (adres 0x0091) ──────────────────────────────────"));
  if (okRPM) {
    uint8_t lo = rawRPM & 0xFF;
    uint8_t hi = (rawRPM >> 8) & 0xFF;
    Serial.print(F("  Raw HEX: ")); PHEX(hi); Serial.print(' '); PHEX(lo);
    Serial.print(F("  uint16=")); Serial.println(rawRPM);
    Serial.print(F("  raw / 4  = ")); Serial.print(rawRPM / 4);   Serial.println(F(" rpm"));
    Serial.print(F("  raw / 8  = ")); Serial.print(rawRPM / 8);   Serial.println(F(" rpm"));
    Serial.print(F("  raw / 2  = ")); Serial.print(rawRPM / 2);   Serial.println(F(" rpm"));
    Serial.print(F("  raw×0.25 = ")); Serial.print(rawRPM * 0.25f, 0); Serial.println(F(" rpm"));
    Serial.print(F("  raw      = ")); Serial.print(rawRPM);        Serial.println(F(" (bez skalowania)"));
  } else {
    Serial.println(F("  ❌ Brak odpowiedzi."));
  }

  Serial.println(F("\n── TEMPERATURA (adres 0x0005) ──────────────────────────"));
  if (okTemp) {
    uint8_t lo = rawTemp & 0xFF;
    uint8_t hi = (rawTemp >> 8) & 0xFF;
    Serial.print(F("  Raw HEX: ")); PHEX(hi); Serial.print(' '); PHEX(lo);
    Serial.print(F("  uint16=")); Serial.println(rawTemp);
    Serial.print(F("  raw16*0.1-40      = ")); Serial.print(rawTemp * 0.1f - 40.0f, 1); Serial.println(F(" °C"));
    Serial.print(F("  LO_byte - 40      = ")); Serial.print((int)lo - 40);               Serial.println(F(" °C"));
    Serial.print(F("  LO_byte - 48      = ")); Serial.print((int)lo - 48);               Serial.println(F(" °C"));
    Serial.print(F("  LO_byte - 50      = ")); Serial.print((int)lo - 50);               Serial.println(F(" °C"));
    Serial.print(F("  HI_byte - 40      = ")); Serial.print((int)hi - 40);               Serial.println(F(" °C (jeśli 1-bajtowy w HI)"));
    Serial.print(F("  raw16 - 40        = ")); Serial.print((int)rawTemp - 40);          Serial.println(F(" °C"));
    Serial.print(F("  raw16/10 - 40     = ")); Serial.print(rawTemp / 10 - 40);          Serial.println(F(" °C"));
  } else {
    Serial.println(F("  ❌ Brak odpowiedzi."));
  }

  Serial.println(F("\n══════════════════════════════════════════════════════════"));
  Serial.println(F("💡 Wskazówka: dla silnika na biegu jałowym (~780 rpm)"));
  Serial.println(F("   raw/8 powinno dać ~780, raw/4 powinno dać ~1560."));
  Serial.println(F("   Dla ciepłego silnika (~90°C) LO_byte-40 = 90 gdy LO=0x82=130."));
}

// ============================================================
//  Opcja 6 – Tryb identyfikacji (20× odczyt, min/max/avg)
// ============================================================
void identificationMode() {
  Serial.println(F("\n🔬 TRYB IDENTYFIKACJI (20× odczyt)"));

  uint16_t addr = readHexAddress();
  if (addr == 0xFFFF) {
    Serial.println(F("❌ Nieprawidłowy adres."));
    return;
  }

  uint8_t addrHi = (addr >> 8) & 0xFF;
  uint8_t addrLo = addr & 0xFF;

  Serial.print(F("Identyfikuję adres: 0x")); PHEX(addrHi); PHEX(addrLo);
  Serial.println(F(" (20 odczytów, 500ms odstęp)..."));

  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU."));
    return;
  }

  byte resp[128];
  uint16_t samples[20];
  int validCount = 0;
  uint32_t sumVal = 0;
  uint16_t minVal = 0xFFFF;
  uint16_t maxVal = 0;

  for (int i = 0; i < 20; i++) {
    // Co 10 odczytów – re-init
    if (i > 0 && (i % 10 == 0)) {
      Serial.println(F("  🔄 Re-init ECU..."));
      fastInitECU();
    }

    int len = query2C(addrHi, addrLo, resp);
    delay(50);
    uint16_t raw = 0;
    bool ok = parseResponse2C(resp, len, &raw);

    Serial.print(F("  ["));
    Serial.print(i + 1);
    Serial.print(F("] "));
    if (ok) {
      Serial.print(F("raw="));
      Serial.print(raw);
      Serial.print(F(" (0x")); PHEX((raw >> 8) & 0xFF); PHEX(raw & 0xFF); Serial.print(F(")"));
      samples[validCount++] = raw;
      sumVal += raw;
      if (raw < minVal) minVal = raw;
      if (raw > maxVal) maxVal = raw;
    } else {
      Serial.print(F("❌ brak odpowiedzi"));
    }
    Serial.println();
    delay(500);
  }

  Serial.println(F("\n── Wyniki ──────────────────────────────────────────────"));
  if (validCount == 0) {
    Serial.println(F("❌ Żaden odczyt się nie powiódł."));
    return;
  }

  uint16_t avgVal = (uint16_t)(sumVal / validCount);
  uint16_t spread = maxVal - minVal;

  Serial.print(F("  Ważnych próbek : ")); Serial.println(validCount);
  Serial.print(F("  MIN            : ")); Serial.println(minVal);
  Serial.print(F("  MAX            : ")); Serial.println(maxVal);
  Serial.print(F("  AVG            : ")); Serial.println(avgVal);
  Serial.print(F("  Rozrzut (MAX-MIN): ")); Serial.println(spread);

  Serial.print(F("  Charakter      : "));
  if (spread <= 10) {
    Serial.println(F("STAŁY – prawdopodobnie napięcie lub sensor statyczny"));
  } else if (spread <= 200) {
    Serial.println(F("MAŁY ROZRZUT – możliwe temperatura lub ciśnienie"));
  } else {
    Serial.println(F("ZMIENNY – możliwe RPM, ciśnienie doładowania lub pedał"));
  }

  Serial.println(F("\n── Sugestia na podstawie AVG ───────────────────────────"));
  printValueHint(avgVal);

  Serial.println(F("\n── Interpretacja AVG ───────────────────────────────────"));
  printAllFormulas(avgVal);
}

// ============================================================
//  Opcja 7 – Skan Service 0x21 (ReadDataByLocalIdentifier)
// ============================================================
void scan21() {
  Serial.println(F("\n📡 SKAN Service 0x21 – Local ID 0x01–0x20"));
  Serial.println(F("══════════════════════════════════════════════════"));

  if (!fastInitECU()) {
    Serial.println(F("❌ Nie można nawiązać połączenia z ECU."));
    return;
  }
  Serial.println(F("✅ ECU gotowe.\n"));

  byte resp[128];

  for (int lid = 0x01; lid <= 0x20; lid++) {
    // Co 10 zapytań – re-init
    if (lid > 1 && ((lid - 1) % 10 == 0)) {
      Serial.println(F("  🔄 Re-init ECU..."));
      if (!fastInitECU()) {
        Serial.println(F("  ❌ Re-init nieudany – przerywam skan."));
        break;
      }
    }

    byte req[6];
    req[0] = 0x82;
    req[1] = TARGET_ADDR;
    req[2] = TESTER_ADDR;
    req[3] = 0x21;
    req[4] = (uint8_t)lid;
    req[5] = calcChecksum(req, 5);

    while (Serial1.available()) Serial1.read();
    int len = sendAndReceive(req, 6, resp);
    delay(50);

    Serial.print(F("  Local ID 0x")); PHEX((uint8_t)lid);
    Serial.print(F(": "));

    bool positive = false;
    bool negative = false;
    for (int i = 0; i < len; i++) {
      if (resp[i] == 0x61) { positive = true; break; }
      if (resp[i] == 0x7F) { negative = true; break; }
    }

    if (positive) {
      Serial.print(F("✅ RAW: "));
      for (int i = 0; i < len; i++) { PHEX(resp[i]); Serial.print(' '); }
      Serial.println();
    } else if (negative) {
      Serial.println(F("❌ negative response (0x7F)"));
    } else if (len == 0) {
      Serial.println(F("⏱ TIMEOUT"));
    } else {
      Serial.print(F("? len=")); Serial.println(len);
    }
  }

  Serial.println(F("\n══════════════════════════════════════════════════"));
  Serial.println(F("✅ Skan Service 0x21 zakończony."));
}

// ============================================================
//  LOOP – menu główne
// ============================================================
void loop() {
  printMenu();

  while (Serial.available() == 0) delay(10);
  char choice = (char)Serial.read();
  while (Serial.available()) Serial.read();

  Serial.println(choice);

  switch (choice) {
    case '1': fullScan();            break;
    case '2': knownScan();           break;
    case '3': liveMonitor();         break;
    case '4': singleTest();          break;
    case '5': formulaComparison();   break;
    case '6': identificationMode();  break;
    case '7': scan21();              break;
    default:
      Serial.println(F("❌ Nieznana opcja. Wpisz 1–7."));
      break;
  }

  delay(1000);
}
