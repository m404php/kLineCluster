// ============================================================
//  PID_Tester.ino – Narzędzie diagnostyczne (Multi-Protocol)
//  Biblioteka: OBD2_KLine | Protokoly: ISO9141, KWP2000
//  Piny: RX=35, TX=38  |  ESP32
// ============================================================

#include "OBD2_KLine.h"

#define RX_PIN       35
#define TX_PIN       38
#define TARGET_ADDR  0x12   // Adres ECU (silnik BMW)
#define TESTER_ADDR  0xF1   // Adres testera

// Znane / potwierdzone adresy KWP2000
#define ADDR_RPM     0x0091
#define ADDR_VOLT    0x0093
#define ADDR_TEMP    0x0005
#define ADDR_BOOST   0x009E

// Pomocnicze makro do wydruku hex
#define PHEX(v)  do { if ((v) < 0x10) Serial.print('0'); Serial.print((v), HEX); } while (0)

// Stalte PID OBD2 (nie sa w OBD2_KLine.h)
const byte ENGINE_LOAD         = 0x04;
const byte ENGINE_COOLANT_TEMP = 0x05;
const byte FUEL_PRESSURE       = 0x0A;
const byte ENGINE_RPM          = 0x0C;
const byte VEHICLE_SPEED       = 0x0D;
const byte INTAKE_AIR_TEMP     = 0x0F;
const byte MAF_FLOW_RATE       = 0x10;
const byte THROTTLE_POSITION   = 0x11;

// Globalny obiekt biblioteki OBD2_KLine
OBD2_KLine obd(Serial1, 10400, RX_PIN, TX_PIN);

// ============================================================
//  Tablica znanych adresow KWP2000 (opcja 2)
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
//  Inicjalizacja ECU przez biblioteke OBD2_KLine
// ============================================================
bool initECU() {
  Serial.println(F("Inicjalizacja ECU..."));
  if (obd.initOBD2()) {
    Serial.println(F("ECU gotowe."));
    return true;
  }
  Serial.println(F("Nie mozna nawiazac polaczenia z ECU. Przerywam."));
  return false;
}

bool reInitECU() {
  obd.setProtocol("Automatic");
  return obd.initOBD2();
}

// ============================================================
//  Wyslij zapytanie Service 0x2C, zwraca liczbe odebranych bajtow
// ============================================================
uint8_t query2C(uint8_t addrHi, uint8_t addrLo) {
  uint8_t frame[] = {0x84, TARGET_ADDR, TESTER_ADDR, 0x2C, 0x10, addrHi, addrLo};
  obd.writeRawData(frame, 7);  // writeRawData dodaje checksum automatycznie
  return obd.readData();
}

// ============================================================
//  Wydobadz raw16 z bufora Service 0x2C
//  Zwraca true jesli odpowiedz pozytywna (zawiera 0x6C)
// ============================================================
bool parseResponse2C(uint8_t len, uint16_t* raw16) {
  for (int i = 0; i < (int)len - 1; i++) {
    if (obd.resultBuffer[i] == 0x6C) {
      if (i + 3 < len) {
        *raw16 = ((uint16_t)obd.resultBuffer[i + 2] << 8) | obd.resultBuffer[i + 3];
      } else if (i + 2 < len) {
        *raw16 = obd.resultBuffer[i + 2];
      } else {
        *raw16 = 0;
      }
      return true;
    }
  }
  return false;
}

// ============================================================
//  Wydruk ramki hex
// ============================================================
void printFrame(const char* label, const uint8_t* buf, int len) {
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

  Serial.println(F("  -- Interpretacje --"));
  Serial.print(F("  Raw uint16 (HI<<8|LO) : ")); Serial.println(raw16);
  Serial.print(F("  Raw HI byte           : 0x")); PHEX(hi); Serial.println();
  Serial.print(F("  Raw LO byte           : 0x")); PHEX(lo); Serial.print(F("  (")); Serial.print(lo); Serial.println(F(")"));

  Serial.println(F("  -- Temperatura --"));
  Serial.print(F("  raw16*0.1-40          : ")); Serial.print(raw16 * 0.1f - 40.0f, 1); Serial.println(F(" C"));
  Serial.print(F("  LO_byte - 40          : ")); Serial.print((int)lo - 40);             Serial.println(F(" C"));
  Serial.print(F("  LO_byte - 48          : ")); Serial.print((int)lo - 48);             Serial.println(F(" C"));
  Serial.print(F("  LO_byte - 50          : ")); Serial.print((int)lo - 50);             Serial.println(F(" C"));
  Serial.print(F("  raw16 - 40            : ")); Serial.print((int)raw16 - 40);          Serial.println(F(" C"));

  Serial.println(F("  -- Cisnienie --"));
  Serial.print(F("  raw16 * 0.136         : ")); Serial.print(raw16 * 0.136f, 1);  Serial.println(F(" hPa"));
  Serial.print(F("  raw16 * 0.0375        : ")); Serial.print(raw16 * 0.0375f, 2); Serial.println(F(" bar"));
  Serial.print(F("  raw16 * 0.1           : ")); Serial.print(raw16 * 0.1f, 1);    Serial.println(F(" kPa"));
  Serial.print(F("  raw16 (surowe kPa)    : ")); Serial.print(raw16);              Serial.println(F(" kPa"));

  Serial.println(F("  -- Napiecie --"));
  Serial.print(F("  raw16 * 0.00268       : ")); Serial.print(raw16 * 0.00268f, 2); Serial.println(F(" V"));
  Serial.print(F("  raw16 * 0.001         : ")); Serial.print(raw16 * 0.001f, 3);   Serial.println(F(" V"));
  Serial.print(F("  LO * 0.1              : ")); Serial.print(lo * 0.1f, 1);        Serial.println(F(" V"));
  Serial.print(F("  LO * 0.08             : ")); Serial.print(lo * 0.08f, 2);       Serial.println(F(" V"));

  Serial.println(F("  -- RPM --"));
  Serial.print(F("  raw16 / 4             : ")); Serial.print(raw16 / 4);   Serial.println(F(" rpm"));
  Serial.print(F("  raw16 / 8             : ")); Serial.print(raw16 / 8);   Serial.println(F(" rpm"));
}

// ============================================================
//  Sugestia na podstawie zakresu wartosci
// ============================================================
void printValueHint(uint16_t raw16) {
  uint8_t lo = raw16 & 0xFF;
  uint8_t hi = (raw16 >> 8) & 0xFF;
  Serial.print(F("  Sugestia: "));
  if (raw16 == 0) {
    Serial.println(F("brak danych / czujnik odlaczony"));
  } else if (lo >= 30 && lo <= 200 && hi == 0) {
    Serial.println(F("prawdopodobnie TEMPERATURA (1 bajt, C po odjeciu offsetu)"));
  } else if (raw16 >= 500 && raw16 <= 8000) {
    Serial.println(F("prawdopodobnie RPM (zakres 500-8000)"));
  } else if (raw16 >= 7000 && raw16 <= 8500) {
    Serial.println(F("prawdopodobnie cisnienie atmosferyczne/boost (zakres hPa)"));
  } else if (raw16 >= 4000 && raw16 <= 6500) {
    Serial.println(F("prawdopodobnie NAPIECIE (raw ~5000 = 13.4V przy x0.00268)"));
  } else {
    Serial.println(F("nieznany zakres - sprawdz recznie"));
  }
}

// ============================================================
//  Nazwa PID OBD2 (dla opcji 9)
// ============================================================
String pidName(byte pid) {
  switch (pid) {
    case 0x01: return "Monitor Status";
    case 0x02: return "Freeze DTC";
    case 0x03: return "Fuel System Status";
    case 0x04: return "Engine Load";
    case 0x05: return "Coolant Temp";
    case 0x06: return "STFT Bank1";
    case 0x07: return "LTFT Bank1";
    case 0x08: return "STFT Bank2";
    case 0x09: return "LTFT Bank2";
    case 0x0A: return "Fuel Pressure";
    case 0x0B: return "MAP Pressure";
    case 0x0C: return "Engine RPM";
    case 0x0D: return "Vehicle Speed";
    case 0x0E: return "Timing Advance";
    case 0x0F: return "Intake Air Temp";
    case 0x10: return "MAF Flow Rate";
    case 0x11: return "Throttle Position";
    case 0x1C: return "OBD Standards";
    case 0x1F: return "Runtime Since Start";
    case 0x21: return "Dist w/ MIL on";
    case 0x22: return "Fuel Rail Pressure";
    case 0x23: return "Fuel Rail Gauge P";
    case 0x2C: return "Commanded EGR";
    case 0x2F: return "Fuel Tank Level";
    case 0x31: return "Dist since cleared";
    case 0x33: return "Barometric Pressure";
    case 0x42: return "Control Module V";
    case 0x45: return "Rel Throttle Pos";
    case 0x46: return "Ambient Air Temp";
    case 0x4D: return "Time w/ MIL on";
    case 0x4E: return "Time since cleared";
    case 0x5C: return "Engine Oil Temp";
    case 0x5E: return "Engine Fuel Rate";
    case 0x61: return "Demanded Torque";
    case 0x62: return "Actual Torque";
    case 0x63: return "Reference Torque";
    default: {
      String s = "PID 0x";
      if (pid < 0x10) s += "0";
      s += String(pid, HEX);
      return s;
    }
  }
}

// ============================================================
//  Wczytaj adres hex od uzytkownika (np. "0B" lub "009E")
//  Zwraca adres jako uint16_t lub 0xFFFF przy bledzie
// ============================================================
uint16_t readHexAddress() {
  Serial.print(F("\nPodaj adres hex (np. 0B lub 009E): "));
  while (Serial.available() == 0) delay(10);
  delay(100);

  char buf[16];
  int idx = 0;
  while (Serial.available() && idx < 15) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') break;
    buf[idx++] = c;
  }
  buf[idx] = '\0';
  while (Serial.available()) Serial.read();

  if (idx == 0) return 0xFFFF;
  return (uint16_t)strtoul(buf, nullptr, 16);
}

// ============================================================
//  Opcja 1 - Pelny skan 0x0001-0x00FF (Service 0x2C)
// ============================================================
void fullScan() {
  Serial.println(F("\nPELNY SKAN KWP2000 - adresy 0x0001-0x00FF (Service 0x2C)"));
  Serial.println(F("============================================================"));
  if (!initECU()) return;
  Serial.println();

  uint8_t  foundAddrs[256];
  uint16_t foundRaw[256];
  int      foundCount = 0;

  for (int addr = 0x01; addr <= 0xFF; addr++) {
    if (addr > 1 && ((addr - 1) % 10 == 0)) {
      Serial.println(F("  Re-init ECU..."));
      if (!reInitECU()) {
        Serial.println(F("  Re-init nieudany - przerywam skan."));
        break;
      }
    }

    uint8_t len = query2C(0x00, (uint8_t)addr);
    delay(50);

    Serial.print(F("  [0x00"));
    PHEX((uint8_t)addr);
    Serial.print(F("] "));

    bool negative = false;
    for (int i = 0; i < len; i++) {
      if (obd.resultBuffer[i] == 0x7F) { negative = true; break; }
    }

    uint16_t raw16 = 0;
    bool positive = parseResponse2C(len, &raw16);

    if (positive) {
      Serial.print(F("OK RAW: "));
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
      Serial.println(F("TIMEOUT"));
    } else if (negative) {
      Serial.println(F("negative response (0x7F)"));
    } else {
      Serial.print(F("? nieznana odpowiedz len="));
      Serial.println(len);
    }
  }

  Serial.println(F("\n============================================================"));
  Serial.print(F("PODSUMOWANIE: znaleziono "));
  Serial.print(foundCount);
  Serial.println(F(" adresow z odpowiedzia pozytywna:"));
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
//  Opcja 2 - Skan znanych adresow (~39 wpisow, Service 0x2C)
// ============================================================
void knownScan() {
  Serial.println(F("\nSKAN ZNANYCH ADRESOW (~39 wpisow, Service 0x2C)"));
  Serial.println(F("===================================================="));
  if (!initECU()) return;
  Serial.println();

  for (int i = 0; i < KNOWN_COUNT; i++) {
    if (i > 0 && (i % 10 == 0)) {
      Serial.println(F("  Re-init ECU..."));
      if (!reInitECU()) {
        Serial.println(F("  Re-init nieudany - przerywam."));
        break;
      }
    }

    uint8_t hi = knownAddrs[i].addrHi;
    uint8_t lo = knownAddrs[i].addrLo;
    uint8_t len = query2C(hi, lo);
    delay(50);

    Serial.print(F("  [0x"));
    PHEX(hi); PHEX(lo);
    Serial.print(F("] "));
    Serial.print(knownAddrs[i].possibleName);
    Serial.print(F(": "));

    uint16_t raw16 = 0;
    bool positive = parseResponse2C(len, &raw16);

    if (positive) {
      uint8_t loB = raw16 & 0xFF;
      Serial.print(F("OK RAW=0x"));
      PHEX((raw16 >> 8) & 0xFF); PHEX(loB);
      Serial.print(F(" ("));
      Serial.print(raw16);
      Serial.print(F(") | Temp(LO-40)="));
      Serial.print((int)loB - 40);
      Serial.print(F("C | Boost*0.136="));
      Serial.print(raw16 * 0.136f, 0);
      Serial.print(F("hPa | V*0.00268="));
      Serial.print(raw16 * 0.00268f, 2);
      Serial.print(F("V | RPM/4="));
      Serial.print(raw16 / 4);
      Serial.print(F(" RPM/8="));
      Serial.println(raw16 / 8);
    } else if (len == 0) {
      Serial.println(F("TIMEOUT"));
    } else {
      bool neg = false;
      for (int j = 0; j < len; j++) if (obd.resultBuffer[j] == 0x7F) neg = true;
      if (neg) Serial.println(F("negative response"));
      else { Serial.print(F("? len=")); Serial.println(len); }
    }
  }
  Serial.println(F("\n===================================================="));
  Serial.println(F("Skan znanych adresow zakonczony."));
}

// ============================================================
//  Opcja 3 - Live Monitor KWP2000 (RPM/Volt/Temp/Boost)
// ============================================================
void liveMonitor() {
  Serial.println(F("\nLIVE MONITOR KWP2000 - RPM / Napiecie / Temperatura / Boost"));
  Serial.println(F("Wyslij dowolny znak aby zatrzymac."));
  Serial.println(F("============================================================"));

  if (!initECU()) return;
  Serial.println(F("Start monitoringu...\n"));

  static const uint8_t pids[4][2] = {
    {0x00, 0x91},  // RPM
    {0x00, 0x93},  // Napiecie
    {0x00, 0x05},  // Temperatura
    {0x00, 0x9E},  // Boost
  };

  int iteration = 0;

  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("\nZatrzymano przez uzytkownika."));
      break;
    }

    if (iteration > 0 && (iteration % 20 == 0)) {
      if (!reInitECU()) {
        Serial.println(F("Re-init nieudany - zatrzymuje monitor."));
        break;
      }
    }

    uint16_t raw[4] = {0, 0, 0, 0};
    bool ok[4] = {false, false, false, false};

    for (int p = 0; p < 4; p++) {
      uint8_t len = query2C(pids[p][0], pids[p][1]);
      delay(40);
      ok[p] = parseResponse2C(len, &raw[p]);
    }

    Serial.print(F("#"));
    Serial.print(iteration);
    Serial.print(F("  RPM(div4)="));
    if (ok[0]) { Serial.print(raw[0] / 4); Serial.print(F("  (div8)=")); Serial.print(raw[0] / 8); }
    else Serial.print(F("---"));

    Serial.print(F("  |  Volt="));
    if (ok[1]) Serial.print(raw[1] * 0.00268f, 2);
    else Serial.print(F("---"));

    Serial.print(F("V  |  Temp(LO-40)="));
    if (ok[2]) {
      uint8_t lo = raw[2] & 0xFF;
      Serial.print((int)lo - 40);
      Serial.print(F("C  (raw16*0.1-40="));
      Serial.print(raw[2] * 0.1f - 40.0f, 1);
      Serial.print(F("C  LO-48="));
      Serial.print((int)lo - 48);
      Serial.print(F("C)"));
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
//  Opcja 4 - Test pojedynczego adresu (Service 0x2C)
// ============================================================
void singleTest() {
  Serial.println(F("\nTEST POJEDYNCZEGO ADRESU (Service 0x2C)"));

  uint16_t addr = readHexAddress();
  if (addr == 0xFFFF) {
    Serial.println(F("Nieprawidlowy adres."));
    return;
  }

  uint8_t addrHi = (addr >> 8) & 0xFF;
  uint8_t addrLo = addr & 0xFF;

  Serial.print(F("Testuje adres: 0x"));
  PHEX(addrHi); PHEX(addrLo);
  Serial.println();

  if (!initECU()) return;

  uint8_t frame[] = {0x84, TARGET_ADDR, TESTER_ADDR, 0x2C, 0x10, addrHi, addrLo};
  printFrame("  TX (bez cs):", frame, 7);  // writeRawData doda checksum

  uint8_t len = query2C(addrHi, addrLo);
  printFrame("  RX:", obd.resultBuffer, len);

  uint16_t raw16 = 0;
  bool positive = parseResponse2C(len, &raw16);

  if (!positive) {
    bool neg = false;
    for (int i = 0; i < len; i++) if (obd.resultBuffer[i] == 0x7F) neg = true;
    if (neg) {
      Serial.println(F("Odpowiedz negatywna (0x7F) - adres nieobslugiwany."));
    } else if (len == 0) {
      Serial.println(F("Timeout - brak odpowiedzi."));
    } else {
      Serial.println(F("Nierozpoznana odpowiedz."));
    }
    return;
  }

  Serial.print(F("Odpowiedz pozytywna (0x6C). Raw uint16 = "));
  Serial.println(raw16);

  printAllFormulas(raw16);
  printValueHint(raw16);
}

// ============================================================
//  Opcja 5 - Porownanie formul (RPM + Temperatura)
// ============================================================
void formulaComparison() {
  Serial.println(F("\nPOROWNANIE FORMUL - RPM (0x0091) i Temperatura (0x0005)"));
  Serial.println(F("=========================================================="));

  if (!initECU()) return;

  uint16_t rawRPM = 0, rawTemp = 0;

  uint8_t lenRPM  = query2C(0x00, 0x91);
  delay(60);
  bool okRPM  = parseResponse2C(lenRPM, &rawRPM);

  uint8_t lenTemp = query2C(0x00, 0x05);
  delay(60);
  bool okTemp = parseResponse2C(lenTemp, &rawTemp);

  Serial.println(F("\n-- RPM (adres 0x0091) --"));
  if (okRPM) {
    uint8_t lo = rawRPM & 0xFF;
    uint8_t hi = (rawRPM >> 8) & 0xFF;
    Serial.print(F("  Raw HEX: ")); PHEX(hi); Serial.print(' '); PHEX(lo);
    Serial.print(F("  uint16=")); Serial.println(rawRPM);
    Serial.print(F("  raw / 4  = ")); Serial.print(rawRPM / 4);   Serial.println(F(" rpm"));
    Serial.print(F("  raw / 8  = ")); Serial.print(rawRPM / 8);   Serial.println(F(" rpm"));
    Serial.print(F("  raw / 2  = ")); Serial.print(rawRPM / 2);   Serial.println(F(" rpm"));
    Serial.print(F("  raw*0.25 = ")); Serial.print(rawRPM * 0.25f, 0); Serial.println(F(" rpm"));
    Serial.print(F("  raw      = ")); Serial.print(rawRPM);        Serial.println(F(" (bez skalowania)"));
  } else {
    Serial.println(F("  Brak odpowiedzi."));
  }

  Serial.println(F("\n-- TEMPERATURA (adres 0x0005) --"));
  if (okTemp) {
    uint8_t lo = rawTemp & 0xFF;
    uint8_t hi = (rawTemp >> 8) & 0xFF;
    Serial.print(F("  Raw HEX: ")); PHEX(hi); Serial.print(' '); PHEX(lo);
    Serial.print(F("  uint16=")); Serial.println(rawTemp);
    Serial.print(F("  raw16*0.1-40      = ")); Serial.print(rawTemp * 0.1f - 40.0f, 1); Serial.println(F(" C"));
    Serial.print(F("  LO_byte - 40      = ")); Serial.print((int)lo - 40);               Serial.println(F(" C"));
    Serial.print(F("  LO_byte - 48      = ")); Serial.print((int)lo - 48);               Serial.println(F(" C"));
    Serial.print(F("  LO_byte - 50      = ")); Serial.print((int)lo - 50);               Serial.println(F(" C"));
    Serial.print(F("  HI_byte - 40      = ")); Serial.print((int)hi - 40);               Serial.println(F(" C (jesli 1-bajtowy w HI)"));
    Serial.print(F("  raw16 - 40        = ")); Serial.print((int)rawTemp - 40);          Serial.println(F(" C"));
    Serial.print(F("  raw16/10 - 40     = ")); Serial.print(rawTemp / 10 - 40);          Serial.println(F(" C"));
  } else {
    Serial.println(F("  Brak odpowiedzi."));
  }

  Serial.println(F("\n=========================================================="));
  Serial.println(F("Wskazowka: dla silnika na biegu jalowym (~780 rpm)"));
  Serial.println(F("   raw/8 powinno dac ~780, raw/4 powinno dac ~1560."));
  Serial.println(F("   Dla cieplego silnika (~90C) LO_byte-40=90 gdy LO=0x82=130."));
}

// ============================================================
//  Opcja 6 - Tryb identyfikacji (20x odczyt, min/max/avg)
// ============================================================
void identificationMode() {
  Serial.println(F("\nTRYB IDENTYFIKACJI (20x odczyt)"));

  uint16_t addr = readHexAddress();
  if (addr == 0xFFFF) {
    Serial.println(F("Nieprawidlowy adres."));
    return;
  }

  uint8_t addrHi = (addr >> 8) & 0xFF;
  uint8_t addrLo = addr & 0xFF;

  Serial.print(F("Identyfikuje adres: 0x")); PHEX(addrHi); PHEX(addrLo);
  Serial.println(F(" (20 odczytow, 500ms odstep)..."));

  if (!initECU()) return;

  uint16_t samples[20];
  int validCount = 0;
  uint32_t sumVal = 0;
  uint16_t minVal = 0xFFFF;
  uint16_t maxVal = 0;

  for (int i = 0; i < 20; i++) {
    if (i > 0 && (i % 10 == 0)) {
      Serial.println(F("  Re-init ECU..."));
      reInitECU();
    }

    uint8_t len = query2C(addrHi, addrLo);
    delay(50);
    uint16_t raw = 0;
    bool ok = parseResponse2C(len, &raw);

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
      Serial.print(F("brak odpowiedzi"));
    }
    Serial.println();
    delay(500);
  }

  Serial.println(F("\n-- Wyniki --"));
  if (validCount == 0) {
    Serial.println(F("Zaden odczyt sie nie powiosl."));
    return;
  }

  uint16_t avgVal = (uint16_t)(sumVal / validCount);
  uint16_t spread = maxVal - minVal;

  Serial.print(F("  Waznych probek : ")); Serial.println(validCount);
  Serial.print(F("  MIN            : ")); Serial.println(minVal);
  Serial.print(F("  MAX            : ")); Serial.println(maxVal);
  Serial.print(F("  AVG            : ")); Serial.println(avgVal);
  Serial.print(F("  Rozrzut(MAX-MIN): ")); Serial.println(spread);

  Serial.print(F("  Charakter      : "));
  if (spread <= 10) {
    Serial.println(F("STALY - prawdopodobnie napiecie lub sensor statyczny"));
  } else if (spread <= 200) {
    Serial.println(F("MALY ROZRZUT - mozliwe temperatura lub cisnienie"));
  } else {
    Serial.println(F("ZMIENNY - mozliwe RPM, cisnienie doladowania lub pedal"));
  }

  Serial.println(F("\n-- Sugestia na podstawie AVG --"));
  printValueHint(avgVal);

  Serial.println(F("\n-- Interpretacja AVG --"));
  printAllFormulas(avgVal);
}

// ============================================================
//  Opcja 7 - Skan Service 0x21 (Local ID 0x01-0x20)
// ============================================================
void scan21() {
  Serial.println(F("\nSKAN Service 0x21 - Local ID 0x01-0x20"));
  Serial.println(F("=========================================="));

  if (!initECU()) return;
  Serial.println(F("ECU gotowe.\n"));

  for (int lid = 0x01; lid <= 0x20; lid++) {
    if (lid > 1 && ((lid - 1) % 10 == 0)) {
      Serial.println(F("  Re-init ECU..."));
      if (!reInitECU()) {
        Serial.println(F("  Re-init nieudany - przerywam skan."));
        break;
      }
    }

    uint8_t frame[] = {0x82, TARGET_ADDR, TESTER_ADDR, 0x21, (uint8_t)lid};
    obd.writeRawData(frame, 5);  // writeRawData dodaje checksum automatycznie
    uint8_t len = obd.readData();
    delay(50);

    Serial.print(F("  Local ID 0x")); PHEX((uint8_t)lid);
    Serial.print(F(": "));

    bool positive = false;
    bool negative = false;
    for (int i = 0; i < len; i++) {
      if (obd.resultBuffer[i] == 0x61) { positive = true; break; }
      if (obd.resultBuffer[i] == 0x7F) { negative = true; break; }
    }

    if (positive) {
      Serial.print(F("OK RAW: "));
      for (int i = 0; i < len; i++) { PHEX(obd.resultBuffer[i]); Serial.print(' '); }
      Serial.println();
    } else if (negative) {
      Serial.println(F("negative response (0x7F)"));
    } else if (len == 0) {
      Serial.println(F("TIMEOUT"));
    } else {
      Serial.print(F("? len=")); Serial.println(len);
    }
  }

  Serial.println(F("\n=========================================="));
  Serial.println(F("Skan Service 0x21 zakonczony."));
}

// ============================================================
//  Opcja 8 - OBD2 Live Data (standardowe PIDy Mode 0x01)
// ============================================================
void obd2LiveData() {
  Serial.println(F("\nOBD2 LIVE DATA (Mode 0x01 - standardowe PIDy)"));
  Serial.println(F("Wyslij dowolny znak aby zatrzymac."));
  Serial.println(F("================================================"));

  if (!initECU()) return;
  Serial.println(F("Start monitoringu OBD2...\n"));

  int iteration = 0;

  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("\nZatrzymano przez uzytkownika."));
      break;
    }

    Serial.print(F("#")); Serial.print(iteration); Serial.println(F(":"));

    float rpm  = obd.getLiveData(ENGINE_RPM);
    float spd  = obd.getLiveData(VEHICLE_SPEED);
    float temp = obd.getLiveData(ENGINE_COOLANT_TEMP);
    float iat  = obd.getLiveData(INTAKE_AIR_TEMP);
    float maf  = obd.getLiveData(MAF_FLOW_RATE);
    float thr  = obd.getLiveData(THROTTLE_POSITION);
    float load = obd.getLiveData(ENGINE_LOAD);
    float fp   = obd.getLiveData(FUEL_PRESSURE);

    Serial.print(F("  RPM          : ")); if (rpm  >= 0) { Serial.print(rpm,  0); Serial.println(F(" rpm"));  } else Serial.println(F("---"));
    Serial.print(F("  Speed        : ")); if (spd  >= 0) { Serial.print(spd,  0); Serial.println(F(" km/h")); } else Serial.println(F("---"));
    Serial.print(F("  Coolant Temp : ")); if (temp >= 0) { Serial.print(temp, 1); Serial.println(F(" C"));    } else Serial.println(F("---"));
    Serial.print(F("  Intake Air T : ")); if (iat  >= 0) { Serial.print(iat,  1); Serial.println(F(" C"));    } else Serial.println(F("---"));
    Serial.print(F("  MAF          : ")); if (maf  >= 0) { Serial.print(maf,  2); Serial.println(F(" g/s"));  } else Serial.println(F("---"));
    Serial.print(F("  Throttle     : ")); if (thr  >= 0) { Serial.print(thr,  1); Serial.println(F(" %"));    } else Serial.println(F("---"));
    Serial.print(F("  Engine Load  : ")); if (load >= 0) { Serial.print(load, 1); Serial.println(F(" %"));    } else Serial.println(F("---"));
    Serial.print(F("  Fuel Press   : ")); if (fp   >= 0) { Serial.print(fp,   0); Serial.println(F(" kPa"));  } else Serial.println(F("---"));

    Serial.println(F("----------------------------------------------------"));
    iteration++;
    delay(1000);
  }
}

// ============================================================
//  Opcja 9 - OBD2 Supported PIDs (wykryj obslugiwane PIDy)
// ============================================================
void obd2SupportedPIDs() {
  Serial.println(F("\nOBD2 SUPPORTED PIDs"));
  Serial.println(F("================================================"));

  if (!initECU()) return;

  // Live Data (Mode 01)
  Serial.println(F("\n-- Mode 01 - Live Data --"));
  uint8_t cnt = obd.readSupportedLiveData();
  Serial.print(F("Znaleziono: ")); Serial.print(cnt); Serial.println(F(" PIDow"));
  for (int i = 0; i < cnt; i++) {
    byte pid = obd.getSupportedData(0x01, i);
    Serial.print(F("  0x")); PHEX(pid); Serial.print(F(" - ")); Serial.println(pidName(pid));
  }

  // Freeze Frame (Mode 02)
  Serial.println(F("\n-- Mode 02 - Freeze Frame --"));
  cnt = obd.readSupportedFreezeFrame();
  Serial.print(F("Znaleziono: ")); Serial.print(cnt); Serial.println(F(" PIDow"));
  for (int i = 0; i < cnt; i++) {
    byte pid = obd.getSupportedData(0x02, i);
    Serial.print(F("  0x")); PHEX(pid); Serial.print(F(" - ")); Serial.println(pidName(pid));
  }

  // Vehicle Info (Mode 09)
  Serial.println(F("\n-- Mode 09 - Vehicle Info --"));
  cnt = obd.readSupportedVehicleInfo();
  Serial.print(F("Znaleziono: ")); Serial.print(cnt); Serial.println(F(" PIDow"));
  for (int i = 0; i < cnt; i++) {
    byte pid = obd.getSupportedData(0x09, i);
    Serial.print(F("  0x")); PHEX(pid); Serial.println();
  }

  Serial.println(F("\n================================================"));
  Serial.println(F("Skan Supported PIDs zakonczony."));
}

// ============================================================
//  Opcja A - OBD2 DTC (odczyt/kasowanie bledow)
// ============================================================
void obd2DTC() {
  Serial.println(F("\nOBD2 DTC - Kody bledow"));
  Serial.println(F("================================================"));
  Serial.println(F("  S - Stored DTC (odczyt zapisanych)"));
  Serial.println(F("  P - Pending DTC (odczyt oczekujacych)"));
  Serial.println(F("  C - Clear DTC (kasowanie bledow)"));
  Serial.print(F("Wybierz: "));

  while (Serial.available() == 0) delay(10);
  char sub = (char)Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(sub);

  if (!initECU()) return;

  if (sub == 'S' || sub == 's') {
    Serial.println(F("\n-- Stored DTCs (Mode 03) --"));
    uint8_t cnt = obd.readStoredDTCs();
    if (cnt == 0) {
      Serial.println(F("Brak zapisanych bledow."));
    } else {
      Serial.print(cnt); Serial.println(F(" blad(ow):"));
      for (int i = 0; i < cnt; i++) {
        Serial.print(F("  ")); Serial.println(obd.getStoredDTC(i));
      }
    }
  } else if (sub == 'P' || sub == 'p') {
    Serial.println(F("\n-- Pending DTCs (Mode 07) --"));
    uint8_t cnt = obd.readPendingDTCs();
    if (cnt == 0) {
      Serial.println(F("Brak oczekujacych bledow."));
    } else {
      Serial.print(cnt); Serial.println(F(" blad(ow):"));
      for (int i = 0; i < cnt; i++) {
        Serial.print(F("  ")); Serial.println(obd.getPendingDTC(i));
      }
    }
  } else if (sub == 'C' || sub == 'c') {
    Serial.println(F("\n-- Clear DTCs (Mode 04) --"));
    Serial.println(F("Kasowanie bledow..."));
    if (obd.clearDTCs()) {
      Serial.println(F("Bledy skasowane."));
    } else {
      Serial.println(F("Nie udalo sie skasowac bledow."));
    }
  } else {
    Serial.println(F("Nieznana opcja."));
  }
}

// ============================================================
//  Opcja B - OBD2 Vehicle Info (VIN, Calibration ID)
// ============================================================
void obd2VehicleInfo() {
  Serial.println(F("\nOBD2 VEHICLE INFO (Mode 09)"));
  Serial.println(F("================================================"));

  if (!initECU()) return;

  Serial.println(F("\n-- VIN (0x02) --"));
  String vin = obd.getVehicleInfo(read_VIN);
  Serial.print(F("VIN: "));
  Serial.println(vin.length() > 0 ? vin : "(brak danych)");

  Serial.println(F("\n-- Calibration ID (0x04) --"));
  String calId = obd.getVehicleInfo(read_ID);
  Serial.print(F("Cal ID: "));
  Serial.println(calId.length() > 0 ? calId : "(brak danych)");

  Serial.println(F("\n================================================"));
}

// ============================================================
//  Opcja C - Zmiana protokolu
// ============================================================
void changeProtocol() {
  Serial.println(F("\nZMIEN PROTOKOL"));
  Serial.println(F("================================================"));
  Serial.println(F("  1 - Automatic (Auto-detect)"));
  Serial.println(F("  2 - ISO9141 (5-baud slow init)"));
  Serial.println(F("  3 - ISO14230_Slow (KWP2000 slow)"));
  Serial.println(F("  4 - ISO14230_Fast (KWP2000 fast - BMW EDC16)"));
  Serial.print(F("Wybierz: "));

  while (Serial.available() == 0) delay(10);
  char sub = (char)Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(sub);

  String proto = "";
  switch (sub) {
    case '1': proto = "Automatic";      break;
    case '2': proto = "ISO9141";        break;
    case '3': proto = "ISO14230_Slow";  break;
    case '4': proto = "ISO14230_Fast";  break;
    default:
      Serial.println(F("Nieznana opcja."));
      return;
  }

  obd.setProtocol(proto);
  Serial.print(F("Protokol ustawiony: ")); Serial.println(proto);
  Serial.println(F("Inicjalizacja z nowym protokolem..."));
  if (obd.initOBD2()) {
    Serial.println(F("Polaczenie nawiazane."));
  } else {
    Serial.println(F("Nie udalo sie polaczyc."));
  }
}

// ============================================================
//  Opcja D - Re-init ECU (ponowna inicjalizacja)
// ============================================================
void reInitMenu() {
  Serial.println(F("\nRE-INIT ECU (ponowna inicjalizacja)"));
  Serial.println(F("================================================"));
  obd.setProtocol("Automatic");
  if (obd.initOBD2()) {
    Serial.println(F("Polaczenie nawiazane."));
  } else {
    Serial.println(F("Inicjalizacja nieudana."));
  }
}

// ============================================================
//  Opcja E - OBD2 Freeze Frame (dane zamrozone)
// ============================================================
void obd2FreezeFrame() {
  Serial.println(F("\nOBD2 FREEZE FRAME (Mode 0x02)"));
  Serial.println(F("================================================"));

  if (!initECU()) return;

  Serial.println(F("Odczyt danych Freeze Frame..."));

  float rpm  = obd.getFreezeFrame(ENGINE_RPM);
  float spd  = obd.getFreezeFrame(VEHICLE_SPEED);
  float temp = obd.getFreezeFrame(ENGINE_COOLANT_TEMP);
  float iat  = obd.getFreezeFrame(INTAKE_AIR_TEMP);
  float load = obd.getFreezeFrame(ENGINE_LOAD);
  float thr  = obd.getFreezeFrame(THROTTLE_POSITION);

  Serial.print(F("  RPM          : ")); if (rpm  >= 0) { Serial.print(rpm,  0); Serial.println(F(" rpm"));  } else Serial.println(F("---"));
  Serial.print(F("  Speed        : ")); if (spd  >= 0) { Serial.print(spd,  0); Serial.println(F(" km/h")); } else Serial.println(F("---"));
  Serial.print(F("  Coolant Temp : ")); if (temp >= 0) { Serial.print(temp, 1); Serial.println(F(" C"));    } else Serial.println(F("---"));
  Serial.print(F("  Intake Air T : ")); if (iat  >= 0) { Serial.print(iat,  1); Serial.println(F(" C"));    } else Serial.println(F("---"));
  Serial.print(F("  Engine Load  : ")); if (load >= 0) { Serial.print(load, 1); Serial.println(F(" %"));    } else Serial.println(F("---"));
  Serial.print(F("  Throttle     : ")); if (thr  >= 0) { Serial.print(thr,  1); Serial.println(F(" %"));    } else Serial.println(F("---"));

  Serial.println(F("\n================================================"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  obd.setDebug(Serial);
  Serial.println(F("\n===================================================="));
  Serial.println(F("  PID_Tester - BMW K-Line Multi-Protocol Tester"));
  Serial.println(F("  Biblioteka: OBD2_KLine"));
  Serial.println(F("  Protokoly: ISO9141 / ISO14230 Slow / Fast"));
  Serial.println(F("===================================================="));
}

// ============================================================
//  Wyswietl menu glowne
// ============================================================
void printMenu() {
  Serial.println(F("\n+-------------------------------------------------------+"));
  Serial.println(F("|  MENU GLOWNE - PID Tester (Multi-Protocol)            |"));
  Serial.println(F("+-------------------------------------------------------+"));
  Serial.println(F("|  1 - PELNY SKAN KWP2000 (0x0001-0x00FF, Srv 0x2C)    |"));
  Serial.println(F("|  2 - SKAN ZNANYCH ADRESOW (~39 wpisow, Srv 0x2C)     |"));
  Serial.println(F("|  3 - LIVE MONITOR KWP2000 (RPM/Volt/Temp/Boost)      |"));
  Serial.println(F("|  4 - TEST POJEDYNCZEGO ADRESU (Service 0x2C)         |"));
  Serial.println(F("|  5 - POROWNANIE FORMUL (RPM + Temperatura)           |"));
  Serial.println(F("|  6 - TRYB IDENTYFIKACJI (20x odczyt, min/max/avg)    |"));
  Serial.println(F("|  7 - SKAN Service 0x21 (Local ID 0x01-0x20)          |"));
  Serial.println(F("|  8 - OBD2 LIVE DATA (standardowe PIDy Mode 0x01)     |"));
  Serial.println(F("|  9 - OBD2 SUPPORTED PIDs (wykryj obslugiwane PIDy)   |"));
  Serial.println(F("|  A - OBD2 DTC (odczyt/kasowanie bledow)              |"));
  Serial.println(F("|  B - OBD2 VEHICLE INFO (VIN, Calibration ID)         |"));
  Serial.println(F("|  C - ZMIEN PROTOKOL (Auto/ISO9141/ISO14230_S/F)      |"));
  Serial.println(F("|  D - RE-INIT ECU (ponowna inicjalizacja)             |"));
  Serial.println(F("|  E - OBD2 FREEZE FRAME (dane zamrozone)              |"));
  Serial.println(F("+-------------------------------------------------------+"));
  Serial.print(F("Wybierz opcje: "));
}

// ============================================================
//  LOOP - menu glowne
// ============================================================
void loop() {
  printMenu();

  while (Serial.available() == 0) delay(10);
  char choice = (char)Serial.read();
  while (Serial.available()) Serial.read();

  Serial.println(choice);

  switch (choice) {
    case '1': fullScan();           break;
    case '2': knownScan();          break;
    case '3': liveMonitor();        break;
    case '4': singleTest();         break;
    case '5': formulaComparison();  break;
    case '6': identificationMode(); break;
    case '7': scan21();             break;
    case '8': obd2LiveData();       break;
    case '9': obd2SupportedPIDs();  break;
    case 'A': case 'a': obd2DTC();         break;
    case 'B': case 'b': obd2VehicleInfo(); break;
    case 'C': case 'c': changeProtocol();  break;
    case 'D': case 'd': reInitMenu();      break;
    case 'E': case 'e': obd2FreezeFrame(); break;
    default:
      Serial.println(F("Nieznana opcja. Wpisz 1-9 lub A-E."));
      break;
  }

  delay(1000);
}
