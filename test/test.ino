// ============================================================================
//  BMW EDC16C31 (M57) - PID TESTER po K-Line
//  Protokół: KWP2000 (ISO 14230) + SAE J1979
//  Testuje: IAT, Boost/MAP, Napięcie Akumulatora + BMW-specific PIDs
// ============================================================================

#define RX_PIN 35
#define TX_PIN 38

#define TARGET_ADDR 0x12   // BMW EDC16 silnik
#define TESTER_ADDR 0xF1   // Nasz tester

// ==================== DEFINICJE PIDów ====================

// --- SAE J1979 Standard PIDs (Service 01) ---
#define SAE_PID_SUPPORTED_01_20   0x00
#define SAE_PID_SUPPORTED_21_40   0x20
#define SAE_PID_SUPPORTED_41_60   0x40
#define SAE_PID_ENGINE_LOAD       0x04
#define SAE_PID_COOLANT_TEMP      0x05
#define SAE_PID_MAP               0x0B  // Manifold Absolute Pressure (Boost)
#define SAE_PID_RPM               0x0C
#define SAE_PID_SPEED             0x0D
#define SAE_PID_IAT               0x0F  // Intake Air Temperature
#define SAE_PID_MAF               0x10
#define SAE_PID_THROTTLE          0x11
#define SAE_PID_BARO_PRESSURE     0x33  // Barometric Pressure (do obliczenia boost)
#define SAE_PID_CTRL_MODULE_VOLT  0x42  // Napięcie akumulatora

// --- BMW-specific Memory Addresses (dla Service 0x2C) ---
#define BMW_ADDR_RPM_HI     0x00
#define BMW_ADDR_RPM_LO     0x91
#define BMW_ADDR_VOLT_HI    0x00
#define BMW_ADDR_VOLT_LO    0x93
#define BMW_ADDR_TEMP_HI    0x00
#define BMW_ADDR_TEMP_LO    0x05
#define BMW_ADDR_BOOST_HI   0x00
#define BMW_ADDR_BOOST_LO   0x9E

// ==================== STRUKTURY ====================

struct SAE_PID_Entry {
  uint8_t pid;
  const char* name;
  const char* unit;
  uint8_t dataBytes;  // 1 = A only, 2 = A+B
};

// PIDy które chcemy testować (SAE J1979)
const SAE_PID_Entry saePidsToTest[] = {
  { SAE_PID_IAT,              "Intake Air Temp (IAT)",      "°C",  1 },
  { SAE_PID_MAP,              "MAP / Boost Pressure",       "kPa", 1 },
  { SAE_PID_CTRL_MODULE_VOLT, "Battery Voltage",            "V",   2 },
  { SAE_PID_BARO_PRESSURE,    "Barometric Pressure",        "kPa", 1 },
  { SAE_PID_COOLANT_TEMP,     "Coolant Temp",               "°C",  1 },
  { SAE_PID_RPM,              "Engine RPM",                 "rpm", 2 },
  { SAE_PID_SPEED,            "Vehicle Speed",              "km/h",1 },
  { SAE_PID_ENGINE_LOAD,      "Engine Load",                "%",   1 },
  { SAE_PID_MAF,              "MAF Flow Rate",              "g/s", 2 },
  { SAE_PID_THROTTLE,         "Throttle Position",          "%",   1 },
};
const int NUM_SAE_PIDS = sizeof(saePidsToTest) / sizeof(saePidsToTest[0]);

struct BMW_PID_Entry {
  uint8_t addrHi;
  uint8_t addrLo;
  const char* name;
  const char* unit;
  float scale;
  float offset;
};

const BMW_PID_Entry bmwPidsToTest[] = {
  { 0x00, 0x91, "RPM",                "rpm",  0.25,   0.0   },
  { 0x00, 0x93, "Battery Voltage",    "V",    0.00268, 0.0  },
  { 0x00, 0x05, "Coolant Temp",       "°C",   0.1,   -40.0  },
  { 0x00, 0x9E, "Boost Pressure",     "hPa",  0.136,  0.0   },
};
const int NUM_BMW_PIDS = sizeof(bmwPidsToTest) / sizeof(bmwPidsToTest[0]);

// ==================== GLOBALS ====================
byte responseBuf[256];

// ==================== HELPER FUNCTIONS ====================

void printHex(uint8_t val) {
  if (val < 0x10) Serial.print("0");
  Serial.print(val, HEX);
}

void printRawFrame(const char* label, byte* data, int len) {
  Serial.print("  [RAW ");
  Serial.print(label);
  Serial.print("] ");
  for (int i = 0; i < len; i++) {
    printHex(data[i]);
    Serial.print(" ");
  }
  Serial.println();
}

byte calcChecksum(byte* data, int len) {
  int sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return (byte)(sum & 0xFF);
}

// ==================== KOMUNIKACJA ECU ====================

bool fastInitECU() {
  Serial1.end();
  Serial1.begin(10400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Fast init: 25ms LOW pulse
  Serial1.updateBaudRate(360);
  Serial1.write(0x00);
  Serial1.flush();
  delay(22);
  Serial1.updateBaudRate(10400);

  while (Serial1.available()) Serial1.read();

  // StartCommunication request (celowane w 0x12 = silnik BMW)
  byte startComm[] = { 0x81, TARGET_ADDR, TESTER_ADDR, 0x81, 0x05 };
  Serial1.write(startComm, 5);
  Serial1.flush();

  // Odczyt odpowiedzi
  unsigned long startTime = millis();
  int count = 0;
  byte rxBuf[64];

  while (millis() - startTime < 200) {
    if (Serial1.available()) {
      rxBuf[count++] = Serial1.read();
      startTime = millis();
    }
  }

  Serial.print("  [INIT RAW] ");
  for (int i = 0; i < count; i++) {
    printHex(rxBuf[i]);
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < count; i++) {
    if (rxBuf[i] == 0xC1) {
      Serial.println("  ✅ ECU wybudzony pomyslnie (0xC1 received)");
      delay(50);
      return true;
    }
  }

  Serial.println("  ❌ ECU nie odpowiedzial");
  return false;
}

int sendAndReceive(byte* req, int reqLen, byte* resp) {
  while (Serial1.available()) Serial1.read();

  Serial1.write(req, reqLen);
  Serial1.flush();

  unsigned long startTime = millis();
  int count = 0;

  while (millis() - startTime < 300) {
    if (Serial1.available()) {
      resp[count++] = Serial1.read();
      startTime = millis();
    }
  }
  return count;
}

// ==================== SAE J1979 FUNCTIONS ====================

// Buduje ramkę KWP2000 dla SAE J1979 Service 01 request
// Format: [0x82] [TARGET] [TESTER] [0x01] [PID] [CHECKSUM]
void buildSAE_Request(byte* buf, uint8_t pid) {
  buf[0] = 0x82;         // Format byte: Physical addressing, 2 data bytes
  buf[1] = TARGET_ADDR;  // ECU address
  buf[2] = TESTER_ADDR;  // Tester address
  buf[3] = 0x01;         // Service 01: Current Data
  buf[4] = pid;          // PID
  buf[5] = calcChecksum(buf, 5);
}

float decodeSAE_PID(uint8_t pid, uint8_t A, uint8_t B) {
  switch (pid) {
    case SAE_PID_IAT:              return A - 40.0f;                      // °C
    case SAE_PID_MAP:              return (float)A;                       // kPa
    case SAE_PID_CTRL_MODULE_VOLT: return ((A * 256.0f) + B) / 1000.0f;  // V
    case SAE_PID_BARO_PRESSURE:    return (float)A;                       // kPa
    case SAE_PID_COOLANT_TEMP:     return A - 40.0f;                      // °C
    case SAE_PID_RPM:              return ((A * 256.0f) + B) / 4.0f;     // rpm
    case SAE_PID_SPEED:            return (float)A;                       // km/h
    case SAE_PID_ENGINE_LOAD:      return A * 100.0f / 255.0f;           // %
    case SAE_PID_MAF:              return ((A * 256.0f) + B) / 100.0f;   // g/s
    case SAE_PID_THROTTLE:         return A * 100.0f / 255.0f;           // %
    default:                       return -999.0f;
  }
}

// Parsowanie odpowiedzi SAE J1979
// Odpowiedź KWP2000: [FMT] [SRC] [TGT] [0x41] [PID] [A] [B?] [CS]
// Ale z echem: echo(6 bytes) + response
// Szukamy bajtu 0x41 w buforze
bool parseSAE_Response(byte* resp, int len, uint8_t expectedPID, uint8_t* A, uint8_t* B) {
  for (int i = 0; i < len - 2; i++) {
    if (resp[i] == 0x41 && resp[i + 1] == expectedPID) {
      *A = (i + 2 < len) ? resp[i + 2] : 0;
      *B = (i + 3 < len) ? resp[i + 3] : 0;
      return true;
    }
  }
  return false;
}

void testSAE_PID(const SAE_PID_Entry& entry) {
  byte request[6];
  buildSAE_Request(request, entry.pid);

  Serial.print("\n  📡 PID 0x");
  printHex(entry.pid);
  Serial.print(" (");
  Serial.print(entry.name);
  Serial.println(")");

  printRawFrame("TX", request, 6);

  int respLen = sendAndReceive(request, 6, responseBuf);

  if (respLen > 0) {
    printRawFrame("RX", responseBuf, respLen);

    uint8_t A = 0, B = 0;
    if (parseSAE_Response(responseBuf, respLen, entry.pid, &A, &B)) {
      float value = decodeSAE_PID(entry.pid, A, B);
      Serial.print("  ✅ Wartość: ");
      Serial.print(value, 2);
      Serial.print(" ");
      Serial.println(entry.unit);

      // Jeśli to MAP, oblicz boost
      if (entry.pid == SAE_PID_MAP) {
        Serial.print("     (Boost = MAP - Baro. Odczytaj PID 0x33 dla obliczenia boost'u)");
        Serial.println();
      }
    } else {
      // Sprawdź czy ECU zwróciło Negative Response (0x7F)
      for (int i = 0; i < respLen; i++) {
        if (responseBuf[i] == 0x7F) {
          Serial.print("  ❌ ECU odrzucil: Negative Response (NRC: 0x");
          if (i + 2 < respLen) printHex(responseBuf[i + 2]);
          Serial.println(")");
          return;
        }
      }
      Serial.println("  ❌ Nie udalo sie zdekodowac odpowiedzi");
    }
  } else {
    Serial.println("  ❌ Brak odpowiedzi (timeout)");
  }

  delay(50);  // P3 timing
}

// ==================== BMW-SPECIFIC (Service 0x2C) ====================

// Buduje ramkę DynamicallyDefineLocalIdentifier
// Format: [0x84] [TARGET] [TESTER] [0x2C] [0x10] [ADDR_HI] [ADDR_LO] [CS]
void buildBMW_Request(byte* buf, uint8_t addrHi, uint8_t addrLo) {
  buf[0] = 0x84;         // Format: 4 data bytes
  buf[1] = TARGET_ADDR;
  buf[2] = TESTER_ADDR;
  buf[3] = 0x2C;         // DynamicallyDefineLocalIdentifier
  buf[4] = 0x10;         // DefineByMemoryAddress
  buf[5] = addrHi;
  buf[6] = addrLo;
  buf[7] = calcChecksum(buf, 7);
}

void testBMW_PID(const BMW_PID_Entry& entry) {
  byte request[8];
  buildBMW_Request(request, entry.addrHi, entry.addrLo);

  Serial.print("\n  📡 BMW Addr 0x");
  printHex(entry.addrHi);
  printHex(entry.addrLo);
  Serial.print(" (");
  Serial.print(entry.name);
  Serial.println(")");

  printRawFrame("TX", request, 8);

  int respLen = sendAndReceive(request, 8, responseBuf);

  if (respLen > 0) {
    printRawFrame("RX", responseBuf, respLen);

    // Szukamy pozytywnej odpowiedzi 0x6C
    for (int i = 0; i < respLen - 2; i++) {
      if (responseBuf[i] == 0x6C) {
        // Dane są 2 bajty po 0x6C (offset zależny od struktury ramki)
        int dataIdx = i + 2;
        if (dataIdx + 1 < respLen) {
          int rawVal = (responseBuf[dataIdx] << 8) | responseBuf[dataIdx + 1];
          float value = (rawVal * entry.scale) + entry.offset;
          Serial.print("  ✅ Raw: 0x");
          printHex(responseBuf[dataIdx]);
          printHex(responseBuf[dataIdx + 1]);
          Serial.print(" → Wartość: ");
          Serial.print(value, 2);
          Serial.print(" ");
          Serial.println(entry.unit);
        }
        return;
      }
    }

    // Negative Response?
    for (int i = 0; i < respLen; i++) {
      if (responseBuf[i] == 0x7F) {
        Serial.print("  ❌ Negative Response (NRC: 0x");
        if (i + 2 < respLen) printHex(responseBuf[i + 2]);
        Serial.println(")");
        return;
      }
    }

    Serial.println("  ❌ Nie znaleziono 0x6C w odpowiedzi");
  } else {
    Serial.println("  ❌ Brak odpowiedzi (timeout)");
  }

  delay(50);
}

// ==================== SKAN SUPPORTED PIDs (SAE) ====================

void scanSupportedPIDs() {
  Serial.println("\n═══════════════════════════════════════════════");
  Serial.println(" SKAN OBSŁUGIWANYCH PIDów SAE J1979 ");
  Serial.println("═══════════════════════════════════════════════");

  uint8_t supportRanges[] = { 0x00, 0x20, 0x40 };
  const char* rangeNames[] = { "PIDs 01-20", "PIDs 21-40", "PIDs 41-60" };

  for (int r = 0; r < 3; r++) {
    byte request[6];
    buildSAE_Request(request, supportRanges[r]);

    Serial.print("\n  Sprawdzam zakres: ");
    Serial.println(rangeNames[r]);
    printRawFrame("TX", request, 6);

    int respLen = sendAndReceive(request, 6, responseBuf);

    if (respLen > 0) {
      printRawFrame("RX", responseBuf, respLen);

      // Szukamy 0x41 + PID w odpowiedzi
      for (int i = 0; i < respLen - 5; i++) {
        if (responseBuf[i] == 0x41 && responseBuf[i + 1] == supportRanges[r]) {
          uint8_t bytes[4] = {
            responseBuf[i + 2], responseBuf[i + 3],
            responseBuf[i + 4], responseBuf[i + 5]
          };

          Serial.print("  Bitmap: ");
          for (int b = 0; b < 4; b++) {
            for (int bit = 7; bit >= 0; bit--) {
              Serial.print((bytes[b] >> bit) & 1);
            }
            Serial.print(" ");
          }
          Serial.println();

          Serial.print("  Obsługiwane PIDs: ");
          for (int b = 0; b < 4; b++) {
            for (int bit = 7; bit >= 0; bit--) {
              if ((bytes[b] >> bit) & 1) {
                uint8_t pidNum = supportRanges[r] + (b * 8) + (7 - bit) + 1;
                Serial.print("0x");
                printHex(pidNum);
                Serial.print(" ");
              }
            }
          }
          Serial.println();
          break;
        }
      }
    } else {
      Serial.println("  ❌ Brak odpowiedzi");
      break;  // Jeśli nie ma odpowiedzi na 0x00, nie próbuj dalszych zakresów
    }

    delay(100);
  }
}

// ==================== CIĄGŁY MONITORING ====================

void continuousMonitor() {
  Serial.println("\n═══════════════════════════════════════════════");
  Serial.println(" CIĄGŁY MONITORING (Wyślij cokolwiek aby stop)");
  Serial.println("═══════════════════════════════════════════════");

  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println("\n  [STOP] Monitoring zatrzymany.");
      return;
    }

    // IAT
    {
      byte req[6];
      buildSAE_Request(req, SAE_PID_IAT);
      int len = sendAndReceive(req, 6, responseBuf);
      uint8_t A, B;
      if (parseSAE_Response(responseBuf, len, SAE_PID_IAT, &A, &B)) {
        Serial.print("IAT: ");
        Serial.print(A - 40);
        Serial.print("°C");
      } else {
        Serial.print("IAT: ---");
      }
    }
    Serial.print(" | ");

    // MAP/Boost
    {
      byte req[6];
      buildSAE_Request(req, SAE_PID_MAP);
      int len = sendAndReceive(req, 6, responseBuf);
      uint8_t A, B;
      if (parseSAE_Response(responseBuf, len, SAE_PID_MAP, &A, &B)) {
        Serial.print("MAP: ");
        Serial.print(A);
        Serial.print("kPa");
      } else {
        Serial.print("MAP: ---");
      }
    }
    Serial.print(" | ");

    // Napięcie
    {
      byte req[6];
      buildSAE_Request(req, SAE_PID_CTRL_MODULE_VOLT);
      int len = sendAndReceive(req, 6, responseBuf);
      uint8_t A, B;
      if (parseSAE_Response(responseBuf, len, SAE_PID_CTRL_MODULE_VOLT, &A, &B)) {
        float volt = ((A * 256.0f) + B) / 1000.0f;
        Serial.print("VOLT: ");
        Serial.print(volt, 1);
        Serial.print("V");
      } else {
        Serial.print("VOLT: ---");
      }
    }
    Serial.print(" | ");

    // Baro (do obliczenia boost)
    {
      byte req[6];
      buildSAE_Request(req, SAE_PID_BARO_PRESSURE);
      int len = sendAndReceive(req, 6, responseBuf);
      uint8_t A, B;
      if (parseSAE_Response(responseBuf, len, SAE_PID_BARO_PRESSURE, &A, &B)) {
        Serial.print("BARO: ");
        Serial.print(A);
        Serial.print("kPa");
      } else {
        Serial.print("BARO: ---");
      }
    }

    Serial.println();
    delay(500);
  }
}

// ==================== MAIN ====================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════╗");
  Serial.println("║  BMW EDC16C31 (M57) - PID TESTER v1.0        ║");
  Serial.println("║  K-Line / KWP2000 / SAE J1979                ║");
  Serial.println("║  IAT • Boost • Battery • BMW-specific        ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
}

void loop() {
  Serial.println("\n──────────────────────────────────────────────");
  Serial.println(" MENU GŁÓWNE:");
  Serial.println("  1 - Test SAE J1979 PIDs (IAT, MAP, Volt...)");
  Serial.println("  2 - Test BMW-specific PIDs (0x2C)");
  Serial.println("  3 - Skan obsługiwanych PIDs (bitmapa)");
  Serial.println("  4 - CIĄGŁY MONITORING (IAT+MAP+VOLT+BARO)");
  Serial.println("  5 - Test POJEDYNCZEGO PID (wpisz hex)");
  Serial.println("  6 - BMW FULL LIVE DATA (RPM+V+Temp+Boost)");
  Serial.println("──────────────────────────────────────────────");

  while (Serial.available() == 0) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();

  if (choice < '1' || choice > '6') return;

  Serial.println("\n  [INIT] Budzenie ECU...");
  if (!fastInitECU()) {
    Serial.println("  ❌ Nie udalo sie wybudzic ECU. Sprobuj ponownie.");
    delay(2000);
    return;
  }

  switch (choice) {
    case '1': {
      Serial.println("\n═══════════════════════════════════════════════");
      Serial.println(" TEST SAE J1979 STANDARD PIDs");
      Serial.println("═══════════════════════════════════════════════");
      for (int i = 0; i < NUM_SAE_PIDS; i++) {
        testSAE_PID(saePidsToTest[i]);
      }
      Serial.println("\n  ✅ Test SAE zakończony.");
      break;
    }

    case '2': {
      Serial.println("\n═══════════════════════════════════════════════");
      Serial.println(" TEST BMW-SPECIFIC PIDs (Service 0x2C)");
      Serial.println("═══════════════════════════════════════════════");
      for (int i = 0; i < NUM_BMW_PIDS; i++) {
        testBMW_PID(bmwPidsToTest[i]);
      }
      Serial.println("\n  ✅ Test BMW zakończony.");
      break;
    }

    case '3': {
      scanSupportedPIDs();
      break;
    }

    case '4': {
      continuousMonitor();
      break;
    }

    case '5': {
      Serial.println("\n  Wpisz PID w hex (np. 0F dla IAT):");
      while (Serial.available() == 0) delay(10);
      delay(100);
      String hexStr = "";
      while (Serial.available()) {
        char c = Serial.read();
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
          hexStr += c;
        }
      }
      if (hexStr.length() > 0) {
        uint8_t pid = (uint8_t)strtol(hexStr.c_str(), NULL, 16);
        SAE_PID_Entry custom = { pid, "Custom PID", "?", 2 };
        testSAE_PID(custom);
      }
      break;
    }

    case '6': {
      Serial.println("\n═══════════════════════════════════════════════");
      Serial.println(" BMW FULL LIVE DATA (Service 0x2C) x3 pętle");
      Serial.println("═══════════════════════════════════════════════");

      for (int loop = 0; loop < 3; loop++) {
        Serial.print("\n  --- Pętla ");
        Serial.print(loop + 1);
        Serial.println("/3 ---");

        float vals[NUM_BMW_PIDS];
        for (int i = 0; i < NUM_BMW_PIDS; i++) {
          byte request[8];
          buildBMW_Request(request, bmwPidsToTest[i].addrHi, bmwPidsToTest[i].addrLo);
          int respLen = sendAndReceive(request, 8, responseBuf);

          Serial.print("  [");
          Serial.print(bmwPidsToTest[i].name);
          Serial.print("] RAW: ");
          for (int j = 0; j < respLen; j++) {
            printHex(responseBuf[j]);
            Serial.print(" ");
          }
          Serial.println();

          vals[i] = -999;
          for (int j = 0; j < respLen - 2; j++) {
            if (responseBuf[j] == 0x6C) {
              int dataIdx = j + 2;
              if (dataIdx + 1 < respLen) {
                int rawVal = (responseBuf[dataIdx] << 8) | responseBuf[dataIdx + 1];
                vals[i] = (rawVal * bmwPidsToTest[i].scale) + bmwPidsToTest[i].offset;
              }
              break;
            }
          }
          delay(40);
        }

        Serial.print("  → RPM: ");
        Serial.print(vals[0], 0);
        Serial.print(" | V: ");
        Serial.print(vals[1], 1);
        Serial.print(" | Temp: ");
        Serial.print(vals[2], 1);
        Serial.print("°C | Boost: ");
        Serial.print(vals[3], 0);
        Serial.println(" hPa");
        Serial.println("  ────────────────────────────────────");

        delay(500);
      }
      break;
    }
  }

  delay(1500);
}