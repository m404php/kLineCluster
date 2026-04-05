// ============================================================
//  PID_Tester.ino – Narzędzie diagnostyczne EDC15 K-Line
//  Biblioteka: OBD2_KLine  |  Definicje PID: PIDs.h
//  Protokoły: ISO9141 / ISO14230_Slow / ISO14230_Fast
//  Piny: RX=35, TX=38  |  ESP32
//  ECU: 0x12 (BMW)  |  Tester: 0xF1
// ============================================================

#include "OBD2_KLine.h"
#include "PIDs.h"

#define RX_PIN      35
#define TX_PIN      38
#define ECU_ADDR    0x12
#define TESTER_ADDR 0xF1

// Pomocnicze makro do wydruku hex
#define PHEX(v)  do { if ((v) < 0x10) Serial.print('0'); Serial.print((v), HEX); } while (0)

// Globalny obiekt biblioteki OBD2_KLine
OBD2_KLine obd(Serial1, 10400, RX_PIN, TX_PIN);

// Aktualnie używany protokół (aktualizowany po każdym udanym połączeniu)
String currentProtocol = "Automatic";

// ============================================================
//  Wzbudzenie ECU z ponowieniami
//  EDC15 gubi sesję po każdym PIDzie – wywoływać przed każdym zapytaniem
// ============================================================
bool initWithRetry(int maxRetries = 3) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    obd.setProtocol(currentProtocol);
    if (obd.initOBD2()) return true;
    Serial.print(F("  ⚠️ Init próba "));
    Serial.print(attempt + 1);
    Serial.println(F(" nieudana, ponawiam..."));
    delay(500);
  }
  Serial.println(F("  ❌ Init nieudany po wszystkich próbach."));
  return false;
}

// ============================================================
//  Nazwa PID OBD2
// ============================================================
String pidName(byte pid) {
  switch (pid) {
    case MONITOR_STATUS_SINCE_DTC_CLEARED:      return F("Monitor Status");
    case FREEZE_DTC:                            return F("Freeze DTC");
    case FUEL_SYSTEM_STATUS:                    return F("Fuel System Status");
    case ENGINE_LOAD:                           return F("Engine Load");
    case ENGINE_COOLANT_TEMP:                   return F("Coolant Temp");
    case SHORT_TERM_FUEL_TRIM_BANK_1:           return F("STFT Bank1");
    case LONG_TERM_FUEL_TRIM_BANK_1:            return F("LTFT Bank1");
    case SHORT_TERM_FUEL_TRIM_BANK_2:           return F("STFT Bank2");
    case LONG_TERM_FUEL_TRIM_BANK_2:            return F("LTFT Bank2");
    case FUEL_PRESSURE:                         return F("Fuel Pressure");
    case INTAKE_MANIFOLD_ABS_PRESSURE:          return F("MAP Pressure");
    case ENGINE_RPM:                            return F("Engine RPM");
    case VEHICLE_SPEED:                         return F("Vehicle Speed");
    case TIMING_ADVANCE:                        return F("Timing Advance");
    case INTAKE_AIR_TEMP:                       return F("Intake Air Temp");
    case MAF_FLOW_RATE:                         return F("MAF Flow Rate");
    case THROTTLE_POSITION:                     return F("Throttle Position");
    case COMMANDED_SECONDARY_AIR_STATUS:        return F("Sec Air Status");
    case OBD_STANDARDS:                         return F("OBD Standards");
    case RUN_TIME_SINCE_ENGINE_START:           return F("Runtime Since Start");
    case DISTANCE_TRAVELED_WITH_MIL_ON:         return F("Dist w/ MIL on");
    case FUEL_RAIL_PRESSURE:                    return F("Fuel Rail Pressure");
    case FUEL_RAIL_GAUGE_PRESSURE:              return F("Fuel Rail Gauge P");
    case COMMANDED_EGR:                         return F("Commanded EGR");
    case EGR_ERROR:                             return F("EGR Error");
    case FUEL_TANK_LEVEL_INPUT:                 return F("Fuel Tank Level");
    case WARMUPS_SINCE_CODES_CLEARED:           return F("Warmups since clrd");
    case DISTANCE_TRAVELED_SINCE_CODES_CLEARED: return F("Dist since cleared");
    case ABSOLUTE_BAROMETRIC_PRESSURE:          return F("Barometric Pressure");
    case CONTROL_MODULE_VOLTAGE:                return F("Control Module V");
    case ABS_LOAD_VALUE:                        return F("Abs Load Value");
    case RELATIVE_THROTTLE_POSITION:            return F("Rel Throttle Pos");
    case AMBIENT_AIR_TEMP:                      return F("Ambient Air Temp");
    case TIME_RUN_WITH_MIL_ON:                  return F("Time w/ MIL on");
    case TIME_SINCE_CODES_CLEARED:              return F("Time since cleared");
    case ENGINE_OIL_TEMP:                       return F("Engine Oil Temp");
    case ENGINE_FUEL_RATE:                      return F("Engine Fuel Rate");
    case DEMANDED_ENGINE_PERCENT_TORQUE:        return F("Demanded Torque");
    case ACTUAL_ENGINE_TORQUE:                  return F("Actual Torque");
    case ENGINE_REFERENCE_TORQUE:               return F("Reference Torque");
    default: {
      String s = "PID 0x";
      if (pid < 0x10) s += "0";
      s += String(pid, HEX);
      return s;
    }
  }
}

// ============================================================
//  Jednostka PID OBD2
// ============================================================
String pidUnit(byte pid) {
  switch (pid) {
    case ENGINE_RPM:                            return F("rpm");
    case VEHICLE_SPEED:                         return F("km/h");
    case ENGINE_COOLANT_TEMP:
    case INTAKE_AIR_TEMP:
    case AMBIENT_AIR_TEMP:
    case ENGINE_OIL_TEMP:                       return F("°C");
    case THROTTLE_POSITION:
    case RELATIVE_THROTTLE_POSITION:
    case ENGINE_LOAD:
    case ABS_LOAD_VALUE:
    case COMMANDED_EGR:
    case EGR_ERROR:
    case FUEL_TANK_LEVEL_INPUT:
    case ACTUAL_ENGINE_TORQUE:
    case DEMANDED_ENGINE_PERCENT_TORQUE:        return F("%");
    case MAF_FLOW_RATE:                         return F("g/s");
    case FUEL_PRESSURE:
    case INTAKE_MANIFOLD_ABS_PRESSURE:
    case ABSOLUTE_BAROMETRIC_PRESSURE:
    case FUEL_RAIL_PRESSURE:                    return F("kPa");
    case CONTROL_MODULE_VOLTAGE:                return F("V");
    case ENGINE_FUEL_RATE:                      return F("L/h");
    case ENGINE_REFERENCE_TORQUE:               return F("Nm");
    case TIMING_ADVANCE:                        return F("°");
    case RUN_TIME_SINCE_ENGINE_START:
    case TIME_RUN_WITH_MIL_ON:
    case TIME_SINCE_CODES_CLEARED:              return F("s");
    case DISTANCE_TRAVELED_WITH_MIL_ON:
    case DISTANCE_TRAVELED_SINCE_CODES_CLEARED: return F("km");
    default:                                    return F("");
  }
}

// ============================================================
//  Wczytaj numer PID hex od użytkownika (np. "0C")
//  Zwraca 0xFF przy błędzie lub pustym wejściu
// ============================================================
uint8_t readHexPID() {
  Serial.print(F("\nPodaj numer PID (hex, np. 0C): "));
  while (Serial.available() == 0) delay(10);
  delay(50);
  char buf[8];
  int idx = 0;
  while (Serial.available() && idx < 7) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') break;
    buf[idx++] = c;
  }
  buf[idx] = '\0';
  while (Serial.available()) Serial.read();
  if (idx == 0) return 0xFF;
  return (uint8_t)strtoul(buf, nullptr, 16);
}

// ============================================================
//  Wydruk ramki bajtów hex
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
//  Opcja 1 – Auto-detect protocol & test connection
// ============================================================
void connectAuto() {
  Serial.println(F("\n📡 AUTO-DETECT PROTOKOŁU I TEST POŁĄCZENIA"));
  Serial.println(F("==========================================="));
  Serial.println(F("🔁 Próbuję Slow Init (ISO9141 / ISO14230_Slow)..."));
  Serial.println(F("   potem Fast Init (ISO14230_Fast) jeśli brak odpowiedzi."));
  obd.setProtocol("Automatic");
  if (obd.initOBD2()) {
    String proto = obd.getConnectedProtocol();
    currentProtocol = proto;
    Serial.print(F("✅ Połączono! Protokół: "));
    Serial.println(proto);

    Serial.println(F("\n🔍 Odczyt obsługiwanych PID-ów..."));
    uint8_t cnt = obd.readSupportedLiveData();
    Serial.print(F("Znaleziono: ")); Serial.print(cnt); Serial.println(F(" PID-ów obsługiwanych przez ECU:"));
    Serial.println();
    for (int i = 0; i < cnt; i++) {
      byte pid = obd.getSupportedData(0x01, i);
      Serial.print(F("  0x")); PHEX(pid);
      Serial.print(F("  ")); Serial.println(pidName(pid));
    }
    Serial.println(F("\n✅ Test połączenia zakończony."));
  } else {
    Serial.println(F("❌ Nie udało się połączyć z ECU."));
    Serial.println(F("   Sprawdź okablowanie lub wybierz protokół ręcznie (opcja 2 lub 3)."));
  }
}

// ============================================================
//  Opcja 2 – Force ISO9141/ISO14230_Slow protocol
// ============================================================
void connectSlowInit() {
  Serial.println(F("\n📡 POŁĄCZ – FORCE ISO9141 / ISO14230_Slow"));
  Serial.println(F("=========================================="));
  Serial.println(F("🔁 Próbuję Slow Init (5-baud, adres 0x12)..."));
  obd.setProtocol("ISO9141");
  if (obd.initOBD2()) {
    currentProtocol = obd.getConnectedProtocol();
    Serial.print(F("✅ Połączono! Protokół: ")); Serial.println(currentProtocol);

    Serial.println(F("\nOdczyt RPM / Coolant / Speed:"));
    if (initWithRetry()) {
      float rpm  = obd.getLiveData(ENGINE_RPM);
      Serial.print(F("  RPM="));
      if (rpm >= 0) { Serial.print((int)rpm); } else { Serial.print(F("---")); }
    }
    if (initWithRetry()) {
      float temp = obd.getLiveData(ENGINE_COOLANT_TEMP);
      Serial.print(F("  Coolant="));
      if (temp >= 0) { Serial.print((int)temp); Serial.print(F("°C")); } else { Serial.print(F("---")); }
    }
    if (initWithRetry()) {
      float spd  = obd.getLiveData(VEHICLE_SPEED);
      Serial.print(F("  Speed="));
      if (spd >= 0) { Serial.print((int)spd); Serial.print(F(" km/h")); } else { Serial.print(F("---")); }
    }
    Serial.println();
  } else {
    Serial.println(F("❌ Nie udało się połączyć przez Slow Init."));
  }
}

// ============================================================
//  Opcja 3 – Force ISO14230_Fast protocol
// ============================================================
void connectFastInit() {
  Serial.println(F("\n📡 POŁĄCZ – FORCE ISO14230_Fast"));
  Serial.println(F("================================"));
  Serial.println(F("🔁 Próbuję Fast Init (puls 25ms na TX)..."));
  obd.setProtocol("ISO14230_Fast");
  if (obd.initOBD2()) {
    currentProtocol = "ISO14230_Fast";
    Serial.println(F("✅ Połączono! Protokół: ISO14230_Fast"));

    Serial.println(F("\nOdczyt RPM / Coolant / Speed:"));
    if (initWithRetry()) {
      float rpm  = obd.getLiveData(ENGINE_RPM);
      Serial.print(F("  RPM="));
      if (rpm >= 0) { Serial.print((int)rpm); } else { Serial.print(F("---")); }
    }
    if (initWithRetry()) {
      float temp = obd.getLiveData(ENGINE_COOLANT_TEMP);
      Serial.print(F("  Coolant="));
      if (temp >= 0) { Serial.print((int)temp); Serial.print(F("°C")); } else { Serial.print(F("---")); }
    }
    if (initWithRetry()) {
      float spd  = obd.getLiveData(VEHICLE_SPEED);
      Serial.print(F("  Speed="));
      if (spd >= 0) { Serial.print((int)spd); Serial.print(F(" km/h")); } else { Serial.print(F("---")); }
    }
    Serial.println();
  } else {
    Serial.println(F("❌ Nie udało się połączyć przez Fast Init."));
  }
}

// ============================================================
//  Opcja 4 – Skan wszystkich standardowych PID-ów OBD2 (Mode 01, 0x00-0x60)
// ============================================================
void scanSupportedPIDs() {
  Serial.println(F("\n🔍 SKAN OBSŁUGIWANYCH PID-ów (Mode 01)"));
  Serial.println(F("======================================="));

  if (!initWithRetry()) {
    Serial.println(F("❌ Brak połączenia z ECU."));
    return;
  }

  uint8_t cnt = obd.readSupportedLiveData();
  Serial.print(F("Znaleziono: ")); Serial.print(cnt); Serial.println(F(" PID-ów obsługiwanych przez ECU:"));
  Serial.println();

  for (int i = 0; i < cnt; i++) {
    byte pid = obd.getSupportedData(0x01, i);
    Serial.print(F("  0x")); PHEX(pid);
    Serial.print(F("  ")); Serial.println(pidName(pid));
  }

  Serial.println(F("\n✅ Skan zakończony."));
}

// ============================================================
//  Pomocnicza – Odczyt LIVE DATA (pojedynczy PID)
//  Dostępna przez menu opcja 4 (scanSupportedPIDs) lub wywoływana ręcznie
// ============================================================
void readSinglePID() {
  Serial.println(F("\n📊 ODCZYT LIVE DATA – Pojedynczy PID"));
  Serial.println(F("====================================="));

  uint8_t pid = readHexPID();
  if (pid == 0xFF) {
    Serial.println(F("❌ Nieprawidłowy PID."));
    return;
  }

  Serial.print(F("🔍 Odczyt PID 0x")); PHEX(pid);
  Serial.print(F(" (")); Serial.print(pidName(pid)); Serial.println(F(")..."));

  if (!initWithRetry()) {
    Serial.println(F("❌ Brak połączenia z ECU."));
    return;
  }

  float val = obd.getLiveData(pid);

  Serial.print(F("  PID 0x")); PHEX(pid);
  Serial.print(F(" = "));
  if (val >= 0) {
    Serial.print(val, 2);
    String unit = pidUnit(pid);
    if (unit.length() > 0) { Serial.print(F(" ")); Serial.print(unit); }
    Serial.println();
  } else if (val == -1) {
    Serial.println(F("--- (timeout – brak odpowiedzi)"));
  } else if (val == -2) {
    Serial.println(F("--- (zły PID w odpowiedzi ECU)"));
  } else {
    Serial.print(F("--- (nieznany PID, kod: ")); Serial.print((int)val); Serial.println(F(")"));
  }
}

// ============================================================
//  Opcja 5 – LIVE MONITOR (RPM / Temp / Speed / Throttle)
// ============================================================
void liveMonitor() {
  Serial.println(F("\n📺 LIVE MONITOR – RPM / Temp / Speed / Throttle"));
  Serial.println(F("Wyślij dowolny znak aby zatrzymać."));
  Serial.println(F("================================================"));
  Serial.println(F("Start monitoringu...\n"));

  int iteration = 0;

  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("\n⏹ Zatrzymano przez użytkownika."));
      break;
    }

    float rpm  = -1;
    float temp = -1;
    float spd  = -1;
    float thr  = -1;

    if (initWithRetry()) rpm  = obd.getLiveData(ENGINE_RPM);
    if (initWithRetry()) temp = obd.getLiveData(ENGINE_COOLANT_TEMP);
    if (initWithRetry()) spd  = obd.getLiveData(VEHICLE_SPEED);
    if (initWithRetry()) thr  = obd.getLiveData(THROTTLE_POSITION);

    Serial.print(F("#")); Serial.print(iteration);

    Serial.print(F("  RPM="));
    if (rpm >= 0) { Serial.print((int)rpm); } else { Serial.print(F("---")); }

    Serial.print(F("  Temp="));
    if (temp >= 0) { Serial.print((int)temp); Serial.print(F("°C")); } else { Serial.print(F("---")); }

    Serial.print(F("  Speed="));
    if (spd >= 0) { Serial.print((int)spd); Serial.print(F(" km/h")); } else { Serial.print(F("---")); }

    Serial.print(F("  Throttle="));
    if (thr >= 0) { Serial.print(thr, 1); Serial.print(F("%")); } else { Serial.print(F("---")); }

    Serial.println();
    iteration++;
    delay(500);
  }
}

// ============================================================
//  Opcja 6 – Odczyt DTC (kody błędów)
// ============================================================
void readDTCsMenu() {
  Serial.println(F("\n🔬 ODCZYT DTC – Kody błędów"));
  Serial.println(F("============================"));

  // Stored DTCs (Mode 03)
  Serial.println(F("\n-- Stored DTC (Mode 03) --"));
  if (initWithRetry()) {
    uint8_t cnt = obd.readStoredDTCs();
    if (cnt == 0) {
      Serial.println(F("Brak zapisanych błędów. ✅"));
    } else {
      Serial.print(cnt); Serial.println(F(" błąd(ów):"));
      for (int i = 0; i < (int)cnt; i++) {
        Serial.print(F("  ")); Serial.println(obd.getStoredDTC(i));
      }
    }
  } else {
    Serial.println(F("❌ Init nieudany."));
  }

  // Pending DTCs (Mode 07)
  Serial.println(F("\n-- Pending DTC (Mode 07) --"));
  if (initWithRetry()) {
    uint8_t cnt = obd.readPendingDTCs();
    if (cnt == 0) {
      Serial.println(F("Brak oczekujących błędów. ✅"));
    } else {
      Serial.print(cnt); Serial.println(F(" błąd(ów) oczekujących:"));
      for (int i = 0; i < (int)cnt; i++) {
        Serial.print(F("  ")); Serial.println(obd.getPendingDTC(i));
      }
    }
  } else {
    Serial.println(F("❌ Init nieudany."));
  }

  Serial.println(F("\n✅ Odczyt DTC zakończony."));
}

// ============================================================
//  Opcja 7 – Kasowanie DTC
// ============================================================
void clearDTCsMenu() {
  Serial.println(F("\n🗑️ KASOWANIE DTC (Mode 04)"));
  Serial.println(F("==========================="));
  Serial.println(F("⚠️  UWAGA: Skasuje WSZYSTKIE kody błędów i dane historyczne!"));
  Serial.print(F("Wpisz 'Y' aby potwierdzić, lub inny znak aby anulować: "));

  while (Serial.available() == 0) delay(10);
  char c = (char)Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(c);

  if (c != 'Y' && c != 'y') {
    Serial.println(F("Anulowano."));
    return;
  }

  Serial.println(F("Kasowanie błędów..."));
  if (!initWithRetry()) {
    Serial.println(F("❌ Brak połączenia z ECU."));
    return;
  }

  if (obd.clearDTCs()) {
    Serial.println(F("✅ Błędy skasowane pomyślnie."));
  } else {
    Serial.println(F("❌ Nie udało się skasować błędów."));
  }
}

// ============================================================
//  Opcja 8 – Raw KWP2000 frame test
// ============================================================
void rawFrameTest() {
  Serial.println(F("\n🛠️ TEST SUROWEJ RAMKI KWP2000"));
  Serial.println(F("=============================="));
  Serial.println(F("Wpisz bajty hex oddzielone spacjami (bez CS, np. 81 12 F1 03)"));
  Serial.println(F("Naciśnij Enter aby wysłać."));
  Serial.print(F("> "));

  while (Serial.available() == 0) delay(10);
  delay(100);

  char buf[64];
  int idx = 0;
  while (Serial.available() && idx < 63) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') break;
    buf[idx++] = c;
  }
  buf[idx] = '\0';
  while (Serial.available()) Serial.read();

  // Parsowanie bajtów hex
  uint8_t frame[32];
  int frameLen = 0;
  char* token = strtok(buf, " ");
  while (token != nullptr && frameLen < 32) {
    frame[frameLen++] = (uint8_t)strtoul(token, nullptr, 16);
    token = strtok(nullptr, " ");
  }

  if (frameLen == 0) {
    Serial.println(F("❌ Brak danych do wysłania."));
    return;
  }

  Serial.print(F("Wysyłam (bez CS): "));
  printFrame("TX:", frame, frameLen);

  if (!initWithRetry()) {
    Serial.println(F("❌ Brak połączenia z ECU."));
    return;
  }

  obd.writeRawData(frame, frameLen);
  uint8_t len = obd.readData();

  if (len == 0) {
    Serial.println(F("❌ Timeout – brak odpowiedzi."));
    return;
  }

  Serial.print(F("RX (")); Serial.print(len); Serial.print(F(" bajtów):"));
  for (int i = 0; i < len; i++) {
    Serial.print(' ');
    PHEX(obd.resultBuffer[i]);
  }
  Serial.println();
}

// ============================================================
//  Opcja 9 – KWP2000 Service 0x2C (DynamicallyDefineLocalIdentifier)
//  Skan BMW-specyficznych adresów: RPM, Napięcie, Boost, Temp, IAT
// ============================================================
void scan2C() {
  Serial.println(F("\n🔬 SERVICE 0x2C – BMW DynamicallyDefineLocalIdentifier"));
  Serial.println(F("======================================================="));
  Serial.println(F("Format ramki: 84 12 F1 2C 10 HI LO CS"));
  Serial.println(F("Oczekiwana odpowiedź: 0x6C (pozytywna)"));
  Serial.println();

  // BMW-specyficzne adresy pamięci
  struct BMWAddr {
    uint16_t addr;
    const char *name;
  };
  const BMWAddr addrs[] = {
    {0x0091, "RPM (raw/4 = obr/min)"},
    {0x0093, "Napięcie (raw*0.00268 = V)"},
    {0x009E, "Boost/MAP (raw*0.136 = hPa)"},
    {0x0005, "Coolant Temp (raw*0.1-40 = °C)"},
    {0x000B, "IAT (raw*0.1-40 = °C)"},
  };
  const int addrCount = sizeof(addrs) / sizeof(addrs[0]);

  for (int i = 0; i < addrCount; i++) {
    uint8_t hi = (addrs[i].addr >> 8) & 0xFF;
    uint8_t lo = addrs[i].addr & 0xFF;

    Serial.print(F("  [0x")); PHEX(hi); PHEX(lo);
    Serial.print(F("] ")); Serial.print(addrs[i].name);
    Serial.print(F(" → "));

    if (!initWithRetry(2)) {
      Serial.println(F("❌ Init nieudany"));
      continue;
    }

    uint8_t frame[] = {0x84, ECU_ADDR, TESTER_ADDR, 0x2C, 0x10, hi, lo};
    obd.writeRawData(frame, sizeof(frame));
    uint8_t len = obd.readData();

    if (len == 0) {
      Serial.println(F("❌ Timeout"));
      continue;
    }

    // Wydruk surowych bajtów
    Serial.print(F("RAW ["));
    for (int j = 0; j < len; j++) { PHEX(obd.resultBuffer[j]); if (j < len-1) Serial.print(' '); }
    Serial.print(F("]"));

    // Sprawdź pozytywną odpowiedź 0x6C
    bool found6C = false;
    for (int j = 0; j < len; j++) {
      if (obd.resultBuffer[j] == 0x6C) { found6C = true; break; }
    }

    // KWP2000 positive response: header(1) + dest(1) + src(1) + svc(0x6C)(1) + subfn(1) + hiVal(1) + loVal(1) = 7 bytes minimum
    if (found6C && len >= 7) {
      uint16_t raw = ((uint16_t)obd.resultBuffer[5] << 8) | obd.resultBuffer[6];
      Serial.print(F("  raw="));
      Serial.print(raw);

      // Conversion factors from BMW ECU specification (confirmed on EDC16C31 hardware)
      if (addrs[i].addr == 0x0091) {
        Serial.print(F("  RPM=")); Serial.print(raw / 4.0f, 0);         // raw / 4 = rpm
      } else if (addrs[i].addr == 0x0093) {
        Serial.print(F("  V=")); Serial.print(raw * 0.00268f, 2);        // raw * 0.00268 = V
      } else if (addrs[i].addr == 0x009E) {
        Serial.print(F("  hPa=")); Serial.print(raw * 0.136f, 0);        // raw * 0.136 = hPa
      } else if (addrs[i].addr == 0x0005 || addrs[i].addr == 0x000B) {
        Serial.print(F("  °C=")); Serial.print(raw * 0.1f - 40.0f, 1);   // raw * 0.1 - 40 = °C
      }
      Serial.println(F("  ✅"));
    } else if (found6C) {
      Serial.println(F("  ✅ (za mało danych do dekodowania)"));
    } else {
      Serial.println(F("  ❌ brak 0x6C"));
    }

    delay(50);
  }

  Serial.println(F("\n✅ Skan Service 0x2C zakończony."));
}

// ============================================================
//  Opcja A – KWP2000 Service 0x21 (ReadDataByLocalIdentifier)
//  Skan Local ID 0x01-0x20
// ============================================================
void scan21() {
  Serial.println(F("\n🔬 SERVICE 0x21 – ReadDataByLocalIdentifier"));
  Serial.println(F("==========================================="));
  Serial.println(F("Format ramki: 82 12 F1 21 LID CS"));
  Serial.println(F("Oczekiwana odpowiedź: 0x61 (pozytywna)"));
  Serial.println(F("Skan Local ID 0x01-0x20..."));
  Serial.println();

  int found = 0;

  for (uint8_t lid = 0x01; lid <= 0x20; lid++) {
    Serial.print(F("  LID 0x")); PHEX(lid); Serial.print(F(" → "));

    if (!initWithRetry(2)) {
      Serial.println(F("❌ Init nieudany – pomijam"));
      continue;
    }

    uint8_t frame[] = {0x82, ECU_ADDR, TESTER_ADDR, 0x21, lid};
    obd.writeRawData(frame, sizeof(frame));
    uint8_t len = obd.readData();

    if (len == 0) {
      Serial.println(F("timeout"));
      continue;
    }

    // Sprawdź pozytywną odpowiedź 0x61
    bool found61 = false;
    for (int j = 0; j < len; j++) {
      if (obd.resultBuffer[j] == 0x61) { found61 = true; break; }
    }

    if (found61) {
      Serial.print(F("✅ DATA ["));
      for (int j = 0; j < len; j++) {
        PHEX(obd.resultBuffer[j]);
        if (j < len - 1) Serial.print(' ');
      }
      Serial.println(F("]"));
      found++;
    } else {
      Serial.print(F("❌ ["));
      for (int j = 0; j < len; j++) {
        PHEX(obd.resultBuffer[j]);
        if (j < len - 1) Serial.print(' ');
      }
      Serial.println(F("]"));
    }

    delay(50);
  }

  Serial.println(F("\n================================================"));
  Serial.print(F("Znaleziono: ")); Serial.print(found);
  Serial.println(F(" Local ID z odpowiedzią 0x61."));
  Serial.println(F("✅ Skan Service 0x21 zakończony."));
}

// ============================================================
//  Wyświetl menu główne
// ============================================================
void printMenu() {
  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════════════════╗"));
  Serial.println(F("║  PID_Tester – EDC15 K-Line (OBD2_KLine Library)     ║"));
  Serial.println(F("║  Protokoły: ISO9141 / ISO14230_Slow / ISO14230_Fast  ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════╝"));
  Serial.print(F("  Protokół: ")); Serial.println(currentProtocol);
  Serial.println();
  Serial.println(F("┌──────────────────────────────────────────────────────┐"));
  Serial.println(F("│  MENU GŁÓWNE                                         │"));
  Serial.println(F("├──────────────────────────────────────────────────────┤"));
  Serial.println(F("│  1 - AUTO-DETECT protokołu + obsługiwane PID-y       │"));
  Serial.println(F("│  2 - FORCE ISO9141/Slow Init + RPM/Temp/Speed        │"));
  Serial.println(F("│  3 - FORCE ISO14230_Fast Init + RPM/Temp/Speed       │"));
  Serial.println(F("│  4 - SKAN OBSŁUGIWANYCH PID-ów (Mode 01)             │"));
  Serial.println(F("│  5 - LIVE MONITOR (RPM/Temp/Speed/Throttle)          │"));
  Serial.println(F("│  6 - ODCZYT DTC (kody błędów)                        │"));
  Serial.println(F("│  7 - KASOWANIE DTC                                   │"));
  Serial.println(F("│  8 - TEST SUROWEJ RAMKI KWP2000                      │"));
  Serial.println(F("│  9 - SERVICE 0x2C (BMW: RPM/Voltage/Boost/Temp)      │"));
  Serial.println(F("│  A - SERVICE 0x21 (ReadDataByLocalIdentifier)        │"));
  Serial.println(F("└──────────────────────────────────────────────────────┘"));
  Serial.print(F("Wybierz opcję: "));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  obd.setDebug(Serial);

  Serial.println(F("\n╔══════════════════════════════════════════════════════╗"));
  Serial.println(F("║  PID_Tester – EDC15 K-Line (OBD2_KLine Library)     ║"));
  Serial.println(F("║  ECU: 0x12 (BMW)  |  Tester: 0xF1  |  K-Line 10400  ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════╝"));
  Serial.println(F("Serial: 115200 baud  |  K-Line: 10400 baud"));
  Serial.print(F("RX: ")); Serial.print(RX_PIN);
  Serial.print(F("  TX: ")); Serial.println(TX_PIN);
  Serial.println(F("Gotowy. Wybierz opcję z menu."));
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
    case '1': connectAuto();       break;
    case '2': connectSlowInit();   break;
    case '3': connectFastInit();   break;
    case '4': scanSupportedPIDs(); break;
    case '5': liveMonitor();       break;
    case '6': readDTCsMenu();      break;
    case '7': clearDTCsMenu();     break;
    case '8': rawFrameTest();      break;
    case '9': scan2C();            break;
    case 'A':
    case 'a': scan21();            break;
    default:
      Serial.println(F("❓ Nieznana opcja. Wpisz 1-9 lub A."));
      break;
  }

  delay(500);
}
