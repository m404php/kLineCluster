// ============================================================
//  PID_Tester.ino – Narzędzie diagnostyczne EDC15 K-Line
//  Biblioteka: OBD2_KLine  |  Definicje PID: PIDs.h
//  Protokoły: ISO9141 / ISO14230_Slow / ISO14230_Fast
//  Piny: RX=35, TX=38  |  ESP32
//  ⚡ EDC15: inicjalizacja ECU przed KAŻDYM zapytaniem PID
// ============================================================

#include "OBD2_KLine.h"
#include "PIDs.h"

#define RX_PIN  35
#define TX_PIN  38

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
//  Opcja 1 – Auto-connect (automatyczna detekcja protokołu)
// ============================================================
void connectAuto() {
  Serial.println(F("\n📡 POŁĄCZ Z ECU – Auto-detekcja protokołu"));
  Serial.println(F("=========================================="));
  Serial.println(F("🔁 Próbuję Slow Init (ISO9141 / ISO14230_Slow)..."));
  Serial.println(F("   potem Fast Init (ISO14230_Fast) jeśli brak odpowiedzi."));
  obd.setProtocol("Automatic");
  if (obd.initOBD2()) {
    String proto = obd.getConnectedProtocol();
    currentProtocol = proto;
    Serial.print(F("✅ Połączono! Protokół: "));
    Serial.println(proto);
  } else {
    Serial.println(F("❌ Nie udało się połączyć z ECU."));
    Serial.println(F("   Sprawdź okablowanie lub wybierz protokół ręcznie (opcja 2)."));
  }
}

// ============================================================
//  Opcja 2 – Połącz z wybranym protokołem
// ============================================================
void connectProtocol() {
  Serial.println(F("\n📡 POŁĄCZ Z WYBRANYM PROTOKOŁEM"));
  Serial.println(F("================================"));
  Serial.println(F("  1 - ISO9141       (Slow Init, 5-baud, KW1=KW2)"));
  Serial.println(F("  2 - ISO14230_Slow (Slow Init, 5-baud, KW1≠KW2)"));
  Serial.println(F("  3 - ISO14230_Fast (Fast Init, puls 25ms na TX)"));
  Serial.print(F("Wybierz: "));

  while (Serial.available() == 0) delay(10);
  char c = (char)Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(c);

  String proto = "";
  switch (c) {
    case '1': proto = "ISO9141";       break;
    case '2': proto = "ISO14230_Slow"; break;
    case '3': proto = "ISO14230_Fast"; break;
    default:
      Serial.println(F("❌ Nieznana opcja."));
      return;
  }

  Serial.print(F("📡 Łączenie przez: ")); Serial.println(proto);
  obd.setProtocol(proto);
  if (obd.initOBD2()) {
    currentProtocol = proto;
    Serial.println(F("✅ Połączono!"));
  } else {
    Serial.println(F("❌ Nie udało się połączyć."));
  }
}

// ============================================================
//  Opcja 3 – Skan obsługiwanych PID-ów (Mode 01)
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
//  Opcja 4 – Odczyt LIVE DATA (pojedynczy PID)
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
//  Opcja 8 – Informacje o pojeździe (VIN)
// ============================================================
void vehicleInfoMenu() {
  Serial.println(F("\n🚗 INFORMACJE O POJEŹDZIE (Mode 09)"));
  Serial.println(F("====================================="));

  Serial.println(F("\n-- VIN (0x02) --"));
  String vin = "";
  if (initWithRetry()) vin = obd.getVehicleInfo(read_VIN);
  Serial.print(F("VIN: "));
  if (vin.length() > 0) { Serial.println(vin); } else { Serial.println(F("(brak danych)")); }

  Serial.println(F("\n-- Calibration ID (0x04) --"));
  String calId = "";
  if (initWithRetry()) calId = obd.getVehicleInfo(read_ID);
  Serial.print(F("Cal ID: "));
  if (calId.length() > 0) { Serial.println(calId); } else { Serial.println(F("(brak danych)")); }

  Serial.println(F("\n✅ Odczyt informacji o pojeździe zakończony."));
}

// ============================================================
//  Opcja 9 – Pełny skan wszystkich PID-ów (0x00–0x6F)
// ============================================================
void fullScanPIDs() {
  Serial.println(F("\n🔬 PEŁNY SKAN PID-ów OBD2 (0x00–0x6F, Mode 01)"));
  Serial.println(F("================================================"));
  Serial.println(F("Inicjalizacja ECU przed każdym PIDem (EDC15)..."));
  Serial.println();

  int found = 0;

  for (byte pid = 0x01; pid <= 0x6F; pid++) {
    // Pomiń adresy "Supported PIDs" (zwracają bitmapę, nie wartość)
    if (pid == SUPPORTED_PIDS_1_20  ||
        pid == SUPPORTED_PIDS_21_40 ||
        pid == SUPPORTED_PIDS_41_60 ||
        pid == SUPPORTED_PIDS_61_80) {
      continue;
    }

    if (!initWithRetry(2)) {
      Serial.print(F("  [0x")); PHEX(pid);
      Serial.println(F("] ❌ Init nieudany – pomijam"));
      continue;
    }

    float val = obd.getLiveData(pid);

    Serial.print(F("  [0x")); PHEX(pid); Serial.print(F("]  "));
    Serial.print(pidName(pid));
    Serial.print(F(": "));

    if (val >= 0) {
      Serial.print(val, 2);
      String unit = pidUnit(pid);
      if (unit.length() > 0) { Serial.print(F(" ")); Serial.print(unit); }
      Serial.println(F("  ✅"));
      found++;
    } else if (val == -1) {
      Serial.println(F("--- (timeout)"));
    } else if (val == -2) {
      Serial.println(F("--- (zły PID)"));
    } else {
      Serial.println(F("--- (nieobsługiwany)"));
    }
  }

  Serial.println(F("\n================================================"));
  Serial.print(F("Podsumowanie: znaleziono "));
  Serial.print(found);
  Serial.println(F(" PID-ów z wartościami."));
  Serial.println(F("✅ Skan zakończony."));
}

// ============================================================
//  Opcja 0 – Test surowej ramki KWP2000
// ============================================================
void rawFrameTest() {
  Serial.println(F("\n🛠️ TEST SUROWEJ RAMKI KWP2000"));
  Serial.println(F("=============================="));
  Serial.println(F("Wpisz bajty hex oddzielone spacjami (np. C2 33 F1 01 0C)"));
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
  Serial.println(F("│  1 - POŁĄCZ Z ECU (Auto-detect protokołu)            │"));
  Serial.println(F("│  2 - POŁĄCZ Z WYBRANYM PROTOKOŁEM                    │"));
  Serial.println(F("│  3 - SKAN OBSŁUGIWANYCH PID-ów (Mode 01)             │"));
  Serial.println(F("│  4 - ODCZYT LIVE DATA (pojedynczy PID)               │"));
  Serial.println(F("│  5 - LIVE MONITOR (RPM/Temp/Speed/Throttle)          │"));
  Serial.println(F("│  6 - ODCZYT DTC (kody błędów)                        │"));
  Serial.println(F("│  7 - KASOWANIE DTC                                   │"));
  Serial.println(F("│  8 - INFORMACJE O POJEŹDZIE (VIN)                    │"));
  Serial.println(F("│  9 - PEŁNY SKAN WSZYSTKICH PID-ów (0x00-0x6F)        │"));
  Serial.println(F("│  0 - TEST SUROWEJ RAMKI KWP2000                      │"));
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
  Serial.println(F("║  ⚡ Init ECU przed każdym PIDem (EDC15 mode)         ║"));
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
    case '1': connectAuto();      break;
    case '2': connectProtocol();  break;
    case '3': scanSupportedPIDs(); break;
    case '4': readSinglePID();    break;
    case '5': liveMonitor();      break;
    case '6': readDTCsMenu();     break;
    case '7': clearDTCsMenu();    break;
    case '8': vehicleInfoMenu();  break;
    case '9': fullScanPIDs();     break;
    case '0': rawFrameTest();     break;
    default:
      Serial.println(F("❓ Nieznana opcja. Wpisz 0-9."));
      break;
  }

  delay(500);
}

