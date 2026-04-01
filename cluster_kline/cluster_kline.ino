// ============================================================================
//  BMW EDC16C31 (M57) – Wyświetlacz zegarów z K-Line
//  ESP32 + K-Line (KWP2000, ISO 14230) @ 10400 baud
//
//  Działanie pinu 21:
//    HIGH (brak masy) → tryb NORMALNY:  GÓRA = ENG Temp, DÓŁ = IAT
//    LOW  (masa)      → tryb K-LINE:    GÓRA = Boost,    DÓŁ = Napięcie
//
//  Potwierdzone adresy BMW EDC16C31 (M57):
//    0x009E – Boost/MAP      → raw16 × 0.136  = hPa  | (hPa – 1013) / 1000 = bar
//    0x0093 – Napięcie akum. → raw16 × 0.00268 = V
//    0x0005 – Temp. płynu    → LO_byte – 40    = °C
//    0x000B – IAT            → LO_byte – 40    = °C
//    0x0091 – RPM            → raw16 / 8       = obr/min
// ============================================================================

// ===========================================================================
// KONFIGURACJA PINÓW
// ===========================================================================
#define RX_PIN          35    // K-Line RX (tylko odczyt)
#define TX_PIN          38    // K-Line TX (wysyłanie)
#define PIN_TRYB        21    // Przełącznik trybu: LOW = masa = tryb K-Line

// ===========================================================================
// PARAMETRY K-LINE / KWP2000
// ===========================================================================
#define TARGET_ADDR     0x12  // Adres ECU silnika BMW EDC16
#define TESTER_ADDR     0xF1  // Adres naszego testera

// ===========================================================================
// ADRESY PAMIĘCI ECU (BMW EDC16C31)
// ===========================================================================
#define ADDR_BOOST_HI   0x00
#define ADDR_BOOST_LO   0x9E  // Ciśnienie doładowania (MAP): raw × 0.136 = hPa

#define ADDR_VOLT_HI    0x00
#define ADDR_VOLT_LO    0x93  // Napięcie akumulatora: raw × 0.00268 = V

#define ADDR_ENG_HI     0x00
#define ADDR_ENG_LO     0x05  // Temperatura płynu chłodzącego: LO_byte – 40 = °C

#define ADDR_IAT_HI     0x00
#define ADDR_IAT_LO     0x0B  // Temperatura powietrza dolotowego: LO_byte – 40 = °C

#define ADDR_RPM_HI     0x00
#define ADDR_RPM_LO     0x91  // Obroty silnika: raw16 / 8 = obr/min

// ===========================================================================
// PARAMETRY PRACY
// ===========================================================================
#define RE_INIT_CO_ODCZYTOW  20    // Re-inicjalizacja ECU co tyle odczytów
#define OPOZNIENIE_MS        500   // Opóźnienie między pełnymi cyklami odczytu [ms]
#define TIMEOUT_ECU_MS       300   // Timeout odpowiedzi ECU [ms]

// ===========================================================================
// ZMIENNE GLOBALNE
// ===========================================================================
static byte   g_rxBuf[64];         // Bufor odpowiedzi K-Line
static int    g_odczytow = 0;       // Licznik odczytów (do re-initu)
static bool   g_ecuGotowe = false;  // Flaga połączenia z ECU

// Ostatnie odczytane wartości
static float  g_boost_hPa = 0.0f;   // Ciśnienie doładowania [hPa]
static float  g_volt = 0.0f;        // Napięcie akumulatora [V]
static float  g_engTemp = 0.0f;     // Temperatura płynu chłodzącego [°C]
static float  g_iat = 0.0f;         // Temperatura powietrza dolotowego [°C]

// ===========================================================================
// FUNKCJE POMOCNICZE K-LINE
// ===========================================================================

// Oblicza sumę kontrolną XOR/arytmetyczną (bajty 0..len-1)
static byte obliczSumKontrolna(byte* dane, int len) {
  int suma = 0;
  for (int i = 0; i < len; i++) suma += dane[i];
  return (byte)(suma & 0xFF);
}

// ===========================================================================
// FAST INIT ECU (KWP2000 ISO 14230-4)
// Wysyła impuls startowy 25ms LOW, potem StartCommunication
// ===========================================================================
bool fastInitECU() {
  // Restart portu szeregowego
  Serial1.end();
  Serial1.begin(10400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Impuls inicjujący: ~25ms @ 360 baud = bajt 0x00
  Serial1.updateBaudRate(360);
  Serial1.write(0x00);
  Serial1.flush();
  delay(22);

  // Przywróć właściwy baud rate
  Serial1.updateBaudRate(10400);

  // Opróżnij bufor RX
  while (Serial1.available()) Serial1.read();

  // StartCommunication: 0x81 TARGET TESTER SID CS
  byte startComm[] = { 0x81, TARGET_ADDR, TESTER_ADDR, 0x81, 0x05 };
  Serial1.write(startComm, sizeof(startComm));
  Serial1.flush();

  // Czekaj na odpowiedź (szukamy 0xC1 = pozytywna odpowiedź ECU)
  unsigned long tStart = millis();
  int ilosc = 0;
  byte bufor[64];

  while (millis() - tStart < 200) {
    if (Serial1.available()) {
      bufor[ilosc++] = Serial1.read();
      tStart = millis(); // Reset timeoutu po każdym bajcie
    }
  }

  for (int i = 0; i < ilosc; i++) {
    if (bufor[i] == 0xC1) {
      delay(50); // Krótka pauza P3
      return true;
    }
  }
  return false;
}

// ===========================================================================
// WYSLIJ ZAPYTANIE I ODBIERZ ODPOWIEDŹ
// ===========================================================================
static int wyslijIOdbierz(byte* req, int reqLen, byte* resp) {
  // Opróżnij bufor RX przed wysłaniem
  while (Serial1.available()) Serial1.read();

  Serial1.write(req, reqLen);
  Serial1.flush();

  unsigned long tStart = millis();
  int ilosc = 0;

  while (millis() - tStart < TIMEOUT_ECU_MS) {
    if (Serial1.available()) {
      resp[ilosc++] = Serial1.read();
      tStart = millis(); // Reset timeoutu po każdym bajcie
      if (ilosc >= 64) break; // Zabezpieczenie bufora
    }
  }
  return ilosc;
}

// ===========================================================================
// ZAPYTANIE SERVICE 0x2C (DynamicallyDefineLocalIdentifier)
// Ramka TX: 84 TARGET TESTER 2C 10 ADDR_HI ADDR_LO CS
// ===========================================================================
static int zapytaj2C(uint8_t addrHi, uint8_t addrLo, byte* resp) {
  byte req[8];
  req[0] = 0x84;
  req[1] = TARGET_ADDR;
  req[2] = TESTER_ADDR;
  req[3] = 0x2C;         // DynamicallyDefineLocalIdentifier
  req[4] = 0x10;         // DefineByMemoryAddress
  req[5] = addrHi;
  req[6] = addrLo;
  req[7] = obliczSumKontrolna(req, 7);
  return wyslijIOdbierz(req, 8, resp);
}

// ===========================================================================
// PARSOWANIE ODPOWIEDZI 0x2C
// Szukamy bajtu 0x6C w odpowiedzi – to pozytywna odpowiedź ECU
// Dane raw16: 2 bajty za 0x6C (offset 2 i 3)
// ===========================================================================
static bool parsujOdpowiedz2C(byte* resp, int len, uint16_t* raw16) {
  for (int i = 0; i < len - 1; i++) {
    if (resp[i] == 0x6C) {
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

// ===========================================================================
// ODCZYT POJEDYNCZEJ WARTOŚCI Z ECU (raw16)
// Zwraca true jeśli odczyt się powiódł
// ===========================================================================
static bool odczytajRaw16(uint8_t addrHi, uint8_t addrLo, uint16_t* raw16) {
  int len = zapytaj2C(addrHi, addrLo, g_rxBuf);
  if (len <= 0) return false;
  return parsujOdpowiedz2C(g_rxBuf, len, raw16);
}

// ===========================================================================
// ODCZYT WSZYSTKICH PARAMETRÓW Z ECU
// Zwraca true jeśli przynajmniej jeden odczyt się powiódł
// ===========================================================================
static bool odczytajParametry(bool trybKLine) {
  uint16_t raw = 0;
  bool sukces = false;

  if (trybKLine) {
    // Tryb K-Line: odczyt Boost i Napięcia
    if (odczytajRaw16(ADDR_BOOST_HI, ADDR_BOOST_LO, &raw)) {
      g_boost_hPa = raw * 0.136f;
      sukces = true;
    }
    delay(40); // Opóźnienie między zapytaniami (timing KWP2000)

    if (odczytajRaw16(ADDR_VOLT_HI, ADDR_VOLT_LO, &raw)) {
      g_volt = raw * 0.00268f;
      sukces = true;
    }
    delay(40);

  } else {
    // Tryb normalny: odczyt Temp. płynu i IAT
    if (odczytajRaw16(ADDR_ENG_HI, ADDR_ENG_LO, &raw)) {
      // Odpowiedź: LO bajt - 40 = temperatura
      uint8_t lo = raw & 0xFF;
      g_engTemp = (float)lo - 40.0f;
      sukces = true;
    }
    delay(40);

    if (odczytajRaw16(ADDR_IAT_HI, ADDR_IAT_LO, &raw)) {
      uint8_t lo = raw & 0xFF;
      g_iat = (float)lo - 40.0f;
      sukces = true;
    }
    delay(40);
  }

  return sukces;
}

// ===========================================================================
// SPRAWDZENIE I EWENTUALNA RE-INICJALIZACJA ECU
// ===========================================================================
static bool sprawdzECU() {
  // Re-inicjalizuj ECU co RE_INIT_CO_ODCZYTOW odczytów lub gdy brak połączenia
  if (!g_ecuGotowe || (g_odczytow >= RE_INIT_CO_ODCZYTOW)) {
    g_odczytow = 0;
    g_ecuGotowe = fastInitECU();
    if (!g_ecuGotowe) {
      Serial.println("[ECU] Brak odpowiedzi ECU – czekam...");
      return false;
    }
    Serial.println("[ECU] Połączenie nawiązane.");
  }
  return true;
}

// ===========================================================================
// ============================================================
// SEKCJA WYŚWIETLACZA – DOSTOSUJ DO SWOJEGO HARDWARE
// ============================================================
// Poniżej: wyświetlanie przez Serial (debugowanie / monitor).
// Aby użyć fizycznego wyświetlacza (np. SSD1306 OLED, TFT itp.):
//   1. Dodaj odpowiednią bibliotekę (np. #include <Adafruit_SSD1306.h>)
//   2. Zastąp implementacje funkcji wyswietlGore() i wyswietlDol()
//      wywołaniami biblioteki wyświetlacza.
// ============================================================

// Wyświetl wartość na górnym wierszu (GÓRA = ENG Temp lub Boost)
static void wyswietlGore(const char* etykieta, const char* wartosc) {
  // --- Miejsce na kod fizycznego wyświetlacza (wiersz górny) ---
  // Przykład dla OLED SSD1306:
  //   display.clearDisplay();
  //   display.setTextSize(2);
  //   display.setCursor(0, 0);
  //   display.print(etykieta);
  //   display.print(wartosc);
  // --- koniec sekcji wyświetlacza ---

  // Debug przez Serial Monitor (można usunąć po podłączeniu wyświetlacza)
  Serial.print("[GORA] ");
  Serial.print(etykieta);
  Serial.print(wartosc);
}

// Wyświetl wartość na dolnym wierszu (DÓŁ = IAT lub Napięcie akumulatora)
static void wyswietlDol(const char* etykieta, const char* wartosc) {
  // --- Miejsce na kod fizycznego wyświetlacza (wiersz dolny) ---
  // Przykład dla OLED SSD1306:
  //   display.setTextSize(2);
  //   display.setCursor(0, 32);
  //   display.print(etykieta);
  //   display.print(wartosc);
  //   display.display();
  // --- koniec sekcji wyświetlacza ---

  // Debug przez Serial Monitor
  Serial.print("  [DOL]  ");
  Serial.print(etykieta);
  Serial.println(wartosc);
}

// Odśwież wyświetlacz po ustawieniu wartości obu wierszy
static void odswiez() {
  // --- Miejsce na display.display() lub odpowiednik ---
  // Przykład dla OLED SSD1306: display.display();
}

// ===========================================================================
// FORMATOWANIE WARTOŚCI DO WYŚWIETLENIA
// ===========================================================================

// Boost w barach względnych: (hPa - 1013) / 1000
// Np. 1092 hPa → +0.08 bar | 1300 hPa → +0.29 bar
static void wyswietlBoost() {
  float bar = (g_boost_hPa - 1013.0f) / 1000.0f;
  char buf[16];
  dtostrf(bar, 5, 2, buf);
  char linia[24];
  snprintf(linia, sizeof(linia), "%s bar", buf);
  wyswietlGore("BOOST: ", linia);
}

// Napięcie z jednym miejscem po przecinku: np. "14.9V"
static void wyswietlNapiecie() {
  char buf[16];
  dtostrf(g_volt, 4, 1, buf);
  char linia[16];
  snprintf(linia, sizeof(linia), "%sV", buf);
  wyswietlDol("VOLT:  ", linia);
}

// Temperatura silnika: np. "92 C"
static void wyswietlEngTemp() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d C", (int)g_engTemp);
  wyswietlGore("ENG:   ", buf);
}

// Temperatura powietrza dolotowego: np. "60 C"
static void wyswietlIAT() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d C", (int)g_iat);
  wyswietlDol("IAT:   ", buf);
}

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  // Debugowanie przez USB
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" BMW EDC16C31 (M57) – K-Line Cluster");
  Serial.println("======================================");
  Serial.println(" Pin 21 LOW (masa) = tryb K-Line");
  Serial.println("   GORA: Boost [bar]");
  Serial.println("   DOL:  Napiecie [V]");
  Serial.println(" Pin 21 HIGH = tryb normalny");
  Serial.println("   GORA: Temp. plynu [C]");
  Serial.println("   DOL:  IAT [C]");
  Serial.println("======================================");

  // Konfiguracja pinu trybu (pull-up, masa = tryb K-Line)
  pinMode(PIN_TRYB, INPUT_PULLUP);

  // Konfiguracja portu K-Line
  Serial1.begin(10400, SERIAL_8N1, RX_PIN, TX_PIN);

  // --- Miejsce na inicjalizację fizycznego wyświetlacza ---
  // Przykład dla OLED SSD1306 (128x64, I2C):
  //   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  //     Serial.println("Blad: brak wyswietlacza SSD1306!");
  //     while (true);
  //   }
  //   display.clearDisplay();
  //   display.setTextColor(SSD1306_WHITE);
  //   display.display();
  // --- koniec sekcji inicjalizacji wyświetlacza ---

  // Pierwsze połączenie z ECU
  Serial.println("[ECU] Inicjalizacja...");
  g_ecuGotowe = fastInitECU();
  if (g_ecuGotowe) {
    Serial.println("[ECU] OK.");
  } else {
    Serial.println("[ECU] Brak odpowiedzi – powtorzymy w petli.");
  }
}

// ===========================================================================
// LOOP
// ===========================================================================
void loop() {
  // Odczytaj stan pinu trybu (LOW = masa = tryb K-Line)
  bool trybKLine = (digitalRead(PIN_TRYB) == LOW);

  // Sprawdź połączenie z ECU (re-init co RE_INIT_CO_ODCZYTOW odczytów)
  if (!sprawdzECU()) {
    delay(1000); // Krótka pauza przed ponowną próbą
    return;
  }

  // Odczytaj parametry z ECU
  bool ok = odczytajParametry(trybKLine);

  if (ok) {
    g_odczytow++;

    if (trybKLine) {
      // ---- TRYB K-LINE: GÓRA = Boost, DÓŁ = Napięcie ----
      wyswietlBoost();
      wyswietlNapiecie();
      odswiez();

    } else {
      // ---- TRYB NORMALNY: GÓRA = ENG Temp, DÓŁ = IAT ----
      wyswietlEngTemp();
      wyswietlIAT();
      odswiez();
    }

  } else {
    // Błąd odczytu – wymuś re-inicjalizację przy następnej iteracji
    Serial.println("[ERR] Blad odczytu K-Line – re-init przy nastepnym cyklu.");
    g_ecuGotowe = false;
  }

  delay(OPOZNIENIE_MS);
}
