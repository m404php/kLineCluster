#define RX_PIN 35
#define TX_PIN 38

#define TARGET_ADDR 0x12
#define TESTER_ADDR 0xF1

bool isConnected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\n==============================================");
  Serial.println(" BMW EDC16 - RENTGEN DANYCH LIVE DATA ");
  Serial.println("==============================================\n");
}

void printHex(uint8_t val) {
  if (val < 0x10) Serial.print("0");
  Serial.print(val, HEX);
}

byte calcChecksum(byte* data, int len) {
  int sum = 0;
  for(int i = 0; i < len; i++) sum += data[i];
  return (byte)(sum & 0xFF);
}

String getFaultDescription(uint16_t errorCode) {
  switch (errorCode) {
    case 0x3F30: return "Czujnik cisnienia szyny";
    case 0x4302: return "Zawor regulacji przeplywu";
    case 0x4332: return "Zawor regulacji cisnienia w plynie";
    case 0x41A2: return "Silownik cisnienia doladowania";
    case 0x3EE0: return "Czujnik temperatury plynu chlodzacego";
    case 0x4000: return "Czujnik temperatury paliwa";
    case 0x4390: return "Czujnik temperatury powietrza doladowujacego";
    case 0x46D3: return "Wewnetrzna jednostka sterujaca 4";
    case 0x4605: return "Systemy wykorzystywane przez firme";
    case 0x3F01: return "Czujnik cisnienia doladowania";
    case 0x3F11: return "Potencjometr modulu pedalu przyspieszenia 1";
    case 0x3F21: return "Potencjometr modulu pedalu przyspieszenia 2";
    case 0x4162: return "Przekaznik pompy zasilajacej";
    case 0x4203: return "Sterownik swiec zarowych";
    default:     return "Nieznany kod bledu";
  }
}

bool connectToECU() {
  Serial1.end();
  Serial1.begin(10400, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial1.updateBaudRate(360);
  Serial1.write(0x00);
  Serial1.flush();
  delay(22);
  Serial1.updateBaudRate(10400);

  while(Serial1.available()) Serial1.read(); 

  byte startComm[] = {0x81, TARGET_ADDR, TESTER_ADDR, 0x81, 0x05};
  Serial1.write(startComm, 5);
  Serial1.flush();

  unsigned long startTime = millis();
  int count = 0;
  byte rxBuf[50];
  
  while(millis() - startTime < 150) {
    if(Serial1.available()) {
      rxBuf[count++] = Serial1.read();
      startTime = millis();
    }
  }

  for (int i = 0; i < count; i++) {
    if (rxBuf[i] == 0xC1) {
      delay(50); 
      return true;
    }
  }
  return false;
}

int sendCommand(byte* req, int reqLen, byte* responseBuf) {
  while(Serial1.available()) Serial1.read(); 

  Serial1.write(req, reqLen);
  Serial1.flush();

  unsigned long startTime = millis();
  int count = 0;
  
  while (millis() - startTime < 200) {
    if (Serial1.available()) {
      responseBuf[count++] = Serial1.read();
      startTime = millis(); 
    }
  }
  return count;
}

void loop() {
  Serial.println("\n----------------------------------------------");
  Serial.println(" WYBIERZ AKCJE (Wpisz cyfre w konsoli):");
  Serial.println(" 1 - Odczyt ID Sterownika");
  Serial.println(" 2 - Odczytaj Bledy (DTCs)");
  Serial.println(" 3 - Skasuj Bledy");
  Serial.println(" 4 - MULTI LIVE DATA (Z RENTGENEM RAMEK)");
  Serial.println("----------------------------------------------");

  while(Serial.available() == 0) { delay(10); }
  char choice = Serial.read();
  while(Serial.available()) Serial.read(); 

  if (choice < '1' || choice > '4') return;

  Serial.println("\n[SYSTEM] Budzenie ECU...");
  if (!connectToECU()) {
    Serial.println("❌ Nie udalo sie wybudzic sterownika.");
    return;
  }

  byte response[256];
  int respLen = 0;

  if (choice == '1') {
      byte reqID[] = {0x82, 0x12, 0xF1, 0x1A, 0x80, 0x00};
      reqID[5] = calcChecksum(reqID, 5);
      respLen = sendCommand(reqID, 6, response);
      if (respLen > 6 && response[6+3] == 0x5A) {
        Serial.print("✅ Wersja Oprogramowania / ID: ");
        for(int i = 6 + 4; i < respLen - 1; i++) {
          byte b = response[i];
          if(b >= 0x20 && b <= 0x7E) Serial.print((char)b);
        }
        Serial.println();
      }
  } 
  else if (choice == '2') {
      byte reqDTC[] = {0x84, 0x12, 0xF1, 0x18, 0x02, 0xFF, 0xFF, 0x00};
      reqDTC[7] = calcChecksum(reqDTC, 7);
      respLen = sendCommand(reqDTC, 8, response);
      if (respLen > 8 && response[8+3] == 0x58) {
        int numDTCs = response[8+4]; 
        Serial.print("✅ Znaleziono bledow: "); Serial.println(numDTCs);
        if(numDTCs > 0) {
          for(int i = 0; i < numDTCs; i++) {
             int idx = (8+5) + (i * 3);
             if(idx + 2 < respLen) {
               uint16_t errorCode = (response[idx] << 8) | response[idx+1];
               Serial.print(" -> Kod: "); 
               printHex(response[idx]); printHex(response[idx+1]); 
               Serial.print(" - ");
               Serial.print(getFaultDescription(errorCode));
               Serial.print(" (Status: "); printHex(response[idx+2]); Serial.println(")");
             }
          }
        }
      }
  } 
  else if (choice == '3') {
      byte clearDTC[] = {0x83, 0x12, 0xF1, 0x14, 0xFF, 0xFF, 0x00};
      clearDTC[6] = calcChecksum(clearDTC, 6);
      respLen = sendCommand(clearDTC, 7, response);
      if (respLen > 7 && response[7+3] == 0x54) Serial.println("✅ PAMIEC BLEDOW SKASOWANA!");
  } 
  else if (choice == '4') {
      Serial.println("\n====== RENTGEN LIVE DATA ======");
      
      // POPRAWIONY ADRES RPM NA 00 91 !!!
      byte reqs[4][8] = {
        {0x84, 0x12, 0xF1, 0x2C, 0x10, 0x00, 0x91, 0x00}, // Zmienione na 00 91 (RPM)
        {0x84, 0x12, 0xF1, 0x2C, 0x10, 0x00, 0x93, 0x00}, // Napięcie
        {0x84, 0x12, 0xF1, 0x2C, 0x10, 0x00, 0x05, 0x00}, // Temperatura
        {0x84, 0x12, 0xF1, 0x2C, 0x10, 0x00, 0x9E, 0x00}  // Ciśnienie Doładowania
      };
      
      float valRPM = 0, valNapiecie = 0, valTemp = 0, valCisnienie = 0;

      for (int loopCount = 0; loopCount < 3; loopCount++) { // Pokaże tylko 3 pętle żeby nie zalać ekranu
        if (Serial.available()) { while(Serial.available()) Serial.read(); break; }

        for(int p = 0; p < 4; p++) {
          reqs[p][7] = calcChecksum(reqs[p], 7);
          respLen = sendCommand(reqs[p], 8, response);
          
          // WYDRUK SUROWYCH DANYCH Z AUTA (Rentgen)
          Serial.print("PID ["); Serial.print(p); Serial.print("] RAW: ");
          for(int i=0; i<respLen; i++) {
            printHex(response[i]); Serial.print(" ");
          }
          Serial.println();

          if (respLen > 8 && response[8+3] == 0x6C) {
             int valRaw = (response[8+5] * 256) + response[8+6];
             
             if(p == 0) valRPM = valRaw / 4.0;
             else if(p == 1) valNapiecie = valRaw * 0.00268;     
             else if(p == 2) valTemp = (valRaw * 0.1) - 40.0;    
             else if(p == 3) valCisnienie = valRaw * 0.136;      
          }
          delay(40); 
        }

        Serial.print("OBLICZONE -> RPM: "); Serial.print(valRPM, 0); 
        Serial.print(" | V: "); Serial.print(valNapiecie, 1); 
        Serial.print(" | C: "); Serial.print(valTemp, 1); 
        Serial.print(" | hPa: "); Serial.println(valCisnienie, 0);
        Serial.println("------------------------------------------------");
        
        delay(500); 
      }
      Serial.println("\nZakonczono Rentgen. Skopiuj mi prosze to co wyplulo.");
  }
  
  delay(1500); 
}