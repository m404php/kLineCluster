#ifndef OBD2_KLINE_H
#define OBD2_KLINE_H

#include <Arduino.h>
#include "PIDs.h"

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#include <AltSoftSerial.h>
#define SerialType AltSoftSerial
#else
#define SerialType HardwareSerial
#endif

class OBD2_KLine {
 public:
  OBD2_KLine(SerialType &serialStream, uint32_t baudRate, uint8_t rxPin, uint8_t txPin);

  void setDebug(Stream &serial);
  void setSerial(bool enabled);
  bool initOBD2();
  bool trySlowInit();
  bool tryFastInit();
  void writeData(uint8_t mode, uint8_t pid);
  void writeRawData(const uint8_t *dataArray, uint8_t length);
  uint8_t readData();
  void send5baud(uint8_t data);

  float getPID(uint8_t mode, uint8_t pid);
  float getLiveData(uint8_t pid);
  float getFreezeFrame(uint8_t pid);

  uint8_t readDTCs(uint8_t mode);
  uint8_t readStoredDTCs();
  uint8_t readPendingDTCs();
  String getStoredDTC(uint8_t index);
  String getPendingDTC(uint8_t index);

  bool clearDTCs();

  String getVehicleInfo(uint8_t pid);

  uint8_t readSupportedLiveData();
  uint8_t readSupportedFreezeFrame();
  uint8_t readSupportedOxygenSensors();
  uint8_t readSupportedOtherComponents();
  uint8_t readSupportedOnBoardComponents();
  uint8_t readSupportedVehicleInfo();
  uint8_t readSupportedData(uint8_t mode);
  uint8_t getSupportedData(uint8_t mode, uint8_t index);

  void setByteWriteInterval(uint16_t interval);
  void setInterByteTimeout(uint16_t interval);
  void setReadTimeout(uint16_t timeoutMs);
  void setProtocol(const String &protocolName);
  void updateConnectionStatus(bool messageReceived);
  String getConnectedProtocol();

  uint8_t resultBuffer[64] = {0};  // Public: accessible after readData()

 private:
  SerialType *_serial;
  uint32_t _baudRate;
  uint8_t _rxPin;
  uint8_t _txPin;
  Stream *_debugSerial = nullptr;  // Debug serial port
  uint8_t unreceivedDataCount = 0;
  bool connectionStatus = false;

  String selectedProtocol = "Automatic";
  String connectedProtocol = "";
  uint16_t _byteWriteInterval = 5;
  uint16_t _interByteTimeout = 60;
  uint16_t _readTimeout = 1000;
  String storedDTCBuffer[32];
  String pendingDTCBuffer[32];

  uint8_t supportedLiveData[32];
  uint8_t supportedFreezeFrame[32];
  uint8_t supportedOxygenSensor[32];
  uint8_t supportedOtherComponents[32];
  uint8_t supportedControlComponents[32];
  uint8_t supportedVehicleInfo[32];

  String decodeDTC(uint8_t input_byte1, uint8_t input_byte2);
  uint8_t calculateChecksum(const uint8_t *dataArray, uint8_t length);
  bool isInArray(const uint8_t *dataArray, uint8_t length, uint8_t value);
  String convertBytesToHexString(const uint8_t *dataArray, uint8_t length);
  String convertHexToAscii(const uint8_t *dataArray, uint8_t length);
  void clearEcho();
  void debugPrint(const char *msg);
  void debugPrint(const __FlashStringHelper *msg);
  void debugPrintln(const char *msg);
  void debugPrintln(const __FlashStringHelper *msg);
  void debugPrintHex(uint8_t val);    // Hexadecimal output
  void debugPrintHexln(uint8_t val);  // Hexadecimal + newline
};

#endif  // OBD2_KLINE_H