#include <MPU6050.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include "FIFO.h"  // Include the FIFO library

BLEService customService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLENotify, 12);
BLECharacteristic stringCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLERead, 8);
BLECharacteristic intervalCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLERead, 4);
MPU6050 mpu;
FIFO fifo;                                // Create an instance of the FIFO class
const int numSample = 100;                // Number of samples per second
const unsigned long sampleInterval = 10;  // 10 ms interval
const float ThresholdVMeanH = 0.14;
const float ThresholdVMeanL = -0.46;
const float ThresholdVvarH = 0.001;
const float ThresholdVvarL = 0.0005;
unsigned long nextIntervalTime = 0;
int count = 0;
int mcount = 0;
int Interval = 10;  // Interval in seconds to send data to mobile app
unsigned long startTime = 0;  // Time when the current timestamp processing started
unsigned long lastSendTime = 0;  // Time when the data was last sent
bool processing = false;  // Flag to indicate if we are currently processing data
float axs = 0.0, ays = 0.0, azs = 0.0;
float varx = 0.0, vary = 0.0, varz = 0.0;
float fsum = 0.0;
float previous = 0;
float meanx, meany, meanz;
float vx, vy, vz;
float fmv;
unsigned long long value = 0;
unsigned long nextSampleTime = 0;
int16_t ax, ay, az;
BLEDevice central;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setRate(7);
  BLE.begin();
  BLE.setLocalName("r-nrf");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(dataCharacteristic);
  customService.addCharacteristic(stringCharacteristic);
  customService.addCharacteristic(intervalCharacteristic);
  BLE.addService(customService);
  BLE.advertise();
  nextSampleTime = millis();
  nextIntervalTime = millis() + (Interval * 1000);
}

void loop() {
  central = BLE.central();

  if (millis() >= nextSampleTime) {
    nextSampleTime += sampleInterval;
    // Read accelerometer data
    mpu.getAcceleration(&ax, &ay, &az);
    float asx = ax / 16384.0;
    float asy = ay / 16384.0;
    float asz = az / 16384.0;
    // Update running totals and variance
    varx += pow(asx, 2.0);
    vary += pow(asy, 2.0);
    varz += pow(asz, 2.0);
    axs += asx;
    ays += asy;
    azs += asz;
    count++;
    if (count == numSample) {
      calvalue();
      count = 0;
    }
  }
  
  // Check if the intervalCharacteristic is written
  if (intervalCharacteristic.written()) {
    int length = intervalCharacteristic.valueLength();
    const uint8_t* data = intervalCharacteristic.value();
    int Received_interval = 0;
    memcpy(&Received_interval, data, min(length, sizeof(Received_interval)));
    if (Received_interval > 0) {
      Interval = Received_interval;
      nextIntervalTime += Interval * 1000;
      Serial.print("New interval set: ");
      Serial.println(Interval);
      byte ackData[] = {0x01};  // Example acknowledgment
      intervalCharacteristic.writeValue(ackData, sizeof(ackData));
    }
  }
  // Check if the stringCharacteristic is written (timestamp received)
  if (stringCharacteristic.written()) {
    int length = stringCharacteristic.valueLength();
    const uint8_t* data = stringCharacteristic.value();
    startTime = millis();  // Start processing from now
    byte epocData[8];
    memcpy(epocData, data, min(length, sizeof(epocData)));
    memcpy(&value, epocData, sizeof(epocData));
    processing = true;    // Set flag to indicate processing phase
    lastSendTime = millis();  // Reset the last send time
    Serial.print("Received timestamp: ");
    Serial.println(value);
  }
  // Process and send data if in processing phase and 10 seconds have elapsed
   if (millis() >= nextIntervalTime) {
    nextIntervalTime += Interval * 1000;
    processing = false;
    sendData();
  }
  // if (mcount == Interval) { // 10 seconds
  //   sendData();
  //   processing = false;  // Reset processing flag
  // }
  // Handle data if disconnected
  if (!BLE.central().connected()) {
    if (processing) {
      // Store epoch value and processed data in FIFO
      unsigned long long epochValue = value + Interval * 1000;
      value = epochValue;
      float processData = fsum / Interval;
      fifo.push(epochValue, processData);
      Serial.println(value);
    }
  }
}

void calvalue() {
  mcount++;
  meanx = axs / numSample;
  meany = ays / numSample;
  meanz = azs / numSample;
  vx = (varx / numSample) - pow(meanx, 2.0);
  vy = (vary / numSample) - pow(meany, 2.0);
  vz = (varz / numSample) - pow(meanz, 2.0);
  fmv = (vx + vy + vz) / 3.0;
  float sum = previous;
  if (meanx > ThresholdVMeanH) {
    sum = 1;
    previous = 1;
  } else if (meanx < ThresholdVMeanL) {
    sum = 0;
    previous = 0;
  }
  if (fmv > ThresholdVvarH) {
    sum *= 1;
  } else if (fmv < ThresholdVvarL) {
    sum *= 0;
  }
  fsum += sum;
  axs = 0.0;
  ays = 0.0;
  azs = 0.0;
  varx = 0.0;
  vary = 0.0;
  varz = 0.0;
}
void sendData() {
  if (BLE.central().connected()) {
     while (!fifo.isEmpty()) {
      unsigned long long storedValue;
      float storedProcessData;
      if (fifo.pop(storedValue, storedProcessData)) {  // Pop data from the FIFO queue
        byte combinedData[12];
        combineData((byte*)&storedValue, storedProcessData, combinedData); // Combine stored timestamp and data
        dataCharacteristic.writeValue(combinedData, sizeof(combinedData)); // Send the stored data
        Serial.print("Sent stored data with timestamp: ");
        Serial.println(storedValue);
      }
    }
    byte epocData[8];
    Serial.println(mcount);
    unsigned long interval = millis()-startTime;
    value = value+interval;
    memcpy(epocData, &value, sizeof(value)); // Convert timestamp to byte array
    float processData = fsum / Interval; // Processed data
    byte combinedData[12];
    combineData(epocData, processData, combinedData); // Combine timestamp and processed data
    dataCharacteristic.writeValue(combinedData, sizeof(combinedData)); // Send data
    Serial.print("Sent data with timestamp: ");
    Serial.println(value);
  } else {
    // Handle disconnection
    if (processing) {
      unsigned long long epochValue = value;
      value = epochValue;
      float processData = fsum / Interval;
      fifo.push(epochValue, processData);
      Serial.println(value);
    }
  }
  fsum = 0.0;
  mcount = 0;
}

void combineData(const byte* epocData, float processedData, byte* combinedData) {
  memcpy(combinedData, epocData, 8);  // Copy epoch data
  byte* floatBytes = (byte*)(&processedData);
  for (int i = 0; i < 4; i++) {
    combinedData[8 + i] = floatBytes[i];
  }
}
