#include <Wire.h>
#include "MPU6050_light_modified.h"
#include <ArduinoBLE.h>
#include "FIFO.h" 
#include "variable.h" 

// BLE setup
BLEService mpuService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Define a BLE service
BLECharacteristic data("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify,12); // Define a BLE characteristic
BLECharacteristic string("19B10002-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLERead, 8);
BLECharacteristic Interval("19B10003-E8F2-537E-4F6C-D104768A1214", BLEWrite | BLERead, 4);

FIFO fifo; 
BLEDevice central;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu1.begin();
  // Initialize arrays
  for (int i = 0; i < windowSize; i++) {
    axValues[i] = 0.0;
    ayValues[i] = 0.0;
    azValues[i] = 0.0;
  }

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  Serial.println("BLE initialized.");
  BLE.setLocalName("r-nrf");
  // Set up BLE service and characteristic
  mpuService.addCharacteristic(data);
  mpuService.addCharacteristic(string);
  mpuService.addCharacteristic(Interval);
  BLE.addService(mpuService);
  BLE.advertise();
  Serial.println("BLE service and characteristic added.");
}

void loop() {
  unsigned long currentMillis = millis(); // Get current time
  central = BLE.central();
  if (currentMillis - previousMillis >= interval) {
    // Time to take a new reading
    previousMillis = currentMillis;

    mpu1.update();

    // Read the current acceleration in all axes
    axValues[sampleIndex] = mpu1.getAccX();
    ayValues[sampleIndex] = mpu1.getAccY();
    azValues[sampleIndex] = mpu1.getAccZ();

    // Calculate rolling mean for each axis
    rollingMeanAx = 0.0;
    rollingMeanAy = 0.0;
    rollingMeanAz = 0.0;
    
    for (int i = 0; i < windowSize; i++) {
      rollingMeanAx += axValues[i];
      rollingMeanAy += ayValues[i];
      rollingMeanAz += azValues[i];
    }
    
    rollingMeanAx /= windowSize;
    rollingMeanAy /= windowSize;
    rollingMeanAz /= windowSize;

    // Calculate rolling variance for each axis
    rollingVarianceAx = 0.0;
    rollingVarianceAy = 0.0;
    rollingVarianceAz = 0.0;
    
    for (int i = 0; i < windowSize; i++) {
      rollingVarianceAx += pow(axValues[i] - rollingMeanAx, 2);
      rollingVarianceAy += pow(ayValues[i] - rollingMeanAy, 2);
      rollingVarianceAz += pow(azValues[i] - rollingMeanAz, 2);
    }
    rollingVarianceAx /= windowSize;
    rollingVarianceAy /= windowSize;
    rollingVarianceAz /= windowSize;

    // Calculate the norm of the rolling variances
    float varianceNorm = sqrt(rollingVarianceAx * rollingVarianceAx +
                              rollingVarianceAy * rollingVarianceAy +
                              rollingVarianceAz * rollingVarianceAz);
    
   

    // Determine output based on thresholds
    outputMean = getOutput(rollingMeanAx, meanAxLowerThreshold, meanAxUpperThreshold, outputMean);
    outputVariance = getOutput(varianceNorm, normVarianceLowerThreshold, normVarianceUpperThreshold, outputVariance);
    

    // Calculate final output
    finalOutput = outputMean * outputVariance;
    sampleIndex = (sampleIndex + 1) % windowSize;

    // Check if the intervalCharacteristic is written
  if (Interval.written()) {
    int length = Interval.valueLength();
    const uint8_t* data = Interval.value();
    int Received_interval = 0;
    memcpy(&Received_interval, data, min(length, sizeof(Received_interval)));
    if (Received_interval > 0) {
      numSeconds = Received_interval;
      Serial.print("New interval set: ");
      Serial.println(interval);
    }
  }
  // Check if the stringCharacteristic is written (timestamp received)
  if (string.written()) {
    int length = string.valueLength();
    const uint8_t* data = string.value();
    got_Timestamp = true;
    startTime = millis(); 
    Serial.println(startTime);
    byte epocData[8];
    memcpy(epocData, data, min(length, sizeof(epocData)));
    memcpy(&value, epocData, sizeof(epocData));
    Serial.print("Received timestamp: ");
    Serial.println(value);
  }
  int Total_interval = numSeconds*numSamplespersec;
    // Store the final output value in the buffer
    if (bufferIndex < Total_interval) {
      finalOutputBuffer[bufferIndex] = finalOutput;
      bufferIndex++;
    }
    // Check if it's time to process and send data via BLE
    if (currentMillis - previousSendMillis >= numSeconds*1000) {
      previousSendMillis = currentMillis;
      
      // Calculate the average final output over the last interval
      long sum = 0;
      for (int i = 0; i < Total_interval; i++) {
        sum += finalOutputBuffer[i];
      }
      final_average = sum / (float)Total_interval;
      // Serial.println(Total_interval);
      // Serial.println(sum);
      // Serial.println(final_average);
      sendData();
      bufferIndex = 0;
    }
  }
}
//Function to send the data
void sendData() {
  if (BLE.central().connected()) {
     while (!fifo.isEmpty()) {
      unsigned long long storedValue;
      float storedProcessData;
      if (fifo.pop(storedValue, storedProcessData)) {  // Pop data from the FIFO queue
        byte combinedData[12];
        combineData((byte*)&storedValue, storedProcessData, combinedData); // Combine stored timestamp and data
        data.writeValue(combinedData, sizeof(combinedData)); // Send the stored data
        Serial.print("Sent stored data with timestamp: ");
        Serial.println(storedValue);
      }
    }
    byte epocData[8];
    unsigned long long sendValue;
    sendValue = value + millis() - startTime;
    memcpy(epocData, &sendValue, sizeof(sendValue)); // Convert timestamp to byte array
    byte combinedData[12];
    combineData(epocData, final_average, combinedData); // Combine timestamp and processed data
    data.writeValue(combinedData, sizeof(combinedData)); // Send data
    Serial.print("Sent data with timestamp: ");
    Serial.println(value);
  } else {
    // Handle disconnection
    if(got_Timestamp){
      unsigned long long storedValue;
      storedValue = value + millis() - startTime;
      Serial.println(startTime);
       Serial.println();
      fifo.push(storedValue, final_average);
    }
  }
}
void combineData(const byte* epocData, float processedData, byte* combinedData) {
  memcpy(combinedData, epocData, 8);  // Copy epoch data
  byte* floatBytes = (byte*)(&processedData);
  for (int i = 0; i < 4; i++) {
    combinedData[8 + i] = floatBytes[i];
  }
}
// Function to determine output based on thresholds
int getOutput(float value, float lowerThreshold, float upperThreshold, int previousOutput) {
  if (value > upperThreshold) {
    return 1;
  } else if (value < lowerThreshold) {
    return 0;
  } else {
    return previousOutput; // Maintain the previous state
  }
}
