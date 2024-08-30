
#include "Arduino.h"
#include "Wire.h"
#include "MPU6050_light_modified.h"

MPU6050 mpu1(Wire);


const int windowSize = 50;
float axValues[windowSize] = {0.0};
float ayValues[windowSize] = {0.0};
float azValues[windowSize] = {0.0};
int sampleIndex = 0;

float rollingMeanAx = 0.0;
float rollingMeanAy = 0.0;
float rollingMeanAz = 0.0;

float rollingVarianceAx = 0.0;
float rollingVarianceAy = 0.0;
float rollingVarianceAz = 0.0;

int outputMean = 0;
int outputVariance = 0;
int finalOutput = 0;

unsigned long startTime;

float final_average;
// Thresholds
const float meanAxLowerThreshold = -0.46;  // Example threshold, adjust as needed
const float meanAxUpperThreshold = 0.14;  // Example threshold, adjust as needed
const float normVarianceLowerThreshold = 0.001;  // Example threshold, adjust as needed
const float normVarianceUpperThreshold = 0.0005;   // Example threshold, adjust as needed
bool got_Timestamp = false;// To check Timestamp is received or not
unsigned long previousMillis = 0; // Stores the last time reading was taken
const unsigned long interval = 10; // Interval at which to read (milliseconds)

unsigned long previousSendMillis = 0; // Stores the last time data was sent via BLE
unsigned long long value = 0;

int numSeconds = 5;
const int numSamplespersec = 100;

int finalOutputBuffer[1000] = {0}; // Buffer to store final output values, max for 10 seconds at 10ms intervals
int bufferIndex = 0;








