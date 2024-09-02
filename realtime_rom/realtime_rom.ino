#include <ArduinoBLE.h>
#include "Wire.h"
#include "variable.h"

#define deviceServiceUuid  "7271f06e-5088-46c9-ab77-4e246b3ea3cb"
#define deviceServiceimudataCharacteristicUuid  "660c4a6f-16d8-4e57-8fdb-a4058934242d"

union IntToBytes {
    uint16_t integer;
    struct {
        uint8_t byte1;
        uint8_t byte2;
    } bytes;
};

void convertIntegersToBytes(const uint16_t* integers, size_t length, uint8_t* bytes){
  for (size_t i = 0; i < length; i++){
    IntToBytes converter;
    converter.integer = integers[i];
    bytes[2 * i] = converter.bytes.byte1;
    bytes[2 * i + 1] = converter.bytes.byte2;
  }
}

BLEService imudataService(deviceServiceUuid);
BLECharacteristic imudata(deviceServiceimudataCharacteristicUuid, BLEWrite | BLERead | BLENotify,24);

void rgbLedRed() {
  digitalWrite(LEDG, HIGH);  // common anode, so high = off
  digitalWrite(LEDB, HIGH);  // common anode, so high = off
  digitalWrite(LEDR, LOW);   // common anode, so high = off
}
void rgbLedoff() {
  digitalWrite(LEDG, HIGH);  // common anode, so high = off
  digitalWrite(LEDB, HIGH);  // common anode, so high = off
  digitalWrite(LEDR, HIGH);   // common anode, so high = off
}

void setup(){
  Wire.begin();
  mpu1.begin(1,0);
  BLE.begin();
  BLE.setAdvertisedService(imudataService);
  imudataService.addCharacteristic(imudata);
  BLE.addService(imudataService);
  BLE.setLocalName("romiumeter");
  BLE.setDeviceName("romiumeter");
  BLE.setConnectionInterval(0x0006, 0x0006);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if ( central.connected() )
  { rgbLedRed();
    mpu1.update();
    unsigned long value =  micros();
    uint16_t integers[] = {mpu1.getrawGyroX(), mpu1.getrawGyroY(),
                            mpu1.getrawGyroZ()};
    size_t numIntegers = sizeof(integers) / sizeof(integers[0]);
    // Array to hold the combined bytes
    byte combinedBytes[sizeof(integers) + sizeof(unsigned long)];
    uint8_t bytes[2*numIntegers];
    convertIntegersToBytes(integers, numIntegers, bytes);
    memcpy(combinedBytes, bytes, sizeof(integers));
    memcpy(combinedBytes + sizeof(integers), &value, sizeof(unsigned long));
    imudata.writeValue(combinedBytes,sizeof(combinedBytes)); 
  }
  else 
  {
    rgbLedoff();
  }
}

