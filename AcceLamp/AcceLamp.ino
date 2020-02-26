//Basic no DMP branch

#include<Wire.h>

// Declare variables
int32_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // raw data from MPU
uint16_t timeMs = 0; // time since last movement
bool ledStatus = 1; // helper variable for blink
int32_t initialAcceleration = 0;

// Declare constants
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const uint8_t ledPin = 13;
const int32_t accelerationThreshold = 300;
const uint16_t ambientLightThreshold = 1000; // TODO: Set 
const uint32_t LEDOffDelayMs = 10000; // time to wait before turnning LED off after last movement

// Functions
int32_t totalAccelerationRaw();
void blink(uint8_t pinNo);
void updateLedState(uint16_t timeMs);
void updateTimeSinceLastMovement(int32_t accleration);

void setup(){
  // Init i2c
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // Init serial
  Serial.begin(115200);
  // Init LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  // read acceleration offset
  initialAcceleration = totalAccelerationRaw();
}
void loop(){
  int32_t AcTot = totalAccelerationRaw() - initialAcceleration; 
  //Serial.print("AcTot = "); 
  //Serial.println(AcTot);

  updateTimeSinceLastMovement(AcTot);
  updateLedState(timeMs);

  delay(100);  
}

void blink(uint8_t pinNo) {
  if(ledStatus == 1) {
    digitalWrite(pinNo, LOW);
    ledStatus = 0;
  } else {
    digitalWrite(pinNo, HIGH);
    ledStatus = 1;
  }
}

int32_t totalAccelerationRaw() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  //return sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);
  return abs(AcX) + abs(AcY) + abs(AcZ);
}

void updateLedState(uint16_t timeMs) {
  if(timeMs < LEDOffDelayMs) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void updateTimeSinceLastMovement(int32_t acceleration) {
  if (abs(acceleration) > accelerationThreshold) {
    timeMs = 0;
  } else {
    timeMs += 100;
  }
}
