
/// define hardware
#define NEOPIXEL_PRESENT true

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#if NEOPIXEL_PRESENT
// Include the Neopixel library
#include <Adafruit_NeoPixel.h>
#endif

/// Declare variables
int32_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // raw data from MPU
uint16_t timeMs = 0; // time since last movement
bool ledStatus = 1; // helper variable for blink function
int32_t initialAcceleration = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#if NEOPIXEL_PRESENT
//NeoPixel
// Declare and initialise globadl GPIO pin constant for Neopixel ring
const byte neoPin = 10;

// Declare and initialise global constant for number of pixels
const byte neoPixels = 24;
int i = 0;

// Declare and initialise variable for Neopixel brightness
byte neoBright = 20;
#endif

/// Declare constants
const int MPU_addr = 0x68; // I2C address of the MPU-6050
const uint8_t ledPin = 13;
const int32_t accelerationThreshold = 6000;
const uint16_t ambientLightThreshold = 1000; // TODO: Set
const uint32_t LEDOffDelayMs = 10000; // time to wait before turnning LED off after last movement

/// Functions
int32_t totalAccelerationRaw();
void blink(uint8_t pinNo);
void updateLedState(uint16_t timeMs);
void updateTimeSinceLastMovement(int32_t accleration);
void dmpDataReady() {
  mpuInterrupt = true;
}
void updateLedRing(float angle);


/// Create instances of classes
// specific I2C addresses may be passed as a parameter here, default is 0x68
MPU6050 mpu;

#if NEOPIXEL_PRESENT
// Create new Neopixel ring object
Adafruit_NeoPixel ring = Adafruit_NeoPixel(neoPixels, neoPin, NEO_GRB);
#endif

void setup() {
  // Init i2c
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // Init serial
  Serial.begin(38400); // 38 400 is max baud for MCU @ 8MHz
  // initialize MPU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(29); //220
  mpu.setYGyroOffset(17); //76
  mpu.setZGyroOffset(23); //-85
  mpu.setZAccelOffset(1041); //1788, 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  // Init LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

#if NEOPIXEL_PRESENT
  // Initialise the ring
  ring.begin();
  ring.setBrightness(neoBright);
  ring.show();
#endif

  // read acceleration offset
  initialAcceleration = totalAccelerationRaw();
}
void loop() {
  int32_t AcTot = totalAccelerationRaw() - initialAcceleration;
  /*
  Serial.print("AcTot = ");
  Serial.print(AcTot);
  Serial.print("A init = ");
  Serial.print(initialAcceleration);
  Serial.print("time = ");
  Serial.println(timeMs);
  */
  updateTimeSinceLastMovement(AcTot);
  updateLedState(timeMs);

  //delay(100);
}

void blink(uint8_t pinNo) {
  if (ledStatus == 1) {
    digitalWrite(pinNo, LOW);
    ledStatus = 0;
  } else {
    digitalWrite(pinNo, HIGH);
    ledStatus = 1;
  }
}

int32_t totalAccelerationRaw() {
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || (fifoCount > (1024 - packetSize))) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
    Serial.println("FIFO overflow!");
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // read DMP data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    // display Euler angles in degrees
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

    // Light LED ring according to yaw angle
    updateLedRing(ypr[0]);
    
/*
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
  */  
  }

  //return sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);
  return abs(aaWorld.x) + abs(aaWorld.y) + abs(aaWorld.z);
}

void updateLedRing(float angle) {
            // Turn off pixels
            ring.setPixelColor(i, ring.Color(0,0,0));
            ring.show();
            
            if (ypr[0] >= 0) {
              i = floor((angle* 180/M_PI) / 15);
            } else {
              i = 24 + floor((angle* 180/M_PI) / 15);
            }
            
            // Turn on pixels
            ring.setPixelColor(i, ring.Color(0,120,190));
            ring.show();

}

void updateLedState(uint16_t timeMs) {
  if (timeMs < LEDOffDelayMs) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

void updateTimeSinceLastMovement(int32_t acceleration) {
  if (abs(acceleration) > accelerationThreshold) {
    timeMs = 0;
  } else {
    timeMs += 10; // estimate of loop time
  }
}
