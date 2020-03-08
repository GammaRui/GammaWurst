/*==========================================================
 * 
 *                      STROLLER PIMP
 * 
 * =========================================================
 * Adafruit Feather 328P with MPU6050
 * 
 * GY-521  Adafruit
 * MPU6050 Feather
 *         328P          Description
 * ======= ==========    ===================================
 * VCC     3V            3.3V seems to work, otherwise connect to BAT
 * GND     GND           Ground
 * SCL     SCL           I2C clock
 * SDA     SDA           I2C data
 * XDA     not connected
 * XCL     not connected
 * AD0     not connected
 * INT     GPIO2         Interrupt pin
 * =========================================================
 */

// define hardware
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

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool ledStatus = false;

/// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/// Declare variables
//int32_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // raw data from MPU
uint16_t motionTimeMs = 0; // time since last movement
int32_t initialAcceleration = 0;
int32_t AcTot = 0;
unsigned long previousYawMillis = 0; 
const long intervalYaw = 500;           // interval at which to check yaw change (milliseconds)
float previousYaw = 0;
const float thresholdYaw = 0.5;
unsigned long previousBlink = 0;
const long blinkOffDelay = 5000;
unsigned long previousBlinkInterval = 0;
const long intervalBlink = 100;
bool blinkLeft = false;
bool blinkRight = false;
int blinkCount = 0;

/// Declare constants
const int MPU_addr = 0x68; // I2C address of the MPU-6050
const int32_t accelerationThreshold = 6000;
const uint16_t ambientLightThreshold = 1000; // TODO: Set
const uint32_t LEDOffDelayMs = 10000; // time to wait before turnning LED off after last movement

/// Functions
int32_t readDMPdata();
void blink(uint8_t pinNo);
void updateLedState();
void updateTimeSinceLastMovement(int32_t accleration);
void updateLedRing(float angle);

/// Create instances of classes
// specific I2C addresses may be passed as a parameter here, default is 0x68
MPU6050 mpu;

#if NEOPIXEL_PRESENT
  //NeoPixel declarations
  const byte neoPinRing = 6; // Declare and initialise globadl GPIO pin constant for Neopixel ring
  const byte neoPinLeft = 10; // Declare and initialise globadl GPIO pin constant for NeoStrip left
  const byte neoPinRight = 9; // Declare and initialise globadl GPIO pin constant for NeoStrip Right
  const byte neoPixelsRing = 24; // Declare and initialise global constant for number of pixels in Ring
  const byte neoPixelsLeft = 8; // Declare and initialise global constant for number of pixels in left strip
  const byte neoPixelsRight = 8; // Declare and initialise global constant for number of pixels in rigth strip
  int neoCurrentLed = 0;
  byte neoBright = 20; // Declare and initialise variable for Neopixel brightness
  // Create new Neopixel ring object
  Adafruit_NeoPixel ring = Adafruit_NeoPixel(neoPixelsRing, neoPinRing, NEO_GRB);
  Adafruit_NeoPixel stripLeft = Adafruit_NeoPixel(neoPixelsLeft, neoPinLeft, NEO_GRB);
  Adafruit_NeoPixel stripRight = Adafruit_NeoPixel(neoPixelsRight, neoPinRight, NEO_GRB);
#endif

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  // Init serial
  Serial.begin(38400); // 38 400 is max baud for MCU @ 8MHz
  // initialize MPU
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
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
  mpu.setXGyroOffset(2); //29
  mpu.setYGyroOffset(7); //17
  mpu.setZGyroOffset(-13); //23
  mpu.setZAccelOffset(468); //1041

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  #if NEOPIXEL_PRESENT
    // Initialise the ring
    ring.begin();
    ring.setBrightness(neoBright);
    ring.show();
    // Initialise the left strip
    stripLeft.begin();
    stripLeft.setBrightness(neoBright);
    stripLeft.show();
    // Initialise the right strip
    stripRight.begin();
    stripRight.setBrightness(neoBright);
    stripRight.show();
  #endif

}
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    AcTot = readDMPdata();
  }

  //Detect turning
  unsigned long currentMillis = millis();
  if (currentMillis - previousYawMillis >= intervalYaw) {

    //Serial.print("Delta rotation / 100 ms:");
    //Serial.println(ypr[0]-previousYaw);
    if (ypr[0]-previousYaw <= (thresholdYaw*-1)) {
      //Left turn detected
      blinkLeft = true;
      Serial.println("Turning left"); 
    }else if (ypr[0]-previousYaw >= thresholdYaw) {
       //Right turn detected
       blinkRight = true;
      Serial.println("Turning right"); 
    }
    
    previousYawMillis = currentMillis;
    previousYaw = ypr[0];

  }
  /*
  Serial.print("AcTot = ");
  Serial.println(AcTot);
  /*
  Serial.print("A init = ");
  Serial.print(initialAcceleration);
  Serial.print("time = ");
  Serial.println(motionTimeMs);
  */
  updateTimeSinceLastMovement(AcTot);
  updateLedState();

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

int32_t readDMPdata() {
  // read DMP data
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  // display Euler angles in degrees
/*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
*/
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

  return abs(aaWorld.x) + abs(aaWorld.y) + abs(aaWorld.z);
}

void updateLedRing(float angle) {
            // Turn off pixels
            ring.setPixelColor(neoCurrentLed, ring.Color(0,0,0));
            ring.show();
            
            if (ypr[0] >= 0) {
              neoCurrentLed = floor((angle* 180/M_PI) / (360/neoPixelsRing));
            } else {
              neoCurrentLed = 24 + floor((angle* 180/M_PI) / (360/neoPixelsRing));
            }
            
            // Turn on pixels
            ring.setPixelColor(neoCurrentLed, ring.Color(0,120,190));
            ring.show();

}

void updateLedState() {
  if (motionTimeMs < LEDOffDelayMs) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Activate blink strips
  if(blinkLeft == true) {
    //Update strip
    stripLeft.setPixelColor(blinkCount, stripLeft.Color(255,255,0));
    stripLeft.setPixelColor(blinkCount+1, stripLeft.Color(255,255,0));
    stripLeft.setPixelColor(blinkCount-1, stripLeft.Color(0,0,0));
    stripLeft.show();

    //Blink interval and next pixel counter
    if (millis() - previousBlinkInterval >= intervalBlink) {
      blinkCount += 1;
      previousBlinkInterval = millis();
      if (blinkCount > 7) {
        stripLeft.setPixelColor(blinkCount-1, stripLeft.Color(0,0,0));
        stripLeft.show();  
        blinkCount = 0;
      } 
    }
    blinkRight = false;      
  } 
  if (blinkRight == true) {
    //Update strip
    stripRight.setPixelColor(blinkCount, stripRight.Color(255,255,0));
    stripRight.setPixelColor(blinkCount+1, stripRight.Color(255,255,0));
    stripRight.setPixelColor(blinkCount-1, stripRight.Color(0,0,0));
    stripRight.show();

    //Blink interval and next pixel counter
    if (millis() - previousBlinkInterval >= intervalBlink) {
      blinkCount += 1;
      previousBlinkInterval = millis();
      if (blinkCount > 7) {
        stripRight.setPixelColor(blinkCount-1, stripRight.Color(0,0,0));
        stripRight.show();  
        blinkCount = 0;
      } 
    }
    blinkLeft = false;
  }

  //Turn off blink strips
  if (millis() - previousBlink > blinkOffDelay) {
    for (int i = 0; i <= 8; i++) {
      stripRight.setPixelColor(i, stripRight.Color(0,0,0));
      stripRight.show(); 
      stripLeft.setPixelColor(i, stripLeft.Color(0,0,0));
      stripLeft.show();
    } 

    //reset flags and timer
    previousBlink = millis();
    blinkLeft = false;
    blinkRight = false;
    blinkCount = 0;
  }
}

void updateTimeSinceLastMovement(int32_t acceleration) {
  if (abs(acceleration) > accelerationThreshold) {
    motionTimeMs = 0;
  } else {
    motionTimeMs += 10; // estimate of loop time
  }
}
