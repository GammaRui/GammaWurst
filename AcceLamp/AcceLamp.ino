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

#if NEOPIXEL_PRESENT
  // Include the Neopixel library
  #include <Adafruit_NeoPixel.h>
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

/// Declare variables
//int32_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // raw data from MPU
uint16_t motionTimeMs = 0; // time since last movement
int32_t initialAcceleration = 0;
int32_t AcTot = 0;
unsigned long previousYawMillis = 0; 
float previousYaw = 0;
unsigned long previousBlink = 0;
unsigned long previousBlinkInterval = 0;
bool ledStatus = false;
bool blinkLeft = false;
bool blinkRight = false;
int blinkCount = 0;

/// Declare constants

const int32_t accelerationThreshold = 6000;
//const uint16_t ambientLightThreshold = 1000; // TODO: Set
const uint32_t LEDOffDelayMs = 10000;   // time to wait before turnning LED off after last movement
const long intervalYaw = 500;           // interval at which to check yaw change (milliseconds)
const float thresholdYaw = 0.4;         // Turn angle in Rad, 1 Rad approx. 60 deg
const long blinkOffDelay = 5000;        // Duration of turn indication
const long intervalBlink = 100;         // Blink frequency

/// Functions
int32_t readDMPdata();
void blink(uint8_t pinNo);
void updateLedState();
void updateTimeSinceLastMovement(int32_t accleration);
void updateLedRing(float angle);
void turnOffBlinkLeft();
void turnOffBlinkRight();

#if NEOPIXEL_PRESENT
  //NeoPixel declarations
  const byte neoPinRing = 6; // Declare and initialise globadl GPIO pin constant for Neopixel ring
  const byte neoPinLeft = 10; // Declare and initialise globadl GPIO pin constant for NeoStrip left
  const byte neoPinRight = 9; // Declare and initialise globadl GPIO pin constant for NeoStrip Right
  
  const byte neoPixelsRing = 24; // Declare and initialise global constant for number of pixels in Ring
  const byte neoPixelsStrip = 8; // Declare and initialise global constant for number of pixels in blink strips
  
  int neoCurrentLed = 0; // Counter used for neoRing
  byte neoBright = 40; // Declare and initialise variable for Neopixel brightness
  
  // Create new Neopixel ring object
  Adafruit_NeoPixel ring = Adafruit_NeoPixel(neoPixelsRing, neoPinRing, NEO_GRB);
  Adafruit_NeoPixel stripLeft = Adafruit_NeoPixel(neoPixelsStrip, neoPinLeft, NEO_GRB);
  Adafruit_NeoPixel stripRight = Adafruit_NeoPixel(neoPixelsStrip, neoPinRight, NEO_GRB);
#endif



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

  //Detect turning
  unsigned long currentMillis = millis();
  if (currentMillis - previousYawMillis >= intervalYaw) {

    //Serial.print("Delta rotation / 100 ms:");
    //Serial.println(ypr[0]-previousYaw);
    if (ypr[0]-previousYaw <= (thresholdYaw*-1)) {
      //Left turn detected
      blinkLeft = true;
      blinkRight = false;
      turnOffBlinkRight();
      Serial.println("Turning left"); 
    }else if (ypr[0]-previousYaw >= thresholdYaw) {
       //Right turn detected
       blinkRight = true;
       blinkLeft = false;
       turnOffBlinkLeft();
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

  return 
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
      if (blinkCount >= neoPixelsStrip) {
        stripLeft.setPixelColor(blinkCount-1, stripLeft.Color(0,0,0));
        stripLeft.show();  
        blinkCount = 0;
      } 
    }     
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
  }
  

  //Maximum blink timer reached
  if (millis() - previousBlink > blinkOffDelay) {
    //Turn off blinks
    turnOffBlinkLeft();
    turnOffBlinkRight();

    //reset flags and timer
    previousBlink = millis();
    blinkLeft = false;
    blinkRight = false;
    blinkCount = 0;
  }
}

void turnOffBlinkLeft() {
    for (int i = 0; i <= neoPixelsStrip; i++) {
      stripLeft.setPixelColor(i, stripLeft.Color(0,0,0));
      stripLeft.show();
    } 
}

void turnOffBlinkRight() {
    for (int i = 0; i <= neoPixelsStrip; i++) {
      stripRight.setPixelColor(i, stripRight.Color(0,0,0));
      stripRight.show(); 
    }
}

void updateTimeSinceLastMovement(int32_t acceleration) {
  if (abs(acceleration) > accelerationThreshold) {
    motionTimeMs = 0;
  } else {
    motionTimeMs += 10; // estimate of loop time
  }
}
