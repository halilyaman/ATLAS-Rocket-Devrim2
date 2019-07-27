/*
 * For making using of sensors easier, we need to import libraries
 * which are LPS, Adafruit_MMA8451 and Adafruit_Sensor.
 * 
 * Wire library is used for making communication between master 
 * and slave arduinos with I2C protocol.
 * 
 * SoftwareSerial library is used for making communication between
 * Arduino NANO and Xbee Pro S2C.
 * 
 * EEPROM library is used because delock time is kept in there. If there is 
 * sudden power failure, system starts from beginning and takes the delock
 * time from EEPROM.
 */

#include <LPS.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Creating LPS25H object which allows us to get altitude info.
LPS lps25h;

// Creating MMA8451 object which allows us to get three axis acceleration.
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Buzzer is used for getting sound respond about system's status.
int buzzer = 3;

/* 
 * xbee is used to communicate with ground station.
 * It sends special packets to ground station with 
 * high safety protocol.
 */
SoftwareSerial xbee(11, 10);

/* 
 *  For keeping system safe, all the system will be activated after launcing.
 *  To provide this, lock boolean value is initialized to true.
 *  After launching, when the rocket reaches 50m, lock will be false and
 *  releasing algorithm starts calculating.
 */
bool lock = true;

/*
 * For logging, when the lock boolean will be false, we will keep this 
 * operation time in delock variable. Also this time is used for activating
 * release procedure if timeout occurs.
 */
float delock;

/*
 * MAX time between delock and apogeeTime. If process exceeds this time,
 * timeout will occur and release procedure will be activated.
 */
int timeoutLimit = 30;

/*
 * Until the systems are ready for launching, it provides infinity loop.
 * And we hear the warning sound from buzzer.
 */
bool isReadyToLaunch = false;

/*
 * Xbee Pro S2C is configured to work in 9600 baud.
 */
static const uint32_t XBEEBaud = 9600;

void setup() {
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);
  Wire.begin();

  while(!isReadyToLaunch) {
    /*
     * Activation and setup of LPS25H which calculates the altitude,
     * pressure and temperature 
     */
    if(!lps25h.init()) {
      // If the LPS25H can not be activated. Buzzer beeps one seconds.
      longBeep(1000);
    } else {
      lps25h.enableDefault();
      // Activation of MMA8451 which measures the acceleration
      if(!mma.begin()){
        // If the accelerometer can not be activated. Buzzer beeps two seconds.
        longBeep(2000);
      } else {
        isReadyToLaunch = true;
        xbee.begin(XBEEBaud);
        /*
         * When all the systems are ready for flying,
         * buzzer beeps two times.
         */
        beepTwice();
        xbee.println("?All systems are ready for launching.\n!");
      }
    }
  }
}

void loop() {
  /*
   * For getting altitude in meters, first we need to get pressure in millibars.
   */
  float pressure = lps25h.readPressureMillibars();
  float altitude = lps25h.pressureToAltitudeMeters(pressure);

  /*
   * When the altitude is 50m, lock will be false and delock time is kept.
   * If delock was saved in EEPROM before, we take it from there.
   * If it is not, then we assign the current millisecond to delock and
   * save the delock to EEPROM.
   * This is necessary for safety.
   */
  if(lock) {
    if(altitude >= 50) {
      lock = false;
      EEPROM.get(0, delock);
      if(delock == 0) {
        delock = millis();
        EEPROM.put(0, delock);
      }
    }
  } else {
    
  }
  delay(100);
}

void beepTwice() {
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
  delay(150);
  digitalWrite(buzzer, HIGH);
  delay(100);
  digitalWrite(buzzer, LOW);
}

void longBeep(int millisecond) {
  digitalWrite(buzzer, HIGH);
  delay(millisecond);
  digitalWrite(buzzer, LOW);
  delay(500);
}

void clearEEPROM() {
  for(int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, '\0');
  }
}
