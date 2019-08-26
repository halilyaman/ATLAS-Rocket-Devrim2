/*
   For making using of sensors easier, we need to import libraries
   which are LPS, Adafruit_MMA8451 and Adafruit_Sensor.

   Wire library is used for making communication between master
   and slave arduinos with I2C protocol.

   SoftwareSerial library is used for making communication between
   Arduino NANO and Xbee Pro S2C.

   EEPROM library is used because delock time is kept in there. If there is
   sudden power failure, system starts from beginning and takes the delock
   time from EEPROM.

   TinyGPS++ library is used for getting location from GPS module.

   SD and SPI libraries are used to log data to microSD card from sensors.
*/

#include <LPS.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Servo.h>

/*
   Releasing mechanism activator.
 */
Servo servo1;

/*
   For communicating DEVRİM-K2.
*/
#define signalPin A0

RTC_DS1307 rtc;

/*
   Creating LPS25H object which allows us to get altitude info.
*/
LPS lps25h;

/*
   Creating MMA8451 object which allows us to get three axis acceleration.
*/
Adafruit_MMA8451 mma = Adafruit_MMA8451();

/*
   Creating gps object which allows us to get location info.
*/
TinyGPSPlus gps;

/*
   Buzzer is used for getting sound respond about system's status.
*/
const int buzzer = 5;

/*
   microSD is used to log data from sensors.
*/
const int microSD = 2;

/*
   Trigger servo controller.
 */
const int servo1Pin = 6;

/*
   xbee is used to communicate with ground station.
   It sends special packets to ground station with
   high safety protocol.
*/
SoftwareSerial xbee(9, 10); // (RXPin, TXPin)

/*
   gps_serial is necessery for making communication between arduino and
   gps module.
*/
SoftwareSerial gps_serial(3, 4); // (RXPin, TXPin)

/*
    For keeping system safe, all the system will be activated after launcing.
    To provide this, lock boolean value is initialized to true.
    After launching, when the rocket reaches 50m, lock will be false and
    releasing algorithm starts calculating.
*/
bool lock = true;

/*
   For logging, when the lock boolean will be false, we will keep this
   operation time in delock variable. Also this time is used for activating
   release procedure if timeout occurs.
*/
float delock;

/*
   MAX time in milliseconds between delock and apogeeTime.
   If process exceeds this time, timeout will occur and release procedure
   will be activated.
*/
const int timeoutLimit = 30000;

/*
   Until the systems are ready for launching, it provides infinity loop.
   And we hear the warning sound from buzzer.
*/
bool isReadyToLaunch = false;

/*
   After releasing procedure, rocket will start to descend and making
   a calculation for releasing will not be needed. So it will be
   changed to true after releasing happened.
*/
bool isDescending = false;

/*
   Providing safety for unexpected accelerometer errors in a very short
   time after launching due to high velocity, rocket need to
   provide the stage1 and stage2 conditions. After stage2 rocket will
   reach over 2000 meters and systems work fine.
*/
bool stage1 = false;
bool stage2 = false;

/*
   This is the calculation to find the apogee point by using acceleration info.
   Accelerometer measures three axis acceleration. If the square of each axis's
   acceleration is taken and added each other, it gives a stabil positive number
   unless depending on it's orientation but the accelerometer is not moving.
   On the apogee point, it will be zero immediately because it will start
   doing free fall.
*/
double AccXYZ;

/*
   If the rocket reaches to apogee, isApogee will be true.
*/
bool isApogee = false;

/*
   If releasing system will not work on a restricted time interval, isTimeout
   will be true and releasing procedure will start working immediately.
   This is unwanted situation.
*/
bool isTimeout = false;

/*
   If releasing procedure happens, isReleased will be true and this procedure
   will not happen again.
*/
bool isReleased = false;

/*
   When the rocket is about 50 meters, isLanded will be true;
*/
bool isLanded = false;

bool isMainParachuteReleased = false;

/*
   Increases by one for each packet.
*/
unsigned int packageCounter = 0;

/*
   The value which makes the altitude zero.
*/
const double altitudeToSeaLevel = 0;

/*
   (Relative Altitude = altitude measured from sensor - altitudeToSeaLevel)
*/
double relativeAltitude;

/*
   Xbee Pro S2C is configured to work in 9600 baud rate.
*/
static const uint32_t XBEEBaud = 9600;

/*
   GPS is working in same baud rate with Xbee Pro S2C.
*/
static const uint32_t GPSBaud = 9600;

File dataFile;
float pressure, altitude;
int signal;
DateTime now;
sensors_event_t event;

void setup() {
  servo1Setup();
  Wire.begin();
  Serial.begin(57600);
  pinMode(buzzer, OUTPUT);

  while (!isReadyToLaunch) {
    /*
       Activation and setup of LPS25H which calculates the altitude,
       pressure and temperature.
    */
    if (!lps25h.init()) {
      /*
         If the LPS25H can not be activated. Buzzer beeps one seconds.
      */
      longBeep(1000);
    } else {
      lps25h.enableDefault();
      delay(1000);
      /*
         Activation of MMA8451 which measures the acceleration.
      */
      if (!mma.begin()) {
        /*
           If the accelerometer can not be activated. Buzzer beeps two seconds.
        */
        longBeep(2000);
      } else {
        isReadyToLaunch = true;

        xbee.begin(XBEEBaud);
        gps_serial.begin(GPSBaud);
        SD.begin(microSD);
        rtc.begin();
        rtc.isrunning();
        mma.setRange(MMA8451_RANGE_2_G);
        
        /*
           When all the systems are ready for flying,
           buzzer beeps two times.
        */
        beepTwice();
        xbee.println(F("?All systems are ready for launching.\n!"));
      }
    }
  }
  dataFile = SD.open(F("datalog.txt"), FILE_WRITE);
  dataFile.println(F("time,id,packageNumber,latitude,longtitude,altitude,AccXYZ,[status]"));
  dataFile.println(F("------------------------------------------------------------------"));
  dataFile.close();
}

void loop() {
  
  signal = analogRead(signalPin);
  if(signal > 1000) {
    isDescending = true;
  }
  now = rtc.now();
  
  
  /*
     For getting altitude in meters, first we need to get pressure in millibars.
  */
  pressure = lps25h.readPressureMillibars();
  altitude = lps25h.pressureToAltitudeMeters(pressure);

  /*
     altitudeToSeaLevel will be updated according to current location.
     relativeAltitude must be zero on ground.
  */
  relativeAltitude = altitude - altitudeToSeaLevel;
  
  /*
     MMA8451 starts to get value.
  */
  mma.read();

  /*
      For getting three axis acceleration, sensor event must be created.
  */
  mma.getEvent(&event);

  /*
     Taking square of each acceleration on different orientation
     and adding them to each other.
  */
  AccXYZ = (event.acceleration.x * event.acceleration.x) +
           (event.acceleration.y * event.acceleration.y) +
           (event.acceleration.z * event.acceleration.z);

  /*
     Sending data to ground station
  */
  sendData();

  /*
     Data logging
  */
  logData();

  /*
     When the altitude is 200m, lock will be false and delock time is kept.
     If delock was saved in EEPROM before, we take it from there.
     If it is not, then we assign the current millisecond to delock and
     save it to EEPROM.
     This is necessary for safety.
  */
  if (lock) {
    if (relativeAltitude >= 200) {
      lock = false;
      delock = millis();
    }
  } else {
    /*
       If DEVRİM_K2 activates the release system. It sends '1' to DEVRİM_K1
       and DEVRİM_K2 won't activate it again.
     */
    
    /*
       isDescending is initially false. This condition will be active after
       apogee point is reached.
    */
    if (isDescending) {
      /*
        Main parachute releasing mechanism.
       */
      if(!isMainParachuteReleased) {
        if(relativeAltitude <= 600) {
          releaseMainParachute();
          isMainParachuteReleased = true;
        }
      } else {
        /*
          Checking landing
        */
        if (relativeAltitude < 50) {
          longBeep(1000);
          isLanded = true;
        }
      }
    } else {
      /*
         This part of code is active while rocket is going up.
      */

      /*
         Releasing algorithm
      */
      if (relativeAltitude > 1000 && relativeAltitude < 2000) {
        stage1 = true;
      }
      if (relativeAltitude > 2000 && relativeAltitude < 3000) {
        stage2 = true;
      }
      if (stage1 && stage2) {
        if (AccXYZ <= 100 && AccXYZ >= 0) {
          isApogee = true;
        }
      }
      /*
         If this condition is true, releasing procedure works anyway.
      */
      if (millis() - delock > timeoutLimit) {
        if (!isReleased) {
          isTimeout = true;
        }
      }

      /*
         Control point for releasing parachute
      */
      if ((stage1 && stage2 && isApogee) || isTimeout) {
        if (!isReleased) {
          releaseDragParachute();
          isReleased = true;
          isDescending = true;
        } else {
          /*
             For guarantee that isDescending is true.
          */
          isDescending = true;
        }
      }
    }
  }
  smartDelay(100);
}

void servo1Setup() {
  servo1.write(10);
  servo1.attach(servo1Pin);
  delay(1000);
  servo1.detach();
}

void releaseDragParachute() {
  
}

void releaseMainParachute() {
  
}

/*
   These  comma seperated values are sended to the ground station.
   Message starts with '?' character and ends with '!' character.
   Package Content: [team_id, package_number,latitude,
   longtitude, altitude, [status]]

   Time, latitude and longtitude values are taken from GPS module.
*/

void sendData() {
  packageCounter++;
  gps_serial.listen();
  xbee.print(F("?"));
  xbee.print(F("DEVRİM-K1"));
  xbee.print(F(","));
  xbee.print(packageCounter);
  xbee.print(F(","));
  xbee.print(gps.location.lat(), 6);
  xbee.print(F(","));
  xbee.print(gps.location.lng(), 6);
  xbee.print(F(","));
  xbee.print(relativeAltitude);
  xbee.print(F(","));
  xbee.print(F("["));
  if (stage1) {
    xbee.print(F("Stage1"));
  } else {
    xbee.print(F("*"));
  }
  if (stage2) {
    xbee.print(F(","));
    xbee.print(F("Stage2"));
  } else {
    xbee.print(F(","));
    xbee.print(F("*"));
  }
  if (isApogee) {
    xbee.print(F(","));
    xbee.print(F("Apogee"));
  } else {
    xbee.print(F(","));
    xbee.print(F("*"));
  }
  if (isDescending) {
    xbee.print(F(","));
    xbee.print(F("Descending"));
  } else {
    xbee.print(F(","));
    xbee.print(F("*"));
  }
  if (isTimeout) {
    xbee.print(F(","));
    xbee.print(F("Time out"));
  } else {
    xbee.print(F(","));
    xbee.print(F("*"));
  }
  if (isLanded) {
    xbee.print(F(","));
    xbee.print(F("Landed"));
  } else {
    xbee.print(F(","));
    xbee.print(F("*"));
  }
  xbee.print(F("]"));
  xbee.print(F("\n"));
  xbee.print(F("!"));
}

void logData() {
  dataFile = SD.open(F("datalog.txt"), FILE_WRITE);
  dataFile.print(now.hour());
  dataFile.print(F(":"));
  dataFile.print(now.minute());
  dataFile.print(F(":"));
  dataFile.print(now.second());
  dataFile.print(F(","));
  dataFile.print(F("DEVRİM-K1"));
  dataFile.print(F(","));
  dataFile.print(packageCounter);
  dataFile.print(F(","));
  dataFile.print(gps.location.lat(), 6);
  dataFile.print(F(","));
  dataFile.print(gps.location.lng(), 6);
  dataFile.print(F(","));
  dataFile.print(relativeAltitude);
  dataFile.print(F(","));
  dataFile.print(AccXYZ);
  dataFile.print(F(","));
  dataFile.print(F("["));
  if(!stage1 && !isTimeout) {
    dataFile.print(F("Standing"));
  } else {
    dataFile.print(F("*"));
  }
  if (stage1) {
    dataFile.print(F(","));
    dataFile.print(F("Stage1"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  if (stage2) {
    dataFile.print(F(","));
    dataFile.print(F("Stage2"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  if (isApogee) {
    dataFile.print(F(","));
    dataFile.print(F("Apogee"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  if (isDescending) {
    dataFile.print(F(","));
    dataFile.print(F("Descending"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  if (isTimeout) {
    dataFile.print(F(","));
    dataFile.print(F("Timeout"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  if (isLanded) {
    dataFile.print(F(","));
    dataFile.print(F("Landed"));
  } else {
    dataFile.print(F(","));
    dataFile.print(F("*"));
  }
  dataFile.println(F("]"));
  
  dataFile.close();
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

/*
   smartDelay is used because of GPS working.
*/
unsigned long start;
static void smartDelay(unsigned long ms)
{
  start = millis();
  do
  {
    while (gps_serial.available())
      gps.encode(gps_serial.read());
  } while (millis() - start < ms);
}
