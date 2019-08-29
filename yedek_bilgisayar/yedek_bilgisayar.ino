#include<LPS.h>
#include<Wire.h>
#include<SoftwareSerial.h>
#include<Servo.h> 

LPS lps25h;

const int signalPin = 5;

double altitude;
double pressure;
double relativeAltitude;
const double altitudeToSeaLevel = 0;
double tempAltitude;

bool isSystemOn = false;
bool isProcessDone = false;
bool isMainParachuteReleased = false;

void setup() {
  Wire.begin();
  Serial.begin(57600);
  pinMode(signalPin, OUTPUT);
  digitalWrite(signalPin, LOW);
  if(lps25h.init()) {
    lps25h.enableDefault();
    delay(1000);
  }
}

void loop() {
  if(isProcessDone) {
    digitalWrite(signalPin, HIGH);

    pressure = lps25h.readPressureMillibars();
    altitude = lps25h.pressureToAltitudeMeters(pressure);
    relativeAltitude = altitude - altitudeToSeaLevel;
    
    if(relativeAltitude <= 600 && !isMainParachuteReleased) {
      releaseMainParachute();
      isMainParachuteReleased = true;
    }
  }
  
  if(!isProcessDone) {
    pressure = lps25h.readPressureMillibars();
    altitude = lps25h.pressureToAltitudeMeters(pressure);
    relativeAltitude = altitude - altitudeToSeaLevel;
    
    if(relativeAltitude > 2000 && !isProcessDone) {
      isSystemOn = true;
    }
  }
  
  while(isSystemOn) {
    calculateAltitudeDifference();
  }
}

void calculateAltitudeDifference() {
  pressure = lps25h.readPressureMillibars();
  altitude = lps25h.pressureToAltitudeMeters(pressure);
  tempAltitude = altitude - altitudeToSeaLevel;
  
  delay(1500);
  
  pressure = lps25h.readPressureMillibars();
  altitude = lps25h.pressureToAltitudeMeters(pressure);
  relativeAltitude = altitude - altitudeToSeaLevel;
  
  if(relativeAltitude - tempAltitude < -1) {
    releaseDragParachute();
    isSystemOn = false;
    isProcessDone = true;
  }
  
}

void releaseDragParachute() {
  
}

void releaseMainParachute() {
  
}
