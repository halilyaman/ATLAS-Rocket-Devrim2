#include<LPS.h>
#include<Wire.h>
#include<SoftwareSerial.h>

LPS lps25h;
SoftwareSerial devrim_k1(6, 7);

const int relay = 5;

double altitude;
double pressure;
double relativeAltitude;
const double altitudeToSeaLevel = 0;
double tempAltitude;

bool isSystemOn = false;
bool isProcessDone = false;

void setup() {
  devrim_k1.begin(57600);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  Wire.begin();
  Serial.begin(9600);
  if(!lps25h.init()) {
    while(1);
  } else {
    lps25h.enableDefault();
  }
}

void loop() {
  if(!isProcessDone) {
    pressure = lps25h.readPressureMillibars();
    altitude = lps25h.pressureToAltitudeMeters(pressure);
    relativeAltitude = altitude - altitudeToSeaLevel;
  }
  if(relativeAltitude > 2000 && !isProcessDone) {
    isSystemOn = true;
  }
  while(isSystemOn) {
    calculateAltitudeDifference();
  }
}

void calculateAltitudeDifference() {
  pressure = lps25h.readPressureMillibars();
  altitude = lps25h.pressureToAltitudeMeters(pressure);
  tempAltitude = altitude - altitudeToSeaLevel;
  
  delay(100);
  
  pressure = lps25h.readPressureMillibars();
  altitude = lps25h.pressureToAltitudeMeters(pressure);
  relativeAltitude = altitude - altitudeToSeaLevel;

  if(relativeAltitude < tempAltitude) {
    releaseProcedure();
    isSystemOn = false;
    isProcessDone = true;
  }
}

void releaseProcedure() {
  digitalWrite(relay, LOW);
  delay(2000);
  digitalWrite(relay, HIGH);
}
