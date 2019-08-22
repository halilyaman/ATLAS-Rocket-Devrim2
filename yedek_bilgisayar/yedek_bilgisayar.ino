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
  Wire.begin();
  Serial.begin(57600);
  devrim_k1.begin(4800);
  if(lps25h.init()) {
    lps25h.enableDefault();
    delay(1000);
    pinMode(relay, OUTPUT);
    digitalWrite(relay, HIGH);
  }
}

void loop() {
  if(isProcessDone) {
    devrim_k1.print(1);
  }
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
