#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;

SoftwareSerial gps_serial(4,3);
SoftwareSerial xbee(10,9);

int packageCounter=0;

void setup() {
  Serial.begin(57600);
  gps_serial.begin(9600);
  xbee.begin(9600);
}

void loop() {
  gps_serial.listen();

  xbee.print(F("?"));
  xbee.print(F("DEVRİM-K2"));
  xbee.print(F(","));
  xbee.print(packageCounter);
  xbee.print(F(","));
  xbee.print(gps.location.lat(), 6);
  xbee.print(F(","));
  xbee.println(gps.location.lng(), 6);
  xbee.print(F("!"));

  packageCounter++;
  smartDelay(1000);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gps_serial.available())
      gps.encode(gps_serial.read());
  } while (millis() - start < ms);
}
