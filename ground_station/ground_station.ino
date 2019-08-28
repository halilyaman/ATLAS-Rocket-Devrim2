#include <SoftwareSerial.h>

SoftwareSerial xbee(10, 11); //(RX, TX)
char val;
bool isMessage = false;


void setup() {
  Serial.begin(115200);
  xbee.begin(9600);
}

void loop() {
  while(xbee.available()) {
    val = xbee.read();
    if(val == '?') {
      isMessage = true;
    }

    if(val == '!') {
      isMessage = false;
    }

    if(isMessage) {
      if(val != '?') Serial.print(val);
    }
  }
}
