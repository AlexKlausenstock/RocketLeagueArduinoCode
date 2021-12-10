#include <Servo.h>
Servo ESC;
int fan;

void setup() {
  Serial.begin(9600);
  fan=0;
  ESC.attach(13,1000,2000);

  while(millis()<3000){
    fan = 255;
    ESC.write(fan);
  }
  while (millis()<6000 && millis()>3000){
    fan = 0;
    ESC.write(fan);
  }
}

void loop() {
  fanSet(fan);
}

void fanSet(int fan){
ESC.write(fan);
}

/*
int readSerialPort() {
  msg = "";
  x_coor = "";
  //debug line
  
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    x_coor = msg.substring(5);
    Serial.flush();

  xcoord= x_coor.toInt();
  return xcoord;
}
return -1;
}
*/
