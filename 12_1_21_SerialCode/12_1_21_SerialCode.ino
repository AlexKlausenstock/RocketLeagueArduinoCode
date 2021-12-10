String nom = "Arduino";
String msg;
String x_coord;
String rad;
int xcoord=0;
int radius=0;

//need to add a time var to remember when the last time a sample was taken to determine how to take data?
unsigned long lastSample=0;
unsigned long Reference=0;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  readSerialPort();

  if (msg != "") {
    sendData();
    Reference = millis();
  }
  //delay(500);
  Serial3.println(radius);
}

void readSerialPort() {
  
  if (Serial.available()) {
    msg = "";
    x_coord = "";
    //lastSample is for external use
    lastSample=millis();
    delay(5);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    //Serial.print(msg.substring(0,3));
    if (msg.substring(0,4) == "CAMX") {
      x_coord = msg.substring(5);
    }
    if (msg.substring(0,4) == "CAMR") {
      rad = msg.substring(5);
    }
    Serial.flush();
    //convert data to int type
    xcoord=x_coord.toInt();
    radius=rad.toInt();
    
  }
}

void sendData() {
  //write data
  Serial.print(nom);
  Serial.print(" received : ");
  Serial.print(msg);
  if (x_coord != "") {
    Serial.print(" X-Coordinate: ");
    Serial.print(x_coord);
  }
}
