// READ ME!!!
// When Puting code together check each ********** flag
// Go down the list and check each section has code or a "//no code" comment
// Finally, check no conflicting variables
// Assume only code is VS code, MotorToSpin, IMU, LaserRangeFinder, Fan, PieToArduino

//VS Code Variables************************************************

// do not copy the includes or using name space std into arduino

// VeloToMotor
//DestiantionToVelo
//GoTo

long reffe=0;
// VelotoMotor
double velo[3] = {0, 0, 0};     // y x theta forward+ right+ clockwise+
double motor[4] = {0, 0, 0, 0}; // motor pwm signals
double norm = 0;                // for calculating max of motor to make sure max(motor)<=255

double robot[3] = {0, 0, 0}; // robot coordinates x y theta (degrees clockwise) datum at +y axis
double enemy[2] = {0, 0};    // enemy cour position
double ball[2] = {0, 0};     // ball court posoition
double ballVelo[2] = {0, 0}; // ball velocity
int MaxSpeed = 255;          // normalizing value for max speed

double destination[3] = {0, 0, 0};  // assigned destination
double dtranslation[3] = {0, 0, 0}; // difference between robot and destiantion for velo calc

double goal[2] = {0, 0};     // goal location
double goal2[2] = {0, 0};    // Enemy goal location
double BallPath[2] = {0, 0}; // angle 1 then 2 of planned ball path stored in radians because it's not needed in degrees

double ballRadius = 27;                            // ball radius built in buffer
double ballRadiusPush = 18;                        // ball radius minus some
double robotRadius = 30;                           // robot max radius
double radiusBallRobot = ballRadius + robotRadius; // radius to set up by the ball for destination calc
double radiusPush = ballRadiusPush + robotRadius;  // radius to push the ball for destination calc

int errorCode = 0; // error code to be used for durring use debug 1= getAngle broke 2

//SetMode
int mode = 0;                       //Control movement mode
double modeError[3] = {2, 2, 5};    // error of robot
double modeGoToBall[3] = {1, 1, 1}; // error of robot to set up on ball destination

//Check Court Conditions Variables

//Get Destination variables

int fan = 0; // Fan Speed 0 to 255

//Filter Camera Data
double angle = 0; // angle from pixle to angle .ino
int errorfilter = 10;
double distance=0; // Distance from radius from camera

//Set IMU angle
double IMU_Read = 0; // value read from IMU
double IMU_ref = 0; // Reference value used to calibrate IMU based on camera data
double IMU_angle = 0; // calibrated robot[2] angle relative to the court

//SetRobotBallEnemy.cpp vars
int robotColor = 0;
int ballColor = 0;
int enemyColor = 0;
//Get Mode
int refff; // ref for time since last got realistic data from camera

int interval = 10000; // time in millis since last camera aquisition before search for ball

//motorToSpin Variables************************************************

//MotorSpin Variables
//PIN VARIABLES
//the motor direction is controled by AIN or BIN 1 & 2
//Driver 1
// motor 1 front left
const int A1IN1 = 22;           //control pin 1
const int A1IN2 = 23;           //control pin 2
const int PWM1A = 3;            //speed control pin on the motor driver
// motor 2 front right
const int B2IN1 = 24;           //control pin 1
const int B2IN2 = 25;           //control pin 2
const int PWM2B = 4;            //speed control pin on the motor driver

// Driver 2
// motor 3 back right
const int A3IN1 = 26;           //control pin 1
const int A3IN2 = 27;           //control pin 2
const int PWM3A = 5;            //speed control pin on the motor driver
// motor 4 back left
const int B4IN1 = 28;          //control pin 1
const int B4IN2 = 29;          //control pin 2
const int PWM4B = 6;           //speed control pin on the motor driver



//IMU Variables************************************************
//LaserRangeFinder Variables************************************************
//Fan Variables************************************************
//PieToArduino Variables************************************************
String nom = "Arduino";
String msg;
String x_coord;
String rad;
int xcoord=0;
int radius=0;

//need to add a time var to remember when the last time a sample was taken to determine how to take data?
unsigned long lastSample=0;
unsigned long Reference=0;


//Radio Code Variables************************************************

//Pixle to Angle Variables************************************************
//SetGoal Variables************************************************
//SetRobotBallEnemy Variables************************************************

void setup() {
  
  //MotorToSpinSetup*******************************************************************************
  pinMode(A1IN1, OUTPUT);
  pinMode(A1IN2, OUTPUT);
  pinMode(PWM1A, OUTPUT);
  // front right
  pinMode(B2IN1, OUTPUT);
  pinMode(B2IN2, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  // back right
  pinMode(A3IN1, OUTPUT);
  pinMode(A3IN2, OUTPUT);
  pinMode(PWM3A, OUTPUT);
  // back left
  pinMode(B4IN1, OUTPUT);
  pinMode(B4IN2, OUTPUT);
  pinMode(PWM4B, OUTPUT);
  //IMU Setup************************************************
  //LaserRangeFinder Setup************************************************
  //Fan Setup************************************************
  //PieToArduino Setup************************************************
  
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(44,INPUT);

  
  
  //Radio Code Setup************************************************
  

  //Pixle to Angle Setup************************************************
  //SetGoal Setup************************************************
  //SetRobotBallEnemy Setup************************************************
}

void loop() {


  
  
  //
//val = map(val, 0, 1023, 0, 255);


//
if(1000+reffe<millis()){
  
  if(digitalRead(44)==HIGH){
  velo[0]=0;
    velo[1]=0;
    velo[2]=255;
  }
  else{
    velo[0]=0;
    velo[1]=0;
    velo[2]=0;
  }
}
else{
  
  if(digitalRead(44)==HIGH){
  if(xcoord>350){
    velo[0]=255;
    velo[1]=0;
    velo[2]=0;
    MaxSpeed=200;
  }
  else if(xcoord<250){
    velo[0]=-255;
    velo[1]=0;
    velo[2]=-10;
    MaxSpeed=200;
  }
  else if (xcoord < 351 && xcoord >249){
    MaxSpeed=255;
    velo[0]=0;
    velo[1]=255;
    velo[2]=10;
  }
}
else{
  velo[0]=0;
    velo[1]=0;
    velo[2]=0;
}

}



//Move in a direction
  
  //Serial3.println(radius);
  //Do Stuff Loop************************************************
  //velo[1]=255;
  VeloToMotor();
  
  //motorToSpin Loop************************************************
  //MotorToSpinSetupLoop*******************************************************************************
  spinMotor1(motor[0]);
  spinMotor2(motor[1]);
  spinMotor3(motor[2]);
  spinMotor4(motor[3]);
  //IMU Loop************************************************
  //LaserRangeFinder Loop************************************************
  //Fan Loop************************************************
  //PieToArduino Loop************************************************
  
  readSerialPort();

  if (msg != "") {
    sendData();
    Reference = millis();
  }
  //delay(500);
  Serial3.println(xcoord);
  
  //Radio Code Loop************************************************
  
  //Pixle to Angle Loop************************************************
  //SetGoal Loop************************************************
  //SetRobotBallEnemy Loop************************************************
  
}

  //VS Code Functions (IN ORDER!!)************************************************
  //VeloToMoror-----------------------------------------------------------------------------------------------------
void VeloToMotor()
{

    // Normalization stuff for driver code

    // velo [0] (front and rear wheels towards and away from eachother) // +x direction
    // velo[1] (all in 1 direction) // +y dir forward*
    // velo[2] (left weels + right wheels -) // rotation clockwise
    // motor control motors[4] Index: (clockwise order, front left first) 
    //Index diagram:
    //[0]---[1]
    //| robot |
    //[4]___[3]

    //1 get desired direction in x y and rotation and convert to motor values via supper position
    //2 normalize to limit max motor vectors to 255
    //3 Order motors to move

    // eliminate very small motion to creat motion dead zone (because it will normalize to max 255)

    // find motor values
    //front left
    motor[0] = velo[1] + velo[2] + velo[0];
    //front right
    motor[1] = velo[1] - velo[2] - velo[0];
    //back right
    motor[2] = velo[1] + velo[2] - velo[0];
    //back left
    motor[3] = velo[1] - velo[2] + velo[0];
    motor[3]=floor(motor[3]*.9);

    // normalize by the max motor value
    // normm = max element of motor
    norm=0;
    for (int i = 0; i <= 3; i++)
    {
        if (abs(motor[i]) > norm)
        {
            norm = abs(motor[i]);
        }
    }
    // Scale / map largest motor value to be 255 only if norm !=0
    //if norm == 0 then all velo=0 and trying to hold still also dividing by zero would cause an error
    if(norm!=0)
    {
        for (int i = 0; i <= 3; i++)
        {
            motor[i] = motor[i] / norm;
            motor[i] = motor[i] * MaxSpeed;
            //MaxSpeed = 255 which is 100% output for pwm in arduino
            
        }
    }

    // round to integer for pwm value
    for (int i = 0; i <= 3; i++)
    {
        motor[i] = floor(motor[i]);
    }
    // call spinMotor function with these motor functions in arduino
}

  //motorToSpin Function************************************************
  

//MotorToSpinFunctions*******************************************************************************
void spinMotor1(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(A1IN1, HIGH);                         //set pin 1 to high
    digitalWrite(A1IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(A1IN1, LOW);                          //set pin 1 to low
    digitalWrite(A1IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(A1IN1, LOW);                          //set pin 1 to low
    digitalWrite(A1IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM1A, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor2(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(B2IN1, HIGH);                         //set pin 1 to high
    digitalWrite(B2IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(B2IN1, LOW);                          //set pin 1 to low
    digitalWrite(B2IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(B2IN1, LOW);                          //set pin 1 to low
    digitalWrite(B2IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM2B, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor3(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(A3IN1, HIGH);                         //set pin 1 to high
    digitalWrite(A3IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(A3IN1, LOW);                          //set pin 1 to low
    digitalWrite(A3IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(A3IN1, LOW);                          //set pin 1 to low
    digitalWrite(A3IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM3A, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor4(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(B4IN1, HIGH);                         //set pin 1 to high
    digitalWrite(B4IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(B4IN1, LOW);                          //set pin 1 to low
    digitalWrite(B4IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(B4IN1, LOW);                          //set pin 1 to low
    digitalWrite(B4IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM4B, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
  //IMU Function************************************************
  //LaserRangeFinder Function************************************************
  //Fan Function************************************************
  //PieToArduino Function************************************************
  
void readSerialPort() {
  
  if (Serial.available()) {
    reffe=millis();
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
  //Radio Code Function************************************************
  //Pixle to Angle Function************************************************
  //SetGoal Function************************************************
  //SetRobotBallEnemy Function************************************************
