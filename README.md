# RocketLeagueArduinoCode
//Hi, This is the code to operate a mechanum robot to play Rocket league (Soccer) in 1v1
//Unfortunatly the code was still buggy at the time of competiton start so see Used in competiton Code for the used code
//or see the Original Plan file to see the thick incomplete buggy code
// It includes Ardunio control code and R Pi Code for computer vision
//NOTE: code us ugly and does not use #include statements for functions because it would introduce aditional bugs (it'd be nice if the functions were in seperate files but they are not)

List of Functions and their purpose:
12_1_21_SerialCode:
This code recieves coputer vision data from a raspbery pie using open CV to track the distance and angle from the robot to a yellow playing ball of 9 inches in diameter
distance is in 1/3's of an inch (ik weird)

FanSet sets the fan speed of an ESC controled Fan

MPU_6050 opperates and reads out the gyroscope data from an acellerometer 

Radio Recieve code was given to me by my professor. It reads radio data from a RFM69HCW Radio. This data included the position of our robot on the court, the enemy robot on the court and the ball coordinate.

Robot Visual Studio is a combined chunk of code. It is a set of motion to motor control logic. It was used to test as much of the code as possible since the robot was not complete untill 3 days before competition.
It included:
VelotoMotor:
This Function took a 3 component velocity vector x y and rotation and converted it to motor magnitudes to move the robot in the desired direction (since it was equaiped with mechanum wheels it could strafe so x and y could be used)
Get Destination:
Calculated where the robot should go based on the court condition passed to it from getMode

GetMode 
which determined which of about 5 different motions it should make (ex: move to behind the ball looking towards the goal, move to an intermediate position so the enemy would not be between it and the ball so it could get behind the ball to push it towards the goal etc.) It did this using a number of geometry criteria that would check if things were lined up.


SetGoal:
Set the xy coordinates of the goal the robot wanted to push the ball into since depending on the match it could be either one. It read in a switch that would set the goal

SetRobotBallEnemy: The radio read in 3 ball locations (red blue yellow I believe they were) Each would correspond to our robot their robot or the ball. Which was which would be set by a switch by the user at the begining of each match

motorToSpin:
Would convert motor vector values to digital and PWM signals used by our motor controller a T B 6 6 1 2 F N G

pixeltoangle:
Would convert the x location pixel pased to the arduino from the Raspberry pie to angle relative to the robot
