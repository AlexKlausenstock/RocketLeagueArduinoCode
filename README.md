# RocketLeagueArduinoCode
//Hi, This is the code to operate a mechanum robot to play Rocket league (Soccer) in 1v1
//Unfortunatly the code was still buggy at the time of competiton start so see Used in competiton Code for the used code
//or see the Original Plan file to see the thick incomplete buggy code
// It includes Ardunio control code and R Pi Code for computer vision

List of Functions and their purpose:
12_1_21_SerialCode:
This code recieves coputer vision data from a raspbery pie using open CV to track the distance and angle from the robot to a yellow playing ball of 9 inches in diameter
distance is in 1/3's of an inch (ik weird)

FanSet sets the fan speed of an ESC controled Fan

MPU_6050 opperates and reads out the gyroscope data from an acellerometer 

Radio Recieve code was given to me by my professor. It reads radio data from a RFM69HCW Radio. This data included the position of our robot on the court, the enemy robot on the court and the ball coordinate.

Robot Visual Studio is a combined chunk of code. It is a set of motion to motor control logic. It was used to test as much of the code as possible since the robot was not complete untill 3 days before competition.

SetGoal:
Set the xy coordinates of the goal the robot wanted to push the ball into since depending on the match it could be either one. It read in a switch that would set the goal
