// Include VS Code
// So goal top = 275 , 12.5
// Goal bottom = 270, 360
// set goals for set up
// if input 0 set goal to low y value ("top")
double goal[2] = {0, 0};     // goal location
double goal2[2] = {0, 0};    // Enemy goal location


void setup() {
  pinMode(7,INPUT);
}

void loop() {
  if (digitalRead(7)==HIGH){
    SetGoals(1);
  }
  else{
    SetGoals(0);
  }
}

void SetGoals(int top){
  //set Goal to low y value
if (top==0){
    goal[0]= 275;
    goal[1]= 12.5;
    goal2[0]= 270;
    goal2[1]= 360;
}
//set Goal to high y value
else{
    goal2[0]= 275;
    goal2[1]= 12.5;
    goal[0]= 270;
    goal[1]= 360;
}
}
