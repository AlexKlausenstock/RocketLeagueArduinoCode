
//our robot is 
//1==green
//2==blue
//yellow (no number)

int gx, gy, bx, by, yx, yy;// Global variables for storing x and y coordinates of green, blue and yellow ball locations
int errorCode=0;
double robot [2]= {0,0};
double enemy [2]= {0,0};
double ball [2]= {0,0};

int robotColor=0;
int enemyColor=0;

void setup() {
  pinMode(8,INPUT);
}

void loop() {
  if(digitalRead(8)==HIGH){
    SetRobotBallEnemyColor(1,2);
    getRobotBallEnemy();
  }
  else{
    SetRobotBallEnemyColor(2,1);
    getRobotBallEnemy();
  }
}


void SetRobotBallEnemyColor(int robot, int enemy){
    if(robot==1){
        robotColor=1;
    }
    else if(robot ==2){
        robotColor=2;
    }
    else{
        errorCode=2;
    }

    if(enemy==1){
        enemyColor=1;
    }
    else if (enemy==2){
        enemyColor=2;
    }
    else{
        errorCode=2;
    }
}

void getRobotBallEnemy(){
        if(robotColor==1){
        robot[0]=gx;
        robot[1]=gy;
    }
    else if(robotColor ==2){
        robot[0]=bx;
        robot[1]=by;
    }
    else{
        errorCode=2;
    }

    ball[0]=yx;
    ball[1]=yy;

    if(enemyColor==1){
        enemy[0]=gx;
        enemy[1]=gy;
    }
    else if (enemyColor==2){
        enemy[0]=bx;
        enemy[1]=by;
    }
    else{
        errorCode=2;
    }
}
