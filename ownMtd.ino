#include <Wire.h>
#include<math.h>

#define servo1  (16>>1)
#define servo2  (18>>1)
#define UART_BAUD_RATE  115200
#define LED 13

#define SERIAL_BUFFER_SIZE  256

void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos);
void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir);
void I2C_SERVOOFFSET(unsigned char servo_num,int value);
void I2C_SERVOSPEED(unsigned char value);
void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos);
void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos);
char I2C_SERVOEND(void);

void CheckEndMovement(void);
void UserCode(void);
void LEDToggle(void);

void ServoSetAngle();

double distanceBtw(double a,double b,double c,double p,double q,double r);
void calcMoves();
void updateLeg(int i);
bool isSame(int i);
void calcMotor2Pos(int i);
void calcAlpha(int i);
void calcBeta(int i);
void calcGamma(int i);
void manualSetAngle(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18);
void displayAngle();

int checkRegion(int x,int y,int i);
void setIncrX(int incr,int i);
void setIncrY(int incr,int i);
void legPosReset(int i);

void standUp();
void lieFlat();
void idleState();
void liftLeg(int i);
void dropLeg(int i);
void rotateLeg(int i,int angle);
void moveAlongX(int incr);
void moveAlongY(int incr);
void rotateAbout(int angle);



bool isIdleState =true;
bool activeLegSet=false; //False-> Set 0(0,3,4) True -> Set 1(1,2,5)

double h=24,C=6,F=17,T=24;
int minR=15,maxR=35;

double legFootPos[6][3]={{35,17.5,h}, {35,-17.5,h}, {0,36.5,h}, {0,-36.5,h}, {-35,17.5,h}, {-35,-17.5,h}};
double destFootPos[6][3]={{35,17.5,h}, {35,-17.5,h}, {0,36.5,h}, {0,-36.5,h}, {-35,17.5,h}, {-35,-17.5,h}};
double motor1Pos[6][3]={{15,8,0}, {15,-8,0}, {0,13.5,0}, {0,-13.5,0}, {-15,8,0}, {-15,-8,0}};
double motor2Pos[6][3]={{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
int zeroError[18]={0,0,0, 0,0,0, 3,0,0, 1,0,0, 3,0,0, -3,0,0};
int offset[18]={0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0};
double S_ang[18]={90,90,90, 90,90,90, 90,90,90, 90,90,90, 90,90,90, 90,90,90}; 

//Temp
int imf=0;

volatile int cnt,c,servoval;
volatile char state,servobuf[36],bytecnt;

char LEDState=0;

#define State_Start  0
#define State_Command  1
#define State_Servoposition  2
#define State_Speed  3
#define State_Servomin  4
#define State_Servomax  5
#define State_Servooffset  6
#define State_Servoreverse  7
#define State_Servonutral  8
#define State_ReadOffsets  9

void setup()
{ 
  cnt=0;
  int i;
  unsigned int x;
  char buffer[10],tmp,tmp1;
  double range;
  Serial.begin(UART_BAUD_RATE);
  Serial.write('S');
  Serial.write('r');
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(8, INPUT);
  digitalWrite(8,1);
  delay(500);
  sei();
  Wire.begin();
  TWSR = 3; // no prescaler
  TWBR = 18;  //Set I2C speed lower to suite I2C Servo controller
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  delay(500);
  state=State_Start;
}

void loop()
{
  UserCode();
  LEDToggle();
//  delay(100);
}

void UserCode(void)
{

  //------------------------------Configuration------------------------------

  I2C_SERVOMAX(1,2500); I2C_SERVOMAX(2,2500); I2C_SERVOMAX(3,2500); I2C_SERVOMAX(4,2500); I2C_SERVOMAX(5,2500); I2C_SERVOMAX(6,2500); I2C_SERVOMAX(7,2500); I2C_SERVOMAX(8,2500); I2C_SERVOMAX(9,2500); I2C_SERVOMAX(10,2500); I2C_SERVOMAX(11,2500); I2C_SERVOMAX(12,2500); I2C_SERVOMAX(13,2500); I2C_SERVOMAX(14,2500); I2C_SERVOMAX(15,2500); I2C_SERVOMAX(16,2500); I2C_SERVOMAX(17,2500); I2C_SERVOMAX(18,2500);    //Maximum Values

  I2C_SERVOMIN(1,500); I2C_SERVOMIN(2,500); I2C_SERVOMIN(3,500); I2C_SERVOMIN(4,500); I2C_SERVOMIN(5,500); I2C_SERVOMIN(6,500); I2C_SERVOMIN(7,500); I2C_SERVOMIN(8,500); I2C_SERVOMIN(9,500); I2C_SERVOMIN(10,500); I2C_SERVOMIN(11,500); I2C_SERVOMIN(12,500); I2C_SERVOMIN(13,500); I2C_SERVOMIN(14,500); I2C_SERVOMIN(15,500); I2C_SERVOMIN(16,500); I2C_SERVOMIN(17,500); I2C_SERVOMIN(18,500);    //Minimum Values

  I2C_SERVOOFFSET(1,1500); I2C_SERVOOFFSET(2,1500); I2C_SERVOOFFSET(3,1500); I2C_SERVOOFFSET(4,1500); I2C_SERVOOFFSET(5,1500); I2C_SERVOOFFSET(6,1500); I2C_SERVOOFFSET(7,1500); I2C_SERVOOFFSET(8,1500); I2C_SERVOOFFSET(9,1500); I2C_SERVOOFFSET(10,1500); I2C_SERVOOFFSET(11,1500); I2C_SERVOOFFSET(12,1500); I2C_SERVOOFFSET(13,1500); I2C_SERVOOFFSET(14,1500); I2C_SERVOOFFSET(15,1500); I2C_SERVOOFFSET(16,1500); I2C_SERVOOFFSET(17,1500); I2C_SERVOOFFSET(18,1500);    //Offset Values

  I2C_SERVOREVERSE(1,1); I2C_SERVOREVERSE(2,1); I2C_SERVOREVERSE(3,0); I2C_SERVOREVERSE(4,0); I2C_SERVOREVERSE(5,0); I2C_SERVOREVERSE(6,1); I2C_SERVOREVERSE(7,1); I2C_SERVOREVERSE(8,1); I2C_SERVOREVERSE(9,0); I2C_SERVOREVERSE(10,0); I2C_SERVOREVERSE(11,0); I2C_SERVOREVERSE(12,1); I2C_SERVOREVERSE(13,1); I2C_SERVOREVERSE(14,1); I2C_SERVOREVERSE(15,0); I2C_SERVOREVERSE(16,0); I2C_SERVOREVERSE(17,0); I2C_SERVOREVERSE(18,1);    //Directions (Servo Reverse)

  I2C_SERVOSPEED(80);
  //------------------------------Code Flow------------------------------
  //  ServoSetAll(,,,,,,,,,,,,,,,,,);
//  ServoSetAll(1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500);   // Line # 0
//  ServoSetAll(,,,,,,,,,,,,,,,,,);
// ServoSetAngle();
  
//  while(1){
//    LEDToggle();
//    delay(100);
//  }
//  idleState(); 
//  delay(1000);
//  standUp();
//  delay(1000);
//  lieFlat();

updateLeg(0);
updateLeg(1);
updateLeg(2);
updateLeg(3);
updateLeg(4);
updateLeg(5);
ServoSetAngle();
delay(500);
moveAlongX(-10);
//moveAlongY(5);
//rotateAbout(-30);
delay(10000);
imf=0;
//delay(2000);

}

void displayAngle(){
  Serial.print("\n");
  Serial.print("\n");
  Serial.print(S_ang[0]);
  Serial.print("\t");
  Serial.print(S_ang[1]);
  Serial.print("\t");
  Serial.print(S_ang[2]);
  Serial.print("\n");
  Serial.print(S_ang[3]);
  Serial.print("\t");
  Serial.print(S_ang[4]);
  Serial.print("\t");
  Serial.print(S_ang[5]);
  Serial.print("\n");
  Serial.print(S_ang[6]);
  Serial.print("\t");
  Serial.print(S_ang[7]);
  Serial.print("\t");
  Serial.print(S_ang[8]);
  Serial.print("\n");
  Serial.print(S_ang[9]);
  Serial.print("\t");
  Serial.print(S_ang[10]);
  Serial.print("\t");
  Serial.print(S_ang[11]);
  Serial.print("\n");
  Serial.print(S_ang[12]);
  Serial.print("\t");
  Serial.print(S_ang[13]);
  Serial.print("\t");
  Serial.print(S_ang[14]);
  Serial.print("\n");
  Serial.print(S_ang[15]);
  Serial.print("\t");
  Serial.print(S_ang[16]);
  Serial.print("\t");
  Serial.print(S_ang[17]);
  Serial.print("\n");
  Serial.print("\n");
}

void ServoSetAngle()
{
  unsigned int Servo1,Servo2,Servo3,Servo4,Servo5,Servo6,Servo7,Servo8,Servo9,Servo10,Servo11,Servo12,Servo13,Servo14,Servo15,Servo16,Servo17,Servo18;
  
  Servo1=map(S_ang[0]+zeroError[0]+offset[0],0,180,500,2500);
  Servo2=map(S_ang[1]+zeroError[1]+offset[1],0,180,500,2500);
  Servo3=map(S_ang[2]+zeroError[2]+offset[2],0,180,500,2500);
  Servo4=map(S_ang[3]+zeroError[3]+offset[3],0,180,500,2500);
  Servo5=map(S_ang[4]+zeroError[4]+offset[4],0,180,500,2500);
  Servo6=map(S_ang[5]+zeroError[5]+offset[5],0,180,500,2500);
  Servo7=map(S_ang[6]+zeroError[6]+offset[6],0,180,500,2500);
  Servo8=map(S_ang[7]+zeroError[7]+offset[7],0,180,500,2500);
  Servo9=map(S_ang[8]+zeroError[8]+offset[8],0,180,500,2500);
  Servo10=map(S_ang[9]+zeroError[9]+offset[9],0,180,500,2500);
  Servo11=map(S_ang[10]+zeroError[10]+offset[10],0,180,500,2500);
  Servo12=map(S_ang[11]+zeroError[11]+offset[11],0,180,500,2500);
  Servo13=map(S_ang[12]+zeroError[12]+offset[12],0,180,500,2500);
  Servo14=map(S_ang[13]+zeroError[13]+offset[13],0,180,500,2500);
  Servo15=map(S_ang[14]+zeroError[14]+offset[14],0,180,500,2500);
  Servo16=map(S_ang[15]+zeroError[15]+offset[15],0,180,500,2500);
  Servo17=map(S_ang[16]+zeroError[16]+offset[16],0,180,500,2500);
  Servo18=map(S_ang[17]+zeroError[17]+offset[17],0,180,500,2500);
  ServoSetAll(Servo1,Servo2,Servo3,Servo4,Servo5,Servo6,Servo7,Servo8,Servo9,Servo10,Servo11,Servo12,Servo13,Servo14,Servo15,Servo16,Servo17,Servo18);
}

double distanceBtw(double a,double b,double c,double p,double q,double r){
  return sqrt(((p-a)*(p-a))+((q-b)*(q-b))+((r-c)*(r-c)));
}

void calcMoves(){}

void updateLeg(int i){
  calcAlpha(i);
  calcMotor2Pos(i);
  calcBeta(i);
  calcGamma(i);
}

bool isSame(int i){
  if((legFootPos[i][0]==destFootPos[i][0])&&(legFootPos[i][1]==destFootPos[i][1])&&(legFootPos[i][2]==destFootPos[i][2]))
    return true;
   return false;
}

void calcMotor2Pos(int i)
{
   double theta,alpha=S_ang[3*i];
   switch(i){
    case 0:theta=alpha-30;break;
    case 1:theta=210-alpha;break;
    case 2:theta=alpha-90;break;
    case 3:theta=270-alpha;break;
    case 4:theta=alpha-150;break;
    case 5:theta=330-alpha;break;
   }
   double rad=theta*PI/180;
   motor2Pos[i][0]=C*sin(rad)+motor1Pos[i][0];
   motor2Pos[i][1]=C*cos(rad)+motor1Pos[i][1];
   motor2Pos[i][2]=motor1Pos[i][2];
}
void calcAlpha(int i)
{
  double l1 =distanceBtw(motor1Pos[i][0],motor1Pos[i][1],motor1Pos[i][2],destFootPos[i][0],destFootPos[i][1],destFootPos[i][2]);
  double hd=distanceBtw(motor1Pos[i][0],motor1Pos[i][1],0,destFootPos[i][0],destFootPos[i][1],0);
  double phi,alpha;
  if((destFootPos[i][0]-motor1Pos[i][0])>=0){
    phi=acos((destFootPos[i][1]-motor1Pos[i][1])/hd)*180/PI;;
  }
  else {
    phi=360-(acos((destFootPos[i][1]-motor1Pos[i][1])/hd)*180/PI);;
  }
  switch(i){
    case 0:alpha=phi+30;break;
    case 1:alpha=210-phi;break;
    case 2:alpha=phi+90;break;
    case 3:alpha=270-phi;break;
    case 4:alpha=phi+150;break;
    case 5:alpha=330-phi;break;
  }
  if(alpha>=360){
    alpha=alpha-360;
  }
  S_ang[3*i]=alpha;
}
  
void calcBeta(int i)
{
  double l2=distanceBtw(motor2Pos[i][0],motor2Pos[i][1],motor2Pos[i][2],destFootPos[i][0],destFootPos[i][1],destFootPos[i][2]); 
  double beta=acos(h/l2)*180/PI;
  beta=beta+(acos(((l2*l2)+(F*F)-(T*T))/(2*F*l2))*180/PI);
  S_ang[3*i+1]=beta;
}

void calcGamma(int i)
{
  double l2=distanceBtw(motor2Pos[i][0],motor2Pos[i][1],motor2Pos[i][2],destFootPos[i][0],destFootPos[i][1],destFootPos[i][2]);  
  double gamma;
  gamma = acos(((F*F)+(T*T)-(l2*l2))/(2*F*T))*180/PI;
  S_ang[3*i+2]=gamma;  
}

void standUp(){
  manualSetAngle(90,170,30,90,170,30,90,170,30,90,170,30,90,170,30,90,170,30);
  ServoSetAngle();
  delay(100);
  idleState();
}
void lieFlat(){
  manualSetAngle(90,120,180,90,120,180,90,120,180,90,120,180,90,120,180,90,120,180);
  ServoSetAngle();
  delay(100);
  manualSetAngle(90,90,180,90,90,180,90,90,180,90,90,180,90,90,180,90,90,180);
  ServoSetAngle();
  delay(100);
}
void idleState(){
  manualSetAngle(90,120,60,90,120,60,90,120,60,90,120,60,90,120,60,90,120,60);
  ServoSetAngle();
  delay(100);
}

int checkRegion(int x,int y,int i){
    double l1 =distanceBtw(motor1Pos[i][0],motor1Pos[i][1],motor1Pos[i][2],destFootPos[i][0]+x,destFootPos[i][1]+y,destFootPos[i][2]);
    double hd=distanceBtw(motor1Pos[i][0],motor1Pos[i][1],0,destFootPos[i][0],destFootPos[i][1],0);
    double phi,alpha;
    if((destFootPos[i][0]-motor1Pos[i][0])>=0){
      phi=acos((destFootPos[i][1]-motor1Pos[i][1])/hd)*180/PI;;
    }
    else {
      phi=360-(acos((destFootPos[i][1]-motor1Pos[i][1])/hd)*180/PI);;
    }
    switch(i){
        case 0:alpha=phi+30;break;
        case 1:alpha=210-phi;break;
        case 2:alpha=phi+90;break;
        case 3:alpha=270-phi;break;
        case 4:alpha=phi+150;break;
        case 5:alpha=330-phi;break;
    }
    if(alpha>=360){
        alpha=alpha-360;
    }
    if(alpha>180 || alpha<0) return 0;
    double d=sqrt((l1*l1)-(h*h));
    if(d<minR) return 1;
    else if(d<=maxR) return 2;
    return 0;
}

void setIncrX(int incr,int i)
{
    int region=checkRegion(incr,0,i);
    if(region==0){
        printf("\n\nERROR\n\n");
        return;
    }
    else if(region==2){
        destFootPos[i][0]+=incr;
        return;
    }
    else{
        double p=sqrt((minR*minR)-(destFootPos[i][0]+incr-motor1Pos[i][0])*(destFootPos[i][0]+incr-motor1Pos[i][0]));
        switch(i){
            case 0: if(S_ang[3*i]>120) p*=-1; break;
            case 1: if(S_ang[3*i]<120) p*=-1; break;
            case 2: break;
            case 3: p*=-1; break;
            case 4: if(S_ang[3*i]<60) p*=-1; break;
            case 5: if(S_ang[3*i]>60) p*=-1; break;
        }
        double y=motor1Pos[i][1]+p;
        destFootPos[i][0]+=incr;
        destFootPos[i][1]=y;
        return;
    }
}
void setIncrY(int incr,int i)
{
    int region=checkRegion(0,incr,i);
    if(region==0){
        printf("\n\nERROR\n\n");
        return;
    }
    else if(region==2){
        destFootPos[i][1]+=incr;
        return;
    }
    else{
        double p=sqrt((minR*minR)-(destFootPos[i][1]+incr-motor1Pos[i][1])*(destFootPos[i][1]+incr-motor1Pos[i][1]));
        switch(i){
            case 0: if(S_ang[3*i]<30) p*=-1; break;
            case 1: if(S_ang[3*i]<30) p*=-1; break;
            case 2: if(S_ang[3*i]<90) p*=-1; break;
            case 3: if(S_ang[3*i]<90) p*=-1; break;
            case 4: if(S_ang[3*i]<150) p*=-1; break;
            case 5: if(S_ang[3*i]<150) p*=-1; break;
        }
        double x=motor1Pos[i][0]+p;
        destFootPos[i][1]+=incr;
        destFootPos[i][0]=x;
        return;
    }
}
void legPosReset(int i){
    destFootPos[i][0]=legFootPos[i][0];
    destFootPos[i][1]=legFootPos[i][1];
    destFootPos[i][2]=legFootPos[i][2];
}
void moveAlongX(int incr){
  bool endMov=false;
  if(imf<15){
    imf++;
  }
  else{
    endMov=true;
  }
  if(isIdleState){
    if(endMov) return;
    isIdleState=false;
    if(activeLegSet){ //move Set 1
      activeLegSet=!activeLegSet;
      liftLeg(1);
      liftLeg(2);
      liftLeg(5);
      ServoSetAngle();
      delay(50);
      setIncrX(incr,1);
      setIncrX(incr,2);
      setIncrX(incr,5);
      updateLeg(1);
      updateLeg(2);
      updateLeg(5);
      ServoSetAngle();
      delay(50);
      dropLeg(1);
      dropLeg(2);
      dropLeg(5);
      ServoSetAngle();
      delay(50);
    }
    else{
      //move set 2
      activeLegSet=!activeLegSet;
      liftLeg(0);
      liftLeg(3);
      liftLeg(4);
      ServoSetAngle();
      delay(50);
      setIncrX(incr,0);
      setIncrX(incr,3);
      setIncrX(incr,4);
      updateLeg(0);
      updateLeg(3);
      updateLeg(4);
      ServoSetAngle();
      delay(50);
      dropLeg(0);
      dropLeg(3);
      dropLeg(4);
      ServoSetAngle();
      delay(50);
    }
    moveAlongX(incr);
  }
  else{
    if(endMov){
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        legPosReset(0);
        legPosReset(3);
        legPosReset(4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        legPosReset(1);
        legPosReset(2);
        legPosReset(5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      isIdleState=true;
      return;
    }
    else{
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        legPosReset(0);
        legPosReset(3);
        legPosReset(4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        setIncrX(incr,1);
        setIncrX(incr,2);
        setIncrX(incr,5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        legPosReset(1);
        legPosReset(2);
        legPosReset(5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        setIncrX(incr,0);
        setIncrX(incr,3);
        setIncrX(incr,4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      moveAlongX(incr);
    }
  }
}

void moveAlongY(int incr){
    bool endMov=false;
  if(imf<15){
    imf++;
  }
  else{
    endMov=true;
  }
  if(isIdleState){
    if(endMov) return;
    isIdleState=false;
    if(activeLegSet){ //move Set 1
      activeLegSet=!activeLegSet;
      liftLeg(1);
      liftLeg(2);
      liftLeg(5);
      ServoSetAngle();
      delay(50);
      setIncrY(incr,1);
      setIncrY(incr,2);
      setIncrY(incr,5);
      updateLeg(1);
      updateLeg(2);
      updateLeg(5);
      ServoSetAngle();
      delay(50);
      dropLeg(1);
      dropLeg(2);
      dropLeg(5);
      ServoSetAngle();
      delay(50);
    }
    else{
      //move set 2
      activeLegSet=!activeLegSet;
      liftLeg(0);
      liftLeg(3);
      liftLeg(4);
      ServoSetAngle();
      delay(50);
      setIncrY(incr,0);
      setIncrY(incr,3);
      setIncrY(incr,4);
      updateLeg(0);
      updateLeg(3);
      updateLeg(4);
      ServoSetAngle();
      delay(50);
      dropLeg(0);
      dropLeg(3);
      dropLeg(4);
      ServoSetAngle();
      delay(50);
    }
    moveAlongY(incr);
  }
  else{
    if(endMov){
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        legPosReset(0);
        legPosReset(3);
        legPosReset(4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        legPosReset(1);
        legPosReset(2);
        legPosReset(5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      isIdleState=true;
      return;
    }
    else{
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        legPosReset(0);
        legPosReset(3);
        legPosReset(4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        setIncrY(incr,1);
        setIncrY(incr,2);
        setIncrY(incr,5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        legPosReset(1);
        legPosReset(2);
        legPosReset(5);
        updateLeg(1);
        updateLeg(2);
        updateLeg(5);
        ServoSetAngle();
        delay(50);
        setIncrY(incr,0);
        setIncrY(incr,3);
        setIncrY(incr,4);
        updateLeg(0);
        updateLeg(3);
        updateLeg(4);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      moveAlongY(incr);
    }
  }
}

void rotateAbout(int angle){
  bool endMov=false;
  if(imf<15){
    imf++;
  }
  else{
    endMov=true;
  }
  if(isIdleState){
    if(endMov) return;
    isIdleState=false;
    if(activeLegSet){ //move Set 1
      activeLegSet=!activeLegSet;
      liftLeg(1);
      liftLeg(2);
      liftLeg(5);
      ServoSetAngle();
      delay(50);
      rotateLeg(1,angle);
      rotateLeg(2,angle);
      rotateLeg(5,angle);      
      ServoSetAngle();
      delay(50);
      dropLeg(1);
      dropLeg(2);
      dropLeg(5);
      ServoSetAngle();
      delay(50);
    }
    else{
      //move set 2
      activeLegSet=!activeLegSet;
      liftLeg(0);
      liftLeg(3);
      liftLeg(4);
      ServoSetAngle();
      delay(50);
      rotateLeg(0,angle);
      rotateLeg(3,angle);
      rotateLeg(4,angle);
      ServoSetAngle();
      delay(50);
      dropLeg(0);
      dropLeg(3);
      dropLeg(4);
      ServoSetAngle();
      delay(50);
    }
    rotateAbout(angle);
  }
  else{
    if(endMov){
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        rotateLeg(0,0);
        rotateLeg(3,0);
        rotateLeg(4,0);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        rotateLeg(1,0);
        rotateLeg(2,0);
        rotateLeg(5,0);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      isIdleState=true;
      return;
    }
    else{
      if(activeLegSet){ //move Set 1
        activeLegSet=!activeLegSet;
        liftLeg(1);
        liftLeg(2);
        liftLeg(5);
        rotateLeg(0,0);
        rotateLeg(3,0);
        rotateLeg(4,0);
        ServoSetAngle();
        delay(50);
        rotateLeg(1,angle);
        rotateLeg(2,angle);
        rotateLeg(5,angle);
        ServoSetAngle();
        delay(50);
        dropLeg(1);
        dropLeg(2);
        dropLeg(5);
        ServoSetAngle();
        delay(50);
      }
      else{
        //move set 2
        activeLegSet=!activeLegSet;
        liftLeg(0);
        liftLeg(3);
        liftLeg(4);
        rotateLeg(1,0);
        rotateLeg(2,0);
        rotateLeg(5,0);
        ServoSetAngle();
        delay(50);
        rotateLeg(0,angle);
        rotateLeg(3,angle);
        rotateLeg(4,angle);
        ServoSetAngle();
        delay(50);
        dropLeg(0);
        dropLeg(3);
        dropLeg(4);
        ServoSetAngle();
        delay(50);
      }
      rotateAbout(angle);
    }
  }  
}

void liftLeg(int i){
  offset[3*i+1]+=20;
}
void dropLeg(int i){
  offset[3*i+1]-=20;
}

void rotateLeg(int i,int angle){
  if(i%2==0) offset[3*i]=-1*angle;
  else offset[3*i]=angle;
}

void manualSetAngle(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
{
  S_ang[0]=Servo1;
  S_ang[1]=Servo2;
  S_ang[2]=Servo3;
  S_ang[3]=Servo4;
  S_ang[4]=Servo5;
  S_ang[5]=Servo6;
  S_ang[6]=Servo7;
  S_ang[7]=Servo8;
  S_ang[8]=Servo9;
  S_ang[9]=Servo10;
  S_ang[10]=Servo11;
  S_ang[11]=Servo12;
  S_ang[12]=Servo13;
  S_ang[13]=Servo14;
  S_ang[14]=Servo15;
  S_ang[15]=Servo16;
  S_ang[16]=Servo17;
  S_ang[17]=Servo18;
}


void LEDToggle(void)
{
  if (LEDState == 0)
    LEDState = 1;
  else
    LEDState = 0;
  digitalWrite(LED, LEDState);
}

void ServoSetAll(unsigned int Servo1, unsigned int Servo2, unsigned int Servo3, unsigned int Servo4, unsigned int Servo5, unsigned int Servo6, unsigned int Servo7, unsigned int Servo8, unsigned int Servo9, unsigned int Servo10, unsigned int Servo11, unsigned int Servo12, unsigned int Servo13, unsigned int Servo14, unsigned int Servo15, unsigned int Servo16, unsigned int Servo17, unsigned int Servo18)
{
  if (Servo1 >= 500) {I2C_SERVOSET(1,Servo1);}
  if (Servo2 >= 500) {I2C_SERVOSET(2,Servo2);}
  if (Servo3 >= 500) {I2C_SERVOSET(3,Servo3);}
  if (Servo4 >= 500) {I2C_SERVOSET(4,Servo4);}
  if (Servo5 >= 500) {I2C_SERVOSET(5,Servo5);}
  if (Servo6 > 500) {I2C_SERVOSET(6,Servo6);}
  if (Servo7 >= 500) {I2C_SERVOSET(7,Servo7);}
  if (Servo8 >= 500) {I2C_SERVOSET(8,Servo8);}
  if (Servo9 >= 500) {I2C_SERVOSET(9,Servo9);}
  if (Servo10 >= 500) {I2C_SERVOSET(10,Servo10);}
  if (Servo11 >= 500) {I2C_SERVOSET(11,Servo11);}
  if (Servo12 >= 500) {I2C_SERVOSET(12,Servo12);}
  if (Servo13 >= 500) {I2C_SERVOSET(13,Servo13);}
  if (Servo14 >= 500) {I2C_SERVOSET(14,Servo14);}
  if (Servo15 >= 500) {I2C_SERVOSET(15,Servo15);}
  if (Servo16 >= 500) {I2C_SERVOSET(16,Servo16);}
  if (Servo17 >= 500) {I2C_SERVOSET(17,Servo17);}
  if (Servo18 >= 500) {I2C_SERVOSET(18,Servo18);}
  //Experiment
  while (!I2C_SERVOEND())
  {
    delay(1);
  }
  LEDToggle();
}
char I2C_SERVOEND(void)
{
  int i, n;
  char buffer;
  Wire.beginTransmission(servo1);
  n = Wire.write(181);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);
  if (n != 0)
    return (n);

  delayMicroseconds(350);
  Wire.requestFrom(servo1, 1, true);
  while(Wire.available())
    buffer=Wire.read();

  return(buffer);
}



void I2C_SERVOMIN(unsigned char servo_num,unsigned int servo_pos)
{
  if(servo_pos<500)
    servo_pos = 500;
  else if(servo_pos>2500)
    servo_pos=2500;
  servo_pos=((servo_pos*2)-1000);

  if(servo_num<19)
    Wire.beginTransmission(servo1);
  else
    Wire.beginTransmission(servo2);
  Wire.write((servo_num-1)+(18*4));
  Wire.write(servo_pos>>8);
  Wire.write(servo_pos & 0XFF);
  Wire.endTransmission();
  delay(20);
}

void I2C_SERVOMAX(unsigned char servo_num,unsigned int servo_pos)
{
  if(servo_pos<500)
    servo_pos = 500;
  else if(servo_pos>2500)
    servo_pos=2500;
  servo_pos=((servo_pos*2)-1000);

  if(servo_num<19)
    Wire.beginTransmission(servo1);
  else
    Wire.beginTransmission(servo2);
  Wire.write((servo_num-1)+(18*3));
  Wire.write(servo_pos>>8);
  Wire.write(servo_pos & 0XFF);
  Wire.endTransmission();
  delay(20);
}
void I2C_SERVOOFFSET(unsigned char servo_num,int value)
{
  value=3000-value;
  value=value-1500;

  if (value<-500)
    value=-500;
  else if (value>500)
    value=500;

  if(value>0)
    value=2000+(value*2);
  else if(value<=0)
    value=-value*2;

  
  if(servo_num<19)
    Wire.beginTransmission(servo1);
  else
    Wire.beginTransmission(servo2);
  Wire.write((servo_num-1)+(18*6));
  Wire.write(value>>8);
  Wire.write(value & 0XFF);
  Wire.endTransmission();
  delay(20);
}
void I2C_SERVOSET(unsigned char servo_num,unsigned int servo_pos)
{
  if(servo_pos<500)
    servo_pos = 500;
  else if(servo_pos>2500)
    servo_pos=2500;

  if(servo_pos>501)
    servo_pos=(((servo_pos-2)*2)-1000);
  else
    servo_pos=0;

  if(servo_num<19)
    Wire.beginTransmission(servo1);
  else
    Wire.beginTransmission(servo2);
  Wire.write(servo_num-1);
  Wire.write(servo_pos>>8);
  Wire.write(servo_pos & 0XFF);
  Wire.endTransmission();
}

//Experiment
void I2C_SERVOSPEED(unsigned char value)
{
  Wire.beginTransmission(servo1);
  Wire.write(18*2);
  Wire.write(value);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(servo2);
  Wire.write(18*2);
  Wire.write(value);
  Wire.write(0);
  Wire.endTransmission();
  delay(20);
}
void I2C_SERVOREVERSE(unsigned char servo_num,unsigned char servo_dir)
{
  if(servo_dir>0)
    servo_dir=1;
  if(servo_num<19)
    Wire.beginTransmission(servo1);
  else
    Wire.beginTransmission(servo2);
  Wire.write((servo_num-1)+(18*7));
  Wire.write(servo_dir);
  Wire.write(0);
  Wire.endTransmission();
  delay(20);
}
