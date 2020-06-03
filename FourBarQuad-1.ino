//FourBarQuad-1 is based on a 4 bars linkage. Symmetrical legs gaits - 03/06/2020
void(* resetFunc) (void) = 0;         // soft reset
#include <Servo.h>
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
int distance=1000;
int stp=1;        // value in millimeter, determines the moving unit
int Speed=1500;   // stp determines the pitch in mm   // Speed determines le speed 

const int nb=8;                        // Number of servos
Servo Srv[nb];                         // Servos table
int OnOff[nb]={1,1,1,1,1,1,1,1};       // Servos table on/off

int FRLS=0, FRRS=1, FLLS=2, FLRS=3, BRLS=4, BRRS=5, BLLS=6, BLRS=7;
// **************** Correction for symmetrical LS and RS servos position     ******************************
// servos      FRLS  FRRS  FLLS  FLRS  BRLS  BRRS  BLLS  BLRS modify these values according to your servos
int Err[nb]={   6,    0,    0,    -1,   0,    -8,    0,    0    };  
// **************** LS must have positive value, RS must have negative value ******************************

void  setup() {
  delay(500);                             // for reset consideration
  Serial.begin(115200);
  lox.begin();                            // Inialisation of the VL53L0X Laser rangefinder 
  pinMode(0,INPUT_PULLUP);                // start/reset button attachment

  Srv[FRLS].attach(2); Srv[FRLS].write(0+Err[FRLS]);   // FR Front Right leg - LS Left   Servo
  Srv[FRRS].attach(3); Srv[FRRS].write(180+Err[FRRS]); // FR Front Right leg - RS Right  Servo
  Srv[FLLS].attach(4); Srv[FLLS].write(0+Err[FLLS]);   // FL Front Left  leg - LS Left   Servo
  Srv[FLRS].attach(5); Srv[FLRS].write(180+Err[FLRS]); // FL Front Left  leg - RS Right  Servo
  Srv[BRLS].attach(6); Srv[BRLS].write(0+Err[BRLS]);   // BR Back  Right leg - LS Left   Servo
  Srv[BRRS].attach(7); Srv[BRRS].write(180+Err[BRRS]); // BR Back  Right leg - RS Right  Servo
  Srv[BLLS].attach(8); Srv[BLLS].write(0+Err[BLLS]);   // BL Back  Left  leg - LS Left   Servo
  Srv[BLRS].attach(9); Srv[BLRS].write(180+Err[BLRS]); // BL Back  Left  leg - RS Right  Servo

  Serial.print("\n\t To start, click on the Start button");
  while( digitalRead(0) );  delay(500);       // waiting for start button pressed 
  Serial.print("\n\t Started");
}

void loop() {
  Walk();
}

void Walk(){
  Laser();
  if(measure.RangeMilliMeter < 500){
    for(int i=0;i<=3;i++) {Backward(35,-5,-10,5);}  delay(200);
    for(int i=0;i<=5;i++) {TurnRight();}            delay(200);
  }else{
    Forward(35,-5,-10,5);
  }
}

void Forward(int Ax, int Ay, int Bx, int By){
  if (! digitalRead(0)) resetFunc();
  FRLS=0; FRRS=1; FLLS=2; FLRS=3; BRLS=4; BRRS=5; BLLS=6; BLRS=7;
  HLine(Ax,Bx,Ay);  VLine(Ax,Ay,Bx,By);
  HLine(Bx,Ax,By);  VLine(Ax,By,Bx,Ay);
}

void Backward(int Ax, int Ay, int Bx, int By){
  if (! digitalRead(0)) resetFunc();
  FRLS=6, FRRS=7, FLLS=4, FLRS=5, BRLS=2, BRRS=3, BLLS=0, BLRS=1;
  HLine(Ax,Bx,Ay);  VLine(Ax,Ay,Bx,By);
  HLine(Bx,Ax,By);  VLine(Ax,By,Bx,Ay);
}

void VLine(int Ax, int Ay, int Bx, int By){
  if(Ay<By){
    for (int i=Ay; i<By;i+=stp){
      InverseKinematics( Bx, i,FRLS,FRRS);
      InverseKinematics(-Bx,-i,BRLS,BRRS);
      InverseKinematics(-Ax,-i,FLLS,FLRS);
      InverseKinematics( Ax, i,BLLS,BLRS);
    }
  }else{
    for (int i=Ay; i>By;i-=stp){
      InverseKinematics( Ax, i,FRLS,FRRS);
      InverseKinematics(-Ax,-i,BRLS,BRRS);
      InverseKinematics(-Bx,-i,FLLS,FLRS);
      InverseKinematics( Bx, i,BLLS,BLRS);
    }
  }
}

void HLine(int Ax, int Bx, int y){
  if(Ax>Bx){
    for(int i=Ax; i>Bx;i-=stp){
      InverseKinematics(         i,  y,FRLS,FRRS);
      InverseKinematics(        -i, -y,BRLS,BRRS);
      InverseKinematics(-(Ax+Bx-i), -y,FLLS,FLRS);
      InverseKinematics(   Ax+Bx-i,  y,BLLS,BLRS);
    }
  }
  else{
    for(int i=Ax; i < Bx; i+=stp){
      InverseKinematics(         i,  y,FRLS,FRRS);
      InverseKinematics(        -i, -y,BRLS,BRRS);      
      InverseKinematics(-(Ax+Bx-i), -y,FLLS,FLRS);
      InverseKinematics(   Ax+Bx-i,  y,BLLS,BLRS);
    }
  }
}

void TurnLeft(){
  int Ax=10, Ay=-7, Bx=-10, By=7;
  int oldSpeed=Speed; Speed=1500;

    for(int i=Ax; i>Bx;i-=stp){
      InverseKinematics( i,  Ay,FRLS,FRRS);
      InverseKinematics(-i, -Ay,BRLS,BRRS);
      InverseKinematics(-i, -Ay,FLLS,FLRS);
      InverseKinematics( i,  Ay,BLLS,BLRS);
    }

    for (int i=Ay; i<By;i+=stp){
      InverseKinematics( Bx, i,FRLS,FRRS);
      InverseKinematics(-Bx,-i,BRLS,BRRS);
      InverseKinematics( Ax,-i,FLLS,FLRS);
      InverseKinematics(-Ax, i,BLLS,BLRS);
    }

    for(int i=Bx; i < Ax; i+=stp){
      InverseKinematics( i,  By,FRLS,FRRS);
      InverseKinematics(-i, -By,BRLS,BRRS);      
      InverseKinematics(-i, -By,FLLS,FLRS);
      InverseKinematics( i,  By,BLLS,BLRS);
    }

    for (int i=By; i>Ay;i-=stp){
      InverseKinematics( Ax, i,FRLS,FRRS);
      InverseKinematics(-Ax,-i,BRLS,BRRS);
      InverseKinematics( Bx,-i,FLLS,FLRS);
      InverseKinematics(-Bx, i,BLLS,BLRS);
    }

  Speed=oldSpeed;
}

void TurnRight(){
  int Ax=10, Ay=-7, Bx=-10, By=7;
  int oldSpeed=Speed; Speed=1500;

    for(int i=Ax; i>Bx;i-=stp){
      InverseKinematics( i, -Ay,FRLS,FRRS);
      InverseKinematics(-i,  Ay,BRLS,BRRS);
      InverseKinematics(-i,  Ay,FLLS,FLRS);
      InverseKinematics( i, -Ay,BLLS,BLRS);
    }

    for (int i=Ay; i<By;i+=stp){
      InverseKinematics( Bx,-i,FRLS,FRRS);
      InverseKinematics(-Bx, i,BRLS,BRRS);
      InverseKinematics( Ax, i,FLLS,FLRS);
      InverseKinematics(-Ax,-i,BLLS,BLRS);
    }

    for(int i=Bx; i < Ax; i+=stp){
      InverseKinematics( i, -By,FRLS,FRRS);
      InverseKinematics(-i,  By,BRLS,BRRS);      
      InverseKinematics(-i,  By,FLLS,FLRS);
      InverseKinematics( i, -By,BLLS,BLRS);
    }

    for (int i=By; i>Ay;i-=stp){
      InverseKinematics( Ax,-i,FRLS,FRRS);
      InverseKinematics(-Ax, i,BRLS,BRRS);
      InverseKinematics( Bx, i,FLLS,FLRS);
      InverseKinematics(-Bx,-i,BLLS,BLRS);
    }

  Speed=oldSpeed;
}


void InverseKinematics(int Px, int Py, int LS, int RS){
  float A1x=0, A1y=130, A2x=0, A2y=130;                   // Values of servos positions
  float a1=112, c1=32, a2=112, c2=32;                     // Values of leg sizes lengths

  float d=A1y-Py, e=Px;                                   // Calculation of inverse kinematics
  float b=sqrt((d*d)+(e*e));                              // Calculation of inverse kinematics
  float S=acos(d/b);  if(e<0)S=(-S);                      // Calculation of inverse kinematics
  float A12=acos(((b*b)+(c1*c1)-(a1*a1))/(2*b*c1));       // Calculation of inverse kinematics
  float A22=acos(((b*b)+(c2*c2)-(a2*a2))/(2*b*c2));       // Calculation of inverse kinematics
  float A11=(PI/2)-A12+S;                                 // Calculation of inverse kinematics
  float A21=(PI/2)-A22-S;                                 // Calculation of inverse kinematics

  int   S1=round(A11*57.296);                             //left servo angle in degree
  int   S2=round(180-(A21*57.296));                       //right servo angle in degree

/* DEBUG
  Serial.print("\n\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);
  Serial.print("\n\t d=");Serial.print(d);Serial.print("\t\t e=");Serial.print(e);
  Serial.print("\t\t b=");Serial.print(b);Serial.print("\t\t S=");Serial.print(S*57.296);
  Serial.print("\n\t A11=");Serial.print(A11*57.296);Serial.print("\t\t A12=");Serial.print(A12*57.296);
  Serial.print("\t\t A22=");Serial.print(A22*57.296);Serial.print("\t\t A21=");Serial.print(A21*57.296);
  Serial.print("\n\t Result of calculations, angles of the servos");
  Serial.print("\n\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°");
*/

  if ( b>(a1+c1) ){
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t b=");Serial.print(b);Serial.print(" > ");
    Serial.print(a1+c1);Serial.print(" is too long. Target impossible to reach   !!!!!");
    return;
  }

  if (S1<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1<0° is not reachable   !!!!!");
    return;
  }
  if (S2>180){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<0° is not reachable   !!!!!");
    return;
  }
  if (S1>140){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1>140° is not reachable   !!!!!");
    return;
  }

  if (S2<40){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<40° is not reachable   !!!!!");
    return;
  }

//  Serial.print("\t executed command");
  if (OnOff[LS]) Srv[LS].write(S1+Err[LS]);   // set target Left servo position if servo switch is On
  if (OnOff[RS]) Srv[RS].write(S2+Err[RS]);   // set target Right servo position if servo switch is On
  delayMicroseconds(Speed);
}

void Laser(){
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {   // phase failures have incorrect data
    Serial.print("\n\t Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {  
    //Serial.println("\n\t Distance out of range "); 
  }
}
