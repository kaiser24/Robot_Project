#include "Servo.h"

//this program will only move the joint to the angles
//that orders comming from the serial port say
//inverse kinematics and trajectories are dealt on python
//to ease things

//Function Declarations
void moveTest(void);

//Variable Declarations
Servo joint1,joint2,joint3,joint4,joint5,gripper;

int q1, q2d, q2i, q3, q4 = 0;
int q1n, q2dn,q2in,q3n,q4n;
int g;
// use joint.write(q); to move a servo motor
// notice that write receives int's. if you want more
// precision use joint.writeMicroseconds()

void setup() {
  // put your setup code here, to run once:
  joint1.attach(3);
  joint2.attach(5);
  joint3.attach(6);
  joint4.attach(9);
  joint5.attach(10);
  gripper.attach(11);
  Serial.begin(9600);
}

int m=0;
int i;
int j;
unsigned int response;
int val = 0;

void loop() {
  moveTest();
  //q1 = 0;
  //q2d = 90; //||
  //q3 = 0;
  //q4 = 0;
  //g = 90;

  //val = int( 0 * 0.8 );
  
  //q1n = 180 - q1;
  //q2dn = 180 - q2d;   //|
  //q2in = 180 - q2in;  //|
  //q3n = 180 - q3;
  //q4n = 180 - q4;
  //joint1.write(q1n);
  //joint2.write(q2dn);
  //joint2.write(val);
  //joint4.write(q3n);
  //joint5.write(q4n);
  //gripper.write(g);
  //info incomming from the python script
  //Serial.available() returns the number of bytes to be read
  //while(Serial.available()>0){
  //    response = Serial.read();
  //    if(response == 'S'){
  //      Serial.println(11,DEC);        
  //    }
  //  } 
}

//Function Definitios
void moveTest(void){
if(m==0){
 for(i=0;i<=180;i=i+1){
      j= 180 - i;
      val = int( i * 0.8 );
      joint3.write(val);
      //joint2.write(j);
      delay(20);
  }
  m=1;
 }else if(m==1){
    for(i=180;i>=0;i=i-1){
      j= 180 - i;
      val = int( i * 0.8 );
      joint3.write(val);
      //joint2.write(j);
      delay(20);
  }
    m=0;
 }
}
