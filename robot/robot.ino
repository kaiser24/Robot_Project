#include "Servo.h"

void setup2(void);

//Variable Declarations
Servo joint1,joint2,joint3,joint4,joint5,gripper;

int q1, q2, q3, q4 = 0;
int q1_p,q2_p,q3_p,q4_p;
int q1n, q2n,q3n,q4n;
int g;
char se = 0;

int state = 0;
unsigned int response;

void setup() {

  joint1.attach(3);
  joint2.attach(5);
  joint3.attach(6);
  joint4.attach(9);
  joint5.attach(10);
  Serial.begin(9600);
  
}


void loop() {
  if(se == 0){
        setup2();
        se = 1;
    }

   
  //info incomming from the python script
  //Serial.available() returns the number of bytes to be read
  while(Serial.available()>0){
      response = Serial.read();
        if(response = 'D' && state == 0){
            
            q1_p = q1n;
             
            q1 = Serial.parseInt();

            q1n = int((q1 - 6)*1.07 );

            if(q1n < 0){
                q1n = 0;
              }

            state = 1;
          }
        if(response = 'D' && state == 1){
            q2_p = q2n;
            
            q2 = Serial.parseInt();

            q2n = 180 - int(q2 * 0.8);   //|
            
            state = 2;
          }
        if(response = 'D' && state == 2){
            q3_p = q3n;
            
            q3 = Serial.parseInt();

            q3n = 180 - q3;   //|

            state = 3;
          }
        if(response = 'D' && state == 3){
            q4_p = q4n;
            
            q4 = Serial.parseInt();

            q4n = 180 - q4;   //|

            state = 4;
          }
        if(response = 'D' && state == 4){
            g = Serial.parseInt();
            state = 5;
          }
        if(state == 5){
            delay(500);
            joint5.write(g);


            if(  (q4n - q4_p) < 0  ){
                 for(q4_p; q4_p >= q4n; q4_p = q4_p - 5){
                    joint4.write(q2_p);
                    delay(50);
                 }
            }else{
                 for(q4_p; q4_p <= q4n; q4_p = q4_p + 5){
                    joint4.write(q4_p);
                    delay(50);
                 } 
            }

            if(  (q3n - q3_p) < 0  ){
                 for(q3_p; q3_p >= q3n; q3_p = q3_p - 5){
                    joint3.write(q3_p);
                    delay(50);
                 }
            }else{
                 for(q3_p; q3_p <= q3n; q3_p = q3_p + 5){
                    joint3.write(q3_p);
                    delay(50);
                 } 
            }

             if(  (q2n - q2_p) < 0  ){
                 for(q2_p; q2_p >= q2n; q2_p = q2_p - 5){
                    joint2.write(q2_p);
                    delay(50);
                 }
            }else{
                 for(q2_p; q2_p <= q2n; q2_p = q2_p + 5){
                    joint2.write(q2_p);
                    delay(50);
                 } 
            }

            if(  (q1 - q1_p) < 0  ){
                 for(q1_p; q1_p >= q1; q1_p = q1_p - 5){
                    joint1.write(q1_p);
                    delay(50);
                 }
            }else{
                 for(q1_p; q1_p <= q1; q1_p = q1_p + 5){
                    joint1.write(q1_p);
                    delay(50);
                 } 
            }
 

              state = 0;
           
          }

    } 
    
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Function Definitios
void setup2(void){
      q1 = 90;
      q2 = 90; //||
      q3 = 0;
      q4 = 0;
      g = 90;

      q1n = int((q1- 6)*1.07143 );
      if(q1n < 0){
            q1n = 0;
          }
      q2n = 180 - (q2 * 0.8);   
      q3n = 180 - q3;
      q4n = 180 - q4;

      joint1.write(q1n);
      joint2.write(q2n);
      joint3.write(q3n);
      joint4.write(q4n);
      joint5.write(g);
  }
