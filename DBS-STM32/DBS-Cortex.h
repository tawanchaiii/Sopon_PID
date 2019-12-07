#include <Servo.h>

#define Ki 0
Servo servo1 ;
Servo servo2 ;
Servo servo3 ;
int Button = 0 ;
int Sensor[10];
#define PWMR  PA10  //   motor R
#define IN1R  PA15  //  
#define IN2R  PB3  //

#define PWML  PA8  //   motor L
#define IN1L  PB15  //  
#define IN2L  PB14  //


void set() {
  Serial.begin(115200);
//  servo1.attach(PB9);
  servo2.attach(PB8);
  servo3.attach(PA9);
  pinMode(PB5, INPUT);
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA2, INPUT);
  pinMode(PA3, INPUT);
  pinMode(PA4, INPUT);
  pinMode(PA5, INPUT);
  pinMode(PA6, INPUT);
  pinMode(PA7, INPUT);
  pinMode(PB0, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PA8, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PA10, OUTPUT);
  pinMode(PA15, OUTPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB12, OUTPUT);
}
void forward(int PWMA , int PWMB) {
  analogWrite(PA8, PWMA);
  analogWrite(PA10, PWMB);
  digitalWrite(PB15, LOW);
  digitalWrite(PB14, HIGH);
  digitalWrite(PA15, LOW);
  digitalWrite(PB3, HIGH);
}
void rearward(int PWMA , int PWMB) {
  analogWrite(PA8, PWMA);
  analogWrite(PA10, PWMB);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, LOW);
  digitalWrite(PA15, HIGH);
  digitalWrite(PB3, LOW);
}
void motors(int PWMA,int PWMB){
  analogWrite(PA8, PWMA);
  //analogWrite(PA10, PWMB);
  digitalWrite(PB15, LOW);
  digitalWrite(PB14, HIGH);
  digitalWrite(PA15, HIGH);
  digitalWrite(PB3, HIGH);
}
void Motor_begin(int sl,int sr)    
   {  
      if(sr>255)
      sr = 255;
      else if(sr<-255)
      sr = -255;
    
      if(sl>255)
      sl = 255;
      else if(sl<-255)
       sl = -255;
    
      if(sl>0)
        {
             digitalWrite(IN1L,HIGH);
             digitalWrite(IN2L,LOW);     
             analogWrite(PWML,sl);
         }
      else if(sl<0)
         {    
             digitalWrite(IN1L,LOW);
             digitalWrite(IN2L,HIGH);
             analogWrite(PWML,-sl);
         }
      else
         {        
             digitalWrite(IN1L,LOW);
             digitalWrite(IN2L,LOW);
             analogWrite(PWML,255);
         }  
  
      if(sr>0)
         {
             digitalWrite(IN1R,HIGH);
             digitalWrite(IN2R,LOW);
             analogWrite(PWMR,sr);
         }
      else if(sr<0)
         {    
            digitalWrite(IN1R,LOW);
            digitalWrite(IN2R,HIGH);
            analogWrite(PWMR,-sr);
          }
       else
          {        
             digitalWrite(IN1R,LOW);
             digitalWrite(IN2R,LOW);
              analogWrite(PWMR,255);
          }    
  }


void left(int PWMA , int PWMB) {
  analogWrite(PA8, PWMA);
  analogWrite(PA10, PWMB);
  digitalWrite(PB15, LOW);
  digitalWrite(PB14, HIGH);
  digitalWrite(PA15, HIGH);
  digitalWrite(PB3, LOW);
}
void right(int PWMA , int PWMB) {
  analogWrite(PA8, PWMA);
  analogWrite(PA10, PWMB);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, LOW);
  digitalWrite(PA15, LOW);
  digitalWrite(PB3, HIGH);
}
void stop() {
  analogWrite(PA8, 255);
  analogWrite(PA10, 255);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PA15, HIGH);
  digitalWrite(PB3, HIGH);
}
void BuzzerON() {
  digitalWrite(PB12, HIGH);
}
void BuzzerOFF() {
  digitalWrite(PB12, LOW);
}
void read() {
  Button = digitalRead(PB5);
  Sensor[0] = analogRead(PA0);
  Sensor[1] = analogRead(PA1);
  Sensor[2] = analogRead(PA2);
  Sensor[3] = analogRead(PA3);
  Sensor[4] = analogRead(PA4);
  Sensor[5] = analogRead(PA5);
  Sensor[6] = analogRead(PA6);
  Sensor[7] = analogRead(PA7);
  Sensor[8] = analogRead(PB0);
  Sensor[9] = analogRead(PB1);
}
void Motor(int PL,int PR)   ////////////////////////////////// คำสั่งควบคุมมอเตอร์ เช่น  Motor(100,100);
   {
      int L = 0;
      int R = 0;
      L = (PL*255)/100;
      R = (PR*255)/100;
      Motor_begin(L,R); 
   }
