#include "DBS-Cortex.h"
#include "NKP_TCSensor.h"
uint8_t numSensor = 4;
int a = 0;

uint16_t setpoint;
float present_position;
float errors = 0;
float output = 0;
float integral ;
float derivative ;
float previous_error ;
void setup() {
  set(); // ฟังชั่น เซ็ทอัพ
  /*while(1){
    if(digitalRead(PB5) == 1) break;
  }
  BuzzerON(); delay(100); BuzzerOFF();*/
  setSensorPins((const int[]){2,3,4,5}, 4);
  _min_sensor_values[0] = 1200, _max_sensor_values[0] = 3600;
  _min_sensor_values[1] = 1000, _max_sensor_values[1] = 3600;
  _min_sensor_values[2] = 1000, _max_sensor_values[2] = 3600;
  _min_sensor_values[3] = 1400, _max_sensor_values[3] = 3600;

  while(1){
    if(digitalRead(PB5) == 1) break;
  }
  while(1) pid(1 ,10,25);
  Motor(0,0);
  delay(1000000);
}
void loop() {
  //read(); // ฟั่งชั่น อ่านค่าปุ่มกด และ เซนเซอร์
  /*for(int i=2;i<=5;i++){
    Serial.print(" ");
    Serial.print(analogRead(i)); // อ่านค่าเซนเซอร์ พอท A0  
    
  }
  Serial.println(" ");*/
  //Serial.println(readline());
  /*if (Button == HIGH) { // เมื่อกดปุ่ม
    servo1.write(0); // สั่งเซอร์โวมอเตอร์ ขา D8 ไปองศาที่ 0
    servo2.write(0); // สั่งเซอร์โวมอเตอร์ ขา D9 ไปองศาที่ 0
    servo3.write(0); // สั่งเซอร์โวมอเตอร์ ขา D10 ไปองศาที่ 0
  }
  else {                  // เมื่อไม่กดปุ่ม
    servo1.write(180); // สั่งเซอร์โวมอเตอร์ ขา D8 ไปองศาที่ 180
    servo2.write(180); // สั่งเซอร์โวมอเตอร์ ขา D9 ไปองศาที่ 180
    servo3.write(180); // สั่งเซอร์โวมอเตอร์ ขา D10 ไปองศาที่ 180
  }
 // BuzzerON(); // เปิดเสียง
  //forward(100, 100); // คำสั่งเดินหน้า
  delay(4000);
  rearward(255, 255); // คำสั่งถอยหลัง
  //delay(4000);
  //left(255, 255); // คำสั่งเลี้ยวซ้าย
  //delay(2000);
  //right(255, 255); // คำสั่งเลี้ยวขวา
  //delay(2000);
  //stop(); // หยุดการเคลื่อนที่
  //BuzzerOFF(); // ปิดเสียง*/
}

void pid(float Kp , float Kd , int speed_max)
{
  present_position = readline() / ((numSensor - 1) * 10) ;
  setpoint = 50.0;
  errors = setpoint - present_position;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = Kp * errors + Ki * integral + Kd * derivative;
  previous_error = errors;
  if (output > 100 )output = 100;
  else if (output < -100)output = -100;
  Motor( speed_max - output, speed_max + output);
  delay(1);
}
void pid2(float Kp , float Kd , int speed_max)
{
  
    if (analogRead(PA1) > 500 && analogRead(PA2) > 500 && analogRead(PA3) > 500 && analogRead(PA4) > 500) {
      Motor( speed_max , speed_max );
    }
    present_position = readline() / ((numSensor - 1) * 10) ;
    setpoint = 50.0;
    errors = setpoint - present_position;
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = Kp * errors + Ki * integral + Kd * derivative;
    previous_error = errors;
    if (output > 100 )output = 100;
    else if (output < -100)output = -100;
    
    Motor( speed_max - output, speed_max + output);
    
    delay(1);
  
}
void pid_T2(float Kp , float Kd , int speed_max, long mil)
{
  long a = millis();
  while (a + mil > millis())pid2(Kp , Kd , speed_max);
}
void pid_T(float Kp , float Kd , int speed_max, long mil)
{
  long a = millis();
  while (a + mil > millis())pid(Kp , Kd , speed_max);
}

void pid_B(float Kp , float Kd , int speed_max)
{
  while (1)
  {
    pid(Kp, Kd, speed_max);
    if ((analogRead(PA4) < 300 && analogRead(PA3) < 300) || (analogRead(PA1) < 300 && analogRead(PA2) < 300))
    {
      break;
    }
  }
}
