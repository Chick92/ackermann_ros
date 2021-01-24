#include "BTS7960.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

double last_time = micros();
double tick_time = 0;
long old_position = 0;

double Setpoint, Output;
double Input = 0;
double Kp=1, Ki=0.5, Kd=0.01;

BTS7960 motorController(6, 5, 10, 11); //en_l en_r l_pwm r_pwm
Encoder myEnc(2, 3);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo myservo;

double calculate_rpm(){
   long new_position = myEnc.read();
   long position_change;
   double RPM;

   if (new_position != old_position) {
       tick_time = (micros() - last_time);
       position_change = old_position - new_position;
       RPM = 1 / ((double(tick_time / position_change) * 18538)/1000000/60); //10041 18538 = ticks per rev, 1 rev = 42.73cm
       old_position = new_position;
       last_time = micros();   
   }
   else{
       RPM = 0.0;
   }
   return RPM;
}


void setup(){
   Serial.begin(9600);
   myservo.attach(9);
   myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
   motorController.Enable();
   Setpoint = 6;
   myPID.SetSampleTime(20);
   myPID.SetMode(AUTOMATIC);
}

void loop(){
   //double RPM;
   //int speed;
   Input = calculate_rpm();
   myPID.Compute();
   if (Setpoint >= 0){
      motorController.TurnLeft(Output);
   }
   else{
      motorController.TurnRight(Output);
   }
   Serial.print("PWM:");
   Serial.print(Output);
   Serial.print(",");
   Serial.print("RPM:");
   Serial.println(Input);
   
   //motorController.Stop();
   //motorController.Disable();
}
