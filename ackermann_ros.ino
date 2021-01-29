#include <BTS7960.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

double last_time = micros();
double tick_time = 0;
long old_position = 0;

double Setpoint, Output;
double Input = 0;
double Kp=1.3, Ki=15, Kd=0.01;
String input_vel = "";
String input_twist = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool twist_flag = false;

double wheel_base_length = 0.34;
double wheel_base_width = 0.22;
double wheel_circumfrance = 0.4273;

double twist = 0.2;
double vel = 0.25;


BTS7960 motorController(6, 4, 5, 11); //en_l en_r l_pwm r_pwm
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

double convert_steering_angle(double twist, double vel){
    double phi;
    double phi_deg;
    phi = atan(-twist/(vel/wheel_base_length));
    phi_deg = (phi * 57296 / 1000) + 90;
    return phi_deg;
}

double covert_vel_rpm(double vel){
  double RPM; 
  RPM = (vel / wheel_circumfrance) * 60;
  return RPM;
}

void setup(){
   Serial.begin(9600);
   //Serial.println("Hello Ben");
   //input_vel.reserve(100);
   //input_twist.reserve(100);
   myservo.attach(9);
   
   myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
   motorController.Enable();
   Setpoint = 0;
   myPID.SetSampleTime(20);
   myPID.SetMode(AUTOMATIC);
   myPID.SetOutputLimits(-255,255);
}

void loop(){
  char debug = 1;
  double OutputMag;

  

  myservo.write(convert_steering_angle(twist, vel));
        
  Setpoint = covert_vel_rpm(vel);
  Input = calculate_rpm();
  myPID.Compute();


  if (0 > Setpoint){
    motorController.TurnLeft(-Output); 
  }
  else{
    motorController.TurnRight(Output); // forwards
  }
       
  if (debug == 1){
    Serial.print("Output_PWM:");
    Serial.print(Output);
    Serial.print(",");
    Serial.print("Input_RPM:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("Setpoint:");
    Serial.println(Setpoint);
  }
        
  if (debug == 2){
    Serial.print("Ticks:");
    Serial.println(myEnc.read());
  }
  
}
