#include <BTS7960.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

double last_time = micros();
double tick_time = 0;
long old_position = 0;

double Setpoint, Output;
double Input = 0;
double Kp=1, Ki=0.5, Kd=0.01;
String input_vel = "";
String input_twist = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool twist_flag = false;

double wheel_base_length = 50;
double wheel_base_width = 20;

int twist = 90;
int vel = 0;


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

int convert_steering_angle(int twist, int vel){
    int phi;
    phi = atan(twist/(vel/wheel_base_length)) + 90;
    return phi;
}

void setup(){
   Serial.begin(9600);
   //Serial.println("Hello Ben");
   input_vel.reserve(100);
   input_twist.reserve(100);
   myservo.attach(9);
   
   myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
   motorController.Enable();
   Setpoint = 0;
   myPID.SetSampleTime(20);
   myPID.SetMode(AUTOMATIC);
   myPID.SetOutputLimits(-255,255);
}

void loop(){
    char debug = 2;
   //double RPM;
   //int speed;
   // int twist = 90;
   // int vel = 0;
    
    //while(1){
        
        if (stringComplete) {
            Serial.println(input_vel);
            Serial.println(input_twist);
            vel = input_vel.toInt();
            twist = input_twist.toInt();
            input_vel = "";
            input_twist = "";
            stringComplete = false;
        }
    
        myservo.write(convert_steering_angle(twist, vel));
        
        Setpoint = vel;
        Input = calculate_rpm();
        myPID.Compute();
        
        if (Output >= 0){
            motorController.TurnLeft(Output);
        }
        else{
            motorController.TurnRight(sqrt(Output * Output));
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
        //motorController.Stop();
        //motorController.Disable();
    //}
}

void serialEvent() {
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        
        if (inChar == '\n') {
            Serial.println("string complete");
            stringComplete = true;
            break;
        
        if (inChar =='t'){
            twist_flag = true;
            break;
        }
        
        if (twist_flag == false){
            input_vel += inChar;
        }
        else{
            input_twist += inChar;
        }
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about:
        
        }
    }
}
