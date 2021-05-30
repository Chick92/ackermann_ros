#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h> 
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include <BTS7960.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>

#define wheel_base_length 0.34
#define wheel_base_width 0.22
#define wheel_circumfrance 0.4273

double last_time = micros();
double tick_time = 0;
long old_position = 0;

double Setpoint, Output;
double Input = 0;

bool twist_flag = false;

int twist = 0; // degrees
int vel = 0; // rpm

double x_pos = 0.0;
double y_pos = 0.0;
double current_yaw = 0.0;
unsigned long p_time = millis();


BTS7960 motorController(6, 4, 5, 11); //en_l en_r l_pwm r_pwm
Encoder myEnc(2, 3);

PID myPID(&Input, &Output, &Setpoint, 1.0, 0.5, 0.01, DIRECT); //P,I,D

Servo myservo;
Servo lights;



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



void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg){
    //0.4273 is circumfrance
    //convert vel to rpm
    //convert steering angle from radians to degrees
    vel  = int((cmd_vel_msg.linear.x / 0.4273) * 60);
    
    if (vel > 0){// only if vehicle is actually moving, as needs forward velocity to turn
        twist = int(cmd_vel_msg.angular.z * 57.296);//convert to degrees
    }
    else{
        twist = 0;
    }    
}


void setup(){
    myservo.attach(9);
    myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
	
    motorController.Enable();
    Setpoint = 0;
    myPID.SetSampleTime(20);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
	
	
}


void loop(){

    myservo.write(convert_steering_angle(twist,vel));
    Setpoint = vel;
    Input = calculate_rpm();
    myPID.Compute();
        
    if (Output >= 0){
        motorController.TurnLeft(Output);
    }
    else{
        motorController.TurnRight(sqrt(Output * Output));
    }
}
