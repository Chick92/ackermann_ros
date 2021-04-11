#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

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

int twist = 0;
int vel = 0;


BTS7960 motorController(6, 5, 10, 11); //en_l en_r l_pwm r_pwm
Encoder myEnc(2, 3);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo myservo;

ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_vel_cb);

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
    phi = arctan(twist/(vel/wheel_base_length)) + 90;
    return phi;
}

void setup(){
    input_vel.reserve(100);
    input_twist.reserve(100);
    myservo.attach(9);
    myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
    motorController.Enable();
    Setpoint = 0;
    myPID.SetSampleTime(20);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255,255);
    nh.initNode();
    nh.subscribe(sub);
}


void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg){
    //0.4273 is circumfrance
    //convert vel to rpm
    //convert steering angle from radians to degrees
    vel  = int((cmd_vel_msg.linear.x / 0.4273) * 60);
    twist = int(cmd_vel_msg.angular.z * 57.296);//convert to degrees
}



void loop(){
    
    myservo.write(convet_steering_angle(twist,vel));
        
    Setpoint = vel;
    Input = calculate_rpm();
    myPID.Compute();
        
    if (Output >= 0){
        motorController.TurnLeft(Output);
    }
    else{
        motorController.TurnRight(sqrt(Output * Output));
    }
        
    nh.spinOnce();
}


