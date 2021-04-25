#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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

bool twist_flag = false;

double wheel_base_length = 50;
double wheel_base_width = 20;

int twist = 0; // degrees
int vel = 0; // rpm

double x_pos = 0.0;
double y_pos = 0.0;
double current_yaw = 0.0;


BTS7960 motorController(6, 5, 10, 11); //en_l en_r l_pwm r_pwm
Encoder myEnc(2, 3);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Servo myservo;

ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_vel_cb);

ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
tf::TransformBroadcaster odom_broadcaster;

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
    
    if (vel > 0); // only if vehicle is actually moving, as needs forward velocity to turn
        twist = int(cmd_vel_msg.angular.z * 57.296);//convert to degrees
    else;
        twist = 0;
}

void calculate_odom(){
    
    //probably getting degrees and radians mixed up a bit here
    
    float speed, x_vel, y_vel;
            
    speed = (vel / 60) * 0.4273;
    
    x_vel = speed * math.cos(current_yaw);
    y_vel = speed * math.sin(current_yaw);
    
    ros::Duration dt = time_duration;
    
    x_pos += x_dot * dt.toSec();
    y_pos += y_dot * dt.toSec();

    current_yaw += twist * dt.toSec();
    
    
    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_yaw);
     //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
  
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
  
    //set the position
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = x_vel;
    odom.twist.twist.linear.y = y_vel;
    odom.twist.twist.angular.z = twist;

    //publish the message
    odom_pub.publish(odom);
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


