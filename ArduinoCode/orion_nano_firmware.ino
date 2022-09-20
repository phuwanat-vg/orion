#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
#define LED_BUILTIN 13


#define L_MOTOR_PWM 6
#define L_MOTOR_DIR1 8
#define L_MOTOR_DIR2 9
#define R_MOTOR_PWM 5
#define R_MOTOR_DIR1 10
#define R_MOTOR_DIR2 7


const float wheel_radius = 0.0321;
const float wheel_base = 0.1175;
const float min_speed = 6.23;
const float max_speed = 9.345;

float v = 0.0;
float w = 0.0;
float vr = 0.0;
float vl = 0.0;
float min_pwm = 0.4;

/*--------------------------------Ultrasonic Sensor--------------------*/
const int trig = 3;
const int echo = 11;
float range = 0.0;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasonic_range", &range_msg);
char frameid[] = "/ultrasonic";
/*---------------------------------------------------------------------*/

void turnWheel( const float wheel_power,
                unsigned int pwm_pin,
                unsigned int dir1_pin,
                unsigned int dir2_pin) {
  float factor = max(min(wheel_power, 1.0f), -1.0f);
  //Serial.print(wheel_power.data);
  if ( factor > 0.0 ) {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);
    analogWrite(pwm_pin, (unsigned int)(255 * factor));
  }else if( factor < 0.0 ){
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
    analogWrite(pwm_pin, (unsigned int)(255 * (-1.0f * factor)));
  } else {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, LOW);
    analogWrite(pwm_pin, 0);
  }
}

void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
  
    turnWheel( wheel_power.data, R_MOTOR_PWM, R_MOTOR_DIR1, R_MOTOR_DIR2 );
    
}

void leftWheelCb( const std_msgs::Float32 &wheel_power ) {

    turnWheel( wheel_power.data, L_MOTOR_PWM, L_MOTOR_DIR1, L_MOTOR_DIR2 );
}

float vel_to_power(float vel) {
  float factor = 0.0;
  float power = 0.0;

  if (vel != 0){
    factor = (abs(vel) - min_speed) / (max_speed - min_speed);
  power = (factor * (1.0 - min_pwm)) + min_pwm;
  }
  else if (vel == 0){
    power = 0.0;
  }
  

  if (vel < 0) {
    power = power * -1;
  }
  return power;
}

void velCb( const geometry_msgs::Twist &vel_data ) {

  float R = wheel_radius;
  float L = wheel_base;

  v = vel_data.linear.x;
  w = vel_data.angular.z;

  vr = ((2.0 * v) + (2.0 * w)) / (2.0 * R);
  vl = ((2.0 * v) - (2.0 * w)) / (2.0 * R);

  vr = max(min(vr, max_speed), -max_speed);
  vl = max(min(vl, max_speed), -max_speed);

  turnWheel(vel_to_power(vl), L_MOTOR_PWM, L_MOTOR_DIR1, L_MOTOR_DIR2);
  turnWheel(vel_to_power(vr), R_MOTOR_PWM, R_MOTOR_DIR1, R_MOTOR_DIR2);
}


void ultrasonic() {
  long duration;

  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  pinMode(echo, INPUT);
  duration = pulseIn(echo, HIGH);

  range = microsecondsToCentimeters(duration);

  Serial.println(range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  range_msg.range = range*0.01;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  delay(5);



}

float microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29.0 / 2.0;
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel",
    &velCb );

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_DIR1, OUTPUT);
  pinMode(L_MOTOR_DIR2, OUTPUT);
  
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR1, OUTPUT);
  pinMode(R_MOTOR_DIR2, OUTPUT);
  
  // Init motors to stop
  digitalWrite(L_MOTOR_DIR1, LOW);
  digitalWrite(L_MOTOR_DIR2, LOW);
  digitalWrite(R_MOTOR_DIR1, LOW);
  digitalWrite(R_MOTOR_DIR2, LOW);
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);

  //ROS setup
  nh.initNode();
  //Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
   nh.subscribe(sub_vel);
   nh.advertise(pub_range);
  delay(20);
}

void loop()
{
   ultrasonic();
   nh.spinOnce();
  
   // wait for a second
   delay(20);
   
}
