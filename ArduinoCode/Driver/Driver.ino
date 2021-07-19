#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include <Servo.h>    //servo addition
ros::NodeHandle nh;

int cam_servo_pin = 11;
Servo cam_servo; //servo add
int cur_angle;
int new_angle;
int pos = 0;
#define LED_BUILTIN 13


#define L_MOTOR_PWM 6
#define L_MOTOR_DIR1 8
#define L_MOTOR_DIR2 9
#define R_MOTOR_PWM 5
#define R_MOTOR_DIR1 10
#define R_MOTOR_DIR2 7

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int dir1_pin,
                unsigned int dir2_pin) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    //Serial.print(wheel_power.data);
    if( factor >= 0 ) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);  
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(255 * (-1.0f * factor)));
    }   
}

void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
  
    turnWheel( wheel_power, R_MOTOR_PWM, R_MOTOR_DIR1, R_MOTOR_DIR2 );
    
}

void leftWheelCb( const std_msgs::Float32 &wheel_power ) {

    turnWheel( wheel_power, L_MOTOR_PWM, L_MOTOR_DIR1, L_MOTOR_DIR2 );
}

void servoCb( const std_msgs::Int16 &angle)
{
  new_angle = angle.data;
  if(new_angle>(cur_angle-90)){
    
  for (pos = cur_angle;pos<new_angle+90; pos+=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
  else if((new_angle<(cur_angle-90))&&(new_angle>=-45)){
  for (pos = cur_angle;pos>new_angle+90; pos-=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );
ros::Subscriber<std_msgs::Int16> sub_servo("servo", &servoCb);

void setup()
{
  cam_servo.attach(cam_servo_pin);
  cur_angle=90;
  cam_servo.write(cur_angle);
  // initialize LED digital pin as an output.
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
  nh.subscribe(sub_servo);

  delay(20);
}

void loop()
{
   //nh.loginfo("Log Me");
   nh.spinOnce();
  
   // wait for a second
   delay(20);
   
}
