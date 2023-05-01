#include <micro_ros_arduino.h>
#include <TeensyThreads.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();

rcl_node_t node;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_init_options_t init_options;


#define PWM1 1
#define PWM2 5
#define PWM3 22
#define PWM4 4

#define INA1 20
#define INA2 6
#define INA3 23
#define INA4 3
int ina1 = 0, ina2 = 0, ina3 = 0, ina4 = 0;

#define INB1 21
#define INB2 8
#define INB3 0
#define INB4 2
int inb1 = 0, inb2 = 0, inb3 = 0, inb4 = 0;

#define limit_s 27

#define limit_s1 18 //top
#define limit_s2 19 //under
#define limit_s3 -1

#define pick_ina 40
#define pick_inb 41
#define pick_up_down_ina -1
#define pick_up_down_inb -1
String pick_state = "up";

float pwmm = 0;
float keep_pwmm = 0;
int state = 0;

bool once = false;

#define shoot_motor_under 14
#define shoot_motor_top 15
#define shoot_spring -1

float prePwm = -1;

static uint32_t preTime;

// linear.x = ล้อซ้ายหน้า
// linear.y = ล้อขวาหน้า
// angular.x = ล้อซ้ายหลัง
// angular.y = ล้อขวาหลัง

// linear.z = เครื่องยิง
// angular.z = เครื่องยก

int lim_switch() {
  return int(digitalRead(limit_s));
}

int lim_switch1() {
  return int(digitalRead(limit_s1));
}

int lim_switch2() {
  return int(digitalRead(limit_s2));

}
int lim_switch3() {
  return int(digitalRead(limit_s3));
}


void subscription_callback(const void * msgin)
{
  //------------------------------------------- drive -----------------------------------------//
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if (msg->linear.x > 0) {
    ina1 = 1; inb1 = 0;
  }
  else if (msg->linear.x == 0) {
    ina1 = 1; inb1 = 1;
  }
  else {
    ina1 = 0; inb1 = 1;
  }
  if (msg->linear.y > 0) {
    ina2 = 1; inb2 = 0;
  }
  else if (msg->linear.y == 0) {
    ina2 = 1; inb2 = 1;
  }
  else {
    ina2 = 0; inb2 = 1;
  }
  if (msg->angular.x > 0) {
    ina3 = 1; inb3 = 0;
  }
  else if (msg->angular.x == 0) {
    ina3 = 1; inb3 = 1;
  }
  else {
    ina3 = 0; inb3 = 1;
  }
  if (msg->angular.y > 0) {
    ina4 = 1; inb4 = 0;
  }
  else if (msg->angular.y == 0) {
    ina4 = 1; inb4 = 1;
  }
  else {
    ina4 = 0; inb4 = 1;
  }
  analogWrite(PWM1, abs(msg->linear.x));
  analogWrite(PWM2, abs(msg->linear.y));
  analogWrite(PWM3, abs(msg->angular.x));
  analogWrite(PWM4, abs(msg->angular.y));
  digitalWrite(INA1, ina1);
  digitalWrite(INB1, inb1);
  digitalWrite(INA2, ina2);
  digitalWrite(INB2, inb2);
  digitalWrite(INA3, ina3);
  digitalWrite(INB3, inb3);
  digitalWrite(INA4, ina4);
  digitalWrite(INB4, inb4);

  //------------------------------------------- shoot -----------------------------------------//
  if (prePwm != msg->linear.z)
  {
    prePwm = msg->linear.z;
    once = true;
  }
  if (msg->linear.z == 0)
  {
    digitalWrite(shoot_spring, LOW);
    if (once)
    {
      analogWrite(shoot_motor_under, abs(msg->linear.z));
      analogWrite(shoot_motor_top, abs(msg->linear.z));
    }
  }
  else if (msg->linear.z > 0)
  {
    digitalWrite(shoot_spring, HIGH);
    if (once)
    {
      analogWrite(shoot_motor_under, abs(msg->linear.z));
      analogWrite(shoot_motor_top, abs(msg->linear.z));
    }
  }
  //------------------------------------------- reload -----------------------------------------//
  if (lim_switch1() != true)
  {
    pick_state = "up";
    digitalWrite(pick_ina, HIGH);
    digitalWrite(pick_inb, HIGH);
  }
  else if (lim_switch2() != true)
  {
    pick_state = "down";
    digitalWrite(pick_ina, HIGH);
    digitalWrite(pick_inb, HIGH);
  }
  else
  {
    if (pick_state != "reload")
    {
      pick_state = "up";
      digitalWrite(pick_ina, LOW);
      digitalWrite(pick_inb, HIGH);
    }
  }
  if (msg->angular.z == 1)
  {
    if (pick_state == "up")
    {
      pick_state = "reload";
      digitalWrite(pick_ina, HIGH);
      digitalWrite(pick_inb, LOW);
    }
    else
    {
      pick_state = "reload";
      digitalWrite(pick_ina, LOW);
      digitalWrite(pick_inb, HIGH);
    }
  }
  //------------------------------------------- up & down -----------------------------------------//
  if (msg->angular.z == 10) //up
  {
    digitalWrite(pick_up_down_ina, HIGH);
    digitalWrite(pick_up_down_inb, LOW);
  }
  else if (msg->angular.z == 20) //low
  {
    digitalWrite(pick_up_down_ina, LOW);
    digitalWrite(pick_up_down_inb, HIGH);
  }
  else
  {
    digitalWrite(pick_up_down_ina, HIGH);
    digitalWrite(pick_up_down_inb, HIGH);
  }
}
// idle
// reload
// shoot

void setup() {
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INA4, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(INB4, OUTPUT);

  pinMode(limit_s, INPUT_PULLUP);
  pinMode(limit_s1, INPUT_PULLUP);
  pinMode(limit_s2, INPUT_PULLUP);

  pinMode(shoot_motor_under, OUTPUT);
  pinMode(shoot_motor_top, OUTPUT);
  pinMode(shoot_spring, OUTPUT);

  pinMode(pick_ina, OUTPUT);
  pinMode(pick_inb, OUTPUT);
  pinMode(pick_up_down_ina, OUTPUT);
  pinMode(pick_up_down_inb, OUTPUT);

  delay(1000);

  set_microros_transports();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  rclc_node_init_default(&node, "drive_uros", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "control_drive_topic");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
}
