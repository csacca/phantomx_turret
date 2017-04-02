#include <Arduino.h>

#include <stdio.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <ax12.h>


#define PAN_ID  1
#define TILT_ID 2

#define TILT_MIN  256
#define TILT_MAX  768

#define PAN_MIN   0
#define PAN_MAX   1023

ros::NodeHandle nh;

float pos_to_rad(int pos) {
  float step = 0.00511326929; // (300 degs / 1024 steps) to rad
  float rad = (float) pos * step;
  return rad;
}

float vel_to_rad(int vel) {
  float step = 0.011623892805; // 0.111 rpm to rad/s

  int dir_mask = 0x400;
  int val_mask = 0x3FF;
  float dir = 0.0;

  if (vel & dir_mask == 0) {
    dir = -1.0;
  } else {
    dir = 1.0;
  }

  float rad = dir * ((float) (vel & val_mask)) * step;
  return rad;
}

float eff_to_Nm(int eff) {
  float step = 0.00146484375; // 1.5 Nm / 1024

  int dir_mask = 0x400;
  int val_mask = 0x3FF;
  float dir = 0.0;

  if (eff & dir_mask == 0) {
    dir = -1.0;
  } else {
    dir = 1.0;
  }

  float force = dir * ((float) (eff & val_mask)) * step;
  return force;
}

void setPanTilt(float pan_f, float tilt_f) {

  //FIXME: do proper conversion from rad to steps
  int pan = (int) pan_f;
  int tilt = (int) tilt_f;

  pan = max(min(pan, PAN_MAX), PAN_MIN);
  tilt = max(min(tilt, TILT_MAX), TILT_MIN);

  int PTData[3][2] = {{PAN_ID, pan},
                      {TILT_ID, tilt},
                     };
   dxlSyncWritePosition(PTData, 2);
}

void cmdCallback(const sensor_msgs::JointState& cmd_msg) {
  nh.loginfo("PTU command callback");
  //FIXME: verify num of elems of msg
  //FIXME: verify names 'pan', 'tilt'

  float pan = cmd_msg.position[0];
  float tilt = cmd_msg.position[1];

  digitalWrite(0, HIGH-digitalRead(0));

  //String pan_Str = String(pan, 3);
  char buf[256];
  snprintf(buf, 256, "pan: %04.3f  tilt: %04.3f", pan, tilt);
  //pan_Str.toCharArray(buf, 256);

  nh.loginfo(buf);

  setPanTilt(pan, tilt);
}

ros::Subscriber<sensor_msgs::JointState> sub_cmd("cmd", &cmdCallback);

sensor_msgs::JointState js_msg;
ros::Publisher pub_state("state", &js_msg);


void publishCurrentState() {
  sensor_msgs::JointState js_msg;

  int pan_pos = dxlGetPosition(PAN_ID);
  int tilt_pos = dxlGetPosition(TILT_ID);
  int pan_speed = dxlGetSpeed(PAN_ID);
  int tilt_speed = dxlGetSpeed(TILT_ID);
  int pan_torque = dxlGetTorque(PAN_ID);
  int tilt_torque = dxlGetTorque(TILT_ID);


  char pan_str[] = "pan";
  char tilt_str[] = "tilt";
  char* name[] = {pan_str, tilt_str};

  float pos[] = {pos_to_rad(pan_pos), pos_to_rad(tilt_pos)};

  float vel[] = {vel_to_rad(pan_speed), vel_to_rad(tilt_speed)};

  float eff[] = {eff_to_Nm(pan_torque), eff_to_Nm(tilt_torque)};

  js_msg.name = name;
  js_msg.position = pos;
  js_msg.velocity = vel;
  js_msg.effort = eff;

  js_msg.name_length = 2;
  js_msg.position_length = 2;
  js_msg.velocity_length = 2;
  js_msg.effort_length = 2;

  pub_state.publish(&js_msg);
}

void setup() {
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);

  dxlInit(1000000);

  nh.initNode();
  nh.advertise(pub_state);
  nh.subscribe(sub_cmd);
}

void loop() {
  // periodically publish state
  // spinOnce
  publishCurrentState();
  nh.spinOnce();
  delay(100);
}
