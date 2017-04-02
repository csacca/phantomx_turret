#include <Arduino.h>

#include <stdio.h>
#include <math.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <ax12.h>


#define PAN_ID  1
#define TILT_ID 2

#define PAN_POS_MIN   0
#define PAN_POS_MAX   1023

#define PAN_VEL_MIN   0
#define PAN_VEL_MAX   1023

#define PAN_EFF_MIN   0
#define PAN_EFF_MAX   1023

#define TILT_POS_MIN  256
#define TILT_POS_MAX  768

#define TILT_VEL_MIN  0
#define TILT_VEL_MAX  1023

#define TILT_EFF_MIN  0
#define TILT_EFF_MAX  1023


#define DEFAULT_VEL  0
#define DEFAULT_EFF  1023

ros::NodeHandle nh;

float pos_to_rad(int pos) {
  float step = 0.00511326929; // (300 degs / 1024 steps) to rad
  float rad = (float) pos * step;
  return rad;
}

int rad_to_pos(float rad) {
  float step = 1.0/0.00511326929;
  int pos = round(rad*step);
  return pos;
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

int rad_to_vel(float rad) {
  float step = 1.0/0.011623892805;

  int dir_mask = 0x400;
  int val_mask = 0x3FF;

  int vel = abs(round(rad*step));
  vel = vel & val_mask;

//  if (rad >= 0) {
//    vel = vel | dir_mask;
//  }

  return vel;
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

int Nm_to_eff(float force) {
  float step = 1.0/0.00146484375;

  int dir_mask = 0x400;
  int val_mask = 0x3FF;

  int eff = abs(round(force*step));
  eff = eff & val_mask;

//  if (force >= 0) {
//    eff = eff | dir_mask;
//  }

  return eff;

}

void setRawPanTiltPos(int pan, int tilt) {
  pan = max(min(pan, PAN_POS_MAX), PAN_POS_MIN);
  tilt = max(min(tilt, TILT_POS_MAX), TILT_POS_MIN);

  int PTData[3][2] = {{PAN_ID, pan},
                      {TILT_ID, tilt},
                     };

  dxlSyncWritePosition(PTData, 2);

  char buf[256];
  snprintf(buf, 256, "Set Position - pan: %d  tilt: %d", pan, tilt);
  nh.loginfo(buf);

}

void setPanTiltPos(float pan_rad, float tilt_rad) {
  int pan = rad_to_pos(pan_rad);
  int tilt = rad_to_pos(tilt_rad);

  setRawPanTiltPos(pan, tilt);
}

void setRawPanTilt(int pan_pos, int tilt_pos, int pan_vel, int tilt_vel, int pan_eff, int tilt_eff) {
  pan_pos = max(min(pan_pos, PAN_POS_MAX), PAN_POS_MIN);
  tilt_pos = max(min(tilt_pos, TILT_POS_MAX), TILT_POS_MIN);
  pan_vel = max(min(pan_vel, PAN_VEL_MAX), PAN_VEL_MIN);
  tilt_vel = max(min(tilt_vel, TILT_VEL_MAX), TILT_VEL_MIN);
  pan_eff = max(min(pan_eff, PAN_VEL_MAX), PAN_VEL_MIN);
  tilt_eff = max(min(tilt_eff, TILT_VEL_MAX), TILT_VEL_MIN);

  int pos_data[3][2] = {{PAN_ID, pan_pos},
                        {TILT_ID, tilt_pos},
                       };

  int vel_data[3][2] = {{PAN_ID, pan_vel},
                        {TILT_ID, tilt_vel},
                       };

  int eff_data[3][2] = {{PAN_ID, pan_eff},
                        {TILT_ID, tilt_eff},
                       };

  dxlSyncWrite(pos_data, 2, AX_GOAL_POSITION_L,2);
  dxlSyncWrite(vel_data, 2, AX_GOAL_SPEED_L,2);
  dxlSyncWrite(eff_data, 2, AX_TORQUE_LIMIT_L,2);


  char buf[256];
  snprintf(buf, 256, "Set Position - pan: %d  tilt: %d", pan_pos, tilt_pos);
  nh.loginfo(buf);

  snprintf(buf, 256, "Set Speed - pan: %d  tilt: %d", pan_vel, tilt_vel);
  nh.loginfo(buf);

  snprintf(buf, 256, "Set Torque - pan: %d  tilt: %d", pan_eff, tilt_eff);
  nh.loginfo(buf);

}

void setPanTilt(float pan_pos_rad, float tilt_pos_rad, float pan_vel_rad, float tilt_vel_rad, float pan_eff_Nm, float tilt_eff_Nm) {
  int pan_pos = rad_to_pos(pan_pos_rad);
  int tilt_pos = rad_to_pos(tilt_pos_rad);
  int pan_vel = rad_to_vel(pan_vel_rad);
  int tilt_vel = rad_to_vel(tilt_vel_rad);
  int pan_eff = Nm_to_eff(pan_eff_Nm);
  int tilt_eff = Nm_to_eff(tilt_eff_Nm);

  setRawPanTilt(pan_pos, tilt_pos, pan_vel, tilt_vel, pan_eff, tilt_eff);
}

void cmdCallback(const sensor_msgs::JointState& cmd_msg) {
  nh.loginfo("PTU command callback");
  //FIXME: verify num of elems of msg
  //FIXME: verify names 'pan', 'tilt'

  float pan_pos = cmd_msg.position[0];
  float tilt_pos = cmd_msg.position[1];
  float pan_vel = cmd_msg.velocity[0];
  float tilt_vel = cmd_msg.velocity[1];
  float pan_eff = cmd_msg.effort[0];
  float tilt_eff = cmd_msg.effort[1];

  digitalWrite(0, HIGH-digitalRead(0));

  char buf[256];
  snprintf(buf, 256, "Recieved command - pan pos: %04.3f  tilt pos: %04.3f", pan_pos, tilt_pos);
  nh.loginfo(buf);

  setPanTilt(pan_pos, tilt_pos, pan_vel, tilt_vel, pan_eff, tilt_eff);
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
