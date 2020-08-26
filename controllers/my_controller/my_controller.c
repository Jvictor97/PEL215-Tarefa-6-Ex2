#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define SPEED 6.28

WbDeviceTag motors[4];

void setSpeed(double* speeds) {
  for (int i = 0; i < 4; i++)
    wb_motor_set_velocity(motors[i], speeds[i]);
}

void front() {
  double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  setSpeed(speeds);
}

void back() {
  double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  setSpeed(speeds);
}

void left() {
  double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  setSpeed(speeds);
}

void right() {
  double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  setSpeed(speeds);
}

void frontLeft() {
  double speeds[4] = {SPEED, 0.0, 0.0, SPEED};
  setSpeed(speeds);
}

void backLeft() {
  double speeds[4] = {0.0, -SPEED, -SPEED, 0.0};
  setSpeed(speeds);  
}

void backRight() {
  double speeds[4] = {-SPEED, 0.0, 0.0, -SPEED};
  setSpeed(speeds);
}

void frontRight() {
  double speeds[4] = {0.0, SPEED, SPEED, 0.0};
  setSpeed(speeds);
}

int main(int argc, char **argv) {
  wb_robot_init();
   
  motors[0] = wb_robot_get_device("wheel1"); // front right
  motors[1] = wb_robot_get_device("wheel2"); // front left
  motors[2] = wb_robot_get_device("wheel3"); // back right
  motors[3] = wb_robot_get_device("wheel4"); // back left
  
  wb_motor_set_position(motors[0], INFINITY);
  wb_motor_set_position(motors[1], INFINITY);
  wb_motor_set_position(motors[2], INFINITY);
  wb_motor_set_position(motors[3], INFINITY);
    
  while (wb_robot_step(TIME_STEP) != -1) {   
    frontLeft();
  };

  wb_robot_cleanup();

  return 0;
}
