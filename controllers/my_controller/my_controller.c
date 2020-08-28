#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64
#define SPEED 6.28
#define DIAGONAL_ZERO -1.13425

WbDeviceTag motors[4];

void setSpeed(double* speeds) {
  for (int i = 0; i < 4; i++)
    wb_motor_set_velocity(motors[i], speeds[i]);
}

void stop() {
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  setSpeed(speeds);
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
  double speeds[4] = {SPEED, DIAGONAL_ZERO, DIAGONAL_ZERO, SPEED};
  setSpeed(speeds);
}

void backLeft() {
  double speeds[4] = {-DIAGONAL_ZERO, -SPEED, -SPEED, -DIAGONAL_ZERO};
  setSpeed(speeds);  
}

void backRight() {
  double speeds[4] = {-SPEED, -DIAGONAL_ZERO, -DIAGONAL_ZERO, -SPEED};
  setSpeed(speeds);
}

void frontRight() {
  double speeds[4] = {DIAGONAL_ZERO, SPEED, SPEED, DIAGONAL_ZERO};
  setSpeed(speeds);
}

bool equal(double firstValue, double secondValue) {
  const double tolerance = 0.1;
  
  return fabs(firstValue - secondValue) < tolerance;
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
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    double x = values[0], z = values[2];
    
    if (equal(x, 0.0) && equal(z, 0.0))
      frontLeft();
    else if (equal(x, 1.25) && equal(z, -1.25))
      left();
    else if (equal(x, 1.25) && equal(z, -2.5))
      backLeft();
    else if (equal(x, 0.0) && equal(z, -3.75))
      back();
    else if (equal(x, -1.25) && equal(z, -3.75))
      backRight();
    else if (equal(x, -2.5) && equal(z, -2.5))
      right();  
    else if (equal(x, -2.5) && equal(z, -1.25))
      frontRight(); 
    else if (equal(x, -1.25) && equal(z, 0.0))
      front();
  };

  wb_robot_cleanup();

  return 0;
}
