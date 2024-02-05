#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define OBSTACLE_THRESHOLD 1000

// time in [ms] of a simulation step
#define TIME_STEP 64

// main function
int main(int argc, char **argv) {
  // initialise the Webots API
  wb_robot_init();

  // internal variables
  int i; // Added this line

  // int avoid_obstacle_counter = 0;

  // initialise distance sensors
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // initialise motors
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    // wb_motor_set_velocity(wheels[i], 0.0); // initially stop the wheels
  }

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    double ds_values[2];
    for (i = 0; i < 2; i++)
      ds_values[i] = wb_distance_sensor_get_value(ds[i]);

    // check if obstacle is detected
    if (ds_values[0] < OBSTACLE_THRESHOLD || ds_values[1] < OBSTACLE_THRESHOLD) {
      // obstacle detected, turn left 90 degrees
      wb_motor_set_velocity(wheels[0], -1.0);
      wb_motor_set_velocity(wheels[1], 1.0);
      wb_motor_set_velocity(wheels[2], -1.0);
      wb_motor_set_velocity(wheels[3], 1.0);
      
      // move forward
      // wb_motor_set_velocity(wheels[0], 1.0);
      // wb_motor_set_velocity(wheels[1], 1.0);
      // wb_motor_set_velocity(wheels[2], 1.0);
      // wb_motor_set_velocity(wheels[3], 1.0);
    } else {
      // no obstacle, move forward
      wb_motor_set_velocity(wheels[0], 8.0);
      wb_motor_set_velocity(wheels[1], 8.0);
      wb_motor_set_velocity(wheels[2], 8.0);
      wb_motor_set_velocity(wheels[3], 8.0);
    }
  }

  // cleanup Webots API
  wb_robot_cleanup();

  return 0;
}