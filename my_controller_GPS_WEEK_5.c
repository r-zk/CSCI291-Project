#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define START_PROXIMITY_THRESHOLD 0.05
#define MIN_DISTANCE_THRESHOLD 0.2  // Minimum distance the robot must move from the start to begin considering it has started moving

// Helper function to calculate the Euclidean distance between two positions
double calculate_distance(const double *pos1, const double *pos2) {
  return sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[2] - pos2[2], 2));  // Use x and z for 2D
}

int main() {
  wb_robot_init();

  // Initialize devices
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int i = 0; i < 8; ++i) {
    sprintf(prox_sensor_name, "ps%d", i);
    prox_sensors[i] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }

  // Record the initial position
  const double *initial_position = wb_gps_get_values(gps);
  printf("Starting position: x=%.2f, z=%.2f\n", initial_position[0], initial_position[2]);

  double right_speed = MAX_SPEED;
  double left_speed = MAX_SPEED;

  bool started_moving = false;
  bool completed_round = false;

  while (wb_robot_step(TIME_STEP) != -1) {
    // Read the current position
    const double *current_position = wb_gps_get_values(gps);

    // Calculate the distance to the starting position
    double distance_to_start = calculate_distance(current_position, initial_position);

    // Check if the robot has started moving away from the starting position
    if (!started_moving && distance_to_start > MIN_DISTANCE_THRESHOLD) {
      started_moving = true;
      printf("Robot has started moving.\n");
    }

    // Check if the robot has completed a full round of the maze
    if (started_moving && !completed_round && distance_to_start < START_PROXIMITY_THRESHOLD) {
      completed_round = true;
      printf("Completed one round of the maze.\n");
    }

    // If the robot has completed the round and is back at the start, stop
    if (completed_round && distance_to_start < START_PROXIMITY_THRESHOLD) {
      printf("Returned to the starting position.\n");
      break;  // Stop the loop
    }

    // Sensor readings
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[1]) > 80;

    // Right-hand wall-following algorithm
    if (front_wall) {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;  // Turn left
    } else if (left_wall) {
      right_speed = MAX_SPEED;
      left_speed = MAX_SPEED;  // Move straight
    } else {
      right_speed = MAX_SPEED / 8;
      left_speed = MAX_SPEED;  // Turn right to find wall
    }

    wb_motor_set_velocity(right_motor, right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
  }

  // Stop the robot
  wb_motor_set_velocity(right_motor, 0);
  wb_motor_set_velocity(left_motor, 0);

  // Cleanup
  wb_robot_cleanup();

  return 0;
}
