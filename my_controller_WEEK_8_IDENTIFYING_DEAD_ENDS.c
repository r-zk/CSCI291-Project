#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>  // Include GPS header
#include <webots/light_sensor.h>  // Include Light Sensor header
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_SENSORS 8   // Number of light sensors
#define WALL_THRESHOLD 80.0 // Threshold for proximity sensor readings
#define DEAD_END_THRESHOLD 50.0 // Threshold for considering walls as part of a dead-end
#define DEAD_END_PROXIMITY_THRESHOLD 0.1 // Threshold distance for detecting the same dead-end (in meters)

// Smooth transition factor for motor speeds
#define SMOOTHING_FACTOR 0.07

// Structure to store dead-end coordinates
typedef struct {
  double x;
  double y;
  double z;
} DeadEnd;

// Array to store detected dead-ends
DeadEnd detected_dead_ends[100];
int num_dead_ends = 0;

// Utility function to smoothly adjust speed
double smooth_speed(double current_speed, double target_speed) {
  return current_speed + SMOOTHING_FACTOR * (target_speed - current_speed);
}

// Function to check if the robot is in a dead-end
bool is_dead_end(double prox_values[]) {
  bool front_wall = prox_values[7] > WALL_THRESHOLD;
  bool left_wall = prox_values[1] > DEAD_END_THRESHOLD;
  bool right_wall = prox_values[5] > DEAD_END_THRESHOLD;

  // Dead-end criteria: walls in front and both left and right sides
  if (front_wall && left_wall && right_wall) {
    return true;
  }

  // False positive check: robot is not really in a dead-end but may be at a corner
  if ((left_wall && right_wall) && !front_wall) {
    double front_distance = prox_values[7];
    if (front_distance > WALL_THRESHOLD && front_distance < DEAD_END_THRESHOLD) {
      return true;
    }
  }

  return false;
}

// Function to calculate the Euclidean distance between two points
double distance_between_points(const double *pos1, const double *pos2) {
  return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
}

// Function to check if the detected dead-end is already recorded
bool is_duplicate_dead_end(const double *position) {
  for (int i = 0; i < num_dead_ends; i++) {
    double dist = distance_between_points(position, (const double*)&detected_dead_ends[i]);
    if (dist < DEAD_END_PROXIMITY_THRESHOLD) {
      return true; // This is a duplicate dead-end
    }
  }
  return false;
}

int main(int argc, char **argv) {
  /* Initialize Webots API */
  wb_robot_init();

  // Get motors
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  // Get distance sensors
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  // Get GPS device
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // Get light sensors
  WbDeviceTag light_sensors[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

  double right_speed = MAX_SPEED;
  double left_speed = MAX_SPEED;

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Read proximity sensor values once per time step
    double prox_values[8];
    for (int i = 0; i < 8; ++i) {
      prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    }

    // Wall-following logic based on proximity sensor values
    bool front_wall = prox_values[7] > WALL_THRESHOLD;
    bool left_wall = prox_values[1] > WALL_THRESHOLD;
    bool right_wall = prox_values[5] > WALL_THRESHOLD;

    if (front_wall) {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;  // Turning left to avoid front wall
    } else if (left_wall) {
      right_speed = MAX_SPEED;
      left_speed = MAX_SPEED;   // Following the left wall
    } else {
      right_speed = MAX_SPEED / 8;
      left_speed = MAX_SPEED;   // Moving forward with slight right bias
    }

    // Smooth transition of speeds
    right_speed = smooth_speed(right_speed, MAX_SPEED);
    left_speed = smooth_speed(left_speed, MAX_SPEED);

    // Read GPS position
    const double *position = wb_gps_get_values(gps);

    // Read light sensor values and compute average
    double light_sum = 0.0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      double light_value = wb_light_sensor_get_value(light_sensors[i]);
      light_sum += light_value;
    }
    double average_light_value = light_sum / NUM_SENSORS;

    // Check if the robot is at a dead-end
    bool dead_end = is_dead_end(prox_values);

    // If it is a new dead-end, check for duplicates
    if (dead_end && !is_duplicate_dead_end(position)) {
      // Store the new dead-end and print position and light intensity with [DEAD-END] label
      detected_dead_ends[num_dead_ends].x = position[0];
      detected_dead_ends[num_dead_ends].y = position[1];
      detected_dead_ends[num_dead_ends].z = position[2];
      num_dead_ends++;

      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f [DEAD-END]\n", 
             position[0], position[1], position[2], average_light_value);
    } else if (!dead_end) {
      // Regular print for position and light intensity without [DEAD-END] label
      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f\n", 
             position[0], position[1], position[2], average_light_value);
    }

    // Set the motor velocities
    wb_motor_set_velocity(right_motor, right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
  }

  wb_robot_cleanup();
  return 0;
}
