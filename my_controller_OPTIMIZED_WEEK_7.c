#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>  // Include GPS header
#include <webots/light_sensor.h>  // Include Light Sensor header
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_SENSORS 8   // Number of light sensors

// Smooth transition factor for motor speeds
#define SMOOTHING_FACTOR 0.07

// Utility function to smoothly adjust speed
double smooth_speed(double current_speed, double target_speed) {
  return current_speed + SMOOTHING_FACTOR * (target_speed - current_speed);
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
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
    // Read proximity sensors for wall detection
    bool right_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
    bool right_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[1]) > 80;
    
    // Wall-following logic
    if (front_wall == true) {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;
    }
    else if (left_wall) {
      right_speed = MAX_SPEED;
      left_speed = MAX_SPEED;
    } else {
      right_speed = MAX_SPEED / 8;
      left_speed = MAX_SPEED;
    }
    
    // Smooth transition of speeds for smooth movement
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

    // Print the current position and average light intensity
    printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f\n", position[0], position[1], position[2], average_light_value);

    // Set the motor velocities
    wb_motor_set_velocity(right_motor, right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
  }

  wb_robot_cleanup();
  return 0;
}
