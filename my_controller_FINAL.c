//including libraries
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

//defining constants
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_SENSORS 8
#define WALL_THRESHOLD 80.0
#define DEAD_END_THRESHOLD 50.0
#define DEAD_END_PROXIMITY_THRESHOLD 0.1
#define MAX_POSITIONS 5000  //prevent overflow, limit the number of recorded positions
#define LIGHT_INTENSITY_THRESHOLD 50.0  //threshold for high light intensity

#define SMOOTHING_FACTOR 0.07 //smoothing motor speed transitions

//structs and global variables
typedef struct {
  double x;
  double y;
  double z;
  double light_intensity;
} PositionLightIntensity;


PositionLightIntensity recorded_positions[MAX_POSITIONS];  //array to store visited positions
int num_recorded_positions = 0; //count of dead ends detected

//struct
typedef struct {
  double x;
  double y;
  double z;
} DeadEnd;

DeadEnd detected_dead_ends[25]; //array to store detected dead ends
int num_dead_ends = 0; //count of dead ends detected

double initial_position[3] = {0.0, 0.0, 0.0}; //initial position of robot

//utilities
//smoothly adjusts motor speed
double smooth_speed(double current_speed, double target_speed) {
  return current_speed + SMOOTHING_FACTOR * (target_speed - current_speed);
}

//calculates distance between two 3d points
double distance_between_points(const double *pos1, const double *pos2) {
  return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
}

//checks if robot is back at starting point
bool is_back_at_start() {
  const double *current_position = wb_gps_get_values(wb_robot_get_device("gps"));
  double dist = distance_between_points(current_position, initial_position);
  return dist < 0.5;  //allow up to 0.5 meters of error
}

//checks if current sensor readings indicate dead end
bool is_dead_end(double prox_values[]) {
  bool front_wall = prox_values[7] > WALL_THRESHOLD;
  bool left_wall = prox_values[1] > DEAD_END_THRESHOLD;
  bool right_wall = prox_values[5] > DEAD_END_THRESHOLD;

//if wall are on three sides
  if (front_wall && left_wall && right_wall) {
    return true;
  }

//for narrow passage
  if ((left_wall && right_wall) && !front_wall) {
    double front_distance = prox_values[7];
    if (front_distance > WALL_THRESHOLD && front_distance < DEAD_END_THRESHOLD) {
      return true;
    }
  }

  return false;
}

//checks if a current position is a previously recorded dead end
bool is_duplicate_dead_end(const double *position) {
  for (int i = 0; i < num_dead_ends; i++) {
    double dist = distance_between_points(position, (double[]){detected_dead_ends[i].x, detected_dead_ends[i].y, detected_dead_ends[i].z});
    if (dist < DEAD_END_PROXIMITY_THRESHOLD) {
      return true;
    }
  }
  return false;
}

//checks if robot has reached position with maximum light intensity
bool is_at_max(const double *target_position, const PositionLightIntensity *best_position) {
  double dist = distance_between_points(target_position, (double[]){best_position->x, best_position->y, best_position->z});
  return dist < 0.01;  //allow up to 0.5 meters of error
}

//main
int main(int argc, char **argv) {
    //initialise robot
  wb_robot_init();

//set up motors
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

//set up proximity sensors
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

//set up gps
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

//set up light sensors
  WbDeviceTag light_sensors[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

//initialise motor speeds
  double right_speed = MAX_SPEED;
  double left_speed = MAX_SPEED;

  const double *initial_gps_position = NULL;
  while (true) {
    initial_gps_position = wb_gps_get_values(gps);
    if (initial_gps_position != NULL && !isnan(initial_gps_position[0]) && !isnan(initial_gps_position[1]) && !isnan(initial_gps_position[2])) {
      break;
    }
    wb_robot_step(TIME_STEP);
  }

  initial_position[0] = initial_gps_position[0];
  initial_position[1] = initial_gps_position[1];
  initial_position[2] = initial_gps_position[2];

  printf("Initial Position: (%.2f, %.2f, %.2f)\n--Exploring Maze--\n", initial_position[0], initial_position[1], initial_position[2]);

//exploration logic
  double start_check_time = wb_robot_get_time() + 27;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (wb_robot_get_time() >= start_check_time) {
      if (is_back_at_start()) {
        printf("Back at start. Stopping.\n--Completed a Round of the Maze--\n");

        //pause for brief moment 
        double pause_time = wb_robot_get_time() + 3.0; 
        while (wb_robot_get_time() < pause_time) {
          wb_robot_step(TIME_STEP);
        }

        //find position with highest light intensity
        double max_light_intensity = -1.0;
        PositionLightIntensity best_position = {0.0, 0.0, 0.0, 0.0};  //initialize best position
        for (int i = 0; i < num_recorded_positions; i++) {
          if (recorded_positions[i].light_intensity > max_light_intensity && recorded_positions[i].light_intensity > LIGHT_INTENSITY_THRESHOLD) {
            max_light_intensity = recorded_positions[i].light_intensity;
            best_position = recorded_positions[i];
          }
        }

        //if a valid best position is found move towards it 
        if (max_light_intensity > -1.0) {
          printf("Moving to position with highest light intensity: (%.2f, %.2f, %.2f)\n", best_position.x, best_position.y, best_position.z);

          const double *target_position = wb_gps_get_values(gps);
          while (distance_between_points(target_position, (double *)&best_position) > 0.2) {
            target_position = wb_gps_get_values(gps);

            //wall-following behavior
            double prox_values[8];
            for (int i = 0; i < 8; ++i) {
              prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
            }

            bool front_wall = prox_values[7] > WALL_THRESHOLD;
            bool left_wall = prox_values[1] > WALL_THRESHOLD;

            if (front_wall) {
              right_speed = MAX_SPEED;
              left_speed = -MAX_SPEED;
            } else if (left_wall) {
              right_speed = MAX_SPEED;
              left_speed = MAX_SPEED;
            } else {
              right_speed = MAX_SPEED / 8;
              left_speed = MAX_SPEED;
            }

            right_speed = smooth_speed(right_speed, MAX_SPEED);
            left_speed = smooth_speed(left_speed, MAX_SPEED);

            wb_motor_set_velocity(right_motor, right_speed);
            wb_motor_set_velocity(left_motor, left_speed);

            //check if robot has reached target position
            if (is_at_max(target_position, &best_position)) {
              break;
            }

            wb_robot_step(TIME_STEP);
          }

          //once arrived at the target stop the motors
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          
          printf("===Reached the position with the highest light intensity: (%.2f, %.2f, %.2f)===\n", best_position.x, best_position.y, best_position.z);
        }

        break;
      } else {
        start_check_time = wb_robot_get_time() + 27;
      }
    }

    //regular wall-following behavior when not moving toward the best light intensity
    //right hand wall following algorithm
    double prox_values[8];
    for (int i = 0; i < 8; ++i) {
      prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    }

    bool front_wall = prox_values[7] > WALL_THRESHOLD;
    bool left_wall = prox_values[1] > WALL_THRESHOLD;

    if (front_wall) {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;
    } else if (left_wall) {
      right_speed = MAX_SPEED;
      left_speed = MAX_SPEED;
    } else {
      right_speed = MAX_SPEED / 8;
      left_speed = MAX_SPEED;
    }

    right_speed = smooth_speed(right_speed, MAX_SPEED);
    left_speed = smooth_speed(left_speed, MAX_SPEED);

    const double *position = wb_gps_get_values(gps);
    double light_sum = 0.0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      light_sum += wb_light_sensor_get_value(light_sensors[i]);
    }
    double average_light_value = light_sum / NUM_SENSORS;

    //check if robot is at dead-end
    bool dead_end = is_dead_end(prox_values);

    //if new dead-end check for duplicates
    if (dead_end && !is_duplicate_dead_end(position)) {
      detected_dead_ends[num_dead_ends].x = position[0];
      detected_dead_ends[num_dead_ends].y = position[1];
      detected_dead_ends[num_dead_ends].z = position[2];
      num_dead_ends++;
      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f [DEAD-END]\n", 
             position[0], position[1], position[2], average_light_value);
    } else if (!dead_end) {
      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f\n", 
             position[0], position[1], position[2], average_light_value);
    }

    //store the current position and light intensity, ensure we don't exceed MAX POSITIONS
    if (num_recorded_positions < MAX_POSITIONS) {
      recorded_positions[num_recorded_positions].x = position[0];
      recorded_positions[num_recorded_positions].y = position[1];
      recorded_positions[num_recorded_positions].z = position[2];
      recorded_positions[num_recorded_positions].light_intensity = average_light_value;
      num_recorded_positions++;
    }

    wb_motor_set_velocity(right_motor, right_speed);
    wb_motor_set_velocity(left_motor, left_speed);
  }

  wb_robot_cleanup();
  return 0;
}