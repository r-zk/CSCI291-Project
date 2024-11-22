#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define NUM_SENSORS 8
#define WALL_THRESHOLD 80.0
#define DEAD_END_THRESHOLD 50.0
#define DEAD_END_PROXIMITY_THRESHOLD 0.1
#define MAX_POSITIONS 5000  // Prevent overflow, limit the number of recorded positions
#define LIGHT_INTENSITY_THRESHOLD 50.0  // Threshold for high light intensity

#define SMOOTHING_FACTOR 0.07

// Structure to store position and light intensity
typedef struct {
  double x;
  double y;
  double z;
  double light_intensity;
} PositionLightIntensity;

PositionLightIntensity recorded_positions[MAX_POSITIONS];  // Store up to MAX_POSITIONS positions
int num_recorded_positions = 0;

// Structure to store dead-end coordinates
typedef struct {
  double x;
  double y;
  double z;
} DeadEnd;

DeadEnd detected_dead_ends[100];
int num_dead_ends = 0;

double initial_position[3] = {0.0, 0.0, 0.0};

double smooth_speed(double current_speed, double target_speed) {
  return current_speed + SMOOTHING_FACTOR * (target_speed - current_speed);
}

double distance_between_points(const double *pos1, const double *pos2) {
  return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
}

bool is_back_at_start() {
  const double *current_position = wb_gps_get_values(wb_robot_get_device("gps"));
  double dist = distance_between_points(current_position, initial_position);
  return dist < 0.5;  // Allow up to 0.5 meters of error
}

bool is_dead_end(double prox_values[]) {
  bool front_wall = prox_values[7] > WALL_THRESHOLD;
  bool left_wall = prox_values[1] > DEAD_END_THRESHOLD;
  bool right_wall = prox_values[5] > DEAD_END_THRESHOLD;

  if (front_wall && left_wall && right_wall) {
    return true;
  }

  if ((left_wall && right_wall) && !front_wall) {
    double front_distance = prox_values[7];
    if (front_distance > WALL_THRESHOLD && front_distance < DEAD_END_THRESHOLD) {
      return true;
    }
  }

  return false;
}

bool is_duplicate_dead_end(const double *position) {
  for (int i = 0; i < num_dead_ends; i++) {
    double dist = distance_between_points(position, (double[]){detected_dead_ends[i].x, detected_dead_ends[i].y, detected_dead_ends[i].z});
    if (dist < DEAD_END_PROXIMITY_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void move_towards_position(const double *current_position, const PositionLightIntensity *target_position) {
  double delta_x = target_position->x - current_position[0];
  double delta_y = target_position->y - current_position[1];

  double target_angle = atan2(delta_y, delta_x);
  double current_angle = current_position[2];  // Assuming Z value is the heading (adjust if necessary)

  double angle_diff = target_angle - current_angle;
  if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

  // Proportional control for turning
  double angular_velocity = angle_diff * 0.5;
  double linear_velocity = 1.0;  // Constant forward speed

  // Set motor velocities based on angle difference
  wb_motor_set_velocity(wb_robot_get_device("left wheel motor"), linear_velocity - angular_velocity);
  wb_motor_set_velocity(wb_robot_get_device("right wheel motor"), linear_velocity + angular_velocity);
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag light_sensors[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }

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

  double start_check_time = wb_robot_get_time() + 27;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (wb_robot_get_time() >= start_check_time) {
      if (is_back_at_start()) {
        printf("Back at start. Stopping.\n--Completed a Round of the Maze--\n");

        // Find the position with the highest light intensity
        double max_light_intensity = -1.0;
        PositionLightIntensity best_position;
        for (int i = 0; i < num_recorded_positions; i++) {
          if (recorded_positions[i].light_intensity > max_light_intensity && recorded_positions[i].light_intensity > LIGHT_INTENSITY_THRESHOLD) {
            max_light_intensity = recorded_positions[i].light_intensity;
            best_position = recorded_positions[i];
          }
        }

        // Move to the position with the highest light intensity
        printf("Moving to position with highest light intensity: (%.2f, %.2f, %.2f)\n", best_position.x, best_position.y, best_position.z);

        // Navigate towards the best position
        const double *current_position = wb_gps_get_values(gps);
        while (distance_between_points(current_position, (double *)&best_position) > 0.2) {
          current_position = wb_gps_get_values(gps);
          move_towards_position(current_position, &best_position);
          wb_robot_step(TIME_STEP);
        }

        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        break;
      } else {
        start_check_time = wb_robot_get_time() + 27;
      }
    }

    double prox_values[8];
    for (int i = 0; i < 8; ++i) {
      prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    }

    bool front_wall = prox_values[7] > WALL_THRESHOLD;
    bool left_wall = prox_values[1] > WALL_THRESHOLD;
    bool right_wall = prox_values[5] > WALL_THRESHOLD;

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

    // Store the current position and light intensity, ensure we don't exceed MAX_POSITIONS
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
