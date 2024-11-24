//including libraries
#include <webots/robot.h>   //robot API for controlling
#include <webots/motor.h>   //motor API for controlling motors
#include <webots/distance_sensor.h>   //distance sensor API to read sensor data
#include <webots/gps.h>   //GPS API for getting robot's position
#include <webots/light_sensor.h>  //light sensor API for light intensity readings
#include <stdio.h>  //imports standard I/O library
#include <stdbool.h>  //imports standard library for using boolean data type (true/false)
#include <math.h>   //math library for math functions

//defining constants
#define TIME_STEP 64  //time step in ms for each simulation cycle
#define MAX_SPEED 6.28  //maximum speed of the robot's motors in radians per second
#define NUM_SENSORS 8   //number of proximity sensors used by robot
#define WALL_THRESHOLD 80.0   //distance threshold for detecting walls
#define DEAD_END_THRESHOLD 50.0   //threshold for detecting dead ends based on sensor values
#define DEAD_END_PROXIMITY_THRESHOLD 0.1  //proximity threshold to detect duplicate dead ends
#define MAX_POSITIONS 5000  //maximum number of positions that can be recorded
#define LIGHT_INTENSITY_THRESHOLD 50.0    //light intensity threshold for detecting significant light levels

#define SMOOTHING_FACTOR 0.07   //factor for smoothing transitions in motor speed

//struct to store position and light intensity at coordinates
typedef struct {
  double x; // x-coordinate of the position
  double y; // y-coordinate of the position
  double z; // z-coordinate of the position
  double light_intensity; //light intensity measured at position
} PositionLightIntensity;

//array to store recorded positions and light intensities
PositionLightIntensity recorded_positions[MAX_POSITIONS]; 
int num_recorded_positions = 0;  //variable to track number of recorded positions

//struct to store coordinates of detected dead ends
typedef struct {
  double x; //x-coordinate of dead end
  double y; //y-coordinate of dead end
  double z; //z-coordinate of dead end
} DeadEnd;

//array to store detected dead ends //size 25
DeadEnd detected_dead_ends[25]; 
int num_dead_ends = 0;  //variable to track number of detected dead ends

double initial_position[3] = {0.0, 0.0, 0.0};  //initial position of robot

//utility functions

//function to smoothly adjust the motor speed from current to target speed
double smooth_speed(double current_speed, double target_speed) {
  return current_speed + SMOOTHING_FACTOR * (target_speed - current_speed);  //smooth transition between speeds
}

//function to calculate distance between points 
double distance_between_points(const double *pos1, const double *pos2) {
  return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));   
  }

//function to check if robot is back at starting position
bool is_back_at_start() {
  const double *current_position = wb_gps_get_values(wb_robot_get_device("gps"));  //get current position from GPS
  double dist = distance_between_points(current_position, initial_position);  //calculate distance from initial position
  return dist < 0.5;  //return true if distance is less than 0.5 meters
}

//function to check if current sensor readings indicate dead end
bool is_dead_end(double prox_values[]) {
  bool front_wall = prox_values[7] > WALL_THRESHOLD;  //check if there is a wall in front of the robot
  bool left_wall = prox_values[1] > DEAD_END_THRESHOLD;  //check if there is a wall on the left
  bool right_wall = prox_values[5] > DEAD_END_THRESHOLD;  //check if there is a wall on the right

  //return true if walls are detected on three sides
  if (front_wall && left_wall && right_wall) {
    return true;
  }

  //if walls are on left and right but not in front, check if it is a narrow passage
  if ((left_wall && right_wall) && !front_wall) {
    double front_distance = prox_values[7];
    if (front_distance > WALL_THRESHOLD && front_distance < DEAD_END_THRESHOLD) {
      return true;
    }
  }

  return false;  //return false if no dead end detected
}

//function to check if a current position is a previously recorded dead end
bool is_duplicate_dead_end(const double *position) {
  for (int i = 0; i < num_dead_ends; i++) {
    double dist = distance_between_points(position, (double[]){detected_dead_ends[i].x, detected_dead_ends[i].y, detected_dead_ends[i].z});
    if (dist < DEAD_END_PROXIMITY_THRESHOLD) {  //if current position is too close to a previously recorded dead end
      return true;
    }
  }
  return false;  //return false if no duplicate dead end detected
}

//function to check if robot has reached target position with maximum light intensity
bool is_at_max(const double *target_position, const PositionLightIntensity *best_position) {
  double dist = distance_between_points(target_position, (double[]){best_position->x, best_position->y, best_position->z});
  return dist < 0.01;  //return true if distance to best position is less than 0.01 meters
}

//main loop
int main(int argc, char **argv) {
    //initialize robot
  wb_robot_init();

  //set up motor devices for controlling robot's wheels
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(right_motor, INFINITY);  //set motors to continuous rotation
  wb_motor_set_position(left_motor, INFINITY);  //set motors to continuous rotation
  wb_motor_set_velocity(right_motor, 0.0);  //set initial velocity to 0 for both motors
  wb_motor_set_velocity(left_motor, 0.0);   //set initial velocity to 0 for both motors

  //set up proximity sensors to detect obstacles around the robot
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind) {  //loop over each proximity sensor
    sprintf(prox_sensor_name, "ps%d", ind);  //construct sensor name dynamically ps0 to ps7
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);  //get sensor device
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);  //enable sensor with given time step
  }

  //set up GPS device for tracking robot's position
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);  //enable GPS with the given time step

  //set up light sensors to measure light intensity in the environment
  WbDeviceTag light_sensors[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {  //loop over each light sensor
    sprintf(sensor_name, "ls%d", i);  //construct sensor name 
    light_sensors[i] = wb_robot_get_device(sensor_name);  //get light sensor device
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);  //enable light sensor with given time step
  }

  //initialize motor speeds
  double right_speed = MAX_SPEED;  //set right motor speed to maximum speed
  double left_speed = MAX_SPEED;   //set left motor speed to maximum speed

  //wait for GPS initialization and ensure valid GPS coordinates
  const double *initial_gps_position = NULL;
  while (true) {
    initial_gps_position = wb_gps_get_values(gps);  //get initial GPS position
    if (initial_gps_position != NULL && !isnan(initial_gps_position[0]) && !isnan(initial_gps_position[1]) && !isnan(initial_gps_position[2])) {
      break;  //break out of loop once valid GPS data is retrieved
    }
    wb_robot_step(TIME_STEP);  //continue simulation step if GPS data is invalid
  }

  //store initial GPS position coordinates
  initial_position[0] = initial_gps_position[0];
  initial_position[1] = initial_gps_position[1];
  initial_position[2] = initial_gps_position[2];

  printf("Initial Position: (%.2f, %.2f, %.2f)\n--Exploring Maze--\n", initial_position[0], initial_position[1], initial_position[2]);

  //start exploration behavior
  double start_check_time = wb_robot_get_time() + 27;  //set a timer to check if robot is back at starting point after 27 seconds

  //main loop for robot behaviour
  while (wb_robot_step(TIME_STEP) != -1) {  //continue running until simulation ends
    if (wb_robot_get_time() >= start_check_time) {  //check if elapsed time exceeds check time
      if (is_back_at_start()) {  //check if robot is back at starting position
        printf("Back at start. Stopping.\n--Completed a Round of the Maze--\n");

        //pause for a brief moment before performing further actions
        double pause_time = wb_robot_get_time() + 3.0; 
        while (wb_robot_get_time() < pause_time) {  //wait for 3 seconds
          wb_robot_step(TIME_STEP);
        }

        //find position with highest light intensity
        double max_light_intensity = -1.0;  //initialize to a low value
        PositionLightIntensity best_position = {0.0, 0.0, 0.0, 0.0};  //initialize best position
        for (int i = 0; i < num_recorded_positions; i++) {  //loop through recorded positions
          if (recorded_positions[i].light_intensity > max_light_intensity && recorded_positions[i].light_intensity > LIGHT_INTENSITY_THRESHOLD) {  //find position with max light intensity above threshold
            max_light_intensity = recorded_positions[i].light_intensity;
            best_position = recorded_positions[i];
          }
        }

        //move robot toward position with highest light intensity
        if (max_light_intensity > -1.0) {  //if a valid position is found
          printf("Moving to position with highest light intensity: (%.2f, %.2f, %.2f)\n", best_position.x, best_position.y, best_position.z);

          const double *target_position = wb_gps_get_values(gps);  //get current GPS position
          while (distance_between_points(target_position, (double *)&best_position) > 0.2) {  //while robot is not at best position
            target_position = wb_gps_get_values(gps);  //get updated GPS position

            //wall-following behavior 
            double prox_values[8]; 
            for (int i = 0; i < 8; ++i) {
              prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);  //read all proximity sensors
            }

            bool front_wall = prox_values[7] > WALL_THRESHOLD;  //check if wall is detected in front
            bool left_wall = prox_values[1] > WALL_THRESHOLD;  //check if wall is detected on left

            //control motor speeds based on wall detection
            if (front_wall) {
              right_speed = MAX_SPEED;  //turn left if front wall detected
              left_speed = -MAX_SPEED;
            } else if (left_wall) {
              right_speed = MAX_SPEED;  //follow  left wall if left wall detected
              left_speed = MAX_SPEED;
            } else {
              right_speed = MAX_SPEED / 8;  //go forward slowly when no walls are detected
              left_speed = MAX_SPEED;
            }

            //smoothly adjust motor speeds
            right_speed = smooth_speed(right_speed, MAX_SPEED);
            left_speed = smooth_speed(left_speed, MAX_SPEED);

            wb_motor_set_velocity(right_motor, right_speed);  //set right motor speed
            wb_motor_set_velocity(left_motor, left_speed);  //set left motor speed

            //check if robot has reached target position with max light intensity
            if (is_at_max(target_position, &best_position)) {
              break;
            }

            wb_robot_step(TIME_STEP);  //continue simulation step
          }

          //once at target stop motors
          wb_motor_set_velocity(left_motor, 0.0);
          wb_motor_set_velocity(right_motor, 0.0);
          
          printf("===Reached the position with the highest light intensity: (%.2f, %.2f, %.2f)===\n", best_position.x, best_position.y, best_position.z);
        }

        break;  //exit loop once the robot reaches target position
      } else {
        start_check_time = wb_robot_get_time() + 27;  //reset check time
      }
    }

    //regular wall-following behavior when not moving towards best light intensity
    double prox_values[8]; 
    for (int i = 0; i < 8; ++i) {
      prox_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);  //get proximity sensor readings
    }

    bool front_wall = prox_values[7] > WALL_THRESHOLD;  //detect wall in front
    bool left_wall = prox_values[1] > WALL_THRESHOLD;  //detect wall on the left

    //control motor speeds based on wall detection
    if (front_wall) {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;  //turn if wall is in front
    } else if (left_wall) {
      right_speed = MAX_SPEED;
      left_speed = MAX_SPEED;  //follow left wall if detected
    } else {
      right_speed = MAX_SPEED / 8;  //move slowly when no walls are detected
      left_speed = MAX_SPEED;
    }

    //smooth transition of motor speeds
    right_speed = smooth_speed(right_speed, MAX_SPEED);
    left_speed = smooth_speed(left_speed, MAX_SPEED);

    const double *position = wb_gps_get_values(gps);  //get current GPS position
    double light_sum = 0.0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      light_sum += wb_light_sensor_get_value(light_sensors[i]);  //sum light sensor values
    }
    double average_light_value = light_sum / NUM_SENSORS;  //compute average light intensity

    //check if robot is at dead-end
    bool dead_end = is_dead_end(prox_values);

    //if a new dead end is detected check for duplicates and store it
    if (dead_end && !is_duplicate_dead_end(position)) {
      detected_dead_ends[num_dead_ends].x = position[0];
      detected_dead_ends[num_dead_ends].y = position[1];
      detected_dead_ends[num_dead_ends].z = position[2];
      num_dead_ends++;
      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f [DEAD-END]\n", 
             position[0], position[1], position[2], average_light_value);
    } else if (!dead_end) {
      //print current position and light intensity if not at dead-end
      printf("At Position: (%.2f, %.2f, %.2f), Average Light Intensity: %.2f\n", 
             position[0], position[1], position[2], average_light_value);
    }

    //store current position and its light intensity if max position limit is not exceeded
    if (num_recorded_positions < MAX_POSITIONS) {
      recorded_positions[num_recorded_positions].x = position[0];
      recorded_positions[num_recorded_positions].y = position[1];
      recorded_positions[num_recorded_positions].z = position[2];
      recorded_positions[num_recorded_positions].light_intensity = average_light_value;
      num_recorded_positions++;  //increment number of recorded positions
    }

    wb_motor_set_velocity(right_motor, right_speed);  //set motor velocities
    wb_motor_set_velocity(left_motor, left_speed);  //set motor velocities
  }

  wb_robot_cleanup();  //clean up after simulation
  return 0;  //end program
}
