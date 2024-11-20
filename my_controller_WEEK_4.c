/*
 * File:          my_controller_WEEK_4.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdbool.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);
  
  WbDeviceTag prox_sensors[8];
  char prox_sensor_name[50];
  for (int ind = 0; ind < 8; ++ind)
  {
    sprintf(prox_sensor_name, "ps%d", ind);
    prox_sensors[ind] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[ind], TIME_STEP);
  }

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
   double right_speed = MAX_SPEED;
   double left_speed = MAX_SPEED;
   
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     
     bool right_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
     bool right_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
     bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
     bool left_wall = wb_distance_sensor_get_value(prox_sensors[1]) > 80;
     
    /* Process sensor data here */
    
    //If wll infront, drive left to follow wall on right
    if (front_wall == true)
    {
      right_speed = MAX_SPEED;
      left_speed = -MAX_SPEED;
    }
    else if (left_wall) {
      //If wall is on right, drive straight
        right_speed = MAX_SPEED;
        left_speed = MAX_SPEED;
      } else {
      //No wall around, driven too far ahead, turn right to find wall
        right_speed = MAX_SPEED/8;
        left_speed = MAX_SPEED;
      }
    
    

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     wb_motor_set_velocity(right_motor, right_speed);
     wb_motor_set_velocity(left_motor, left_speed);
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
