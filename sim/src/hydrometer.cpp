#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

double gen_data(int curr_state) {
  
  double ans = -1;

  switch (curr_state){
    case 0:
      ans = (rand()%20)/100.0; /*ans = [0,0.19]*/
      break;
    case 1:
      ans = (rand()%20+20)/100.0; /*ans = [0.20,0.39]*/
      break;
    case 2:
      ans = (rand()%20+40)/100.0; /*ans = [0.40,0.59]*/
      break;
    case 3:
      ans = (rand()%20+60)/100.0; /*ans = [0.60,0.79]*/
      break;
    case 4:
      ans = (rand()%20+80)/100.0; /*ans = [0.80,0.99]*/
      break;
    default:
      throw "Current state not recognized, couldn't retrieve data.";
      break;
  }

  ans *= 24.85; // Scaling factor: (Qmax - Qmin)/100, Qmax = 2500 and Qmin = 15

  return ans;
}

int change_state (int curr_state) {

  int new_state;
  int p = (rand()%100)+1;

  switch (curr_state){
    case 0:
      if (p <= 70){
        new_state = 0;
      } else if (p <= 90) {
        new_state = 1;
      } else if (p <= 97) {
        new_state = 2;
      } else if (p <= 100) {
        new_state = 3;
      } else {
        throw "Probability out of bounds!";
      }
      break;
    case 1:
      if (p <= 70){
        new_state = 1;
      } else if (p <= 80) {
        new_state = 0;
      } else if (p <= 90) {
        new_state = 2;
      } else if (p <= 95) {
        new_state = 3;
      } else if (p <= 100) {
        new_state = 4;
      } else {
        throw "Probability out of bounds!";
      }
      break;
    case 2:
      if (p <= 70){
        new_state = 2;
      } else if (p <= 80) {
        new_state = 1;
      } else if (p <= 90) {
        new_state = 3;
      } else if (p <= 95) {
        new_state = 0;
      } else if (p <= 100) {
        new_state = 4;
      } else {
        throw "Probability out of bounds!";
      }
      break;
    case 3:
      if (p <= 70){
        new_state = 3;
      } else if (p <= 80) {
        new_state = 2;
      } else if (p <= 90) {
        new_state = 4;
      } else if (p <= 95) {
        new_state = 1;
      } else if (p <= 100) {
        new_state = 0;
      } else {
        throw "Probability out of bounds!";
      }
      break;
    case 4:
      if (p <= 70){
        new_state = 4;
      } else if (p <= 90) {
        new_state = 3;
      } else if (p <= 97) {
        new_state = 2;
      } else if (p <= 100) {
        new_state = 1;
      } else {
        throw "Probability out of bounds!";
      }
      break;
    default:
      throw "Current state not recognized, couldn't retrieve data.";
      break;
  }

  return new_state;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "hydrometer");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);

  srand(time(NULL));
  int curr_state = 0;
  double sensor_data = 0.0;
  ros::Time now;
  while (ros::ok()) {
    
    /*
     * Gerar o dado baseado no estado atual 
     */
    curr_state = change_state(curr_state);
    sensor_data = gen_data(curr_state);
    now = ros::Time::now();

    /*
     * Enviar o dado
     */

    /*
     * Imprimir o dado
     */ 
    ROS_INFO("--- HYDROMETER ---");
    ROS_INFO("flow rate: %.2f L/h", sensor_data);
    ROS_INFO("now: %.2f ns", now.toSec());
    ROS_INFO("------------------");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}