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

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "dmc");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher hydro1 = n.advertise<std_msgs::String>("hydro1_topic", 1000);
  ros::Publisher hydro2 = n.advertise<std_msgs::String>("hydro2_topic", 1000);
  ros::Publisher hydro3 = n.advertise<std_msgs::String>("hydro3_topic", 1000);
  ros::Publisher hydro4 = n.advertise<std_msgs::String>("hydro4_topic", 1000);
  ros::Publisher hydro5 = n.advertise<std_msgs::String>("hydro5_topic", 1000);

  ros::Rate loop_rate(10);

  srand(time(NULL));
  int curr_state = 0;
  double sensor_data = 0.0;
  while (ros::ok()) {

    curr_state = change_state(curr_state);
    sensor_data = gen_data(curr_state);
    ROS_INFO("%f L/h generated.", sensor_data);

    /*
    std::stringstream ss;
    ss << sensor_data << " L/h";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.str());
    */

    /* Flow rate messges with weights (1,2,3,4,5)
     */
    std_msgs::String nil;
    std::vector<std_msgs::String> msg_vec(5,nil);
    std::stringstream ss;
    for (int i = 1; i < 6; i++){
      ss.str("");
      ss << (sensor_data*i)/15;
      msg_vec.at(i-1).data = ss.str();
    }

    hydro1.publish(msg_vec.at(0));
    ROS_INFO("Hydro1: %s L/h", msg_vec.at(0).data.c_str());
    hydro2.publish(msg_vec.at(1));
    ROS_INFO("Hydro2: %s L/h", msg_vec.at(1).data.c_str());
    hydro3.publish(msg_vec.at(2));
    ROS_INFO("Hydro3: %s L/h", msg_vec.at(2).data.c_str());
    hydro4.publish(msg_vec.at(3));
    ROS_INFO("Hydro4: %s L/h", msg_vec.at(3).data.c_str());
    hydro5.publish(msg_vec.at(4));
    ROS_INFO("Hydro5: %s L/h", msg_vec.at(4).data.c_str());

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}