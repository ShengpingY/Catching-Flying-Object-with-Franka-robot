#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include "examples_common.cpp"
#include "geometry_msgs/Point.h"
#include "csvreader.cpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


    // read current position and publish them to trajplan under topic state
    franka::RobotState State =  robot.readOnce();
    std::array<double, 16> current_state = State.O_T_EE_c;
    geometry_msgs::Point cur;
    cur.x = current_state[12]; cur.y = current_state[13]; cur.z = current_state[14];
    ros::init(argc,argv,"robotmotion");
    ros::NodeHandle nh;
    ros::Publisher pub_robotstate = nh.advertise<geometry_msgs::Point> ("state",1000);
    pub_robotstate.publish(cur);
    ROS_INFO("Robot's Endeffector's current state has been published with x = %f, y = %f, z = %f", cur.x, cur.y, cur.z);

    // recieve traj data
    std::vector<std::vector<double>> trjdata_position;
    trjdata_position = read_csv("./src/trajectoryplan/src/traj_for_position.csv");
    //std::vector<std::vector<double>> trjdata_velocity = read_csv("./src/trajectoryplan/src/traj_for_velocity.csv");
    //std::vector<std::vector<double>> trjdata_acceleration = read_csv("./src/trajectoryplan/src/traj_for_acceleration.csv");
    //std::vector<std::vector<double>> trjdata_jerk = read_csv("./src/trajectoryplan/src/traj_for_jerk .csv");

    std::array<double, 16> initial_pose;
    //double time = 0.0;
    int time_counter = 0; // under miliseconds 
    robot.control([&time_counter, &initial_pose, &trjdata_position](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {

                                    
      int delta_t = period.toSec() * 1000;                             
      time_counter += delta_t;
      ROS_INFO("delta t is %d miliseconds",delta_t);
      int x_size = trjdata_position[0].size();
      int y_size = trjdata_position[1].size();
      int z_size = trjdata_position[2].size();


      double x = trjdata_position[0][time_counter];
      double y = trjdata_position[1][time_counter];
      double z = trjdata_position[2][time_counter];

      // update cartesian
      std::array<double, 16> new_pose = initial_pose;
      if (time_counter < x_size) {
        new_pose[12] = x;
      }
      if (time_counter < y_size) {
        new_pose[13] = y;
      }
      if (time_counter < z_size) {
        new_pose[14] = z;
      }
      if ((time_counter >= y_size-1) & (time_counter >= x_size-1) & (time_counter >= z_size-1)){
        std::cout << std::endl << "Position readched! Holding Position" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}