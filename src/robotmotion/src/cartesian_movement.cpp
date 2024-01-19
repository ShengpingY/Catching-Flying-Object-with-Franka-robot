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
    std::cout << "WARNING: This programm will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
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
    trjdata_position = read_csv("./src/trajectoryplan/src/traj_for_position");
    std::vector<std::vector<double>> trjdata_velocity = read_csv("./src/trajectoryplan/src/traj_for_velocity");
    std::vector<std::vector<double>> trjdata_acceleration = read_csv("./src/trajectoryplan/src/traj_for_acceleration");
    std::vector<std::vector<double>> trjdata_jerk = read_csv("./src/trajectoryplan/src/traj_for_jerk");

    // cartesianpose movement
    //double time = 0.0;
    int time_counter = 0; // under miliseconds 
    double time = 0.0;
    // robot.control([&time_counter, &trjdata_position, &trjdata_velocity, &trjdata_acceleration,&trjdata_jerk](const franka::RobotState& robot_state,
    //                                      franka::Duration period) -> franka::CartesianPose {

                                    
    //   double delta_t = period.toSec();  
    //   ROS_INFO("delta t is %f miliseconds",delta_t);                           
    //   time_counter += delta_t * 1000;
    //   ROS_INFO("time counter is %d miliseconds",time_counter);
      
    //   int x_size = trjdata_position[0].size();
    //   ROS_INFO("size of x traj is %d miliseconds",x_size);
    //   int y_size = trjdata_position[1].size();
    //   int z_size = trjdata_position[2].size();


    //   double x = trjdata_position[0][time_counter];
    //   ROS_INFO("x's coordinates is %f",x);
    //   double y = trjdata_position[1][time_counter];
    //   double z = trjdata_position[2][time_counter];

    //   double dx = trjdata_velocity[0][time_counter];
    //   double dy = trjdata_velocity[1][time_counter];
    //   double dz = trjdata_velocity[2][time_counter];
    //   double ddx = trjdata_acceleration[0][time_counter];
    //   double ddy = trjdata_acceleration[1][time_counter];
    //   double ddz = trjdata_acceleration[2][time_counter];
    //   double dddx = trjdata_jerk[0][time_counter];
    //   double dddy = trjdata_jerk[1][time_counter];
    //   double dddz = trjdata_jerk[2][time_counter];



    //   // update cartesian
    //   std::array<double, 16> new_pose = robot_state.O_T_EE_c;
    //   if (time_counter < x_size) {
    //     new_pose[12] = x;
    //     ROS_INFO("x's coordinates is %f",x);
    //     ROS_INFO("x's velocity is %f",dx);
    //     ROS_INFO("x's accerlaration is %f",ddx);
    //     ROS_INFO("x's jerk is %f",dddx);
    //   }
    //   // if (time_counter < y_size) {
    //   //   new_pose[13] = y;
    //   //   ROS_INFO("y's coordinates is %f",y);
    //   // }
    //   // if (time_counter < z_size) {
    //   //   new_pose[14] = z;
    //   //   ROS_INFO("z's coordinates is %f",z);
    //   // }
    //   if ((time_counter >= y_size-1) & (time_counter >= x_size-1) & (time_counter >= z_size-1)){
    //     std::cout << std::endl << "Position readched! Holding Position" << std::endl;
    //     return franka::MotionFinished(new_pose);
    //   }
    //   return new_pose;
    // });
    double secs =ros::Time::now().toSec();
    double secs_last =ros::Time::now().toSec();  
    robot.control([&secs_last, &secs, &time, &time_counter, &trjdata_position, &trjdata_velocity, &trjdata_acceleration,&trjdata_jerk](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianVelocities {
                                          
      // time upsate with ROS if see more precise   
      secs =ros::Time::now().toSec();
      ROS_INFO("Now time is %f miliseconds",secs);
      double delta_t = secs - secs_last;
      ROS_INFO("delta t is %f seconds",delta_t);  
      time += delta_t * 1000;
      time_counter = round(time);
      ROS_INFO("time counter is %d miliseconds",time_counter);
      secs_last = secs;       

      // double delta_t = period.toMSec();  
      // ROS_INFO("delta t is %f miliseconds",delta_t);                           
      // time_counter += delta_t;
      // ROS_INFO("time counter is %d miliseconds",time_counter);
      
      int x_size = trjdata_position[0].size();
      int y_size = trjdata_position[1].size();
      int z_size = trjdata_position[2].size();
      double x = trjdata_position[0][time_counter];
      double y = trjdata_position[1][time_counter];
      double z = trjdata_position[2][time_counter];
      double dx = trjdata_velocity[0][time_counter];
      double dy = trjdata_velocity[1][time_counter];
      double dz = trjdata_velocity[2][time_counter];
      double ddx = trjdata_acceleration[0][time_counter];
      double ddy = trjdata_acceleration[1][time_counter];
      double ddz = trjdata_acceleration[2][time_counter];
      double dddx = trjdata_jerk[0][time_counter];
      double dddy = trjdata_jerk[1][time_counter];
      double dddz = trjdata_jerk[2][time_counter];

      double v_x,v_y,v_z;


      // update cartesian

      if (time_counter < x_size) {
        v_x = dx;
        ROS_INFO("x's coordinates is %f",x);
        ROS_INFO("x's velocity is %f",dx);
        ROS_INFO("x's accerlaration is %f",ddx);
        ROS_INFO("x's jerk is %f",dddx);
      }
      if (time_counter < y_size) {
        v_y = dy;
        ROS_INFO("y's coordinates is %f",y);
        ROS_INFO("y's velocity is %f",dy);
        ROS_INFO("y's accerlaration is %f",ddy);
        ROS_INFO("y's jerk is %f",dddy);
      }
      if (time_counter < z_size) {
        v_z = dz;
        ROS_INFO("z's coordinates is %f",z);
        ROS_INFO("z's velocity is %f",dz);
        ROS_INFO("z's accerlaration is %f",ddz);
        ROS_INFO("z's jerk is %f",dddz);
        
      }
      franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
      if ((time_counter >= y_size-1) & (time_counter >= x_size-1) & (time_counter >= z_size-1)){
        std::cout << std::endl << "Position readched! Holding Position" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}