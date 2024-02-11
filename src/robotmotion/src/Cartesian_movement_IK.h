#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include "examples_common.cpp"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include "panda_ik.hpp"

class Cartesian_IK {
private:
    franka::Robot robot;
    franka::RobotState state;
    std::array<double, 7> state_for_IK_q;
    std::array<double, 16> state_for_IK_tfmatrix;
    std::array<double, 7> result_q;
    bool msg_received_flag;

public:
    Cartesian_IK(const std::string& hostname,int argc, char **argv) : robot(hostname), msg_received_flag(false) {
      try {
          ros::init(argc, argv, "robotmotion");
          state = robot.readOnce();
          state_for_IK_q = state.q;
          state_for_IK_tfmatrix = state.O_T_EE;
          ROS_INFO("Current q pose is %f,%f,%f,%f,%f,%f,%f", state_for_IK_q[0], state_for_IK_q[1], state_for_IK_q[2],
                   state_for_IK_q[3], state_for_IK_q[4], state_for_IK_q[5], state_for_IK_q[6]);
  
          ros::NodeHandle nh;
          ros::Publisher pub_robotstate = nh.advertise<geometry_msgs::Point>("state", 1000);
          geometry_msgs::Point cur;
          cur.x = state_for_IK_tfmatrix[12];
          cur.y = state_for_IK_tfmatrix[13];
          cur.z = state_for_IK_tfmatrix[14];
          pub_robotstate.publish(cur);
          ROS_INFO("Robot's Endeffector's current state has been published with x = %f, y = %f, z = %f", cur.x, cur.y,
                   cur.z);
  
          ros::Subscriber destinationSubscriber = nh.subscribe("/transformed_coord",1, &Cartesian_IK::IK_callback, this);
          std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        //   std::array<double, 7> q_goal = {{1.347, 1.017, -2.106, -1.757, 0.8869, 1.2355, 0.1}};
          MotionGenerator motion_generator(0.2, q_goal);
          std::cout << "WARNING: This example will move the robot! "
                       "Please make sure to have the user stop button at hand!" << std::endl
                    << "Press Enter to continue..." << std::endl;
          std::cin.ignore();
          robot.control(motion_generator);
          ros::spinOnce();
          std::cout << "Initial Pose reached! "
                       "WARNING: This example will move the robot! "
                       "Please make sure to have the user stop button at hand!" << std::endl
                    << "Press Enter to continue..." << std::endl;
          std::cin.ignore();
          while(ros::ok()) {
            // sleep(0.1);
            ros::spinOnce();
            state = robot.readOnce();
            state_for_IK_q = state.q;
            state_for_IK_tfmatrix = state.O_T_EE;
            // ROS_INFO("Current q pose is %f,%f,%f,%f,%f,%f,%f", state_for_IK_q[0], state_for_IK_q[1], state_for_IK_q[2],
            //          state_for_IK_q[3], state_for_IK_q[4], state_for_IK_q[5], state_for_IK_q[6]);
            // ros::spinOnce();
            // ROS_INFO("ROS OK!");
            // ROS_INFO("Message received: %d", msg_received_flag);
            if(msg_received_flag){
                MotionGenerator catch_motion(1.35, result_q);
                robot.control(catch_motion);
                msg_received_flag = false;
            } 
          }

      } catch (const franka::Exception& e) {
            std::cout << "Initialization failed: " << e.what() << std::endl;
            throw;
        }
    }

    // void movement_together(int argc, char **argv){
    //     try {
    //       setDefaultBehavior(robot);
    //       robot.setCollisionBehavior(
    //               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
          
    //       std::array<double, 7> q_goal = {{1.564, -0.026, -1.62, -1.02, 0, 1.011, -0.931}};
    //       MotionGenerator motion_generator(0.2, q_goal);
    //       std::cout << "WARNING: This example will move the robot! "
    //                    "Please make sure to have the user stop button at hand!" << std::endl
    //                 << "Press Enter to continue..." << std::endl;
    //       std::cin.ignore();
    //       robot.control(motion_generator);
    //       ros::spinOnce();
    //       std::cout << "Initial Pose reached! "
    //                    "WARNING: This example will move the robot! "
    //                    "Please make sure to have the user stop button at hand!" << std::endl
    //                 << "Press Enter to continue..." << std::endl;
    //       std::cin.ignore();
    //       while(ros::ok()) {
    //         state = robot.readOnce();
    //         state_for_IK_q = state.q;
    //         state_for_IK_tfmatrix = state.O_T_EE;
    //         ROS_INFO("Current q pose is %f,%f,%f,%f,%f,%f,%f", state_for_IK_q[0], state_for_IK_q[1], state_for_IK_q[2],
    //                  state_for_IK_q[3], state_for_IK_q[4], state_for_IK_q[5], state_for_IK_q[6]);
    //         ros::spinOnce();
    //         // ROS_INFO("ROS OK!");
    //         // ROS_INFO("Message received: %d", msg_received_flag);
    //         // if(msg_received_flag){
    //         //     MotionGenerator catch_motion(0.3, result_q);
    //         //     robot.control(catch_motion);
    //         //     msg_received_flag = false;
    //         // }    
    //       }
    //     } catch (const franka::Exception& e) {
    //             std::cout << "Initialization failed: " << e.what() << std::endl;
    //             throw;
    //     }
    // }

    // void initial_pose() {
    //   try {
    //       setDefaultBehavior(robot);
    //       robot.setCollisionBehavior(
    //               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
    //               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
    //               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
          
    //       std::array<double, 7> q_goal = {{1.564, -0.026, -1.62, -1.02, 0, 1.011, -0.931}};
    //       MotionGenerator motion_generator(0.2, q_goal);
    //       std::cout << "WARNING: This example will move the robot! "
    //                    "Please make sure to have the user stop button at hand!" << std::endl
    //                 << "Press Enter to continue..." << std::endl;
    //       std::cin.ignore();
    //       robot.control(motion_generator);
    //   } catch (const franka::Exception& e) {
    //         std::cout << "Initialization failed: " << e.what() << std::endl;
    //         throw;
    //     }

    // }
    // void run() {
    //   try {
    //       ros::spinOnce();
    //       ROS_INFO("Destination receieved is %d", msg_received_flag);
    //       if(msg_received_flag){
    //         MotionGenerator catch_motion(0.3, result_q);
    //         robot.control(catch_motion);
    //         msg_received_flag = false;
    //       }
    //   } catch (const franka::Exception& e) {
    //         std::cout << "Initialization failed: " << e.what() << std::endl;
    //         throw;
    //     }
    // }

private:
    void IK_callback(const geometry_msgs::Point &msg) {
        msg_received_flag = true;
        ROS_INFO("Destination receieved is %f,%f,%f", msg.x, msg.y, msg.z);
        std::array<double, 7> result;
        // std::array<double, 6> xyzrpy = {{0.5, 0, 0.5, 0, -180, -180}};
        std::array<double, 6> xyzrpy = {{msg.x, msg.y, msg.z, 0, -180, -180}};
        result = compute_inverse_kinematics(xyzrpy, state_for_IK_q);
        if (result[0] == 999999) {
            msg_received_flag = false;
            std::cout << "NO SOLUTIONS FOUND" << std::endl;
        }
        else {
            result_q = result;
            ROS_INFO("Destination q pose is %f,%f,%f,%f,%f,%f,%f", result_q[0], result_q[1], result_q[2], result_q[3], result_q[4], result_q[5], result_q[6]);
        }

    }
};

// int main(int argc, char **argv) {
//     if (argc != 2) {
//         std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//         return -1;
//     }

//     ros::init(argc, argv, "robotmotion");
//     Cartesian_IK movement(argv[1]);
//     movement.initial_pose();
//     while(ros::ok()){
//       movement.run();
//     }
//     return 0;
// }