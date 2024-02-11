#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <franka/exception.h>
#include <franka/robot.h>
// #include "examples_common.h"
// #include "examples_common.cpp"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
// #include "panda_ik.hpp"
#include "Cartesian_movement_IK.h"

// use class Cartesian_IK to avoid global variable
int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    // ros::init(argc, argv, "robotmotion");
    Cartesian_IK movement(argv[1],argc,argv);
    // movement.movement_together(argc,argv);
    // movement.initial_pose();
    // while(ros::ok()){
    //   ROS_INFO("ROS OK!");
    //   movement.run();
    // }
    return 0;
}

// std::array<double, 7> result_q;
// std::array<double, 16> state_for_IK_tfmatrix;
// std::array<double, 7> state_for_IK_q;
// bool msg_received_flag = false;


// void IK_callback(const geometry_msgs::Point& msg) {
//   msg_received_flag = true;
//   ROS_INFO("Current q pose is %f,%f,%f,%f,%f,%f,%f",state_for_IK_q[0],state_for_IK_q[1],state_for_IK_q[2],state_for_IK_q[3],state_for_IK_q[4],state_for_IK_q[5],state_for_IK_q[6]);
//   ROS_INFO("Destination receieved is %f,%f,%f",msg.x, msg.y, msg.z);
//   std::array<double,6> xyzrpy = {{msg.x, msg.y, msg.z, 0, -180, -180}};
//   result_q = compute_inverse_kinematics(xyzrpy, state_for_IK_q);
//   ROS_INFO("Destination q pose is %f,%f,%f,%f,%f,%f,%f",result_q[0],result_q[1],result_q[2],result_q[3],result_q[4],result_q[5],result_q[6]);
//   }

// bool all_near_zero(const std::array<double, 7>& dq){
//   for (const auto& elem : dq) {
//     if (std::abs(elem) > 1e-4) {
//       return false;
//     }
//   }
//   return true;
// }


// // int main(int argc, char** argv) {
// //   ros::init(argc,argv,"test");
// //   ros::NodeHandle nh;
// //   ros::Subscriber desetinationSubscriber = nh.subscribe("matlab",1,dest_coord_sub_Callback);
// //   while(ros::ok()) {
// //         ros::spinOnce();    
// //     }
// // }

// // int main(int argc, char** argv) {
// //   if (argc != 2) {
// //     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
// //     return -1;
// //   }
// //   try {
// //     franka::Robot robot(argv[1]);
// //     setDefaultBehavior(robot);
    
// //     robot.setCollisionBehavior(
// //         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
// //         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
// //         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
// //         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


// //     // read current position and publish them to trajplan under topic state

// //     ros::init(argc,argv,"robotmotion");
// //     ros::NodeHandle nh;
// //     ros::Publisher pub_robotstate = nh.advertise<geometry_msgs::Point> ("state",1000);
// //     while(ros::ok()) {
// //       ros::spinOnce();    
// //       franka::RobotState State =  robot.readOnce();
// //       std::array<double, 16> current_state = State.O_T_EE;
// //       geometry_msgs::Point cur;
// //       cur.x = current_state[12]; cur.y = current_state[13]; cur.z = current_state[14];
// //       pub_robotstate.publish(cur);
// //       ROS_INFO("Robot's Endeffector's current state has been published with x = %f, y = %f, z = %f", cur.x, cur.y, cur.z);
// //     }
    
    


    
// //   } catch (const franka::Exception& e) {
// //     std::cout << e.what() << std::endl;
// //     return -1;
// //   }
// //   return 0;
// // }

// int main(int argc, char** argv) {
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//     return -1;
//   }
//   try {
//     franka::Robot robot(argv[1]);
//     setDefaultBehavior(robot);
//     franka::RobotState State =  robot.readOnce();
//     state_for_IK_q = State.q;
//     ROS_INFO("Current q pose is %f,%f,%f,%f,%f,%f,%f",state_for_IK_q[0],state_for_IK_q[1],state_for_IK_q[2],state_for_IK_q[3],state_for_IK_q[4],state_for_IK_q[5],state_for_IK_q[6]);
  

//     // First move the robot to a suitable joint configuration
//     std::array<double, 7> q_goal = {{1.564, -0.026, -1.62, -1.02, 0, 1.011, -0.931}};
//     MotionGenerator motion_generator(0.2, q_goal);// 0.3 stands for the velocity
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(motion_generator);
//     // std::cout << "Finished moving to initial joint configuration." << std::endl;
//     // std::cout << "WARNING: This programm will move the robot! "
//     //           << "Please make sure to have the user stop button at hand!" << std::endl
//     //           << "Press Enter to continue..." << std::endl;
//     // std::cin.ignore();
//     // Set additional parameters always before the control loop, NEVER in the control loop!
//     // Set collision behavior.
//     robot.setCollisionBehavior(
//         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
//         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


//     // read current position and publish them to trajplan under topic state
//     // franka::RobotState State =  robot.readOnce();
    
//     state_for_IK_tfmatrix = State.O_T_EE;
//     state_for_IK_q = State.q;
//     geometry_msgs::Point cur;
//     cur.x = state_for_IK_tfmatrix[12]; cur.y = state_for_IK_tfmatrix[13]; cur.z = state_for_IK_tfmatrix[14];
//     ros::init(argc,argv,"robotmotion");
//     ros::NodeHandle nh;
//     ros::Publisher pub_robotstate = nh.advertise<geometry_msgs::Point> ("state",1000);
//     pub_robotstate.publish(cur);
//     ROS_INFO("Robot's Endeffector's current state has been published with x = %f, y = %f, z = %f", cur.x, cur.y, cur.z);
//     ros::Subscriber desetinationSubscriber = nh.subscribe("matlab",1,IK_callback);
//     std::cout << "WARNING: This programm will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     int status_a = 0,status_b = 0;
//     bool action_executed = true;
//     std::array<double, 7> q_cur_velocity;
//     // while(ros::ok()){
//     //   while(action_executed) {
//     //     ros::spinOnce();
//     //     State =  robot.readOnce();
//     //     state_for_IK_tfmatrix = State.O_T_EE;
//     //     state_for_IK_q = State.q;
//     //     if (msg_received_flag) {
//     //       status_a++;
//     //       ROS_INFO("status a is %d",status_a);
//     //       action_executed = false;
//     //       MotionGenerator motion_generator(0.5, result_q);
//     //       robot.control(motion_generator);
//     //       std::cout << "Finished moving to initial joint configuration." << std::endl;
//     //       std::cout << "WARNING: This programm will move the robot! "
//     //           << "Please make sure to have the user stop button at hand!" << std::endl
//     //           << "Press Enter to continue..." << std::endl;
//     //       std::cin.ignore();
          
//     //       // status_b++;
          
//     //       // ROS_INFO("status b is %d",status_b);
//     //       // std::cout << "Finished moving to initial joint configuration." << std::endl;
//     //       // State =  robot.readOnce();
//     //       // q_cur_velocity = State.dq;
//     //       // if (all_near_zero(q_cur_velocity)){
//     //       //   action_executed = true;
//     //       // }
//     //     }
//     //     // sleep(5);
//     //   }
//     // }
//     while(ros::ok()){
//       while(!msg_received_flag){
//       ros::spinOnce();
//         }
//       State =  robot.readOnce();
//       state_for_IK_tfmatrix = State.O_T_EE;
//       state_for_IK_q = State.q;
//       status_a++;
//       ROS_INFO("status a is %d",status_a);
//       action_executed = false;
//       MotionGenerator catch_motion(0.3, result_q);
//       robot.control(catch_motion);
//       status_b++;
//       ROS_INFO("status a is %d",status_b);
//       msg_received_flag = false;
      
//     }
//     // while(!msg_received_flag){
//     //   ros::spinOnce();
//     // }
//     // State =  robot.readOnce();
//     // state_for_IK_tfmatrix = State.O_T_EE;
//     // state_for_IK_q = State.q;
//     // status_a++;
//     // ROS_INFO("status a is %d",status_a);
//     // action_executed = false;
//     // MotionGenerator catch_motion(0.05, result_q);
//     // robot.control(catch_motion);
    
    
    

    

    
//   } catch (const franka::Exception& e) {
//     std::cout << e.what() << std::endl;
//     return -1;
//   }
//   return 0;
// }