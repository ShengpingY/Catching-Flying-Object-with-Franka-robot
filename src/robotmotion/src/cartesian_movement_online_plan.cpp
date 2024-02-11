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
#include "online_doubleS.cpp"

//define global variables
geometry_msgs::Point dest_sub;

double decimal3(double x) {
  double y = std::floor(x*1000)/1000;
  return y;
}

void dest_coord_callback(const geometry_msgs::Point& msg) {
        dest_sub = msg;
        // ROS_INFO("dest_coord is being called");
        // calculate traj
}

//implement an interpolation function to get corresponding value between two points in a vector
double interpolate(std::vector<double> vector, double position){
  //round position to the lower integer
  int position_left = floor(position);
  //round position to the upper integer
  int position_right = ceil(position);
  //get the value of the left and right position
  double value_left = vector[position_left];
  double value_right = vector[position_right];
  //get the value of the position
  double value = value_left + (value_right - value_left) * (position - position_left)/(position_right - position_left);
  
  return value;
}

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
    std::array<double, 16> current_state = State.O_T_EE;
    geometry_msgs::Point cur;
    cur.x = current_state[12]; cur.y = current_state[13]; cur.z = current_state[14];
    ros::init(argc,argv,"robotmotion");
    ros::NodeHandle nh;
    ros::Publisher pub_robotstate = nh.advertise<geometry_msgs::Point> ("state",1000);
    pub_robotstate.publish(cur);
    ROS_INFO("Robot's Endeffector's current state has been published with x = %f, y = %f, z = %f", cur.x, cur.y, cur.z);
    ros::Subscriber desetinationSubscriber = nh.subscribe("matlab",10,dest_coord_callback);
    std::cout << "WARNING: This programm will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // cartesianpose movement


    Trajectory::State state_last_x, state_last_y, state_last_z;
    Trajectory::State state_current_x, state_current_y, state_current_z;
    Trajectory::State state_dest_x, state_dest_y, state_dest_z;
    state_dest_x.q = dest_sub.x; state_dest_y.q = dest_sub.y; state_dest_z.q = dest_sub.z;
    ROS_INFO("Robot's Endeffector's current sdestination has been published with x = %f, y = %f, z = %f", dest_sub.x, dest_sub.y, dest_sub.z);
    state_dest_x.dq = 0.0; state_dest_y.dq = 0.0; state_dest_z.dq = 0.0;
    state_dest_x.ddq = 0.0; state_dest_y.ddq = 0.0; state_dest_z.ddq = 0.0;
    state_dest_x.dddq = 0.0; state_dest_y.dddq = 0.0; state_dest_z.dddq = 0.0;
    state_last_x.q = cur.x; state_last_y.q = cur.y; state_last_z.q = cur.z;
    state_last_x.dq = 0.0; state_last_y.dq = 0.0; state_last_z.dq = 0.0;
    state_last_x.ddq = 0.0; state_last_y.ddq = 0.0; state_last_z.ddq = 0.0;
    state_last_x.dddq = 0.0; state_last_y.dddq = 0.0; state_last_z.dddq = 0.0;

    int t_count = 0;
    double secs =ros::Time::now().toSec();
    double secs_last =ros::Time::now().toSec();
    robot.control([&t_count,&secs, &secs_last, &state_last_x,&state_last_y,&state_last_z,&state_current_x, &state_current_y, &state_current_z, &state_dest_x, &state_dest_y, &state_dest_z](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianVelocities {
      ros::spinOnce();
      state_dest_x.q = dest_sub.x; state_dest_y.q = dest_sub.y; state_dest_z.q = dest_sub.z;
      ROS_INFO("Robot's Endeffector's current's destination has been updated with x = %f, y = %f, z = %f", dest_sub.x, dest_sub.y, dest_sub.z); 
      Trajectory plan;

      


      //time stamp
      secs =ros::Time::now().toSec();
      ROS_INFO("Now time is %f miliseconds",secs);
      double delta_t;
      if (t_count == 0){
        delta_t = 999999999999;
        t_count = 1;
        ROS_INFO("Current step is initial step");
      } 
      else {
        delta_t = secs - secs_last;
        ROS_INFO("delta t is %f seconds",delta_t);
      }
      ROS_INFO("delta t is %f seconds",delta_t);
      secs_last = secs; 

    // time += delta_t * 1000;
    // time_counter = round(time);
    // ROS_INFO("time counter is %d miliseconds",time_counter);


      //get current state and dest state for online trajplan
      std::array<double, 16> current_state = robot_state.O_T_EE;
      state_current_x.q = current_state[12]; state_current_y.q = current_state[13]; state_current_z.q = current_state[14];

      double delta_x = decimal3(state_current_x.q - state_last_x.q), delta_y = decimal3(state_current_y.q - state_last_y.q), delta_z = decimal3(state_current_z.q - state_last_z.q);
      state_current_x.dq = delta_x/delta_t; state_current_y.dq = delta_y/delta_t; state_current_z.dq = delta_z/delta_t;

      double delta_vx = decimal3(state_current_x.dq - state_last_x.dq), delta_vy = decimal3(state_current_y.dq - state_last_y.dq), delta_vz = decimal3(state_current_z.dq - state_last_z.dq);
      state_current_x.ddq = delta_vx/delta_t; state_current_y.ddq = delta_vy/delta_t; state_current_z.ddq = delta_vz/delta_t;

      double delta_ax = decimal3(state_current_x.ddq - state_last_x.ddq), delta_ay = decimal3(state_current_y.ddq - state_last_y.ddq), delta_az = decimal3(state_current_z.ddq - state_last_z.ddq);
      state_current_x.dddq = delta_ax/delta_t; state_current_y.dddq = delta_ay/delta_t; state_current_z.dddq = delta_az/delta_t;
      
      state_last_x = state_current_x; state_last_y = state_current_y; state_last_z = state_current_z;
      ROS_INFO("x's current position is %f",state_current_x.q );
      ROS_INFO("x's current estimated velocity is %f",state_current_x.dq );
      ROS_INFO("x's current estimated acceleration is %f",state_current_x.ddq );
      ROS_INFO("x's current estimated jerk is %f",state_current_x.dddq );
      state_current_x = plan.OnlinePlanning(state_current_x, state_dest_x, 0.3, 0.001);
      ROS_INFO("x's planned position is %f",state_current_x.q );
      ROS_INFO("x's planned velocity is %f",state_current_x.dq );
      ROS_INFO("x's planned acceleration is %f",state_current_x.ddq );
      ROS_INFO("x's planned jerk is %f",state_current_x.dddq );
      // state_current_y = plan.OnlinePlanning(state_current_y, state_dest_y, 0.3, 0.001);
      // state_current_z = plan.OnlinePlanning(state_current_z, state_dest_z, 0.3, 0.001);
       


      double v_x,v_y,v_z;
      // update cartesian
      bool position_reached_x = (abs(state_current_x.q - state_dest_x.q) < 0.005);
      bool position_reached_y = (abs(state_current_y.q - state_dest_y.q) < 0.005);
      bool position_reached_z = (abs(state_current_z.q - state_dest_z.q) < 0.005);
      // if position not reached, update the velocity
      if (!position_reached_x) {
        v_x = state_current_x.dq;
        ROS_INFO("x's command velocity is %f",state_current_x.dq );
        ROS_INFO("distance in x is %f",(state_current_x.q - state_dest_x.q));
        // ROS_INFO("x's velocity is %f",v_x);
        // ROS_INFO("x's accerlaration is %f",ddx);
        // ROS_INFO("x's jerk is %f",dddx);
      }
      // if (!position_reached_y) {
      //   v_y = state_current_y.dq;
      //   ROS_INFO("y's command velocity is %f",state_current_y.dq );
      //   ROS_INFO("distance in y is %f",(state_current_y.q - state_dest_y.q));
      //   // ROS_INFO("y's velocity is %f",v_y);
      //   // ROS_INFO("y's accerlaration is %f",ddy);
      //   // ROS_INFO("y's jerk is %f",dddy);
      // }
      // if (!position_reached_z) {
      //   v_z = state_current_z.dq;
      //   ROS_INFO("z's command velocity is %f",state_current_z.dq );
      //   ROS_INFO("distance in z is %f",(state_current_z.q - state_dest_z.q));
      //   //ROS_INFO("z's velocity is %f",v_z);
      //   // ROS_INFO("z's accerlaration is %f",ddz);
      //   // ROS_INFO("z's jerk is %f",dddz);
        
      // }
      franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
      if (position_reached_x & position_reached_x & position_reached_x){
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