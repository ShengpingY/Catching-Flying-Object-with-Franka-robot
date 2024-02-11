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
#include <Eigen/Dense>

class CartesianParabolic {
public:
    RobotMotionController(const std::string& robot_hostname) : robot(robot_hostname) {
        //initialize ros
        ros::init(argc, argv, "robotmotion");
        nh.subscribe("/transformed_coord", 1, &RobotMotionController::callback, this);
    }

    void run() {
        setDefaultBehavior(robot);
        
        // move robot to initial position
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // set Collision behavior
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        // control loop
        try {
            if(msg_received_flag){
                robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
                    // get destination between target position and current position
                    double dx = dest_x - robot_state.O_T_EE[12];
                    double dy = dest_y - robot_state.O_T_EE[13];
                    double dz = dest_z - robot_state.O_T_EE[14];

                    // use jacobian matrix and joints' velocity to get current cartesian velocity
                    std::array<double, 7> joints_velocity = robot_state.dq;
                    Eigen::Vector3d end_effector_velocity = calculateEndEffectorVelocity(jacobian, joints_velocity);
                    double vx = end_effector_velocity[0];
                    double vy = end_effector_velocity[1];
                    double vz = end_effector_velocity[2];

                    // update speed
                    double v_x = update_velocity(vx, dx, a, t, vmax);
                    double v_y = update_velocity(vy, dy, a, t, vmax);
                    double v_z = update_velocity(vz, dz, a, t, vmax);

                    franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};

                    // check if target position is reached
                    if (std::abs(dx) <= 0.005) & (std::abs(dy) <= 0.005) & (std::abs(dz)<= 0.005) {
                        std::cout << std::endl << "Position reached! Holding Position" << std::endl;
                        return franka::MotionFinished(output);
                    }

                    return output;
                });
                msg_received_flag = false;
            } 
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

private:
    franka::Robot robot;
    ros::NodeHandle nh;
    double dest_x, dest_y, dest_z;
    const double a = 10;
    const double vmax = 3;
    const double t = 0.001;
    bool msg_received_flag = false;

    // callback function for subscriber
    void callback(const geometry_msgs::Point& msg) {
        msg_received_flag = true;
        dest_x = msg.x;
        dest_y = msg.y;
        dest_z = msg.z;
        ROS_INFO("Destination received is %f,%f,%f", msg.x, msg.y, msg.z);
    }

    // function definition of speed update
    double update_velocity(double v, double ds, double a, double dt, double vmax) {
        if (v * ds < 0) {
            if (v >= 0) {
                return v - a * dt;
            } else {
                return v + a * dt;
            }
        } else {
            if (0.5 * v * v / a < std::abs(ds)) {
                if (v >= 0) {
                    return std::min(vmax, v + a * dt);
                } else {
                    return std::max(-vmax, v - a * dt);
                }
            } else {
                if (v >= 0) {
                    return v - a * dt;
                } else {
                    return v + a * dt;
                }
            }
        }
    }
};

