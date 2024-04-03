#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>


#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include "examples_common.h"
#include "examples_common.cpp"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include "std_msgs/Float32MultiArray.h"





void array_2_vec_7(std::vector<double>& vec, const std::array<double,7>& arr) {
  for (const auto& elem : arr) {
    vec.push_back(elem);
  }
}

void array_2_vec_16(std::vector<double>& vec, const std::array<double,16>& arr) {
  for (const auto& elem : arr) {
    vec.push_back(elem);
  }
}

void CSV_write_matrix(const std::vector<std::vector<double>>& matrix, const std::string& file_path){
	//check whether file exiets
	std::ifstream checkFile(file_path);
	if (!checkFile){
		std::ofstream createFile(file_path);
		createFile.close();
	}
	//open the file in output mode
	std::ofstream file(file_path);
	//check if the file is open successfully
	if (file.is_open()){
        // delete all data there
        std::ofstream file(file_path, std::ofstream::trunc); // delete all data there
		//loop through matrix
		for(size_t i=0; i<matrix.size();i++){
			for (size_t j = 0; j < matrix[i].size(); j++){
				//write each element to the file,sperated by commas
				file << matrix[i][j];
				//add a comma after each element except the last one
				if (j < matrix[i].size()-1){
					file << ",";
				}
			}
			// new line after each row except the last one
            if (i < matrix.size()-1){
                file << "\n";
            }
		}
		// close the file
		file.close();
	}

	else{
	//handle the error
	std::cerr << "Could not open the file" << file_path << "\n";
	}

}


class RobotMotionController {
private:
    franka::Robot robot;
    
    double dest_x, dest_y, dest_z;
    const double a = 0.6;
    const double vmax = 1;
    // const double t = 0.001;
    bool msg_received_flag;
    // const double dest_x = 0.4;
    // const double dest_y = 0.4;
    // const double dest_z = 0.4;

public:
    RobotMotionController(const std::string& hostname,int argc, char **argv) : robot(hostname), msg_received_flag(false) {
        //initialize ros
        ros::init(argc, argv, "robotmotion");
        ros::NodeHandle nh;
        ros::Subscriber destinationSubscriber = nh.subscribe("/transformed_coord_mp", 1, &RobotMotionController::callback, this);
        // ros::Subscriber destinationSubscriber = nh.subscribe("/matlab", 1, &RobotMotionController::callback, this);
        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel();
        // franka::RobotState State =  robot.readOnce();
        // move robot to initial position
        // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        std::array<double, 7> q_goal = {{0, -0.7894, 0, -2.8395, 0, 2.0495, 0.785}};
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
        

        std::cin.ignore();

        while(!msg_received_flag) {
            ros::spinOnce();
        }

        

        // control loop

        while(ros::ok()) {
            try { 
                if((std::abs(dest_x) > 0.68 && std::abs(dest_y) > 0.68) or (std::abs(dest_x) > 0.68 && std::abs(dest_z) > 0.53) or (std::abs(dest_y) > 0.68 && std::abs(dest_z) > 0.53)
                    or (dest_x *dest_x + dest_y * dest_y + dest_z * dest_z) > 1 ){
                    ROS_INFO("destination out of reach");
                    ros::spinOnce();  
                    continue;
                }
                if(msg_received_flag){
                    double time=0;
                    double destination_x = dest_x - 0.03; // + 0.0x Offset necessary for better accuracy
                    double destination_y = dest_y - 0.03; //+ 0.0x Offset necessary for better accuracy
                    double destination_z = dest_z;
                    franka::RobotState State_current =  robot.readOnce();
                    double dx = destination_x - State_current.O_T_EE[12];
                    double dy = destination_y - State_current.O_T_EE[13];
                    double dz = destination_z - State_current.O_T_EE[14];
                    double time_total_x = sqrt(std::abs(dx)/a);
                    double time_total_y = sqrt(std::abs(dy)/a);
                    double time_total_z = sqrt(std::abs(dz)/a);
                    double time_total = time_total_x;
                    double v_x;
                    double v_y;
                    double v_z;
                    double a_x, a_y, a_z;
                    a_x = a;
                    a_y = a * (abs(dy)/abs(dx));
                    a_z = a * (abs(dz)/abs(dx));                    
                    ROS_INFO("timex is %f",time_total_x);
                    ROS_INFO("timey is %f",time_total_y);
                    ROS_INFO("timez is %f",time_total_z);
                    double kx = 1, ky = 1, kz = 1;
                    if (dx <= 0) { kx = -1;}
                    if (dy <= 0) { ky = -1;}
                    if (dz <= 0) { kz = -1;}

                    std::vector<std::vector<double>> joint_position;
                    std::vector<std::vector<double>> joint_velocity;
                    std::vector<std::vector<double>> ef_position;


                    std::array<double, 7> current_position;
                    std::array<double, 7> current_velocity;

                    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        ros::spinOnce();
                        // std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                        // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                        // // get destination between target position and current position
                        

                        // // use jacobian matrix and joints' velocity to get current cartesian velocity
                        // std::array<double, 7> joints_velocity = robot_state.dq;
                        // Eigen::Vector3d end_effector_velocity = calculateEndEffectorVelocity(jacobian, joints_velocity);
                        // double vx = end_effector_velocity[0];
                        // double vy = end_effector_velocity[1];
                        // double vz = end_effector_velocity[2];

                        // ROS_INFO("current calculated vx is %f",vx);

                        double t = period.toSec();
                        time = time + t;
                        // ROS_INFO("time is %f",time);
                        //update speed
                        // double v_x = update_velocity(vx, dx, a, t, vmax);
                        // ROS_INFO("vx for next step is %f",v_x);
                        // double v_y = update_velocity(vy, dy, a, t, vmax);
                        // double v_z = update_velocity(vz, dz, a, t, vmax);
                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        double v_x, v_y, v_z;
 

                        if (time >= time_total) {
                            if(time <= time_total * 2) {
                                v_x = a * time_total - a_x * (time - time_total);
                            }
                            else{
                                v_x = 0;
                            }
                        }
                        else {
                            v_x = a_x * time;
                        }
                        // ROS_INFO("v_x is %f",v_x);

                        if (time >= time_total) {
                            if(time <= time_total * 2) {
                                v_y = a_y * time_total - a_y * (time - time_total);
                            }
                            else{
                                v_y = 0;
                            }
                        }
                        else {v_y = a_y * time;}
                        // ROS_INFO("v_y is %f",v_y);

                        if (time >= time_total) {
                            if(time <= time_total * 2) {
                                v_z = a_z * time_total - a_z * (time - time_total);
                            }
                            else{
                                v_z = 0;
                            }
                        }
                        else {v_z = a_z * time;}
                        // ROS_INFO("v_z is %f",v_z);

                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        franka::CartesianVelocities output = {{kx * v_x, ky * v_y, kz * v_z, 0.0, 0.0, 0.0}};

                        // get current distance
                        double dist_x = destination_x - robot_state.O_T_EE[12];
                        double dist_y = destination_y - robot_state.O_T_EE[13];
                        double dist_z = destination_z - robot_state.O_T_EE[14];
                        
                        // // following code for data collection
                        // current_position = robot_state.q;
                        // current_velocity = robot_state.dq;
                        // std::array<double, 16> current_pose = robot_state.O_T_EE;
                        // // store them in matrix
                        // std::vector<double> vec_cur_jposition;
                        // std::vector<double> vec_cur_jvelocity;
                        // std::vector<double> vec_cur_efposition;
                        // array_2_vec_7(vec_cur_jposition,current_position );
                        // array_2_vec_7(vec_cur_jvelocity,current_velocity);
                        // array_2_vec_16(vec_cur_efposition,current_pose);
                        // joint_position.push_back(vec_cur_jposition);
                        // joint_velocity.push_back(vec_cur_jvelocity);
                        // ef_position.push_back(vec_cur_efposition);

                        // check if target position is reached
                        if ((std::abs(dist_x) <= 0.001) & (std::abs(dist_y) <= 0.001) & (std::abs(dist_z)<= 0.001)) {
                            std::cout << std::endl << "Position reached! Holding Position" << std::endl;
                            ROS_INFO("Current Joint Position is %f,%f,%f,%f,%f,%f,%f", robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6]);
                            // CSV_write_matrix(joint_position,"./src/robotmotion/src/data/jointposition_car");
                            // CSV_write_matrix(joint_velocity,"./src/robotmotion/src/data/jointvelocity_car");
                            // CSV_write_matrix(ef_position,"./src/robotmotion/src/data/EndEffectorposition_car");
                            return franka::MotionFinished(output);
                            ROS_INFO("Current Position is %f,%f,%f", robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
                        }

                        return output;
                    });
                    // msg_received_flag = false;

                }
            } catch (const franka::Exception& e) {
                std::cout << e.what() << std::endl;
                ROS_INFO("Current Position is %f,%f,%f", robot.readOnce().O_T_EE[12], robot.readOnce().O_T_EE[13], robot.readOnce().O_T_EE[14]);

                robot.automaticErrorRecovery();
                // std::cin.ignore();
            }
        }
            
        
    }

private:


    // callback function for subscriber
    void callback(const geometry_msgs::Point& msg) {
        msg_received_flag = true;
        dest_x = msg.x;
        dest_y = msg.y;
        dest_z = msg.z;
        // ROS_INFO("Destination received is %f,%f,%f", msg.x, msg.y, msg.z);
    }

    // function to calculate Endeffector velocity
    Eigen::Vector3d calculateEndEffectorVelocity(const Eigen::MatrixXd& jacobian, const std::array<double,7>& joints_velocity) {
        Eigen::Map<const Eigen::VectorXd> joints_velocity_eigen(joints_velocity.data(),7);

        Eigen::VectorXd end_effector_velocity = jacobian * joints_velocity_eigen;
        return end_effector_velocity.head(3);
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
class RobotMotionController_Trig {
private:
    franka::Robot robot;
    
    double dest_x, dest_y, dest_z;
    double t_target;
    // const double a = 0.3;
    // const double vmax = 1;
    const double t_set = 0.8; //define target time restriction for trignomic interpolation
    bool msg_received_flag;
    // const double dest_x = 0.4;
    // const double dest_y = 0.4;
    // const double dest_z = 0.4;

public:
    RobotMotionController_Trig(const std::string& hostname,int argc, char **argv) : robot(hostname), msg_received_flag(false) {
        //initialize ros
        ros::init(argc, argv, "robotmotion");
        ros::NodeHandle nh;
        ros::Subscriber destinationSubscriber = nh.subscribe("/transformed_coord_mp", 1, &RobotMotionController_Trig::callback, this);

        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel();
        // franka::RobotState State =  robot.readOnce();
        // move robot to initial position
        // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        std::array<double, 7> q_goal = {{0, -0.7894, 0, -2.8395, 0, 2.0495, 0.785}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration."
                  << "Press Enter to continue..."  << std::endl;

        // set Collision behavior
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
        

        std::cin.ignore();

        while(!msg_received_flag) {
            ros::spinOnce();
        }

        

        // control loop
        while(ros::ok()) {
            try { 
                if((std::abs(dest_x) > 0.68 && std::abs(dest_y) > 0.68) or (std::abs(dest_x) > 0.68 && std::abs(dest_z) > 0.53) or (std::abs(dest_y) > 0.68 && std::abs(dest_z) > 0.53)
                    or (dest_x *dest_x + dest_y * dest_y + dest_z * dest_z) > 0.68 ){
                    ROS_INFO("destination out of reach");
                    ros::spinOnce();  
                    continue;
                }
                if(msg_received_flag){
                    
                    // get target position
                    double destination_x = dest_x - 0.03;
                    double destination_y = dest_y - 0.03;
                    double destination_z = dest_z;
                    // ger current position
                    franka::RobotState State_current =  robot.readOnce();
                    double dx = destination_x - State_current.O_T_EE[12];
                    double dy = destination_y - State_current.O_T_EE[13];
                    double dz = destination_z - State_current.O_T_EE[14];
                    double vmax_x = 2 * dx / t_target;
                    double vmax_y = 2 * dy / t_target;
                    double vmax_z = 2 * dz / t_target;
                    
                    double max_distance = std::abs(dx);
                    if(std::abs(dy) > max_distance) {
                        max_distance = std::abs(dy);
                    }
                    if(std::abs(dz) > max_distance) {
                        max_distance = std::abs(dz);
                    }
                    ROS_INFO("max_distance is  %f",max_distance);
                    t_target = max_distance / 0.3 * t_set; 
                    // t_target = t_set;
                    ROS_INFO("t_target is %f",t_target);

                    // double time_total_x = sqrt(std::abs(dx)/a);
                    // double time_total_y = sqrt(std::abs(dy)/a);
                    // double time_total_z = sqrt(std::abs(dz)/a);
                    // double time_total = time_total_x;
                    // double v_x;
                    // double v_y;
                    // double v_z;
                    // double a_x, a_y, a_z;
                    // a_x = a;
                    // a_y = a * (abs(dy)/abs(dx));
                    // a_z = a * (abs(dz)/abs(dx));                    
                    ROS_INFO("current position is x = %f, y = %f, z = %f",State_current.O_T_EE[12],State_current.O_T_EE[13],State_current.O_T_EE[14]);
                    ROS_INFO("current destination is x = %f, y = %f, z = %f",destination_x,destination_y,destination_z);

                    // double kx = 1, ky = 1, kz = 1;
                    // if (dx <= 0) { kx = -1;}
                    // if (dy <= 0) { ky = -1;}
                    // if (dz <= 0) { kz = -1;}

                    std::vector<std::vector<double>> joint_position;
                    std::vector<std::vector<double>> joint_velocity;
                    std::vector<std::vector<double>> ef_position;


                    std::array<double, 7> current_position;
                    std::array<double, 7> current_velocity;
                    double time=0;
                    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        ros::spinOnce();
                        // std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                        // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                        // // get destination between target position and current position
                        

                        // // use jacobian matrix and joints' velocity to get current cartesian velocity
                        // std::array<double, 7> joints_velocity = robot_state.dq;
                        // Eigen::Vector3d end_effector_velocity = calculateEndEffectorVelocity(jacobian, joints_velocity);
                        // double vx = end_effector_velocity[0];
                        // double vy = end_effector_velocity[1];
                        // double vz = end_effector_velocity[2];

                        // ROS_INFO("current calculated vx is %f",vx);

                        double t = period.toSec();
                        time = time + t;
                        // ROS_INFO("time is %f",time);
                        //update speed
                        // double v_x = update_velocity(vx, dx, a, t, vmax);
                        // ROS_INFO("vx for next step is %f",v_x);
                        // double v_y = update_velocity(vy, dy, a, t, vmax);
                        // double v_z = update_velocity(vz, dz, a, t, vmax);
                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        double v_x, v_y, v_z;
                        // velocity profile in sin
                        // v_x = -dx/2*M_PI/t_target*sin(M_PI/t_target*time+M_PI); 
                        // v_y = -dy/2*M_PI/t_target*sin(M_PI/t_target*time+M_PI); 
                        // v_z = -dz/2*M_PI/t_target*sin(M_PI/t_target*time+M_PI); 

                        // velocity profile in cos, stop at max velocity
                        // v_x = -(2*M_PI/t_target*dx/2*cos(2*M_PI/t_target*time)-2*M_PI/t_target*dx/2);
                        // v_y = -(2*M_PI/t_target*dy/2*cos(2*M_PI/t_target*time)-2*M_PI/t_target*dy/2);
                        // v_z = -(2*M_PI/t_target*dz/2*cos(2*M_PI/t_target*time)-2*M_PI/t_target*dz/2);
                        
                        v_x = 2 * dx * M_PI / t_target * sin(M_PI/t_target*time) * cos(M_PI/t_target*time);
                        v_y = 2 * dy * M_PI / t_target * sin(M_PI/t_target*time) * cos(M_PI/t_target*time);
                        v_z = 2 * dz * M_PI / t_target * sin(M_PI/t_target*time) * cos(M_PI/t_target*time);

 



                        // ROS_INFO("v_z is %f",v_z);

                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        ROS_INFO("calculated velocity is vx = %f, vy = %f, vz = %f",v_x,v_y,v_z);
                        franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};

                        // get current distance
                        double dist_x = destination_x - robot_state.O_T_EE[12];
                        double dist_y = destination_y - robot_state.O_T_EE[13];
                        double dist_z = destination_z - robot_state.O_T_EE[14];
                        ROS_INFO("current distance is dx = %f, dy = %f, dz = %f",dist_x,dist_y,dist_z);
                        // following code for data collection
                        current_position = robot_state.q;
                        current_velocity = robot_state.dq;
                        std::array<double, 16> current_pose = robot_state.O_T_EE;

                        // store them in matrix
                        std::vector<double> vec_cur_jposition;
                        std::vector<double> vec_cur_jvelocity;
                        std::vector<double> vec_cur_efposition;
                        array_2_vec_7(vec_cur_jposition,current_position );
                        array_2_vec_7(vec_cur_jvelocity,current_velocity);
                        array_2_vec_16(vec_cur_efposition,current_pose);
                        joint_position.push_back(vec_cur_jposition);
                        joint_velocity.push_back(vec_cur_jvelocity);
                        ef_position.push_back(vec_cur_efposition);

                        // check if target position is reached
                        // double dt = time - t_target/2;
                        // if (dt >= 0) {
                        if ((std::abs(dist_x) <= 0.001) & (std::abs(dist_y) <= 0.001) & (std::abs(dist_z)<= 0.001)) {
                            std::cout << std::endl << "Position reached! Holding Position" << std::endl;
                            ROS_INFO("Current Joint Position is %f,%f,%f,%f,%f,%f,%f", robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6]);
                            CSV_write_matrix(joint_position,"./src/robotmotion/src/data/jointposition_car_trig");
                            CSV_write_matrix(joint_velocity,"./src/robotmotion/src/data/jointvelocity_car_trig");
                            CSV_write_matrix(ef_position,"./src/robotmotion/src/data/EndEffectorposition_car_trig");
                            return franka::MotionFinished(output);
                            ROS_INFO("Current Position is %f,%f,%f", robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
                        }

                        return output;
                    });
                    // msg_received_flag = false;

                }
            } catch (const franka::Exception& e) {
                std::cout << e.what() << std::endl;
                ROS_INFO("Current Position is %f,%f,%f", robot.readOnce().O_T_EE[12], robot.readOnce().O_T_EE[13], robot.readOnce().O_T_EE[14]);

                robot.automaticErrorRecovery();
                // std::cin.ignore();
            }
        }
            
        
    }

private:
    // callback function for subscriber
    void callback(const geometry_msgs::Point& msg) {
        msg_received_flag = true;
        dest_x = msg.x;
        dest_y = msg.y;
        dest_z = msg.z;
        // ROS_INFO("Destination received is %f,%f,%f", msg.x, msg.y, msg.z);
    }

    // function to calculate Endeffector velocity
    Eigen::Vector3d calculateEndEffectorVelocity(const Eigen::MatrixXd& jacobian, const std::array<double,7>& joints_velocity) {
        Eigen::Map<const Eigen::VectorXd> joints_velocity_eigen(joints_velocity.data(),7);

        Eigen::VectorXd end_effector_velocity = jacobian * joints_velocity_eigen;
        return end_effector_velocity.head(3);
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
class RobotMotionController_constantjerk {
private:
    franka::Robot robot;
    
    double dest_x, dest_y, dest_z;
    const double j = 7.2;
    // const double t = 0.001;
    bool msg_received_flag;
    // const double dest_x = 0.4;
    // const double dest_y = 0.4;
    // const double dest_z = 0.4;

public:
    RobotMotionController_constantjerk(const std::string& hostname,int argc, char **argv) : robot(hostname), msg_received_flag(false) {
        //initialize ros
        ros::init(argc, argv, "robotmotion");
        ros::NodeHandle nh;
        ros::Subscriber destinationSubscriber = nh.subscribe("/transformed_coord_mp", 1, &RobotMotionController_constantjerk::callback, this);
        // ros::Subscriber destinationSubscriber = nh.subscribe("/matlab", 1, &RobotMotionController_constantjerk::callback, this);
        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel();
        // franka::RobotState State =  robot.readOnce();
        // move robot to initial position
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        // std::array<double, 7> q_goal = {{0, -0.7894, 0, -2.8395, 0, 2.0495, 0.785}};
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
        

        std::cin.ignore();

        while(!msg_received_flag) {
            ros::spinOnce();
        }

        // control loop

        while(ros::ok()) {
            try { 
                if((std::abs(dest_x) > 0.68 && std::abs(dest_y) > 0.68) or (std::abs(dest_x) > 0.68 && std::abs(dest_z) > 0.53) or (std::abs(dest_y) > 0.68 && std::abs(dest_z) > 0.53)
                    or (dest_x *dest_x + dest_y * dest_y + dest_z * dest_z) > 1 ){
                    ROS_INFO("destination out of reach");
                    ros::spinOnce();  
                    continue;
                }
                if(msg_received_flag){
                    
                    double destination_x = dest_x + 0.03; // + 0.0x Offset necessary for better accuracy
                    double destination_y = dest_y - 0.05; //+ 0.0x Offset necessary for better accuracy
                    double destination_z = dest_z;
                    franka::RobotState State_current =  robot.readOnce();
                    double dx = destination_x - State_current.O_T_EE[12];
                    double dy = destination_y - State_current.O_T_EE[13];
                    double dz = destination_z - State_current.O_T_EE[14];

                    // double time_total = time_total_x;
                    // double v_x;
                    // double v_y;
                    // double v_z;
                    // double a_x, a_y, a_z;
                    // a_x = a;
                    // a_y = a * (abs(dy)/abs(dx));
                    // a_z = a * (abs(dz)/abs(dx));                    
                    // ROS_INFO("timex is %f",time_total_x);
                    // ROS_INFO("timey is %f",time_total_y);
                    // ROS_INFO("timez is %f",time_total_z);
                    double kx = 1, ky = 1, kz = 1;
                    if (dx <= 0) { kx = -1;}
                    if (dy <= 0) { ky = -1;}
                    if (dz <= 0) { kz = -1;}
                    // ROS_INFO("step 1.0");
                    double time_total_x = cal_tmax(kx * dx,j);
                    // ROS_INFO("step 1");
                    double time_total_y = cal_tmax(ky * dy,j);
                    // ROS_INFO("step 1.1");
                    double time_total_z = cal_tmax(kz * dz,j);
                    // ROS_INFO("step 1.2");
                    std::vector<std::vector<double>> joint_position;
                    std::vector<std::vector<double>> joint_velocity;
                    std::vector<std::vector<double>> ef_position;


                    std::array<double, 7> current_position;
                    std::array<double, 7> current_velocity;

                    double time=0;
                    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
                        ros::spinOnce();
                        // std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                        // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

                        // // get destination between target position and current position
                        

                        // // use jacobian matrix and joints' velocity to get current cartesian velocity
                        // std::array<double, 7> joints_velocity = robot_state.dq;
                        // Eigen::Vector3d end_effector_velocity = calculateEndEffectorVelocity(jacobian, joints_velocity);
                        // double vx = end_effector_velocity[0];
                        // double vy = end_effector_velocity[1];
                        // double vz = end_effector_velocity[2];

                        // ROS_INFO("current calculated vx is %f",vx);

                        double t = period.toSec();
                        time = time + t;
                        // ROS_INFO("time is %f",time);
                        //update speed
                        // double v_x = update_velocity(vx, dx, a, t, vmax);
                        // ROS_INFO("vx for next step is %f",v_x);
                        // double v_y = update_velocity(vy, dy, a, t, vmax);
                        // double v_z = update_velocity(vz, dz, a, t, vmax);
                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        double v_x, v_y, v_z;
                        // ROS_INFO("step 2");
                        v_x = cal_velocity(time,time_total_x,j);
                        // ROS_INFO("step 2.1");
                        v_y = cal_velocity(time,time_total_y,j);
                        // ROS_INFO("step 2.2");
                        v_z = cal_velocity(time,time_total_z,j);
                        // ROS_INFO("step 2.3");

                        // ROS_INFO("v_z is %f",v_z);

                        // double v_x = std::min(time,0.16);
                        // double v_y = std::min(time,0.16);
                        // double v_z = std::min(time,0.16);
                        franka::CartesianVelocities output = {{kx * v_x, ky * v_y, kz * v_z, 0.0, 0.0, 0.0}};

                        // get current distance
                        double dist_x = destination_x - robot_state.O_T_EE[12];
                        double dist_y = destination_y - robot_state.O_T_EE[13];
                        double dist_z = destination_z - robot_state.O_T_EE[14];
                        
                        // // following code for data collection
                        // current_position = robot_state.q;
                        // current_velocity = robot_state.dq;
                        // std::array<double, 16> current_pose = robot_state.O_T_EE;
                        // // store them in matrix
                        // std::vector<double> vec_cur_jposition;
                        // std::vector<double> vec_cur_jvelocity;
                        // std::vector<double> vec_cur_efposition;
                        // array_2_vec_7(vec_cur_jposition,current_position );
                        // array_2_vec_7(vec_cur_jvelocity,current_velocity);
                        // array_2_vec_16(vec_cur_efposition,current_pose);
                        // joint_position.push_back(vec_cur_jposition);
                        // joint_velocity.push_back(vec_cur_jvelocity);
                        // ef_position.push_back(vec_cur_efposition);

                        // check if target position is reached
                        if ((std::abs(dist_x) <= 0.001) & (std::abs(dist_y) <= 0.001) & (std::abs(dist_z)<= 0.001)) {
                            std::cout << std::endl << "Position reached! Holding Position" << std::endl;
                            ROS_INFO("Current Joint Position is %f,%f,%f,%f,%f,%f,%f", robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6]);
                            // CSV_write_matrix(joint_position,"./src/robotmotion/src/data/jointposition_car");
                            // CSV_write_matrix(joint_velocity,"./src/robotmotion/src/data/jointvelocity_car");
                            // CSV_write_matrix(ef_position,"./src/robotmotion/src/data/EndEffectorposition_car");
                            return franka::MotionFinished(output);
                            ROS_INFO("Current Position is %f,%f,%f", robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
                        }

                        return output;
                    });
                    // msg_received_flag = false;

                }
            } catch (const franka::Exception& e) {
                std::cout << e.what() << std::endl;
                ROS_INFO("Current Position is %f,%f,%f", robot.readOnce().O_T_EE[12], robot.readOnce().O_T_EE[13], robot.readOnce().O_T_EE[14]);

                robot.automaticErrorRecovery();
                // std::cin.ignore();
            }
        }
            
        
    }

private:


    // callback function for subscriber
    void callback(const geometry_msgs::Point& msg) {
        msg_received_flag = true;
        dest_x = msg.x;
        dest_y = msg.y;
        dest_z = msg.z;
        // ROS_INFO("Destination received is %f,%f,%f", msg.x, msg.y, msg.z);
    }

    // function to calculate Endeffector velocity
    Eigen::Vector3d calculateEndEffectorVelocity(const Eigen::MatrixXd& jacobian, const std::array<double,7>& joints_velocity) {
        Eigen::Map<const Eigen::VectorXd> joints_velocity_eigen(joints_velocity.data(),7);

        Eigen::VectorXd end_effector_velocity = jacobian * joints_velocity_eigen;
        return end_effector_velocity.head(3);
    }

    // totaltime
    
    double cal_tmax(double distance, double j) {
        return pow(32*distance/j,1.0/3.0);
    }

    double cal_velocity(double t, double tmax, double j) {
        if(t > tmax) {
            return cal_velocity(tmax, tmax, j);
        }
        if (t <= 0.25 * tmax) {
            return 0.5 * j * t * t;
        }
        else if (t <= 0.5 * tmax) {
            return 0.5 * j * (0.25 * tmax) * (0.25 * tmax) + (j * (0.25 * tmax) + j * (0.25 * tmax) + j * (0.25 * tmax) - j * t) * 0.5 * (t-0.25*tmax);
        }
        else {
            return cal_velocity(tmax-t, tmax, j);
        }
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

    // 
};

