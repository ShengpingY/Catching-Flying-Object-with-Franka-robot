#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include "examples_common.h"
#include "examples_common.cpp"



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
    

    // std::array<double, 7> q_target = {{0.5327, 0.087, 0.002, -1.521, 0.005, 1.588, 1.316}};
    // std::array<double, 7> q_target = {{-0.5, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // double total_time = 2;
    // franka::RobotState initial_state = robot.readOnce();
    // std::array<double, 7> initial_position = initial_state.q;
    // std::array<double, 7> delta_q;
    // for (size_t i = 0; i < q_target.size(); ++i) {
    //   delta_q[i] = q_target[i] - initial_position[i];
    // }

    // std::array<double, 7> a;
    // for (size_t i = 0; i < 7; i++) {
    //   a[i] = 2 * delta_q[i] / std::pow(total_time, 2);
    // }

    std::vector<std::vector<double>> joint_position;
    std::vector<std::vector<double>> joint_velocity;
    std::vector<std::vector<double>> ef_position;


    std::array<double, 7> current_position;
    std::array<double, 7> current_velocity;

    // double time = 0.0;
    // robot.control([&](const franka::RobotState& robot_state,
    //                                          franka::Duration period) -> franka::JointPositions {
    //   time += period.toSec();
    //   if (time == 0.0) {
    //     initial_position = robot_state.q_d;
    //   }
    //   std::cout << "Current time is" << time << "second" << std::endl;


    //   franka::JointPositions output = robot_state.q_d;
    //   if (time < total_time/2) {
    //     std::array<double, 7> desired_q;
    //     for (size_t i = 0; i < 7; i++) {
    //       desired_q[i] = initial_position[i] + 0.5 * a[i] * std::pow(time, 2);
    //     }

    //     // read the current state, postion and velocity of each joint and also position of end effector's frame
    //     current_position = robot_state.q;
    //     current_velocity = robot_state.dq;
    //     std::array<double, 16> current_pose = robot_state.O_T_EE;
    //     // store them in matrix
    //     std::vector<double> vec_cur_jposition;
    //     std::vector<double> vec_cur_jvelocity;
    //     std::vector<double> vec_cur_efposition;
    //     array_2_vec_7(vec_cur_jposition,current_position );
    //     array_2_vec_7(vec_cur_jvelocity,current_velocity);
    //     array_2_vec_16(vec_cur_efposition,current_pose);


    //     joint_position.push_back(vec_cur_jposition);
    //     joint_velocity.push_back(vec_cur_jvelocity);
    //     ef_position.push_back(vec_cur_efposition);


    //     franka::JointPositions output = {{desired_q[0],desired_q[1],desired_q[2],desired_q[3],desired_q[4],desired_q[5],desired_q[6]}};
    //     return output;
    //   } else if ((time >= total_time/2) & (time < total_time)){
    //     std::array<double, 7> desired_q;
    //     for (size_t i = 0; i < 7; i++) {
    //       desired_q[i] = initial_position[i] + delta_q[i] - 0.5 * a[i] * std::pow(total_time - time, 2);
    //     }
    //     std::cout << "Desired q is" << desired_q[1] << "second" << std::endl;

        // // read the current state, postion and velocity of each joint and also position of end effector's frame
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

    //     franka::JointPositions output = {{desired_q[0],desired_q[1],desired_q[2],desired_q[3],desired_q[4],desired_q[5],desired_q[6]}};
    //     return output;
    //   }
    //   if (time >= total_time) {
    //     franka::JointPositions output = robot_state.q_d;
    //     CSV_write_matrix(joint_position,"./src/robotmotion/src/data/jointposition");
    //     CSV_write_matrix(joint_velocity,"./src/robotmotion/src/data/jointvelocity");
    //     CSV_write_matrix(ef_position,"./src/robotmotion/src/data/EndEffectorposition");
    //     std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
    //     return franka::MotionFinished(output);
    //   }


    

      
    //   return output;
    // });
    std::array<double, 7> initial_position;
    double time = 0.0;
    robot.control([&](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0] + 0.5 * delta_angle, initial_position[1] + 0.5 * delta_angle,
                                        initial_position[2] + delta_angle, initial_position[3] +  delta_angle,
                                        initial_position[4] , initial_position[5],
                                        initial_position[6] }};
        // read the current state, postion and velocity of each joint and also position of end effector's frame
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

      if (time >= 2.5) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        std::cout << std::endl << "Current Positon is "<< robot_state.O_T_EE[12]<< ", " << robot_state.O_T_EE[13]<< "," <<robot_state.O_T_EE[14] << std::endl;
        CSV_write_matrix(joint_position,"./src/robotmotion/src/data/jointposition");
        CSV_write_matrix(joint_velocity,"./src/robotmotion/src/data/jointvelocity");
        CSV_write_matrix(ef_position,"./src/robotmotion/src/data/EndEffectorposition");
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