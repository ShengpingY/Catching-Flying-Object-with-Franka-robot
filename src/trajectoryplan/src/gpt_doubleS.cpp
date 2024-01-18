#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
// #include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Point.h"
#include <unistd.h> // for sleep

std::vector<double> getColumn(const std::vector<std::vector<double>>& matrix, size_t column_num){
	std::vector<double> column;
	for(const auto& row : matrix){
		//check if there is enough column
		if(column_num < row.size()){
			column.push_back(row[column_num]);
		}else {
			std::cerr << "Column index out of range." << std::endl;
			return {};
		}
	}
	return column;
}

// class MySubscriber : public rclcpp::Node {
// public:
//   MySubscriber() : Node("traj_subscriber") {
//     // Create a subscriber for the "/matlab" topic
//     subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
//         "/matlab", 10, std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));// Define the callback function,10 is the queue size,bind is to bind the callback function to the subscriber
//   }

//   geometry_msgs::msg::Point getLastPoint(){
// 	  return last_received_point;
//   }

// private:
//   void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
//     // Process the received Point message
//     RCLCPP_INFO(get_logger(), "Received Point message: (%f, %f, %f)", msg->x, msg->y, msg->z); // Print the received Point message
//     last_received_point = *msg;
//     //writeCoordinatesToCSV(msg,"/home/panda/Project_ws/src/test/src/coordinates.csv");
//     // You can use the received Point data in your main function here
//     // execute(*msg);
//   }
//   rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_; // Define the subscriber
//   geometry_msgs::msg::Point last_received_point;
// };

struct Coordinates{
	double x;
	double y;
	double z;
};

Coordinates readLastCoordinatesFromCSV(const std::string& filePath){
    Coordinates last_coord = {0.0, 0.0, 0.0};

    std::ifstream file(filePath, std::ios::ate); // Open file at the end
    if (file.is_open()) {
        char ch;
        long long endPos = file.tellg(); // Get current position (end of file)
        long long curPos = endPos;

        while (curPos > 0) { // While not at start of file
            file.seekg(--curPos); // Move one character back
            file.get(ch); // Read the character

            if (ch == '\n' && curPos != endPos - 1) { // If newline is found and it's not the last character in the file
                break; // Stop, curPos is at the start of the last line
            }
        }

        std::string line;
        std::getline(file, line); // Read the last line

        std::istringstream iss(line); // Convert string to stream
        std::string x_str, y_str, z_str; // Store the data in string

        if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',') && std::getline(iss, z_str)) { // Read the data from stream
            last_coord.x = std::stod(x_str);
            last_coord.y = std::stod(y_str);
            last_coord.z = std::stod(z_str);
        }
    }

    return last_coord;
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

std::vector<std::vector<double>> DoubleSTrajectoryFunction(
    double T_s, double q_0, double q_1,
    double v_0, double v_1, double v_max, double v_min,
    double a_max, double a_min, double j_max, double j_min
) {
	std::vector<std::vector<double>> matrix_res;


    double Ts = T_s;

    double qd_0 = q_0, qd_1 = q_1;
    double vd_0 = v_0, vd_1 = v_1;

    double vd_max = v_max, vd_min = v_min;
    double ad_max = a_max, ad_min = a_min;
    double jd_max = j_max, jd_min = j_min;

    // Given the initial conditions (qd_0, qd_1, vd_0, vd_1), compute
    // BLOCK 1
    double sigma = std::copysign(1.0, qd_1 - qd_0);

    q_0 = sigma * qd_0;
    q_1 = sigma * qd_1;
    v_0 = sigma * vd_0;
    v_1 = sigma * vd_1;

    v_max = (sigma + 1) / 2 * vd_max + (sigma - 1) / 2 * vd_min;
    v_min = (sigma + 1) / 2 * vd_min + (sigma - 1) / 2 * vd_max;
    a_max = (sigma + 1) / 2 * ad_max + (sigma - 1) / 2 * ad_min;
    a_min = (sigma + 1) / 2 * ad_min + (sigma - 1) / 2 * ad_max;
    j_max = (sigma + 1) / 2 * jd_max + (sigma - 1) / 2 * jd_min;
    j_min = (sigma + 1) / 2 * jd_min + (sigma - 1) / 2 * jd_max;

    // Assuming that v_max and a_max are reached, compute the time intervals

    // acceleration part
    double Tj_1, Ta, Tj_2, Td;
    if (((v_max - v_0) * j_max) < a_max * a_max) {
        Tj_1 = std::sqrt((v_max - v_0) / j_max);
        Ta = 2 * Tj_1;
    } else {
        Tj_1 = a_max / j_max;
        Ta = Tj_1 + (v_max - v_0) / a_max;
    }

    // deceleration part
    if (((v_max - v_1) * j_max) < a_max * a_max) {
        Tj_2 = std::sqrt((v_max - v_1) / j_max);
        Td = 2 * Tj_2;
    } else {
        Tj_2 = a_max / j_max;
        Td = Tj_2 + (v_max - v_1) / a_max;
    }
    // finally it is possible to determine the time duration of the constant
    // Time duration of the constant velocity phase
    double Tv = (q_1 - q_0) / v_max - Ta / 2 * (1 + v_0 / v_max) - Td / 2 * (1 + v_1 / v_max);

    // Now checking if Tv > 0 or smaller 0
    // v_max is not reached
    if (Tv < 0) {
        // set Tv = 0 (because of errors in computation later)
        Tv = 0;

        // Reset the loop counter
        int count = 0;

        // Do this loop until the break condition holds
        for (double gamma = 1.0; gamma >= 0.0; gamma -= 0.001) {
            a_max = gamma * a_max;
            a_min = gamma * a_min;

            // v_max is not reached => check other things
            Tj_1 = a_max / j_max;
            Tj_2 = a_max / j_max;
            double Tj = a_max / j_max;
            double delta = a_max * a_max * a_max * a_max / (j_max * j_max) + 2 * (v_0 * v_0 + v_1 * v_1) +
                           a_max * (4 * (q_1 - q_0) - 2 * a_max / j_max * (v_0 + v_1));
            Ta = (a_max * a_max / j_max - 2 * v_0 + std::sqrt(delta)) / (2 * a_max);
            Td = (a_max * a_max / j_max - 2 * v_1 + std::sqrt(delta)) / (2 * a_max);

            if (Ta < 0) {
                Ta = 0;
                Td = 2 * (q_1 - q_0) / (v_1 + v_0);
                Tj_2 = (j_max * (q_1 - q_0) - std::sqrt(j_max * (j_max * (q_1 - q_0) * (q_1 - q_0) +
                        (v_1 + v_0) * (v_1 - v_0)))) / (j_max * (v_1 + v_0));
            } else if (Td < 0) {
                Td = 0;
                Ta = 2 * (q_1 - q_0) / (v_1 + v_0);
                Tj_1 = (j_max * (q_1 - q_0) - std::sqrt(j_max * (j_max * (q_1 - q_0) * (q_1 - q_0) -
                        (v_1 + v_0) * (v_1 - v_0)))) / (j_max * (v_1 + v_0));
            } else {
                if ((Ta > 2 * Tj) && (Td > 2 * Tj)) {
                    break;
                } else {
                    count = count + 1;
                }
            }
        }
    }

    // Now compute the trajectory
    double a_lim_a = j_max * Tj_1;
    double a_lim_d = -j_max * Tj_2;
    double v_lim = v_0 + (Ta - Tj_1) * a_lim_a;

    double T = Ta + Tv + Td;

    // round final time to discrete ticks
    T = round(T * 1000) / 1000;

    // time goes from zero in Ts (sample Time) steps to final time T
    std::vector<double> time;
    for (double t = 0; t <= T; t += Ts) {
        time.push_back(t);
    }

    // starting with tic 1
    int i = 0;

    // reading size of time array
    int timeSize = time.size();

    // preallocating memory
    std::vector<double> q(timeSize, 0.0);
    std::vector<double> qp(timeSize, 0.0);
    std::vector<double> qpp(timeSize, 0.0);
    std::vector<double> qppp(timeSize, 0.0);
    std::vector<double> qd(timeSize, 0.0);
    std::vector<double> qdp(timeSize, 0.0);
    std::vector<double> qdpp(timeSize, 0.0);
    std::vector<double> qdppp(timeSize, 0.0);


    // calculating the whole trajectory
    for (double t : time) {
        // ACCELERATION PHASE
        // t element of [0, Tj_1]
        if (t <= Tj_1) {
            q[i] = q_0 + v_0 * t + j_max * t * t * t / 6;
            qp[i] = v_0 + j_max * t * t / 2;
            qpp[i] = j_max * t;
            qppp[i] = j_max;
        }

        // t element of [Tj_1, Ta - Tj_1]
        if ((t > Tj_1) && (t <= (Ta - Tj_1))) {
            q[i] = q_0 + v_0 * t + a_lim_a / 6 * (3 * t * t - 3 * Tj_1 * t + Tj_1 * Tj_1);
            qp[i] = v_0 + a_lim_a * (t - Tj_1 / 2);
            qpp[i] = j_max * Tj_1;
            qppp[i] = 0;
        }

        // t element of [Ta - Tj_1, Ta]
        if ((t > (Ta - Tj_1)) && (t <= Ta)) {
            q[i] = q_0 + (v_lim + v_0) * Ta / 2 - v_lim * (Ta - t) - j_min * std::pow((Ta - t), 3) / 6;
            qp[i] = v_lim + j_min * std::pow((Ta - t), 2) / 2;
            qpp[i] = -j_min * (Ta - t);
            qppp[i] = j_min;
        }

        // CONSTANT VELOCITY PHASE
        // t element of [Ta, Ta + Tv]
        if ((t > Ta) && (t <= (Ta + Tv))) {
            q[i] = q_0 + (v_lim + v_0) * Ta / 2 + v_lim * (t - Ta);
            qp[i] = v_lim;
            qpp[i] = 0;
            qppp[i] = 0;
        }

        // DECELERATION PHASE
        // t element of [T - Td, T - Td + Tj_2]
        if ((t > (Ta + Tv)) && (t <= (Ta + Tv + Tj_1))) {
            q[i] = q_1 - (v_lim + v_1) * Td / 2 + v_lim * (t - T + Td) -
                   j_max * std::pow((t - T + Td), 3) / 6;
            qp[i] = v_lim - j_max * std::pow((t - T + Td), 2) / 2;
            qpp[i] = -j_max * (t - T + Td);
            qppp[i] = j_min;
        }

        // t element of [T - Td + Tj_2, T - Tj_2]
        if ((t > (Ta + Tv + Tj_2)) && (t <= (Ta + Tv + (Td - Tj_2)))) {
            q[i] = q_1 - (v_lim + v_1) * Td / 2 + v_lim * (t - T + Td) +
                   a_lim_d / 6 * (3 * (t - T + Td) * (t - T + Td) - 3 * Tj_2 * (t - T + Td) + Tj_2 * Tj_2);
            qp[i] = v_lim + a_lim_d * (t - T + Td - Tj_2 / 2);
            qpp[i] = -j_max * Tj_2;
            qppp[i] = 0;
        }

        // t element of [T - Tj_2, T]
        if ((t > (Ta + Tv + (Td - Tj_2))) && (t <= T)) {
            q[i] = q_1 - v_1 * (T - t) - j_max * std::pow((T - t), 3) / 6;
            qp[i] = v_1 + j_max * std::pow((T - t), 2) / 2;
            qpp[i] = -j_max * (T - t);
            qppp[i] = j_max;
        }

        // mark the end of trajectory
        if (t > T) {
            q[i] = q_1;
            qp[i] = v_1;
            qpp[i] = 0;
            qppp[i] = 0;
        }

        qd[i] = sigma * q[i];
        qdp[i] = sigma * qp[i];
        qdpp[i] = sigma * qpp[i];
        qdppp[i] = sigma * qppp[i];

        i = i + 1;
    }
    matrix_res.push_back(qd); matrix_res.push_back(qdp); matrix_res.push_back(qdpp);matrix_res.push_back(qdppp);
    return matrix_res;
}

// int main(int argc, char** argv) {
//     // Initialization of system parameter
//     double T_s = 0.001;  // Sample time
//     double v_0 = 0.0, v_1 = 0.0;  // Initial and final velocities
//     double v_max = 4.0, v_min = -4.0;  // Velocity constraints
//     double a_max = 7.5, a_min = -7.5;  // Acceleration constraints
//     double j_max = 3700.0, j_min = -3700.0;  // Jerk constraints
//     double vx_0,vx_1,vy_0,vy_1,vz_0,vz_1;
//     std::vector<std::vector<double>> matrix_tmpx;
//     std::vector<std::vector<double>> matrix_tmpy;
//     std::vector<std::vector<double>> matrix_tmpz;
//     std::vector<std::vector<double>> matrix_p;
//     std::vector<std::vector<double>> matrix_dp;
//     std::vector<std::vector<double>> matrix_ddp;
//     std::vector<std::vector<double>> matrix_dddp;

//     // initialization of subscriber from matlab
//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<MySubscriber>();
//     std::cerr << "1" << std::endl;
//     // create a seperate thread to contiuously spin the node
//     std::thread spin_thread([node](){
//         rclcpp::spin(node);
//     });
//     std::cerr << "2" << std::endl;
//     // main thread

//     while (rclcpp::ok()){
//         std::cerr << "3" << std::endl;
//         // read coordinates from subscriber for matlab
//         geometry_msgs::msg::Point dest = node->getLastPoint();
//         vx_1 = dest.x;
//         vy_1 = dest.y;
//         vz_1 = dest.z;
//         std::cerr << "destination is %f" << vx_1 << std::endl;
//         // read current robot state
//         Coordinates pos_0 = readLastCoordinatesFromCSV("./src/test/src/robotarm_positions.csv");
//         vx_0 = pos_0.x;
//         vy_0 = pos_0.y;
//         vz_0 = pos_0.z;
//         std::cerr << "current position is %f" << vx_0 << std::endl;

//         // only for offline test
//         vx_1 = 0.3; vy_1 = 0.2; vz_1 = 0.5;
//         vx_0 = 0.8; vy_0 = 0.6; vz_0 = 0.2;

//         //trajectory plan for x,y,z coordinates
//         //x
//         double qx_0 = vx_0, qx_1 = vx_1;  // Initial and final positions
//         std::vector<double> qdx, qdpx, qdppx, qdpppx;
//         matrix_tmpx = DoubleSTrajectoryFunction(T_s, qx_0, qx_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
//         qdx = matrix_tmpx[0];
//         qdpx = matrix_tmpx[1];
//         qdppx = matrix_tmpx[2];
//         qdpppx = matrix_tmpx[3];
//         if(qdx.size()== 0){
//             std::cout << "output is zero" << std::endl;
//         }

//         //y
//         double qy_0 = vy_0, qy_1 = vy_1;  // Initial and final positions
//         std::vector<double> qdy, qdpy, qdppy, qdpppy;
//         matrix_tmpy = DoubleSTrajectoryFunction(T_s, qy_0, qy_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
//         qdy = matrix_tmpy[0];
//         qdpy = matrix_tmpy[1];
//         qdppy = matrix_tmpy[2];
//         qdpppy = matrix_tmpy[3];
//         //z
//         double qz_0 = vz_0, qz_1 = vz_1;  // Initial and final positions
//         std::vector<double> qdz, qdpz, qdppz, qdpppz;
//         matrix_tmpz = DoubleSTrajectoryFunction(T_s, qz_0, qz_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
//         qdz = matrix_tmpz[0];
//         qdpz = matrix_tmpz[1];
//         qdppz = matrix_tmpz[2];
//         qdpppz = matrix_tmpz[3];

//         std::cerr << "finished calculateion" << qdx[14] << std::endl;
//         // combine vectors into matrix
//         matrix_p.push_back(qdx); matrix_p.push_back(qdy); matrix_p.push_back(qdz);
//         matrix_dp.push_back(qdpx); matrix_dp.push_back(qdpy); matrix_dp.push_back(qdpz);
//         matrix_ddp.push_back(qdppx); matrix_ddp.push_back(qdppy); matrix_ddp.push_back(qdppz);
//         matrix_dddp.push_back(qdpppx); matrix_dddp.push_back(qdpppy); matrix_dddp.push_back(qdpppz);
//         std::cerr << "-2" << std::endl;
//         // save the traj as matrix into csv files
//         CSV_write_matrix(matrix_p,"./src/trajectoryplan/src/traj_for_position");
//         std::cerr << "-1" << std::endl;
//         CSV_write_matrix(matrix_dp,"./src/trajectoryplan/src/traj_for_velocity");
//         CSV_write_matrix(matrix_ddp,"./src/trajectoryplan/src/traj_for_acceleration");
//         CSV_write_matrix(matrix_dddp,"./src/trajectoryplan/src/traj_for_jerk");
//         std::cerr << "end" << std::endl;
//         usleep(100);
//     }
//     rclcpp::shutdown();
//     // End
//     return 0;
// }

void doubleS(const geometry_msgs::Point& v0, const geometry_msgs::Point& v1) {
    // Initialization of system parameter
    double T_s = 0.001;  // Sample time
    double v_0 = 0.0, v_1 = 0.0;  // Initial and final velocities
    double v_max = 4.0, v_min = -4.0;  // Velocity constraints
    double a_max = 7.5, a_min = -7.5;  // Acceleration constraints
    double j_max = 3700.0, j_min = -3700.0;  // Jerk constraints
    double vx_0,vx_1,vy_0,vy_1,vz_0,vz_1;
    std::vector<std::vector<double>> matrix_tmpx;
    std::vector<std::vector<double>> matrix_tmpy;
    std::vector<std::vector<double>> matrix_tmpz;
    std::vector<std::vector<double>> matrix_p;
	std::vector<std::vector<double>> matrix_dp;
	std::vector<std::vector<double>> matrix_ddp;
	std::vector<std::vector<double>> matrix_dddp;


    std::cerr << "3" << std::endl;
    // read coordinates from subscriber
    vx_1 = v1.x;
    vy_1 = v1.y;
    vz_1 = v1.z;
    vx_0 = v0.x;
    vy_0 = v0.y;
    vz_0 = v0.z;
    std::cerr << "current position is %f" << vx_0 << std::endl;

    // // only for offline test
    // vx_1 = 0.3; vy_1 = 0.2; vz_1 = 0.5;
    // vx_0 = 0.8; vy_0 = 0.6; vz_0 = 0.2;

    //trajectory plan for x,y,z coordinates
    //x
    double qx_0 = vx_0, qx_1 = vx_1;  // Initial and final positions
    std::vector<double> qdx, qdpx, qdppx, qdpppx;
    matrix_tmpx = DoubleSTrajectoryFunction(T_s, qx_0, qx_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
    qdx = matrix_tmpx[0];
    qdpx = matrix_tmpx[1];
    qdppx = matrix_tmpx[2];
    qdpppx = matrix_tmpx[3];
    if(qdx.size()== 0){
        std::cout << "output is zero" << std::endl;
    }

    //y
    double qy_0 = vy_0, qy_1 = vy_1;  // Initial and final positions
    std::vector<double> qdy, qdpy, qdppy, qdpppy;
    matrix_tmpy = DoubleSTrajectoryFunction(T_s, qy_0, qy_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
    qdy = matrix_tmpy[0];
    qdpy = matrix_tmpy[1];
    qdppy = matrix_tmpy[2];
    qdpppy = matrix_tmpy[3];
    if(qdy.size()== 0){
        std::cout << "size of y is zero" << std::endl;
    }
    //z
    double qz_0 = vz_0, qz_1 = vz_1;  // Initial and final positions
    std::vector<double> qdz, qdpz, qdppz, qdpppz;
    matrix_tmpz = DoubleSTrajectoryFunction(T_s, qz_0, qz_1, v_0, v_1, v_max, v_min, a_max, a_min, j_max, j_min);
    qdz = matrix_tmpz[0];
    qdpz = matrix_tmpz[1];
    qdppz = matrix_tmpz[2];
    qdpppz = matrix_tmpz[3];

    std::cerr << "finished calculateion" << qdx[14] << std::endl;
    // combine vectors into matrix
    matrix_p.push_back(qdx); matrix_p.push_back(qdy); matrix_p.push_back(qdz);
    matrix_dp.push_back(qdpx); matrix_dp.push_back(qdpy); matrix_dp.push_back(qdpz);
    matrix_ddp.push_back(qdppx); matrix_ddp.push_back(qdppy); matrix_ddp.push_back(qdppz);
    matrix_dddp.push_back(qdpppx); matrix_dddp.push_back(qdpppy); matrix_dddp.push_back(qdpppz);
    std::cerr << "-2" << std::endl;
    // save the traj as matrix into csv files
    CSV_write_matrix(matrix_p,"./src/trajectoryplan/src/traj_for_position");
    std::cerr << "-1" << std::endl;
    CSV_write_matrix(matrix_dp,"./src/trajectoryplan/src/traj_for_velocity");
    CSV_write_matrix(matrix_ddp,"./src/trajectoryplan/src/traj_for_acceleration");
    CSV_write_matrix(matrix_dddp,"./src/trajectoryplan/src/traj_for_jerk");
    std::cerr << "end" << std::endl;

    return;
}
