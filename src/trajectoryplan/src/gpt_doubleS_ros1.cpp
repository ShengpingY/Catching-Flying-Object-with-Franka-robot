#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <ros/ros.h>
//#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Point.h"
#include <unistd.h> // for sleep

#include "gpt_doubleS.cpp"


class trajplan{
public:
    trajplan(){
        msg_empty_flag_0 = false;
        msg_empty_flag_1 = false;
    }

    void cur_coord_sub_Callback(const geometry_msgs::Point& msg) {
        current_sub = msg;
        msg_empty_flag_0 = true;
        // calculate traj
        ROS_INFO("cur_coord is being called");
        plan();
    }

    void dest_coord_sub_Callback(const geometry_msgs::Point& msg) {
        dest_sub = msg;
        msg_empty_flag_1 = true;
        ROS_INFO("dest_coord is being called");
        // calculate traj
        plan();
    }

    void plan() {
        if ( !msg_empty_flag_1 or !msg_empty_flag_0) {
            return;
        }
        doubleS(current_sub,dest_sub);
    }


private:
    geometry_msgs::Point current_sub;
    geometry_msgs::Point dest_sub;
    bool msg_empty_flag_0;
    bool msg_empty_flag_1;
};

int main(int argc, char** argv) {
    ros::init(argc,argv,"trajectoryplanner");
    ros::NodeHandle nh;

    trajplan planner;

    ros::Subscriber desetinationSubscriber = nh.subscribe("matlab",10,&trajplan::dest_coord_sub_Callback, &planner);
    ros::Subscriber robotstateSubscriber = nh.subscribe("state",10,&trajplan::cur_coord_sub_Callback, &planner);

    // here is publisher only for offline test
    ros::Publisher pubtest0 = nh.advertise<geometry_msgs::Point> ("matlab",1000);
    ros::Publisher pubtest1 = nh.advertise<geometry_msgs::Point> ("state",1000);
    
    int k = 0;
    while(ros::ok()) {
        geometry_msgs::Point destt;
        geometry_msgs::Point currt;
        destt.x = 0.55; destt.y = -0.4; destt.z = 0.45;
        currt.x = 0.306891; currt.y = -0.00005; currt.z = 0.486880;
        pubtest0.publish(destt);
        pubtest1.publish(currt);
        ROS_INFO("Publisher is publishing msgs, destination x is: %f", destt.x);
        // if (k > 10){
        //     return 0;
        // }
        // k++;
        ros::spinOnce();    
    }


    
    
    return 0;
    

    
}