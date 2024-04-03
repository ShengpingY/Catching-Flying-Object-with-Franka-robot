% start ros1 core

	open new terminal
	source /opt/ros/noetic/setup.bash
	roscore

% start camera

% start virtual machine

% start publishing coordinates in virtual machine

% start matlab script (coordinate_subscribe_and_publish.m) to publish coordinates from camera

% start communication with ros2 network

	% open new terminal
	source /opt/ros/noetic/setup.bash
	. ~/ros2_humble/install/setup.bash
	export ROS_MASTER_URI=http://localhost:11311
	ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics

% coordinate transformation

	% open new terminal
	cd Project_ws_ros1/
	source /opt/ros/noetic/setup.bash
	source devel/setup.bash
	rosrun robotmotion state_pub 172.16.0.3 %this is ip of robot which is used
	
	% open new terminal
	cd Project_ws_ros1/
	source /opt/ros/noetic/setup.bash
	source devel/setup.bash
	rosrun camera_robot_tf camera_robot_tf_publisher.py % or rosrun camera_robot_tf camera_robot_tf_publisher_morepoint.py % depends on which method is chosen

% catching Ball!

	% start matlab script test_with_publisher_and_iterative_curvfitting3D.m
 
	% open new terminal
	cd Project_ws_ros1/
	source /opt/ros/noetic/setup.bash
	source devel/setup.bash
	rosrun robotmotion cartesian_move 172.16.0.3 % change the executable("cartesian_move") according to your algorithm
		
Then you can throw the ball!!!!!!!!!!!!!!!!!!!
	

