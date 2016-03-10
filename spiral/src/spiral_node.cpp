#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <pixy_node/Servo.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
//#include <unistd.h>

 
float distance = 0;
float servo_ang = 0;
int servoOffset = 20; //630
const double degreesPerServo = 0.147540984; //90/610

double x_pos =0;
double y_pos = 0;




void distanceCallback(const altitude_sensor::sensor_data::ConstPtr& msg){
    distance = msg->altitude;
    //ROS_INFO("Distance: [%f]", distance);
}

void angleCallback(const pixy_node::Servo::ConstPtr& msg){
    servo_ang = (msg->position - servoOffset)*degreesPerServo;
    //ROS_INFO("Angle: [%d]", angle);
}


//void positionUpdate


int main(int argc, char** argv) {



    //creating the node
	ros::init(argc, argv, "spiral_node");
	ros::NodeHandle nh;
	
    //creating a publisher for the position
 
    //Creating a subscriber for the angle and distance
    ros::Subscriber distance_sub = nh.subscribe("/altitude", 20, distanceCallback);

	ros::Subscriber angle_sub = nh.subscribe("/servo", 20, angleCallback);

	ros::Publisher Pose_pub=nh.advertise<geometry_msgs::Pose>("/position", 20);

	geometry_msgs::Pose position_msg;

	ros::Rate loop_rate(20); 

    //setting default device path for the sensor
	//nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	

	
	while(ros::ok()) {
		//ROS_INFO("Distance: [%f]      Angle: [%f]", distance, servo_ang);
		//ROS_INFO("X: [%f]      Y: [%f]", distance*cos(servo_ang*3.14159/180), distance*sin(servo_ang*3.14159/180));

		x_pos = distance*cos(servo_ang*3.14159/180);
		y_pos = distance*sin(servo_ang*3.14159/180);

		position_msg.position.x = x_pos;
		position_msg.position.y = y_pos;


		// if (abs(position_msg.position.x) <1 && abs(position_msg.position.y-5)<1){
		// 	ROS_INFO("FIRE");
		// }

		Pose_pub.publish(position_msg);

		loop_rate.sleep(); 
		ros::spinOnce();
	}
	
	

	return 0; 
}