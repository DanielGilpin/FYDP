#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <pixy_node/Servo.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
//#include <unistd.h>

 
float distance = 0;
float servo_ang = 0;
int servoOffset = 20; //630
const double degreesPerServo = 0.147540984; //90/610

double x_pos =0;
double y_pos = 0;

Eigen::MatrixXf A(2,2);
Eigen::MatrixXf B(2,2);

Eigen::MatrixXf R(2,2);
Eigen::MatrixXf Q(2,2);

Eigen::MatrixXf Ht(2,2);

Eigen::MatrixXf Ke(2,2);
Eigen::MatrixXf S(2,2);
Eigen::MatrixXf Sp(2,2);

Eigen::MatrixXf I(2,2);

Eigen::MatrixXf u(2,1);
Eigen::MatrixXf mu(2,1);
Eigen::MatrixXf mup(2,1);
Eigen::MatrixXf y(2,1);
Eigen::MatrixXf temp(2,1);

float dt = 0.05;





void distanceCallback(const altitude_sensor::sensor_data::ConstPtr& msg){
    y(0,0) = msg->altitude;
    //ROS_INFO("Distance: [%f]", distance);
}

void angleCallback(const pixy_node::Servo::ConstPtr& msg){
    y(1,0) = (msg->position - servoOffset)*degreesPerServo*3.14159/180.0;
    //ROS_INFO("Angle: [%d]", angle);
}


//void positionUpdate


int main(int argc, char** argv) {

	A << 1,0,0,1;
	B << dt,0,0,dt;

	//ROS_INFO("[%f], [%f],[%f],[%f]", A(0,0), A(0,1), A(1,0), A(1,1));

	R << 1,0,0,1;
	R = R*0.000001;
	Q <<1,0,0,1;
	Q = Q*0.000001;

	S <<1,0,0,1;
	S = S*0.001;
	I <<1,0,0,1;

	u << 0,2;
	mu << 2,2;

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

		// x_pos = distance*cos(servo_ang*3.14159/180);
		// y_pos = distance*sin(servo_ang*3.14159/180);

		ros::spinOnce();


		mup = A*mu + B*u;
		Sp = A*S*A.transpose() + R;

		Ht(0,0) = mup(0,0)/(sqrt(pow(mup(0,0),2) + pow(mup(1,0),2)));
		Ht(0,1) = mup(1,0)/(sqrt(pow(mup(0,0),2) + pow(mup(1,0),2)));
		Ht(1,0) = -pow(mup(0,0),2)/(1 + pow(mup(1,0)/mup(0,0),2));
		Ht(1,1) = 1/(1 + pow(mup(1,0)/mup(0,0),2));

		Ke = Sp*(Ht.transpose())*((Ht*Sp*Ht.transpose() + Q).inverse());


		temp(0,0) = sqrt(pow(mup(0,0),2) + pow(mup(1,0),2));

		temp(1,0) = atan2(mup(1,0),mup(0,0));
//ROS_INFO ("HIT");

		mu = mup + Ke*(y - temp);

		S = (I - Ke*Ht)*Sp;

		position_msg.position.x = mu(0,0);
		position_msg.position.y = mu(1,0);




		// if (abs(position_msg.position.x) <1 && abs(position_msg.position.y-5)<1){
		// 	ROS_INFO("FIRE");
		// }

		Pose_pub.publish(position_msg);

		loop_rate.sleep(); 
		
	}
	
	

	return 0; 
}