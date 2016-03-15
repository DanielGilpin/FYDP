#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <altitude_sensor/sensor_data.h>
#include <pixy_node/Servo.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
//#include <unistd.h>

 
float distance = 0;
float servo_ang = 0;
int servoOffset = 20; //630
const double degreesPerServo = 0.147540984; //90/610

double x_pos =0;
double y_pos = 0;
float x_setpoint = -4;
float y_setpoint =9;

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
Eigen::MatrixXf setpoint(2,1);
Eigen::MatrixXf error(2,1);
Eigen::MatrixXf y(2,1);
Eigen::MatrixXf temp(2,1);

float dt = 1.0/30.0;

bool distanceFlag = 0;
bool angleFlag = 0;

float flightTime = 0; //time of flight of the ball
const float loadingTime = 0.5;
float errorTime = 0;

void distanceCallback(const altitude_sensor::sensor_data::ConstPtr& msg){
    y(0,0) = msg->altitude;
    //ROS_INFO("Distance: [%f]", distance);
    distanceFlag = 1;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg){
    y(1,0) = msg->data *3.14159/180.0;
    //ROS_INFO("Angle: [%d]", angle);
    angleFlag = 1;
}

void setpointCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	flightTime = 0.4403 + 0.0463*msg->data[0]; //flight time in seconds from distance in yards
	setpoint(0,0) = msg->data[3]*0.9144; //setpoint in metres to catch ball
	setpoint(1,0) = msg->data[4]*0.9144;

	mu(0,0) = msg->data[5]*0.9144; //initial guess in metres to catch ball
	mu(1,0) = msg->data[6]*0.9144;
	ROS_INFO("MU : [%f], [%f]",mu(0,0),mu(1,0));
}

///read speed in m/s
void speedCallback(const std_msgs::Float32::ConstPtr& msg){

	u(0,0) = 0; //setpoint in metres to catch ball
	u(1,0) = msg->data;
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
	S = S*0.01;
	I <<1,0,0,1;

	// u << 0,4.4;
	// mu << -4,2;

	// setpoint(0,0) = x_setpoint;
	// setpoint(1,0) = y_setpoint;

    //creating the node
	ros::init(argc, argv, "spiral_node");
	ros::NodeHandle nh;
	
    //creating a publisher for the position
 
    //Creating a subscriber for the angle and distance
    ros::Subscriber distance_sub = nh.subscribe("/altitude", 1, distanceCallback);

	ros::Subscriber angle_sub = nh.subscribe("/servo", 1, angleCallback);

	ros::Subscriber setpoint_sub = nh.subscribe("/input", 1, setpointCallback);

	ros::Subscriber speed_sub = nh.subscribe("/speedInput", 1, speedCallback);

	ros::Publisher Pose_pub=nh.advertise<geometry_msgs::Pose>("/position", 20);

	ros::Publisher launch_pub=nh.advertise<std_msgs::UInt16>("/launch", 20);

	geometry_msgs::Pose position_msg;

	std_msgs::UInt16 launch_msg;

	ros::Rate loop_rate(30); 

	launch_msg.data = 1; 

    //setting default device path for the sensor
	//nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	

	
	while(ros::ok()) {
		//ROS_INFO("Distance: [%f]      Angle: [%f]", distance, servo_ang);
		//ROS_INFO("X: [%f]      Y: [%f]", distance*cos(servo_ang*3.14159/180), distance*sin(servo_ang*3.14159/180));

		// x_pos = distance*cos(servo_ang*3.14159/180);
		// y_pos = distance*sin(servo_ang*3.14159/180);

		ros::spinOnce();

		if(distanceFlag&&angleFlag){
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

			position_msg.orientation.x = y(0,0)*cos(y(1,0));
			position_msg.orientation.y = y(0,0)*sin(y(1,0));

			error = setpoint - mu; //error in units of metres between setpoint and state estimate
 
			//ROS_INFO("Setpoint [%f]    Mu    [%f]", setpoint(1,0), mu(1,0));

			errorTime = error(1,0)/u(1,0);
			//ROS_INFO("Error Time [%f]", error(1,0));

			if (abs(errorTime - flightTime - loadingTime) <0.1){
				launch_msg.data = 2;//Throw football	
				ROS_INFO("FIRE");
			}

			Pose_pub.publish(position_msg);
			launch_pub.publish(launch_msg);
		}
		loop_rate.sleep(); 
			
	}
	
	

	return 0; 
}