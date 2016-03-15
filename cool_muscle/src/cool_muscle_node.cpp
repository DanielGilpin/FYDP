#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>
#include <keyboard/Key.h>
#include <std_msgs/UInt16.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000
	
	const unsigned int sensor_frequency = 60; /*sensor frequency in hz*/

	double altitude;
	double voltage;
	int launch =0;
	std::string serial_port;

	cereal::CerealPort device;
	
	std::string load_position = "P=80000\r\n";
	std::string ready_position = "P=25000\r\n";
	std::string launch_position = "P=0\r\n";
	std::string acceleration = "A=0\r\n";
	std::string speed = "S=1000\r\n";
	std::string go = "^\r\n";

void setup(){
    try{ device.open(serial_port.c_str() , 38400); }
    catch(cereal::Exception& e)
    {
    ROS_FATAL("Failed to open the serial port.");
    ROS_BREAK();
    }

	try{ device.write(acceleration.c_str(), acceleration.length()); }
	catch(cereal::Exception& e)
		{
        ROS_ERROR("Error!");
    }
	try{ device.write(speed.c_str(), speed.length()); }
	catch(cereal::Exception& e)
		{
        ROS_ERROR("Error!");
    }


}

void sendGo(){
	try{ device.write(go.c_str(), go.length()); }
	catch(cereal::Exception& e)
	{
		ROS_ERROR("Error!");
	}
}

void loadFootball(){
	try{ device.write(load_position.c_str(), load_position.length()); }
	catch(cereal::Exception& e)
	{
	ROS_ERROR("Error!");
	}
}

void launchFootball(){
	try{ device.write(launch_position.c_str(), launch_position.length()); }
	catch(cereal::Exception& e)
	{
    ROS_ERROR("Error!");
    }
}

void readyFootball(){
	try{ device.write(ready_position.c_str(), ready_position.length()); }
	catch(cereal::Exception& e)
	{
	ROS_ERROR("Error!");
	}
}


void fireCallback(const keyboard::Key::ConstPtr& msg){
	setup();
	if (msg->code == 273 || msg->code == 274||msg->code==275){
        if (msg->code == 273){
        	launchFootball();
        }
        else if (msg->code == 274){ 
        	loadFootball();
        }
        else { 
        	readyFootball();
        }
	}         
	sendGo();
        
}

void launchCallback(const std_msgs::UInt16::ConstPtr& msg)
{
	
	setup();
    if (msg->data == 2){
    	launchFootball();
    }
    else if (msg->data == 1){ 
    	readyFootball();
    }
    else { 
    	loadFootball();
    }       
	sendGo();
}


int main(int argc, char** argv) {



    //creating the nodde
	ros::init(argc, argv, "cool_muscle_node");
	ros::NodeHandle nh;
	
    //creating a subscriber
	ros::Subscriber keyboard_sub = nh.subscribe("/keyboard/keydown", 1000, fireCallback);

	ros::Subscriber launch_sub = nh.subscribe("/launch", 20, launchCallback);
	
	ros::Rate loop_rate(sensor_frequency); 
	
    //setting default device path for the sensor
	nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	

        //writing to the serial port
	try{ device.open(serial_port.c_str() , 38400); }
        catch(cereal::Exception& e)
        {
        ROS_FATAL("Failed to open the serial port.");
        ROS_BREAK();
        }
        ROS_INFO("The serial port is opened!!!");


	loadFootball();

 		try{ device.write(acceleration.c_str(), acceleration.length()); }
    		catch(cereal::Exception& e)
       		{
                ROS_ERROR("Error!");
                }
 		try{ device.write(speed.c_str(), speed.length()); }
    		catch(cereal::Exception& e)
       		{
                ROS_ERROR("Error!");
                }

 		try{ device.write(go.c_str(), go.length()); }
    		catch(cereal::Exception& e)
       		{
                ROS_ERROR("Error!");
                }
	
        //converting string into float
	while(ros::ok()) {
			
			
		loop_rate.sleep(); 
		ros::spinOnce();
		
	}
	
	ros::spin();

	return 0; 
}
