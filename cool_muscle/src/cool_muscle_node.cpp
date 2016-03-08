#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>
#include <keyboard/Key.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000
	
	const unsigned int sensor_frequency = 60; /*sensor frequency in hz*/

	double altitude;
	double voltage;
	std::string serial_port;

	cereal::CerealPort device;
	
	std::string load_position = "P=80000\r\n";
	std::string launch_position = "P=0\r\n";
	std::string acceleration = "A=0\r\n";
	std::string speed = "S=1000\r\n";
	std::string go = "^\r\n";

void fireCallback(const keyboard::Key::ConstPtr& msg){
	ROS_INFO("MADE IT");
	if (msg->code == 273 || msg->code == 274){
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
        if (msg->code == 273){
	 		try{ device.write(launch_position.c_str(), launch_position.length()); }
	    		catch(cereal::Exception& e)
	       		{
	                ROS_ERROR("Error!");
	                }
        }
        else{ 
		 		try{ device.write(load_position.c_str(), load_position.length()); }
	    		catch(cereal::Exception& e)
	       		{
	                ROS_ERROR("Error!");
	                }

        }

 		try{ device.write(go.c_str(), go.length()); }
    		catch(cereal::Exception& e)
       		{
                ROS_ERROR("Error!");
            }
	}          

        
}


int main(int argc, char** argv) {



    //creating the nodde
	ros::init(argc, argv, "cool_muscle_node");
	ros::NodeHandle nh;
	
    //creating a subscriber
	ros::Subscriber keyboard_sub = nh.subscribe("/keyboard/keydown", 1000, fireCallback);
	
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


 		try{ device.write(load_position.c_str(), load_position.length()); }
    		catch(cereal::Exception& e)
       		{
                ROS_ERROR("Error!");
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
