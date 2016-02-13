#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>

#define REPLY_SIZE 20
#define TIMEOUT 1000
	
	const unsigned int sensor_frequency = 60; /*sensor frequency in hz*/

	double altitude;
	double voltage;
	std::string serial_port;



int main(int argc, char** argv) {



    //creating the nodde
	ros::init(argc, argv, "cool_muscle_node");
	ros::NodeHandle nh;
	
    //creating a publisher
	//ros::Publisher value_pub=nh.advertise<altitude_sensor::sensor_data>("altitude", 5);
	
	ros::Rate loop_rate(sensor_frequency); 
	
	cereal::CerealPort device;
	
	std::string position = "P=0\r\n";
	std::string acceleration = "A=0\r\n";
	std::string speed = "S=1000\r\n";
	std::string go = "^\r\n";
	
	

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


 		try{ device.write(position.c_str(), position.length()); }
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
		
	}
	
	ros::spin();

	return 0; 
}
