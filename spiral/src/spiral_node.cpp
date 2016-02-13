#include <ros/ros.h>
#include <std_msgs/String.h>
#include <altitude_sensor/sensor_data.h>
#include <cereal_port/CerealPort.h>
//#include <spiral/ftd2xx.h>
#include <unistd.h>

#define REPLY_SIZE 256
#define TIMEOUT 1000
#define MSG_LENGTH 256
	

	std::string serial_port;


int main(int argc, char** argv) {



    //creating the nodde
	ros::init(argc, argv, "spiral_node");
	ros::NodeHandle nh;
	
    //creating a publisher
	//ros::Publisher value_pub=nh.advertise<altitude_sensor::sensor_data>("altitude", 5);
	
	ros::Rate loop_rate(20); 
	
	//cereal::CerealPort device;
	//char reply[REPLY_SIZE];
	
	cereal::CerealPort device;

    //setting default device path for the sensor
	nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0"));
	

	try{ device.open(serial_port.c_str() , 38400); }
        catch(cereal::Exception& e)
        {
        ROS_FATAL("Failed to open the serial port.");
        ROS_BREAK();
        }
        ROS_INFO("The serial port is opened."); 


		std::stringstream ss1;
		std::stringstream ss2;
		char reply1[] = "S=1000";
		char reply2[] = "A=100";
		char reply3[] = "P=1000";
		char reply4[] = "^";
		// reply = "A=100";
		
 		try{ device.write(reply1, 6);}
    		catch(cereal::TimeoutException& e)
       		{
                ROS_ERROR("Not sent to the football launcher");
            }
            sleep(1);
            ROS_INFO("MESSAGE 1");
         		try{ device.write(reply2, 5);}
    		catch(cereal::TimeoutException& e)
       		{
                ROS_ERROR("Not sent to the football launcher");
            }
            sleep(1);
            ROS_INFO("MESSAGE 2");
             		try{ device.write(reply3,6);}
    		catch(cereal::TimeoutException& e)
       		{
                ROS_ERROR("Not sent to the football launcher");
            }
            sleep(1);
            ROS_INFO("MESSAGE 3");
             		try{ device.write(reply4, 1);}
    		catch(cereal::TimeoutException& e)
       		{
                ROS_ERROR("Not sent to the football launcher");
            }
            ROS_INFO("MESSAGE 4");


	
        //converting string into float
	while(ros::ok()) {
		

		
		loop_rate.sleep(); 
		
	}
	
	ros::spin();

	return 0; 
}
