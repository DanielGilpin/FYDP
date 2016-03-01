/*
 * pixy_node is part of the pixy_ros package for interfacing with
 * a CMUcam5 pixy with ROS.
 * Copyright (C) 2014 Justin Eskesen
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <pixy_msgs/PixyData.h>
#include <pixy_msgs/PixyBlock.h>
#include <pixy_msgs/Servo.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include "pixy.h"
#define BLOCK_BUFFER_SIZE 1

#define PIXY_MAX_X 640
#define PIXY_MIN_X 0
#define PIXY_MAX_Y 480
#define PIXY_MIN_Y 0
#define PIXY_X_CENTER              ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define PIXY_Y_CENTER              ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#define PIXY_RCS_PAN_CHANNEL        0
#define PIXY_RCS_TILT_CHANNEL       1
#define PIXY_RCS_CENTER_POS 500
#define PIXY_RCS_MAX_X 999
#define PIXY_RCS_MIN_X 1
#define PIXY_RCS_MAX_Y 1000
#define PIXY_RCS_MIN_Y 0


// PID control parameters //
#define PAN_PROPORTIONAL_GAIN     400
#define PAN_DERIVATIVE_GAIN       300
#define TILT_PROPORTIONAL_GAIN    500
#define TILT_DERIVATIVE_GAIN      400

   int     pixy_init_status;
  char    buf[128];
  int     frame_index = 0;
  int     result;
  int     pan_error;
  int     tilt_error;
  int     blocks_copied;
  //int     index;

  const double Kp = 0.5;
  const double Kd = 10;

// Pixy Block Buffer //
struct Block  blocks [BLOCK_BUFFER_SIZE];

static bool run_flag = true;

struct Gimbal {
  int32_t position;
  int32_t previous_error;
  int32_t proportional_gain;
  int32_t derivative_gain;
};

// PID control variables //

struct Gimbal pan;
struct Gimbal tilt;

void initialize_gimbals()
{
  pan.position           = 0;
  pan.previous_error     = 0;
  pan.proportional_gain  = PAN_PROPORTIONAL_GAIN;
  pan.derivative_gain    = PAN_DERIVATIVE_GAIN;
  tilt.position          = PIXY_RCS_CENTER_POS;
  tilt.previous_error    = 0x80000000L;
  tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
  tilt.derivative_gain   = TILT_DERIVATIVE_GAIN;
}

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

void gimbal_update(struct Gimbal *  gimbal, int32_t error)
{
  long int control;
  int32_t  error_delta;
  int32_t  P_gain;
  int32_t  D_gain;

  if(gimbal->previous_error != 0x80000000L) {

    error_delta = error - gimbal->previous_error;
    P_gain      = gimbal->proportional_gain;
    D_gain      = gimbal->derivative_gain;

    /* Using the proportional and derivative gain for the gimbal,
       calculate the change to the position.  */
    control = (error * P_gain + error_delta * D_gain) >> 10;

    gimbal->position += control;

    if (gimbal->position > PIXY_RCS_MAX_X) {
      gimbal->position = PIXY_RCS_MAX_X;
    } else if (gimbal->position < PIXY_RCS_MIN_X) {
      gimbal->position = PIXY_RCS_MIN_X;
    }
  } 

  gimbal->previous_error = error;
}

class PixyNode
{
public:
	PixyNode();

	void spin();

private:
	void update();
	void setServo(const pixy_msgs::Servo& msg) {pixy_rcs_set_position(msg.channel, msg.position);}
	

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;

	ros::Rate rate_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher publisher_;
	ros::Publisher servo_publisher_;
	ros::Subscriber servo_subscriber_;
	std::string frame_id;

	bool use_servos_;

};

PixyNode::PixyNode() :
		node_handle_(),
		private_node_handle_("~"),
		use_servos_(true),
		rate_(1)
{

	private_node_handle_.param<std::string>(std::string("frame_id"), frame_id,
			std::string("pixy_frame"));

	double rate;
	private_node_handle_.param("rate", rate, 1000.0);
	rate_=ros::Rate(rate);

    private_node_handle_.param("use_servos", use_servos_, false); 

    if(use_servos_)
    {
        servo_subscriber_ = node_handle_.subscribe("servo_cmd", 20, &PixyNode::setServo, this);
    }

	int ret = pixy_init();
	if (ret != 0)
	{
		ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
				__FUNCTION__, ret);
		ROS_BREAK();
	}
    publisher_ = node_handle_.advertise<pixy_msgs::PixyData>("block_data", 50.0);
    servo_publisher_ = node_handle_.advertise<pixy_msgs::Servo>("servo", 50.0);


}



void PixyNode::update()
{

	// Pixy Block buffer //
	struct Block blocks[BLOCK_BUFFER_SIZE];
	// if(!pixy_blocks_are_new() )
	// {
		// Get blocks from Pixy //
		int blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, blocks);

		pixy_msgs::PixyData data;

		if (blocks_copied > 0)
		{
			data.header.stamp = ros::Time::now();
			for (int i = 0; i < blocks_copied; i++)
			{
				pixy_msgs::PixyBlock pixy_block;
				pixy_block.type = blocks[i].type;
				pixy_block.signature = blocks[i].signature;
				pixy_block.roi.x_offset = blocks[i].x;
				pixy_block.roi.y_offset = blocks[i].y;
				pixy_block.roi.height = blocks[i].height;
				pixy_block.roi.width = blocks[i].width;
				pixy_block.roi.do_rectify = false;
				pixy_block.angle =
						(pixy_block.type == TYPE_COLOR_CODE) ?
								angles::from_degrees((double) blocks[i].angle) :
								0.0;

				data.blocks.push_back(pixy_block);
			}
	//****************************************************************************************************
			//Pan/tilt stuff

			if (data.blocks[0].roi.height>15){
			pan_error  = 160 - data.blocks[0].roi.x_offset ;
			ROS_INFO("Error[%d]", pan_error);
	      tilt_error = data.blocks[0].roi.y_offset - PIXY_Y_CENTER;

	      // Apply corrections to the pan/tilt with the goal //
	      // of putting the target in the center of          //
	      // Pixy's focus.                                   //

	      gimbal_update(&pan, pan_error);
	      gimbal_update(&tilt, tilt_error);

	      result = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, pan.position);

	      ROS_INFO("Actual Pan Position [%d]", pixy_rcs_get_position(0));
	      ROS_INFO("Commanded Pan Position [%d]", pan.position);

	      
	      if (result < 0) {
	        printf("Error: pixy_rcs_set_position() [%d] ", result);
	        pixy_error(result);
	        fflush(stdout);
	      }

	      // result = pixy_rcs_set_position(PIXY_RCS_TILT_CHANNEL, tilt.position);
	      // if (result<0) {
	      //   printf("Error: pixy_rcs_set_position() [%d] ", result);
	      //   pixy_error(result);
	      //   fflush(stdout);
	      // }
	  }

		}
		else if(blocks_copied < 0)
		{
			ROS_INFO("Pixy read error.");
			return;
		}
	// }


	// publish the message
	publisher_.publish(data);

	//*******************************************************************************
	//Read the servo angle for channel 0 (pan) and publish it under servo
	pixy_msgs::Servo servoData;
	servoData.channel = 0;
	servoData.position = pixy_rcs_get_position(0);

	servo_publisher_.publish(servoData);
}


void PixyNode::spin()
{

	while (node_handle_.ok())
	{
		update();

		ros::spinOnce();
		rate_.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pixy_node");

	ROS_INFO("PixyNode for ROS");
	initialize_gimbals();
	pixy_rcs_set_position(0, pan.position);
	PixyNode myPixy;
	myPixy.spin();
	

	return (0);
}

// EOF
