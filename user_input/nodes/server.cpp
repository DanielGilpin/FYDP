#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <user_input/TutorialsConfig.h>
//#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>


volatile int dis = 0;
volatile int x = 0;
volatile int y = 0;
volatile int x_0 = 0;
volatile int y_0 = 0;

volatile int wayX = 0;
volatile int wayY = 0;

volatile int ang = 0;
volatile int ldata[9] = {0,0,0,0,0,0,0,0,0}; //[dis,ang,initiate,x,y,x0,y0,wayX,wayY]
volatile float speed = 0;

// static int disTable[26] = {
//   0,30,50,60,70,80
//     ,90,100,105,110,113
//     ,117,120,127,132,136
//     ,140,144,148,152,156
//     ,160,163,165,169,170};

void callback(user_input::TutorialsConfig &config, uint32_t level) {
  // ROS_INFO("Reconfigure Request: %d %d", 
            // config.Distance, config.Angle);
            // config.str_param.c_str(), 
            // config.bool_param?"True":"False", 
            // config.size);
  x = (int)config.Xf;
  y = config.Yf;

  x_0 = config.X0;
  y_0 = config.Y0;

  ldata[5] = x_0;
  ldata[6] = y_0;

  wayX = config.cutX;
  wayY = config.cutY;

  ldata[7] = wayX;
  ldata[8] = wayY;

  dis = sqrt(x*x + y*y);
  ang = (atan2(y,x))*180/3.14159;

  ldata[0] = dis;
  ldata[1] = ang;
  if(config.Initiate == true)
  {
    ldata[2] = 1;
  }
  else
  {
    ldata[2] = 0;
  }
  ldata[3] = x;
  ldata[4] = y;
  speed = (float)config.Speed;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "user_input");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<user_input::TutorialsConfig> server;
  dynamic_reconfigure::Server<user_input::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // std_msgs::Int32MultiArray launchData;
  std_msgs::Int16MultiArray launchData;
  std_msgs::Float32 speedData;
  // launchData.layout.dim_length = 1;
  // launchData.data_length =3;
  launchData.data.clear();
  launchData.data.resize(9);
  // launchData.data.resize(2);
  // launchData.MultiArrayLayout.MultiArrayDimension
  // ros::Publisher pub_message = nh.advertise<std_msgs::Int32MultiArray>("Int32MultiArray", 100, &launchData);
  ros::Publisher pub = nh.advertise<std_msgs::Int16MultiArray>("input", 100, &launchData);
  ros::Publisher speedPub = nh.advertise<std_msgs::Float32>("speedInput", 100, &speedData);



  ROS_INFO("Spinning node");
  ros::Rate r(5);
  while (nh.ok())
  {
    // Publish the message.
    // launchData.data[0] = ldata[0];
    // launchData.data[1] = ldata[1];
    for (int i = 0; i < 9; ++i)
    {
      launchData.data[i] = ldata[i];
    }
    speedData.data = speed;
    // pub_message.publish(launchData);
    pub.publish(launchData);
    speedPub.publish(speedData);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

