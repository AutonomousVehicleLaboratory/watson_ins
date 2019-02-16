#ifndef WATSON_INS
#define WATSON_INS
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

class INS{
  public:
    INS();


    ros::NodeHandle n;

    //Subscribers
     
    //Publishers
    ros::Publisher imu_pub;
    ros::Publisher nav_pub;
     
    std::string m_DeviceName;

};

#endif
