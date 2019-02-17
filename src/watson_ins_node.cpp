#include "watson_ins.h"
#include "serialIO.h"
#include <math.h>
#include <iostream>
#include <string>
#include "std_msgs/String.h"

#include <iostream>


INS::INS()
{
  //Advertise INS Data
  imu_pub = n.advertise<sensor_msgs::Imu>("/ins/imu", 20); 
  nav_pub = n.advertise<sensor_msgs::NavSatFix>("/ins/gps", 20);

}


//Define callbacks



int main(int argc, char *argv[]){
  ros::init(argc, argv, "watson_ins");
  INS ins;
  serialIO serial_comm;
  serial_comm.openSerial();
    

  ros::Rate loop_rate(5); 
  while(ros::ok()){
    ros::spinOnce();
    
    if(serial_comm.insAvailability()) 
      serial_comm.readSerial();
     
  }
  loop_rate.sleep();
  
}
