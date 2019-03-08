#include "watson_ins.h"
#include "serialIO.h"
#include <math.h>
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/NavSatFix.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>


INS::INS()
{
  //Advertise INS Data
  imu_pub = n.advertise<sensor_msgs::Imu>("/ins/imu", 20);
  nav_pub = n.advertise<sensor_msgs::NavSatFix>("/ins/gps", 20);
  test_pub = n.advertise<std_msgs::String>("/ins/String", 20);

}


//Define callbacks

//parse data string from serial port
string* parse_ins_data(string data)
{
  std::string delimiter = ' ';
  size_t pos = 0;
  std:string token;
  static string serial_out[10];

  i=0;
  while ((pos=data.find(delimiter)) != std::string::npos)
  {
    token = data.substr(0,pos);
    serial_out[i] = token; 
    data.erase(0, pos+delimiter.length());
    i++;
  }
  serial_out[i] = data;

  return serial_out;
}

bool check_plus_minus(string data)
{
  if data.substr(0,1) == '+'
  {
    return true;
  }
  else
  {
    return false;
  }
}

void populate_gps_data(sensor_msgs::NavSatFix &gps_msg, string* parsed_data)
{
  //see if value is positive or negative and turn to float accordingly 
  if (check_plus_minus(parsed_data[6]))
  {
    gps_msg.latitude = std::stof(parsed_data[6].substr(1));
  }
  else
  {
    gps_msg.latitude = std::stof(parsed_data[6].substr(1))*-1;
  }

  if (check_plus_minus(parsed_data[7]))
  {
    gps_msg.longitude = std::stof(parsed_data[7].substr(1));
  }
  else
  {
    gps_msg.longitude = std::stof(parsed_data[7].substr(1))*-1;
  }

  if (check_plus_minus(parsed_data[8]))
  {
    gps_msg.altitude = std::stof(parsed_data[8].substr(1));
  }
  else
  {
    gps_msg.altitude = std::stof(parsed_data[8].substr(1))*-1;
  }

  return
}

//*****am I using correct Euler angles????******////
void populate_imu_data(sensor_msgs::Imu &imu_msg, string* parsed_data)
{
  float roll, pitch, yaw;
  tf2::Quaternion myQuaternion;

  if (check_plus_minus(parsed_data[2]))
  {
    yaw = std::stof(parsed_data[2].substr(1))*(M_PI/180);
  }
  else
  {
    yaw = std::stof(parsed_data[2].substr(1))*(M_PI/-180);
  }

  if (check_plus_minus(parsed_data[3]))
  {
    pitch = std::stof(parsed_data[3].substr(1))*(M_PI/180);
  }
  else
  {
    pitch = std::stof(parsed_data[3].substr(1))*(M_PI/-180);
  }
    
  roll = std::stof(parsed_data[4])*(M_PI/180);
  myQuaternion.setRPY(roll, pitch, yaw);
  imu_msg.orientation = myQuaternion;

  return
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "watson_ins");
  INS ins;
  serialIO serial_comm;
  serial_comm.openSerial();


  string* parsed_data;
  ros::Rate loop_rate(5);
  while(ros::ok()){
    ros::spinOnce();
    sensor_msgs::NavSatFix gps_msg;
    sensor_msgs::Imu imu_msg;
    std_msgs::String ins_msg;

    if(serial_comm.insAvailability()) {
      ins_msg.data = serial_comm.readSerial();
      parsed_data = parse_ins_data(ins_msg.data);

///Take into account +/- and *******
      if (parsed_data[0] == 'G')
      {
        populate_gps_data(gps_msg, parsed_data)
        populate_imu_data(imu_msg, parsed_data)
      }

      else if (parsed_data[0] == 'T' || parsed_data[0] == 'I' )
      {
        populate_imu_data(imu_msg, parsed_data)
      }


      ins.imu_pub.publish(gps_msg);
      ins.nav_pub.publish(imu_msg);
      ins.test_pub.publish(ins_msg);
    }
    loop_rate.sleep();

  }

}