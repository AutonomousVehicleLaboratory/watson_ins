#include "watson_ins.h"
#include "serialIO.h"
#include <math.h>
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <cstdlib>


INS::INS()
{
  //Advertise INS Data
  imu_pub = n.advertise<sensor_msgs::Imu>("/ins/imu", 20);
  nav_pub = n.advertise<sensor_msgs::NavSatFix>("/ins/gps", 20);
  test_pub = n.advertise<std_msgs::String>("/ins/String", 20);

}


//parse data string from serial port
std::string* parse_ins_data(std::string data)
{
  char delimiter = ' ';
  size_t pos = 0;
  std::string token;
  static std::string serial_out[20];

  int i=0;
  while ((pos=data.find(delimiter)) != std::string::npos)
  {
    token = data.substr(0,pos);
    serial_out[i] = token; 
    data.erase(0, pos+1);
    i++;
  }
  //serial_out[i] = data;

  return serial_out;
}

//checks if string is positive or negative number
bool check_plus_minus(std::string data)
{
  std::string plus = "+";
  if (data.substr(0,1) == plus)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//converts feet to meters
float feet_to_meters(float num)
{
  float meters = num/3.2808; 
  return meters;
}

//converts acceleration in G's to m/s^2
float gs_to_ms2(float num)
{
  float ms2 = num/0.101972;
  return ms2;
}

void populate_gps_data(INS &ins, std::string* parsed_data)
{
  //see if value is positive or negative and turn to float accordingly  

  sensor_msgs::NavSatFix gps_msg; 

  if (check_plus_minus(parsed_data[16]))
  {
    gps_msg.latitude = std::atof((parsed_data[16].substr(1)).c_str());
  }
  else
  {
    gps_msg.latitude = std::atof((parsed_data[16].substr(1)).c_str())*-1;
  }

  if (check_plus_minus(parsed_data[17]))
  {
    gps_msg.longitude = std::atof((parsed_data[17].substr(1)).c_str());
  }
  else
  {
    gps_msg.longitude = std::atof((parsed_data[17].substr(1)).c_str())*-1;
  }


  //if (check_plus_minus(parsed_data[18]))
  //{
  gps_msg.altitude = feet_to_meters(std::atof((parsed_data[18].substr(1)).c_str()));
  //}
  //else
  //{
    //gps_msg.altitude = feet_to_meters(std::atof((parsed_data[18].substr(1)).c_str()))*-1;
  //}

  ins.nav_pub.publish(gps_msg);
}

void populate_imu_data(INS &ins, std::string* parsed_data)
{
  sensor_msgs::Imu imu_msg;
  float roll, pitch, yaw;
  float x_angular, y_angular, z_angular;
  float x_accel, y_accel, z_accel;
  float forward_accel, lateral_accel, vertical_accel;
  tf::Quaternion myQuaternion;

  if (check_plus_minus(parsed_data[2]))
  {
    yaw = std::atof((parsed_data[2].substr(1)).c_str())*(M_PI/180);
  }
  else
  {
    yaw = std::atof((parsed_data[2].substr(1)).c_str())*(M_PI/-180);
  }

  if (check_plus_minus(parsed_data[3]))
  {
    pitch = std::atof((parsed_data[3].substr(1)).c_str())*(M_PI/180);
  }
  else
  {
    pitch = std::atof((parsed_data[3].substr(1)).c_str())*(M_PI/-180);
  }
    
  roll = std::atof(parsed_data[4].c_str())*(M_PI/180);


  if (check_plus_minus(parsed_data[5]))
  {
    x_accel = gs_to_ms2(std::atof((parsed_data[5].substr(1)).c_str()));
  }
  else
  {
    x_accel = gs_to_ms2(std::atof((parsed_data[5].substr(1)).c_str())*-1);
  }

  if (check_plus_minus(parsed_data[6]))
  {
    y_accel = gs_to_ms2(std::atof((parsed_data[6].substr(1)).c_str()));
  }
  else
  {
    y_accel = gs_to_ms2(std::atof((parsed_data[6].substr(1)).c_str())*-1);
  }

  if (check_plus_minus(parsed_data[7]))
  {
    z_accel = gs_to_ms2(std::atof((parsed_data[7].substr(1)).c_str()));
  }
  else
  {
    z_accel = gs_to_ms2(std::atof((parsed_data[7].substr(1)).c_str())*-1);
  }

  if (check_plus_minus(parsed_data[8]))
  {
    forward_accel = gs_to_ms2(std::atof((parsed_data[8].substr(1)).c_str()));
  }
  else
  {
    forward_accel = gs_to_ms2(std::atof((parsed_data[8].substr(1)).c_str())*-1);
  }

  if (check_plus_minus(parsed_data[9]))
  {
    lateral_accel = gs_to_ms2(std::atof((parsed_data[9].substr(1)).c_str()));
  }
  else
  {
    lateral_accel = gs_to_ms2(std::atof((parsed_data[9].substr(1)).c_str())*-1);
  }

  if (check_plus_minus(parsed_data[10]))
  {
    vertical_accel = gs_to_ms2(std::atof((parsed_data[10].substr(1)).c_str()));
  }
  else
  {
    vertical_accel = gs_to_ms2(std::atof((parsed_data[10].substr(1)).c_str())*-1);
  }
  
  if (check_plus_minus(parsed_data[11]))
  {
    x_angular = std::atof((parsed_data[11].substr(1)).c_str())*(M_PI/180);
  }
  else
  {
    x_angular = std::atof((parsed_data[11].substr(1)).c_str())*(M_PI/-180);
  }

  if (check_plus_minus(parsed_data[12]))
  {
    y_angular = std::atof((parsed_data[12].substr(1)).c_str())*(M_PI/180);
  }
  else
  {
    y_angular = std::atof((parsed_data[12].substr(1)).c_str())*(M_PI/-180);
  }

  if (check_plus_minus(parsed_data[13]))
  {
    z_angular = std::atof((parsed_data[13].substr(1)).c_str())*(M_PI/180);
  }
  else
  {
    z_angular = std::atof((parsed_data[13].substr(1)).c_str())*(M_PI/-180);
  }

  myQuaternion.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(myQuaternion, imu_msg.orientation);

  imu_msg.angular_velocity.x = x_angular;
  imu_msg.angular_velocity.y = y_angular;
  imu_msg.angular_velocity.z = z_angular;

  imu_msg.linear_acceleration.x = x_accel;
  imu_msg.linear_acceleration.y = y_accel;
  imu_msg.linear_acceleration.z = z_accel;

  ins.imu_pub.publish(imu_msg);
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "watson_ins");
  INS ins;
  serialIO serial_comm;
  serial_comm.openSerial();


  std::string* parsed_data;
  ros::Rate loop_rate(5);
  while(ros::ok()){
    ros::spinOnce();
    //sensor_msgs::NavSatFix gps_msg;
    //sensor_msgs::Imu imu_msg;
    std_msgs::String ins_msg;

    if(serial_comm.insAvailability()) {
      ins_msg.data = serial_comm.readSerial();
      parsed_data = parse_ins_data(ins_msg.data);

///Take into account +/- and *******
      //both gps and imu data are available
      if (parsed_data[0] == "G" || parsed_data[0] == "T")
      {
        populate_gps_data(ins, parsed_data);
        populate_imu_data(ins, parsed_data);
      }

      //gps data not available so only populate imu data
      else if (parsed_data[0] == "I" )
      {
        populate_imu_data(ins, parsed_data);
      }

      ins.test_pub.publish(ins_msg);
    }
    loop_rate.sleep();

  }

}
