/*
 * Watson DMS-SGP02 INS ROS Driver
 */
#include "watson_ins_driver.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

WatsonINSDriver::WatsonINSDriver(std::string path, int baud, int timeout_ms)
{
  ROS_INFO("Connecting to Watson INS...");
  // perform initialization sequence?
  serDev = new serial::Serial(path, baud, serial::Timeout::simpleTimeout(timeout_ms));
  ROS_INFO("Successfully connected to Watson INS");

  gpsMsgSeq = 0;
  memset(&insData, 0, sizeof(struct ins_data_t));
}

#define Hour2Sec(H) (H*60*60)
#define Min2Sec(H) (H*60)
#define TSec2Nanosec(TS) (TS*100*1000)
void WatsonINSDriver::parseGPS(sensor_msgs::NavSatFix &gpsMsg)
{
  int h, m, s, ts;
  sscanf(insData.timestamp, "%2d%2d%2d.%1d", &h, &m, &s, &ts);

  gpsMsg.header.seq = gpsMsgSeq++;
  gpsMsg.header.stamp.sec = Hour2Sec(h) + Min2Sec(m) + s; 
  gpsMsg.header.stamp.nsec = TSec2Nanosec(ts);

  // Nav Sat Status
  sensor_msgs::NavSatStatus navSatStatus;
  navSatStatus.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
  // GPS Msg
  gpsMsg.status = navSatStatus;
  sscanf(insData.lat, "%lf", &gpsMsg.latitude);
  sscanf(insData.lon, "%lf", &gpsMsg.longitude);
  sscanf(insData.alt, "%lf", &gpsMsg.altitude);
}

#define Degree2Radians(D) (D * M_PI/180)
#define Gs2Meters2(G) (G / 0.101972)
void WatsonINSDriver::parseIMU(sensor_msgs::Imu &imuMsg)
{
  double roll, pitch, yaw; 
  double x_angular, y_angular, z_angular;
  double x_accel, y_accel, z_accel;
  double forward_accel, lateral_accel, vertical_accel;

  sscanf(insData.yaw, "%lf", &yaw);
  sscanf(insData.pitch, "%lf", &pitch);
  sscanf(insData.roll, "%lf", &roll);
  tf::Quaternion quat_tf;
  quat_tf.setRPY(roll, pitch, yaw);

  sscanf(insData.xAccel, "%lf", &x_accel);
  sscanf(insData.yAccel, "%lf", &y_accel);
  sscanf(insData.zAccel, "%lf", &z_accel);
  sscanf(insData.xAngleRate, "%lf", &x_angular);
  sscanf(insData.yAngleRate, "%lf", &y_angular);
  sscanf(insData.zAngleRate, "%lf", &z_angular);

  imuMsg.header.stamp = ros::Time::now();
  tf::quaternionTFToMsg(quat_tf, imuMsg.orientation);
  imuMsg.angular_velocity.x = Degree2Radians(x_angular);
  imuMsg.angular_velocity.y = Degree2Radians(y_angular);
  imuMsg.angular_velocity.z = Degree2Radians(z_angular);
  imuMsg.linear_acceleration.x = Gs2Meters2(x_accel);
  imuMsg.linear_acceleration.y = Gs2Meters2(y_accel);
  imuMsg.linear_acceleration.z = Gs2Meters2(z_accel);
}

void WatsonINSDriver::read(sensor_msgs::Imu &imuMsg,
                           sensor_msgs::NavSatFix &fixMsg,
                           bool &validImu, bool &validFix)
{
  validImu = validFix = false;

  memset(&insData, 0, sizeof(struct ins_data_t));
  if (sizeof(struct ins_data_t) != serDev->read((uint8_t*)&insData,
                                                sizeof(struct ins_data_t))) {
    return;
  }

  switch (insData.dataType[0]) {
    case 'G':
    case 'T':
      ROS_INFO("Both GPS and IMU data is available");
      parseGPS(fixMsg);
      parseIMU(imuMsg);
      validImu = validFix = true;
      break;
    case 'I':
      // Only IMU data is available
      parseIMU(imuMsg);
      validImu = true;
      break;
    case 'g':
    case 't':
    case 'i':
    case 'r':
      ROS_WARN("Calculated altitude/heading error exceeds ranges");
      break;
    default:
      return;
  }
}

WatsonINSDriver::~WatsonINSDriver()
{
  ROS_INFO("Closing Watson INS Driver");
  serDev->close();
  delete serDev;
}
