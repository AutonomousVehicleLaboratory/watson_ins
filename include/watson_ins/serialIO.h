#ifndef SERIAL_IO 
#define SERIAL_IO 
#include <stdio.h>
#include <sstream>
#include "serial/serial.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

#define BUFF_SIZE 63

class serialIO{
  public:
    serialIO();
    
    std::string m_DeviceName;
    int openSerial();
    int insAvailability();  
    std::string readSerial();
    sensor_msgs::NavSatFix getGPS(std::string raw_ascii); 
    sensor_msgs::Imu getImu(std::string raw_ascii); 
  private:
    serial::Serial m_Device;



};

#endif
