#include <math.h>
#include <iostream>
#include <string>
#include "serialIO.h"
#include <ros/console.h>


/**
 * RS-232 Output Format:
 * ASCII characters set asynchronously at regular intervals.
 * String sent at 9600 baud with eight data bits, one stop bit and no parity.
 *    - Maximum rate is 71.11 strings per second (dependent on baud rate)
 *
 */

serialIO::serialIO()
{  
  m_DeviceName = "/dev/ttyUSB0";
}

int serialIO::openSerial()
{
  //m_Device = open(m_DeviceName.c_str(), std::ios::in, std::ios::out);
  try
  { 
    m_Device.setPort(m_DeviceName);
    m_Device.setBaudrate(9600);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    m_Device.setTimeout(time_out);
    m_Device.open();
  }
  catch(serial::IOException& e)
  {
    ROS_ERROR("Error Opening Device: %s", m_DeviceName.c_str());
    return -1; 
  }
  
  if(m_Device.isOpen())
    ROS_INFO("INS Port Open"); 
  
  return 0;
}

int serialIO::insAvailability()
{
  if(m_Device.available())
    ROS_INFO("Reading"); 
    return 1;
  
  return 0;
}

std::string serialIO::readSerial()
{
  //m_Device.available() returns the number of characters in the buffer
  size_t num_chars = m_Device.available();
  ROS_INFO("Chars read: %d", num_chars); 
  return m_Device.read(m_Device.available());
  //std::string buff_new = buff.substr(0,63);
  //ROS_INFO("String: %s", buff.c_str());
}
