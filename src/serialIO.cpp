#include <math.h>
#include <iostream>
#include <string>
#include "serialIO.h"
#include <ros/console.h>

#include <iostream>
#include <sys/types.h>
#include <linux/serial.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

serialIO::serialIO()
{  
  m_DeviceName = "/dev/ttyUSB0";
}

int serialIO::openSerial()
{
  m_Device = open(m_DeviceName.c_str(), std::ios::in, std::ios::out);
  if(m_Device < 0)
  {
    ROS_ERROR("Error Opening Device: %s", m_DeviceName.c_str());    
  }

  return 1;
}

/**
 * RS-232 Output Format:
 * ASCII characters set asynchronously at regular intervals.
 * String sent at 9600 baud with eight data bits, one stop bit and no parity.
 *    - Maximum rate is 71.11 strings per second (dependent on baud rate)
 *
 */
int serialIO::readSerial(char *buff)
{
  ssize_t  BRead = read(m_Device, buff, BUFF_SIZE);
  ROS_INFO("buff: [ %s ]", buff); 
}

