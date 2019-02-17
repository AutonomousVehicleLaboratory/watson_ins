#ifndef SERIAL_IO 
#define SERIAL_IO 
#include <stdio.h>
#include <sstream>
#include "serial/serial.h"
#define BUFF_SIZE 63

class serialIO{
  public:
    serialIO();
    
    std::string m_DeviceName;
    int openSerial();
    int insAvailability();  
    std::string readSerial();  
    
  private:
    serial::Serial m_Device;



};

#endif
