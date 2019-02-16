#ifndef SERIAL_IO 
#define SERIAL_IO 
#include <stdio.h>
#include <sstream>

#define BUFF_SIZE 63

class serialIO{
  public:
    serialIO();
    std::string m_DeviceName;
    int m_Device;
    unsigned char ins_buf[BUFF_SIZE];    
    int openSerial();
    
    int readSerial(char *buff);  





};

#endif
