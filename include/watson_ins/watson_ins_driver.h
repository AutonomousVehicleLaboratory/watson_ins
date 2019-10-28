#ifndef WATSON_INS_H
#define WATSON_INS_H
#include <serial/serial.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

struct ins_data_t {
  char       dataType[2]; // G, T, g, t, i, r
  char      timestamp[9]; // HHMMSS.S
  char            yaw[7]; // 2
  char          pitch[7]; // 3
  char           roll[6]; // 4
  char         xAccel[6]; // 5
  char         yAccel[6]; // 6
  char         zAccel[6]; // 7
  char     xAngleRate[6];
  char     yAngleRate[6];
  char     zAngleRate[6];
  char            lat[10];
  char            lon[11];
  char            alt[6];
  char            _cr[1];
};

class WatsonINSDriver {
  public:
    WatsonINSDriver(std::string path, int baud, int timeout_ms);  
    ~WatsonINSDriver();
    void read(sensor_msgs::Imu &imuMsg,
              sensor_msgs::NavSatFix &fixMsg,
              bool &validImu, bool &validFix);

  private:
    serial::Serial* serDev;
    int gpsMsgSeq;
    struct ins_data_t insData;

    void parseIMU(sensor_msgs::Imu &imuMsg);
    void parseGPS(sensor_msgs::NavSatFix &gpsMsg);
};
#endif /* WATSON_INS_H */
