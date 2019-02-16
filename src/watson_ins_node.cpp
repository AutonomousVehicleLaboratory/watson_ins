#include "watson_ins.h"
#include <string>
#include "std_msgs/String.h"

INS::INS(){
  
  //Set subscribers

}


//Define callbacks



int main(int argc, char *argv[]){
  ros::init(argc, argv, "watson_ins");
  INS ins;
  ros::spin();
  
  return 0;
}
