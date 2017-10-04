#include <cpplogging/cpplogging.h>
#include <testing/comms/utils.hpp>

// ROS
#include "ros/ros.h"
// end ROS

using namespace cpplogging;

LoggerPtr Log;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_rov");
  ros::NodeHandle nh("~");

  Log = CreateLogger("ROVMain");
  Log->SetLogLevel(debug);

  std::string logPrefix;
  bool logToFileEnabled;
  if (!nh.getParam("logPrefix", logPrefix)) {
    Log->Error("Failed to get param 'logPrefix'");
    return 1;
  } else {
    if (logPrefix == "") {
      Log->Info("Do not log to file");
      logToFileEnabled = false;
    } else {
      Log->Info("Log prefix: {}", logPrefix);
      logToFileEnabled = true;
    }
  }

  if (logToFileEnabled) {
    Log->LogToFile(logPrefix + "_main");
  }
}
