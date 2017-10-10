#include <cpplogging/cpplogging.h>
#include <dccomms/dccomms.h>
#include <dccomms_utils/S100Stream.h>
#include <testing/comms/utils.hpp>

// ROS
#include "ros/ros.h"
// end ROS

using namespace cpplogging;
using namespace dccomms_utils;
using namespace dccomms;
using namespace teleop1;

LoggerPtr Log;

int main(int argc, char **argv) {
  ros::init(argc, argv, "master");
  ros::NodeHandle nh("~");

  auto logLevel = info;

  Log = CreateLogger("S100Master");
  Log->SetLogName(Log->GetLogName() + ":Main");
  Log->SetLogLevel(logLevel);

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

  std::string modemPort;
  if (!nh.getParam("modemPort", modemPort)) {
    Log->Error("Failed to get param 'modemPort'");
    return 1;
  } else {
    Log->Info("modem port: {}", modemPort);
  }

  int modemBaudrate;
  if (!nh.getParam("modemBaudrate", modemBaudrate)) {
    Log->Error("Failed to get param 'modemBaudrate'");
    return 1;
  } else {
    Log->Info("modem baudrate: {}", modemBaudrate);
  }

  auto portBaudrate = SerialPortStream::BAUD_2400;

  Ptr<S100Stream> stream =
      CreateObject<S100Stream>(modemPort, portBaudrate, modemBaudrate);
  ;

  stream->SetLogLevel(logLevel);
  stream->FlushLogOn(logLevel);
  stream->SetLogName(":S100Stream");

  if (logToFileEnabled) {
    stream->LogToFile(logPrefix + "_device");
  }
  stream->LogConfig();

  Ptr<SlavePacket> rxpkt = CreateObject<SlavePacket>();
  Ptr<MasterPacket> txpkt = CreateObject<MasterPacket>();

  stream->Open();
  std::thread txwork([txpkt, stream]() {
    int i = 0;
    while (1) {
      *txpkt->GetPayloadBuffer() = '0' + i;
      txpkt->UpdateFCS();
      Log->Info("Transmitting Heartbeat '{}'",
                (char)*txpkt->GetPayloadBuffer());
      *stream << txpkt;
      i = (i + 1) % 10;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::thread rxwork([rxpkt, stream]() {
    while (1) {
      *stream >> rxpkt;
      if (rxpkt->PacketIsOk()) {
        Log->Info("Data received: '{}'", (char *)rxpkt->GetPayloadBuffer());
      } else {
        Log->Warn("Packet received with errors");
      }
    }
  });

  while (1) {
    Utils::Sleep(10000);
  }
}
