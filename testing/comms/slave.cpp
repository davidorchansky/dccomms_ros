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

  Log = CreateLogger("S100Slave");
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

  Ptr<MasterPacket> rxpkt = CreateObject<MasterPacket>();
  Ptr<SlavePacket> txpkt = CreateObject<SlavePacket>();

  stream->Open();

  std::thread work([rxpkt, txpkt, stream]() {
    int i = 0;
    while (1) {
      *stream >> rxpkt;
      if (rxpkt->PacketIsOk()) {
        Log->Info("Heartbeat received ({} bytes): '{}'", rxpkt->GetPacketSize(),
                  (char)*rxpkt->GetPayloadBuffer());
        const char seq = *rxpkt->GetPayloadBuffer();
        std::string msg = seq + std::string(": Hello world!") +
                          std::string("; 1234567890 1234567890");
        txpkt->SetPayload(msg.c_str(), msg.length());
        txpkt->UpdateFCS();
        Log->Info("Transmitting packet with data ({} bytes): '{}'",
                  txpkt->GetPacketSize(), txpkt->GetPayloadBuffer());
        *stream << txpkt;
      } else {
        Log->Warn("Packet received with errors");
      }
    }

    //      std::string msg = std::to_string(i) + std::string(": Hello world!")
    //      +
    //                        std::string("; 1234567890 1234567890");
    //      txpkt->SetPayload(msg.c_str(), msg.length());
    //      txpkt->UpdateFCS();
    //      Log->Info("Transmitting packet with data ({} bytes): '{}'",
    //                txpkt->GetPacketSize(), txpkt->GetPayloadBuffer());
    //      *stream << txpkt;
    //      // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    //      i = (i + 1) % 10;
    // }
  });

  while (1) {
    Utils::Sleep(10000);
  }
}
