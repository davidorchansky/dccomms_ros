#include <cstdio>
#include <cstdio>
#include <iostream>
#include <signal.h>
#include <sys/types.h>

#include <dccomms_ros/simulator/ROSCommsSimulator.h>

// ROS
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace dccomms_ros;
using namespace std;

static std::shared_ptr<spd::logger> Log =
    spd::stdout_color_mt("CommsSimulatorTest");
ROSCommsSimulator *sim;

void SIGINT_handler(int sig) {
  printf("Received %d signal\n", sig);
  printf("Log messages flushed.\n");
  exit(0);
}

void setSignals() {
  if (signal(SIGINT, SIGINT_handler) == SIG_ERR) {
    printf("SIGINT install error\n");
    exit(1);
  }
}

int main(int argc, char **argv) {
  setSignals();

  //// GET PARAMS
  ros::init(argc, argv, "dccomms_netsim");
  ros::NodeHandle nh("~");

  auto packetBuilder =
      dccomms::CreateObject<DataLinkFramePacketBuilder>(DataLinkFrame::crc16);

  auto sim = dccomms::CreateObject<ROSCommsSimulator>(nh, packetBuilder);
  sim->SetLogName("netsim");
  sim->LogToFile("netsim_log");

  Log->set_level(spdlog::level::debug);
  Log->flush_on(spd::level::info);

  sim->SetTransmitPDUCb([](int linkType, dccomms::PacketPtr dlf) {
    Log->info("Transmitting PDU");
  });
  sim->SetReceivePDUCb(
      [](int linkType, dccomms::PacketPtr dlf) { Log->info("PDU Received"); });
  sim->SetErrorPDUCb([](int linkType, dccomms::PacketPtr dlf) {
    Log->warn("PDU Received with errors");
  });

  sim->Start();

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
