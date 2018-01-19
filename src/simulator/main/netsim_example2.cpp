/*
 * This example configures and starts the ROS interface
 * without configuring and starting a network. The simulated
 * network will be configured and started by calling to ROS
 * services. All packets are of the type dccomms::DataLinkFrame
 * with crc16.
 */


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

int main(int argc, char **argv) {
  //// GET PARAMS
  ros::init(argc, argv, "dccomms_netsim");
  ros::NodeHandle nh("~");

  auto packetBuilder =
      dccomms::CreateObject<dccomms::DataLinkFramePacketBuilder>(
          dccomms::DataLinkFrame::crc16);

  auto sim = dccomms::CreateObject<ROSCommsSimulator>(nh);
  sim->SetLogName("netsim");
  sim->LogToFile("netsim_log");

  Log->set_level(spdlog::level::debug);
  Log->flush_on(spd::level::info);

  sim->SetDefaultPacketBuilder(packetBuilder);

  sim->StartROSInterface();
  Log->info("ROS Interface started.");

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
