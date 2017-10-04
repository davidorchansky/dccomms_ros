
//#include <iostream>

//#include <cpplogging/Loggable.h>
#include <dccomms_ros/ns3-testing.h>
#include <ns3/core-module.h>
#include <ns3/node-container.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ScratchSimulator");

int main(int argc, char **argv) {
  NS_LOG_UNCOND("Scratch Simulator");

  NodeContainer nodes;
  Simulator::Run();
  Simulator::Destroy();
}
