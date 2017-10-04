
//#include <iostream>

//#include <cpplogging/Loggable.h>
#include <ns3/core-module.h>
#include <ns3/node-container.h>
#include <testing/simulator/ns3-testing.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ScratchSimulator");

int main(int argc, char **argv) {
  NS_LOG_UNCOND("Scratch Simulator");

  NodeContainer nodes;
  Simulator::Run();
  Simulator::Destroy();
}
