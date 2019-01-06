/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Connecticut
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

#include "ns3/applications-module.h"
#include "ns3/aqua-sim-ng-module.h"
#include "ns3/callback.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h" //may not be needed here...
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include <cpplogging/cpplogging.h>
#include <fstream>
#include <thread>
/*
 * BroadCastMAC
 *
 * N ---->  N  -----> N -----> N* -----> S
 *
 */

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BMac");

class Test : virtual public cpplogging::Logger {
public:
  Test();
  void RunTest();
  void RoutingPacketTx(std::string context, Ptr<const Packet>);
  void RoutingPacketRx(std::string context, Ptr<const Packet>);
  void MacPacketTx(std::string context, Ptr<const Packet>);
  void MacPacketRx(std::string context, Ptr<const Packet>);
  void CourseChange(std::string context, Ptr<const MobilityModel> model);
  void GetSimTime(const char *format, std::string &datetime,
                  double &secsFromStart);
  void SetSimulationStartDateTime();
  void SendPacket(ns3::Ptr<NetDevice> &dev, Address &addr) {
    auto pkt = ns3::Create<ns3::Packet>(200);
    dev->Send(pkt, addr, 0);
  }

private:
  cpplogging::Logger routingLog, macLog, phyLog, mobilityLog;
  const char timeFormat[100] = "%Y-%m-%d %H:%M:%S";
  std::chrono::high_resolution_clock::time_point start;
};

Test::Test() {
  SetLogName("Aqua-Sim test 0");
  routingLog.SetLogName("Routing");
  macLog.SetLogName("Mac");
  phyLog.SetLogName("Phy");
  mobilityLog.SetLogName("Mobility");

  cpplogging::LogLevel level = cpplogging::LogLevel::debug;

  SetLogLevel(level);
  routingLog.SetLogLevel(level);
  macLog.SetLogLevel(level);
  phyLog.SetLogLevel(level);
  mobilityLog.SetLogLevel(level);
}

void Test::SetSimulationStartDateTime() {
  start = std::chrono::high_resolution_clock::now();
}
void Test::GetSimTime(const char *format, std::string &datetime,
                      double &secsFromStart) {
  auto simTime = Simulator::Now();
  secsFromStart = simTime.GetSeconds();
  auto tstamp = simTime.GetTimeStep(); // nanoseconds

  auto simNanos = std::chrono::nanoseconds(tstamp);
  auto curpoint = start + simNanos;
  auto t = std::chrono::high_resolution_clock::to_time_t(curpoint);
  auto localEventTime = std::localtime(&t);
  char mbstr[100];
  auto count = std::strftime(mbstr, sizeof(mbstr), format, localEventTime);
  char *mp = mbstr + count;

  auto durationFromEpoch = curpoint.time_since_epoch();
  auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(durationFromEpoch) -
      std::chrono::duration_cast<std::chrono::seconds>(durationFromEpoch);
  sprintf(mp, ".%ld", millis.count());
  datetime = mbstr;
}

void Test::RoutingPacketRx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto size = ash.GetSize();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info(
      "({} secs; {}) {}: (Addr: {}) Received packet from {} ; {} bytes", secs,
      datetime, context, daddr, saddr, size);
}

void Test::MacPacketRx(std::string context, Ptr<const Packet> pkt) {
  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: Received packet; {} bytes", secs, datetime,
                  context, pkt->GetSize());
}

void Test::MacPacketTx(std::string context, Ptr<const Packet> pkt) {
  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: {} bytes", secs, datetime, context,
                  pkt->GetSize());
}

void Test::CourseChange(std::string context, Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);
  mobilityLog.Info("({} secs; {}) {}: [x,y,z] = [{},{},{}]", secs, datetime,
                   context, position.x, position.y, position.z);
}
void Test::RoutingPacketTx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto nhaddr = ash.GetNextHop().GetAsInt();
  auto size = ash.GetSize();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: (Addr: {}) Transmitting packet to {}. "
                  "Next hop: {} ; {} bytes",
                  secs, datetime, context, saddr, daddr, nhaddr, size);
}

void Test::RunTest() {
  int nodes = 3;
  double range = 100;
  uint devBitRate = 10e4;

  std::string asciiTraceFile = "bMAC-trace.asc";

  /*
   * **********
   * Node -> NetDevice -> AquaSimNetDeive -> etc.
   * Note: Nodelist keeps track of all nodes created.
   * ---Also need to look into id of nodes and assignment of this
   * ---need to look at assignment of address and making it unique per node.
   *
   *
   *  Ensure to use NS_LOG when testing in terminal. ie. ./waf --run
   * broadcastMAC_example NS_LOG=Node=level_all or export
   * 'NS_LOG=*=level_all|prefix_func'
   *  *********
   */

  LogComponentEnable("AquaSimFama", LogLevel(LOG_ALL | LOG_PREFIX_ALL));

  std::cout << "-----------Initializing simulation-----------\n";

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));

  NodeContainer nodesCon;
  nodesCon.Create(nodes);

  // establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();

  asHelper.SetChannel(channel.Create());
  asHelper.SetMac("ns3::AquaSimFama");
  asHelper.SetMacAttribute("BitRate", DoubleValue(devBitRate * 0.8));
  asHelper.SetMacAttribute("EncodingEfficiency", DoubleValue(1));

  MobilityHelper mobility;
  NetDeviceContainer devices;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();

  // Static Y and Z dimension for now
  Vector boundry = Vector(0, 0, 0);

  std::cout << "Creating Nodes\n";

  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++) {
    Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
    position->Add(boundry);
    devices.Add(asHelper.CreateWithoutRouting(*i, newDevice));
    boundry.x += 20;

    auto phy = newDevice->GetPhy();
    phy->SetTransRange(range);
    phy->SetAttribute("PT", ns3::DoubleValue(0.2818));
    phy->SetAttribute("Frequency", ns3::DoubleValue(25));
    phy->SetAttribute("L", ns3::DoubleValue(0));
    phy->SetAttribute("K", ns3::DoubleValue(2.0));
    phy->SetAttribute("TurnOnEnergy", ns3::DoubleValue(0));
    phy->SetAttribute("TurnOffEnergy", ns3::DoubleValue(0));
    phy->SetAttribute("Preamble", ns3::DoubleValue(0));
    auto em = phy->EM();
    em->SetAttribute("InitialEnergy", ns3::DoubleValue(900000));
    em->SetAttribute("RxPower", ns3::DoubleValue(0.660));
    em->SetAttribute("TxPower", ns3::DoubleValue(0.395));
    em->SetAttribute("IdlePower", ns3::DoubleValue(0.0));
    ns3::Ptr<AquaSimModulation> modulation = phy->Modulation(NULL);
    modulation->SetAttribute("CodingEff", ns3::DoubleValue(1));
    modulation->SetAttribute("SPS", ns3::UintegerValue(devBitRate));
    modulation->SetAttribute("BER", ns3::DoubleValue(0.0));
  }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);

  ns3::Ptr<NetDevice> dev0 = devices.Get(0);
  ns3::Address sinkAddr = devices.Get(nodes - 1)->GetAddress();

  Packet::EnablePrinting();
  std::ofstream ascii(asciiTraceFile.c_str());
  if (!ascii.is_open()) {
    NS_FATAL_ERROR("Could not open trace file.");
  }
  asHelper.EnableAsciiAll(ascii);

  std::cout << "-----------Running Simulation-----------\n";

  Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketReceived",
                  MakeCallback(&Test::RoutingPacketRx, this));
  Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketTransmitting",
                  MakeCallback(&Test::RoutingPacketTx, this));

  ns3::Config::Connect("/NodeList/*/DeviceList/0/Phy/MacRx",
                       MakeCallback(&Test::MacPacketRx, this));

  ns3::Config::Connect("/NodeList/*/DeviceList/0/Phy/MacTx",
                       MakeCallback(&Test::MacPacketTx, this));

  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",
                  MakeCallback(&Test::CourseChange, this));

  std::thread app([&]() {
    std::this_thread::sleep_for(std::chrono::seconds(20));
    while (1) {
      Simulator::Schedule(Seconds(1),
                          MakeEvent(&Test::SendPacket, this, dev0, sinkAddr));
      std::this_thread::sleep_for(std::chrono::seconds(4));
    }
  });
  app.detach();

  //  Simulator::Schedule(Seconds(60),
  //                      MakeEvent(&Test::SendPacket, this, dev0, addr3));

  Simulator::Schedule(Seconds(0),
                      MakeEvent(&Test::SetSimulationStartDateTime, this));
  Simulator::Run();
  ;
  Simulator::Destroy();
}

int main(int argc, char *argv[]) {
  Test test;
  test.RunTest();
  return 0;
}
