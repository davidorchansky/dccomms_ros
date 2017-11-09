#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>

namespace dccomms_ros {

AcousticROSCommsDevice::AcousticROSCommsDevice(ROSCommsSimulatorPtr s,
                                               PacketBuilderPtr pb)
    : ROSCommsDevice(s, pb) {
  _started = false;
  _nodeListIndex = ns3::NodeList::GetNNodes();
  _node = ns3::CreateObject<ns3::Node>();
  _mobh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  _mobh.Install(_node);

  _asHelper = ns3::AquaSimHelper::Default();
  _asHelper.SetMac("ns3::AquaSimBroadcastMac");
  _asHelper.SetRouting("ns3::AquaSimRoutingDummy");
  _device = ns3::CreateObject<ns3::AquaSimNetDevice>();
}

void AcousticROSCommsDevice::_Recv(std::string context,
                                   ns3::Ptr<const ns3::Packet> pkt) {
  ns3::AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();

  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  Info("({} secs; {}) {}: (Addr: {}) Received packet from {}", secs, datetime,
       context, daddr, saddr);
}

DEV_TYPE AcousticROSCommsDevice::GetDevType() {
  return DEV_TYPE::ACOUSTIC_UNDERWATER_DEV;
}

void AcousticROSCommsDevice::DoSetMac(uint32_t mac) {
  _aquaSimAddr = ns3::AquaSimAddress(static_cast<uint16_t>(mac));
}

void AcousticROSCommsDevice::DoSend(dccomms::PacketPtr pkt) {
  while (!_started)
    this_thread::sleep_for(chrono::milliseconds(100));
  auto packetSize = pkt->GetPacketSize();
  auto packetBuffer = pkt->GetBuffer();
  ns3::Ptr<ns3::Packet> ns3pkt =
      ns3::Create<ns3::Packet>(packetBuffer, packetSize);

  // TODO: check pkt destination addres (if exists). At the moment we
  // consider a
  // broadcast
  // this does not work (traced callbacks length = 0 in this thread)
  _device->Send(ns3pkt, AquaSimAddress::GetBroadcast(), 0);

  //  Simulator::Schedule(Seconds(0),
  //                      MakeEvent(&ns3::AquaSimNetDevice::Send, (&*_device),
  //                                ns3pkt, AquaSimAddress::GetBroadcast(), 0));
}

void AcousticROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {

    AcousticCommsChannelPtr acChannel =
        static_pointer_cast<AcousticCommsChannel>(channel);
    _channel = acChannel->GetAquaSimChannel();
  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}

void AcousticROSCommsDevice::DoStart() {
  _asHelper.SetChannel(_channel);
  _asHelper.Create(_node, _device);
  _device->SetAddress(_aquaSimAddr);
  _macLayer = _device->GetMac();
  _routingLayer = _device->GetRouting();
  _mobility = _node->GetObject<ns3::MobilityModel>();

  _device->GetPhy()->SetTransRange(20);
  _mobility->SetPosition(Vector3D(10 * _nodeListIndex, 0, 0));
  // TODO: set receive callback
  ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                           "/DeviceList/0/Routing/PacketReceived",
                       MakeCallback(&AcousticROSCommsDevice::_Recv, this));
  _started = true;
}
}
