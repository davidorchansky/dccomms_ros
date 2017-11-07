#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <ns3/aqua-sim-helper.h>
#include <ns3/mobility-helper.h>

namespace dccomms_ros {

AcousticROSCommsDevice::AcousticROSCommsDevice(ROSCommsSimulatorPtr s,
                                               PacketBuilderPtr pb)
    : ROSCommsDevice(s, pb) {

  _node = ns3::CreateObject<ns3::Node>();
  ns3::MobilityHelper mobh;
  mobh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobh.Install(_node);

  ns3::AquaSimHelper asHelper;
  asHelper = ns3::AquaSimHelper::Default();
  asHelper.SetMac("ns3::AquaSimBroadcastMac");
  asHelper.SetRouting("ns3::AquaSimRoutingDummy");
  _device = ns3::CreateObject<ns3::AquaSimNetDevice>();

  asHelper.Create(_node, _device);

  // TODO: set receive callback
}

DEV_TYPE AcousticROSCommsDevice::GetDevType() {
  return DEV_TYPE::ACOUSTIC_UNDERWATER_DEV;
}

void AcousticROSCommsDevice::DoSetMac(uint32_t mac) {
  ns3::AquaSimAddress aquaSimAddr =
      ns3::AquaSimAddress(static_cast<uint16_t>(mac));
  _device->SetAddress(aquaSimAddr);
}

void AcousticROSCommsDevice::DoSend(dccomms::PacketPtr pkt) {
  ns3::Ptr<ns3::Packet> ns3pkt =
      ns3::Create<ns3::Packet>(pkt->GetBuffer(), pkt->GetPacketSize());

  // TODO: check pkt destination addres (if exists). At the moment we
  // consider a
  // broadcast
  _device->Send(ns3pkt, AquaSimAddress::GetBroadcast(), 0);
  _macLayer->RecvProcess(ns3pkt);
}

void AcousticROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {

    AcousticCommsChannelPtr acChannel =
        static_pointer_cast<AcousticCommsChannel>(channel);
    _channel = acChannel->GetAquaSimChannel();
    _device->SetChannel(_channel);
  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}

void AcousticROSCommsDevice::DoStart() {
  // DO NOTHING
}
}
