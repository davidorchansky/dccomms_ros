#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>

namespace dccomms_ros {

AcousticROSCommsDevice::AcousticROSCommsDevice(ROSCommsSimulatorPtr s,
                                               PacketBuilderPtr txpb,
                                               PacketBuilderPtr rxpb)
    : ROSCommsDevice(s, txpb, rxpb) {
  _started = false;
  _nodeListIndex = ns3::NodeList::GetNNodes();
  _node = ns3::CreateObject<ns3::Node>();
  _mobh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  _mobh.Install(_node);

  _asHelper = ns3::AquaSimHelper::Default();
  _asHelper.SetMac("ns3::AquaSimBroadcastMac");
  _asHelper.SetRouting("ns3::AquaSimRoutingDummy");
  _routingType = AQS_ROUTING_DUMMY;
  _device = ns3::CreateObject<ns3::AquaSimNetDevice>();
}

void AcousticROSCommsDevice::_SendTrace(string context,
                                        ns3::Ptr<const ns3::Packet> pkt) {
  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto nhaddr = ash.GetNextHop().GetAsInt();

  Info("({} secs; {}) {}: (Addr: {}) Transmitting packet to {}. Next hop: {} ; "
       "{} bytes",
       secs, datetime, context, saddr, daddr, nhaddr, pkt->GetSize());
  FlushLog();
}

void AcousticROSCommsDevice::_Recv(std::string context,
                                   ns3::Ptr<const ns3::Packet> pkt) {
  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  auto packet = pkt->Copy();
  switch (_routingType) {
  case AQS_ROUTING_DUMMY: {
    ns3::AquaSimHeader ash;
    auto psize = packet->GetSize();
    packet->RemoveHeader(ash);
    auto saddr = ash.GetSAddr().GetAsInt();
    auto daddr = ash.GetDAddr().GetAsInt();
    auto numForwards = ash.GetNumForwards();
    while (ash.GetNumForwards() > 0) {
      packet->RemoveHeader(ash);
    }
    char ser[5000];
    auto size = packet->GetSize();
    packet->CopyData((uint8_t *)ser, size);
    auto dccommsPacket = _rxpb->CreateFromBuffer(ser);
    Info("({} secs; {}) {}: (Own Addr: {} Dest. Addr: {}) Received packet from "
         "{} ({} forwards) ({} bytes)",
         secs, datetime, context, GetMac(), daddr, saddr, numForwards, psize);
    ReceiveFrame(dccommsPacket);
    FlushLog();
    break;
  }
  case AQS_ROUTING_VBF: {
    break;
  default:
    break;
  }
  }
}

void AcousticROSCommsDevice::_RxError(std::string context,
                                      ns3::Ptr<const ns3::Packet> pkt) {
  Warn("Packet received with errors!");
  //TODO: send the packet to de upper layer despite the errors
}

DEV_TYPE AcousticROSCommsDevice::GetDevType() {
  return DEV_TYPE::ACOUSTIC_UNDERWATER_DEV;
}

void AcousticROSCommsDevice::DoSetMac(uint32_t mac) {
  _aquaSimAddr = ns3::AquaSimAddress(static_cast<uint16_t>(mac));
}

void AcousticROSCommsDevice::DoSend(dccomms::PacketPtr pkt) {
  while (!_started) {
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  auto packetSize = pkt->GetPacketSize();
  auto packetBuffer = pkt->GetBuffer();
  ns3::Ptr<ns3::Packet> ns3pkt =
      ns3::Create<ns3::Packet>(packetBuffer, packetSize);

  uint16_t daddr = pkt->GetDestAddr();
  switch (_routingType) {
  case AQS_ROUTING_DUMMY: {
    ns3::AquaSimHeader ash;
    ash.SetNumForwards(0);
    ns3pkt->AddHeader(ash);
    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
                                        &ns3::AquaSimNetDevice::Send, _device,
                                        ns3pkt, AquaSimAddress(daddr), 0);
    break;
  }
  case AQS_ROUTING_VBF: {
    break;
  default:
    break;
  }
  }
}

void AcousticROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel, CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {

    AcousticCommsChannelPtr acChannel =
        static_pointer_cast<AcousticCommsChannel>(channel);
    _channel = acChannel->GetAquaSimChannel();
  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}

void AcousticROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  if (_started) {
    double x = position.getX(), y = position.getY(), z = position.getZ();
    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
                                        &ns3::MobilityModel::SetPosition,
                                        _mobility, ns3::Vector3D(x, y, z));
  }
}

void AcousticROSCommsDevice::_PositionUpdated(
    std::string context, ns3::Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition();

  std::string datetime;
  double secs;
  _sim->GetSimTime(datetime, secs);

  Info("({} secs; {}) {}: [x,y,z] = [{},{},{}]", secs, datetime, context,
       position.x, position.y, position.z);
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
  ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                           "/DeviceList/0/Routing/PacketReceived",
                       MakeCallback(&AcousticROSCommsDevice::_Recv, this));
  ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                           "/DeviceList/0/Routing/PacketTransmitting",
                       MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
//  ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
//                           "/DeviceList/0/Phy/RxError",
//                       MakeCallback(&AcousticROSCommsDevice::_RxError, this));
//  Config::Connect(
//      "/NodeList/" + std::to_string(_nodeListIndex) +
//          "/$ns3::MobilityModel/CourseChange",
//      MakeCallback(&AcousticROSCommsDevice::_PositionUpdated, this));
  _started = true;
}

bool AcousticROSCommsDevice::DoStarted() { return _started; }
}
