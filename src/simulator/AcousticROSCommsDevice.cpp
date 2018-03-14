#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <ns3/aqua-sim-header.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>

namespace dccomms_ros {

using namespace ns3;
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
  _routingType = AQS_NOROUTING;
  _device = ns3::CreateObject<ns3::AquaSimNetDevice>();
  _pT = 0.2818;
  _freq = 25;
  _L = 0;
  _K = 2.0;
  _turnOnEnergy = 0;
  _turnOffEnergy = 0;
  _preamble = 0;
  _pTConsume = 0.660;
  _pRConsume = 0.395;
  _pIdle = 0.0;
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

  Debug(
      "({} secs; {}) {}: (Addr: {}) Transmitting packet to {}. Next hop: {} ; "
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
  case AQS_NOROUTING: {
    ns3::AquaSimHeader ash;
    auto psize = packet->GetSize();
    packet->RemoveHeader(ash);
    auto saddr = ash.GetSAddr().GetAsInt();
    auto daddr = ash.GetDAddr().GetAsInt();
    Debug("Packet received ({} bytes ; {} bytes ; {} bytes) , S:{} ; D:{}", psize,
         packet->GetSize(), ash.GetSize(), saddr, daddr);
    char ser[5000];
    auto size = packet->GetSize();
    packet->CopyData((uint8_t *)ser, size);
    auto dccommsPacket = _rxpb->CreateFromBuffer(ser);
    Debug(
        "({} secs; {}) {}: (Own Addr: {} Dest. Addr: {}) Received packet from "
        "{} ({} bytes)",
        secs, datetime, context, GetMac(), daddr, saddr, psize);
    ReceiveFrame(dccommsPacket);
    FlushLog();
    break;
  }
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
    Debug(
        "({} secs; {}) {}: (Own Addr: {} Dest. Addr: {}) Received packet from "
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
  // TODO: send the packet to de upper layer despite the errors
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
  case AQS_NOROUTING: {
    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
                                        &ns3::AquaSimNetDevice::Send, _device,
                                        ns3pkt, AquaSimAddress(daddr), 0);
    break;
  }
  case AQS_ROUTING_DUMMY: {
//    ns3::AquaSimHeader ash;
//    ash.SetSize(pkt->GetPacketSize());
//    ash.SetNumForwards(0);
//    ns3pkt->AddHeader(ash);
//    ns3::Simulator::ScheduleWithContext(GetMac(), Seconds(0),
//                                        &ns3::AquaSimNetDevice::Send, _device,
//                                        ns3pkt, AquaSimAddress(daddr), 0);
    break;
  }
  case AQS_ROUTING_VBF: {
    break;
  default:
    break;
  }
  }
}

void AcousticROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel,
                                             CHANNEL_LINK_TYPE linkType) {
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

void AcousticROSCommsDevice::SetMACProtocol(const std::string &name) {
  _macP = GetMACPType(name);
}
void AcousticROSCommsDevice::SetRange(double value) { _range = value; }
void AcousticROSCommsDevice::SetPT(double value) { _pT = value; }
void AcousticROSCommsDevice::SetFreq(double value) { _freq = value; }
void AcousticROSCommsDevice::SetL(double value) { _L = value; }
void AcousticROSCommsDevice::SetK(double value) { _K = value; }
void AcousticROSCommsDevice::SetTurnOnEnergy(double value) {
  _turnOnEnergy = value;
}
void AcousticROSCommsDevice::SetTurnOffEnergy(double value) {
  _turnOffEnergy = value;
}
void AcousticROSCommsDevice::SetPreamble(double value) { _preamble = value; }
void AcousticROSCommsDevice::SetPTConsume(double value) { _pTConsume = value; }
void AcousticROSCommsDevice::SetPRConsume(double value) { _pRConsume = value; }
void AcousticROSCommsDevice::SetPIdle(double value) { _pIdle = value; }

void AcousticROSCommsDevice::DoStart() {

  _asHelper.SetMac(_macP);
  _asHelper.SetChannel(_channel);
  if (_routingType == AQS_NOROUTING)
    _asHelper.CreateWithoutRouting(_node, _device);
  else
    _asHelper.Create(_node, _device);
  _device->SetAddress(_aquaSimAddr);
  _macLayer = _device->GetMac();
  _mobility = _node->GetObject<ns3::MobilityModel>();

  auto phy = _device->GetPhy();
  phy->SetTransRange(_range);
  phy->SetAttribute("PT", ns3::DoubleValue(_pT));
  phy->SetAttribute("Frequency", ns3::DoubleValue(_freq));
  phy->SetAttribute("L", ns3::DoubleValue(_L));
  phy->SetAttribute("K", ns3::DoubleValue(_K));
  phy->SetAttribute("TurnOnEnergy", ns3::DoubleValue(_turnOnEnergy));
  phy->SetAttribute("TurnOffEnergy", ns3::DoubleValue(_turnOffEnergy));
  phy->SetAttribute("Preamble", ns3::DoubleValue(_preamble));
  phy->SetAttribute("PRConsume", ns3::DoubleValue(_pRConsume));
  phy->SetAttribute("PTConsume", ns3::DoubleValue(_pTConsume));
  phy->SetAttribute("PIdle", ns3::DoubleValue(_pIdle));

  _mobility->SetPosition(Vector3D(10 * _nodeListIndex, 0, 0));
  if (_routingType != AQS_NOROUTING) {
    _routingLayer = _device->GetRouting();
    ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                             "/DeviceList/0/Routing/PacketReceived",
                         MakeCallback(&AcousticROSCommsDevice::_Recv, this));
    ns3::Config::Connect(
        "/NodeList/" + std::to_string(_nodeListIndex) +
            "/DeviceList/0/Routing/PacketTransmitting",
        MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
  } else {
    ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
                             "/DeviceList/0/Mac/RoutingRx",
                         MakeCallback(&AcousticROSCommsDevice::_Recv, this));
    ns3::Config::Connect(
        "/NodeList/" + std::to_string(_nodeListIndex) +
            "/DeviceList/0/Mac/RoutingTx",
        MakeCallback(&AcousticROSCommsDevice::_SendTrace, this));
  }
  //  ns3::Config::Connect("/NodeList/" + std::to_string(_nodeListIndex) +
  //                           "/DeviceList/0/Phy/RxError",
  //                       MakeCallback(&AcousticROSCommsDevice::_RxError,
  //                       this));
  //  Config::Connect(
  //      "/NodeList/" + std::to_string(_nodeListIndex) +
  //          "/$ns3::MobilityModel/CourseChange",
  //      MakeCallback(&AcousticROSCommsDevice::_PositionUpdated, this));
  _started = true;
}

bool AcousticROSCommsDevice::DoStarted() { return _started; }

std::string AcousticROSCommsDevice::DoToString() {
  int maxBuffSize = 1024;
  char buff[maxBuffSize];
  string txChannelLinked;
  if (_txChannel)
    txChannelLinked = "Type: " + ChannelType2String(_txChannel->GetType()) +
                      " ; Id: " + to_string(_txChannel->GetId());
  else
    txChannelLinked = "not linked";

  dccomms::Ptr<AcousticCommsChannel> acousticChannel =
      std::static_pointer_cast<AcousticCommsChannel>(_txChannel);
  int n;
  if (_txChannel) {
    n = snprintf(buff, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                    "\tMAC ....................... %d\n"
                                    "\tDevice type ............... %s\n"
                                    "\tFrame ID: ................. '%s'\n"
                                    "\tChannel: .................. '%s'\n"
                                    "\t  Bandwidth: .............. %.03f Hz\n"
                                    "\t  Temperature: ............ %.01f ºC\n"
                                    "\t  Salinity: ............... %.02f ppt\n"
                                    "\t  Noise Level: ............ %.01f dB\n"
                                    "\tTx Fifo Size: ............. %d bytes\n"
                                    "\tMAC protocol: ............. %s\n"
                                    "\tMax. Range: ............... %.02f m\n"
                                    "\tPT: ....................... %.03f W\n"
                                    "\tFreq: ..................... %.03f KHz\n"
                                    "\tL: ........................ %.03f\n"
                                    "\tK: ........................ %.03f\n"
                                    "\tTurnOnEnergy: ............. %.03f J\n"
                                    "\tTurnOffEnergy: ............ %.03f J\n"
                                    "\tPreamble: ................. %.03f\n"
                                    "\tPTConsume: ................ %.03f W\n"
                                    "\tPRConsume: ................ %.03f W\n"
                                    "\tPIdle: .................... %.03f W\n",
                 _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
                 _tfFrameId.c_str(), txChannelLinked.c_str(),
                 acousticChannel->GetBandwidth(),
                 acousticChannel->GetTemperature(),
                 acousticChannel->GetSalinity(),
                 acousticChannel->GetNoiseLevel(), GetMaxTxFifoSize(),
                 _macP.c_str(), _range, _pT, _freq, _L, _K, _turnOnEnergy,
                 _turnOffEnergy, _preamble, _pTConsume, _pRConsume, _pIdle);
  } else {
    n = snprintf(buff, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                    "\tMAC ....................... %d\n"
                                    "\tDevice type ............... %s\n"
                                    "\tFrame ID: ................. '%s'\n"
                                    "\tChannel: .................. '%s'\n"
                                    "\tMAC protocol: ............. %s\n"
                                    "\tMax. Range: ............... %.02f m\n"
                                    "\tPT: ....................... %.03f W\n"
                                    "\tFreq: ..................... %.03f KHz\n"
                                    "\tL: ........................ %.03f\n"
                                    "\tK: ........................ %.03f\n"
                                    "\tTurnOnEnergy: ............. %.03f J\n"
                                    "\tTurnOffEnergy: ............ %.03f J\n"
                                    "\tPreamble: ................. %.03f\n"
                                    "\tPTConsume: ................ %.03f W\n"
                                    "\tPRConsume: ................ %.03f W\n"
                                    "\tPIdle: .................... %.03f W\n",
                 _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
                 _tfFrameId.c_str(), txChannelLinked.c_str(), _macP.c_str(),
                 _range, _pT, _freq, _L, _K, _turnOnEnergy, _turnOffEnergy,
                 _preamble, _pTConsume, _pRConsume, _pIdle);
  }

  return std::string(buff);
}
}
