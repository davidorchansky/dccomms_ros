#include <cstdio>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/types.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ROSCommsDevice"); //NS3 DOES NOT WORK (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(ROSCommsDevice);

ns3::TypeId ROSCommsDevice::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("ns3::ROSCommsDevice")
          .SetParent<Object>()
          .AddTraceSource(
              "PacketReceived", "Trace source indicating a packet has been "
                                "delivered to the upper layer.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_rxCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketReceivedCallback")
          .AddTraceSource(
              "PacketTransmitting", "Trace source indicating a packet has been "
                                    "delivered to the lower layer.",
              ns3::MakeTraceSourceAccessor(&ROSCommsDevice::_txCbTrace),
              "dccomms_ros::ROSCommsDevice::PacketTransmittingCallback");
  return tid;
}

ROSCommsDevice::ROSCommsDevice(ROSCommsSimulatorPtr s, PacketBuilderPtr txpb,
                               PacketBuilderPtr rxpb)
    : _sim(s), _txserv(this) {
  _rxpb = rxpb;
  _txpb = txpb;
  _device = CommsDeviceService::BuildCommsDeviceService(
      _txpb, CommsDeviceService::IPHY_TYPE_PHY);
  _device->SetLogLevel(cpplogging::LogLevel::info);
  SetLogLevel(cpplogging::info);
  _txserv.SetWork(&ROSCommsDevice::_TxWork);
  _commonStarted = false;
  _position = tf::Vector3(0, 0, 0);
  LogComponentEnable("ROSCommsDevice", LOG_LEVEL_ALL); //NS3 DOES NOT WORK (TODO: FIX IT)
  SetLogLevel(debug);
}

ROSCommsDevice::~ROSCommsDevice() {}

void ROSCommsDevice::_StartDeviceService() {
  auto startWorker = std::thread([this]() {
    _device->Start();
    _commonStarted = true;
    _device->SetPhyLayerState(CommsDeviceService::READY);
  });
  startWorker.detach();

  auto waitRemainingNodesToGetReadyWorker = std::thread([this]() {
    while (!_sim->Ready(this->GetDevType()))
      std::this_thread::sleep_for(chrono::milliseconds(100));
    _StartNodeWorker();
  });

  waitRemainingNodesToGetReadyWorker.detach();
}

void ROSCommsDevice::Start() {
  _ownPtr = this; // shared_from_this();
  _StartDeviceService();
  DoStart();
}

bool ROSCommsDevice::Started() { return _commonStarted && DoStarted(); }

void ROSCommsDevice::_StartNodeWorker() { _txserv.Start(); }

void ROSCommsDevice::ReceiveFrame(ns3PacketPtr packet) {
  Debug("ROSCommsDevice: Frame received");
  char ser[5000];
  _rxCbTrace(_ownPtr, packet);
  NetsimHeader header;
  packet->RemoveHeader(header);
  auto size = packet->GetSize();
  packet->CopyData((uint8_t *)ser, size);
  auto dccommsPacket = _rxpb->CreateFromBuffer(ser);
  _receiveFrameMutex.lock();
  _device << dccommsPacket;
  _receiveFrameMutex.unlock();
}

void ROSCommsDevice::SetBitRate(uint32_t v) {
  _bitRate = v; // bps
  _nanosPerByte = static_cast<uint64_t>(std::round(8 * 1e9 / _bitRate));
}

void ROSCommsDevice::SetDccommsId(const std::string name) {
  _name = name;
  _txdlf = _txpb->Create();
  _device->SetCommsDeviceId(_name);
}

std::string ROSCommsDevice::GetDccommsId() { return _name; }

void ROSCommsDevice::SetMaxTxFifoSize(uint32_t size) {
  _device->SetMaxQueueSize(size);
}

uint32_t ROSCommsDevice::GetMaxTxFifoSize() {
  return _device->GetMaxQueueSize();
}

void ROSCommsDevice::SetMac(uint32_t mac) {
  _mac = mac;
  DoSetMac(_mac);
}

void ROSCommsDevice::LinkToChannel(CommsChannelPtr channel,
                                   CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::ACOUSTIC_UNDERWATER_CHANNEL) {
    linkType = CHANNEL_LINK_TYPE::CHANNEL_TXRX;
  }
  switch (linkType) {
  case CHANNEL_LINK_TYPE::CHANNEL_TXRX: {
    _rxChannel = channel;
    _txChannel = channel;
    break;
  }
  case CHANNEL_LINK_TYPE::CHANNEL_TX: {
    _txChannel = channel;
    break;
  }
  case CHANNEL_LINK_TYPE::CHANNEL_RX: {
    _rxChannel = channel;
    break;
  }
  }
  DoLinkToChannel(channel, linkType);
}

CommsChannelPtr ROSCommsDevice::GetLinkedTxChannel() { return _txChannel; }

uint32_t ROSCommsDevice::GetMac() { return _mac; }

void ROSCommsDevice::_TxWork() {
  NS_LOG_FUNCTION(this);
  _device->WaitForFramesFromRxFifo();
  _device->SetPhyLayerState(CommsDeviceService::BUSY);
  unsigned int txFifoSize;
  do {
    _device >> _txdlf;
    txFifoSize = _device->GetRxFifoSize();
    //    Log->info("(dccommsId: {}) received packet from the upper layer. tx
    //    fifo "
    //              "size: {} bytes",
    //              GetDccommsId(), txFifoSize);
    PacketPtr txdlf = _txpb->CreateFromBuffer(_txdlf->GetBuffer());
    if (txdlf->PacketIsOk()) {
      // PACKET OK
      auto header = NetsimHeader::Build(txdlf);
      auto pkt =
          ns3::Create<ns3::Packet>(txdlf->GetBuffer(), txdlf->GetPacketSize());
      pkt->AddHeader(header);
      _txCbTrace(_ownPtr, pkt);
      NS_LOG_DEBUG("Send packet");
      Debug("ROSCommsDevice: Send frame");
      DoSend(pkt);
      uint32_t packetSize = static_cast<uint32_t>(txdlf->GetPacketSize());
      uint64_t transmissionTime = packetSize * _nanosPerByte;
      std::this_thread::sleep_for(std::chrono::nanoseconds(transmissionTime));
    } else {
      Log->critical("packet received with errors from the upper layer!");
    }
  } while (txFifoSize > 0);

  _device->SetPhyLayerState(CommsDeviceService::READY);
}

void ROSCommsDevice::SetLogName(std::string name) {
  Loggable::SetLogName(name);
  _device->SetLogName(name + ":CommsDeviceService");
}

void ROSCommsDevice::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
 // _device->SetLogLevel(_level);
}

void ROSCommsDevice::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  //_device->LogToConsole(c);
}

void ROSCommsDevice::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  _device->LogToFile(filename);
}

void ROSCommsDevice::FlushLog() {
  Loggable::FlushLog();
  _device->FlushLog();
}

void ROSCommsDevice::FlushLogOn(cpplogging::LogLevel level) {
  Loggable::FlushLogOn(level);
  _device->FlushLogOn(level);
}

void ROSCommsDevice::SetTfFrameId(const string &id) { _tfFrameId = id; }

void ROSCommsDevice::SetPosition(const tf::Vector3 &position) {
  _position = position;
  DoSetPosition(position);
}

tf::Vector3 ROSCommsDevice::GetPosition() { return _position; }

std::string ROSCommsDevice::GetTfFrameId() { return _tfFrameId; }

std::string ROSCommsDevice::ToString() { return DoToString(); }
}
