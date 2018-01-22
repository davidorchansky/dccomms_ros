#include <cstdio>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/types.h>

namespace dccomms_ros {

ROSCommsDevice::ROSCommsDevice(ROSCommsSimulatorPtr s, PacketBuilderPtr txpb,
                               PacketBuilderPtr rxpb)
    : _sim(s), _txserv(this) {
  _rxpb = rxpb;
  _txpb = txpb;
  _device = CommsDeviceService::BuildCommsDeviceService(
      _txpb, CommsDeviceService::IPHY_TYPE_PHY);
  _device->SetLogLevel(LogLevel::info);
  _txserv.SetWork(&ROSCommsDevice::_TxWork);
  _commonStarted = false;
  _position = tf::Vector3(0, 0, 0);
}

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
  _StartDeviceService();
  DoStart();
}

bool ROSCommsDevice::Started() { return _commonStarted && DoStarted(); }

void ROSCommsDevice::_StartNodeWorker() { _txserv.Start(); }

void ROSCommsDevice::ReceiveFrame(PacketPtr dlf) {
  _sim->ReceivePDUCb(this, dlf);
  _receiveFrameMutex.lock();
  _device << dlf;
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

void ROSCommsDevice::LinkToChannel(CommsChannelPtr channel) {
  _channel = channel;
  DoLinkToChannel(channel);
}

CommsChannelPtr ROSCommsDevice::GetLinkedChannel() { return _channel; }

uint32_t ROSCommsDevice::GetMac() { return _mac; }

void ROSCommsDevice::_TxWork() {
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
      _sim->TransmitPDUCb(this, txdlf);
      DoSend(txdlf);
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
  _device->SetLogLevel(_level);
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

std::string ROSCommsDevice::ToString() {
  int maxBuffSize = 1024;
  char buff[maxBuffSize];
  string channelLinked;
  if (_channel)
    channelLinked = "Type: " + ChannelType2String(_channel->GetType()) +
                    " ; Id: " + to_string(_channel->GetId());
  else
    channelLinked = "not linked";

  int n =
      snprintf(buff, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                  "\tMAC ....................... %d\n"
                                  "\tDevice type ............... %s\n"
                                  "\tFrame ID: ................. '%s'\n"
                                  "\tChannel: .................. '%s'\n"
                                  "\tTx Fifo Size: ............. %d bytes",
               _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
               _tfFrameId.c_str(), channelLinked.c_str(), GetMaxTxFifoSize());
  return std::string(buff);
}
}
