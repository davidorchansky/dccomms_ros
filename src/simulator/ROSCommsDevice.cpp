#include <cstdio>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/types.h>

namespace dccomms_ros {

ROSCommsDevice::ROSCommsDevice(ROSCommsSimulatorPtr s, PacketBuilderPtr pb)
    : _sim(s), _pb(pb), _txserv(this) {
  _device = CommsDeviceService::BuildCommsDeviceService(
      pb, CommsDeviceService::IPHY_TYPE_PHY);
  _txserv.SetWork(&ROSCommsDevice::_TxWork);
  _txdlf = _sim->GetPacketBuilder()->Create();
  _commonStarted = false;
  _position = tf::Vector3(0,0,0);
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
  _receiveFrameMutex.lock();
  _device << dlf;
  _receiveFrameMutex.unlock();
}

void ROSCommsDevice::SetMaxBitRate(uint32_t v) {
  _maxBitRate = v; // bps
  _millisPerByte = static_cast<uint32_t>(std::round(1000. / _maxBitRate * 8));
}

void ROSCommsDevice::SetDccommsId(const std::string name) {
  _name = name;
  _device->SetCommsDeviceId(_name);
}

std::string ROSCommsDevice::GetDccommsId() { return _name; }

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
  do {
    _device >> _txdlf;
    PacketPtr txdlf =
        _sim->GetPacketBuilder()->CreateFromBuffer(_txdlf->GetBuffer());
    if (txdlf->PacketIsOk()) {
      // PACKET OK
      DoSend(txdlf);
      uint32_t packetSize = static_cast<uint32_t>(txdlf->GetPacketSize());
      uint32_t transmissionTime = packetSize * _millisPerByte;
      std::this_thread::sleep_for(std::chrono::milliseconds(transmissionTime));
    } else {
      // PACKET WITH ERRORS
    }
  } while (_device->GetRxFifoSize() > 0);

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

  int n = snprintf(buff, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                      "\tMAC ....................... %d\n"
                                      "\tDevice type ............... %s\n"
                                      "\tFrame ID: ................. '%s'\n"
                                      "\tChannel: .................. '%s'",
                   _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
                   _tfFrameId.c_str(), channelLinked.c_str());
  return std::string(buff);
}
}
