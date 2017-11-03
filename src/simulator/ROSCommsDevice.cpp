#include <cstdio>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/types.h>

namespace dccomms_ros {

ROSCommsDevice::ROSCommsDevice(ROSCommsSimulatorPtr s, PacketBuilderPtr pb,
                               DEV_TYPE devType)
    : _sim(s), _pb(pb), _txserv(this), _devType(devType) {
  _device = CommsDeviceService::BuildCommsDeviceService(
      pb, CommsDeviceService::IPHY_TYPE_PHY);

  _txserv.SetWork(&ROSCommsDevice::_TxWork);
  switch (_devType) {
  case ACOUSTIC_UNDERWATER_DEV: {
    break;
  }
  }
}

void ROSCommsDevice::StartDeviceService() {
  _device->Start();
  _device->SetPhyLayerState(CommsDeviceService::READY);
}

void ROSCommsDevice::StartNodeWorker() { _txserv.Start(); }

void ROSCommsDevice::ReceiveFrame(PacketPtr dlf) {
  _receiveFrameMutex.lock();
  *_device << dlf;
  _receiveFrameMutex.unlock();
}

void ROSCommsDevice::SetChannel(CommsChannelPtr channel) { _channel = channel; }
void ROSCommsDevice::SetMaxBitRate(uint32_t v) { _maxBitRate = v; }
uint32_t ROSCommsDevice::GetMaxBitRate() { return _maxBitRate; }

void ROSCommsDevice::SetDccommsId(const std::string name) { _name = name; }

std::string ROSCommsDevice::GetDccommsId() { return _name; }

void ROSCommsDevice::SetDevType(DEV_TYPE type) { _devType = type; }

DEV_TYPE ROSCommsDevice::GetDevType() { return _devType; }

void ROSCommsDevice::SetMac(int mac) { _mac = mac; }

int ROSCommsDevice::GetMac() { return _mac; }

void ROSCommsDevice::_TxWork() {
  _device->WaitForFramesFromRxFifo();
  _device->SetPhyLayerState(CommsDeviceService::BUSY);
  do {
    *_device >> _txdlf;
    PacketPtr txdlf =
        _sim->GetPacketBuilder()->CreateFromBuffer(_txdlf->GetBuffer());
    if (txdlf->PacketIsOk()) {
      // PACKET OK
      // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      _Transmit(txdlf);
    } else {
      // PACKET WITH ERRORS
      // Log->critical("TX: INTERNAL ERROR: frame received with errors from the
      // upper layer!");
    }
  } while (_device->GetRxFifoSize() > 0);

  _device->SetPhyLayerState(CommsDeviceService::READY);
}

void ROSCommsDevice::_Transmit(PacketPtr dlf) {}
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
                   _name.c_str(), _mac, DevType2String(_devType).c_str(),
                   _tfFrameId.c_str(), channelLinked.c_str());
  return std::string(buff);
}
}
