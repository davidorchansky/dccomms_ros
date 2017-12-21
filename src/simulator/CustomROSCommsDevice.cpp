#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>

namespace dccomms_ros {

CustomROSCommsDevice::CustomROSCommsDevice(ROSCommsSimulatorPtr sim,
                                           PacketBuilderPtr pb)
    : ROSCommsDevice(sim, pb), _ownPtr(this), _erDist(0.0, 1.0) {}

DEV_TYPE CustomROSCommsDevice::GetDevType() { return DEV_TYPE::CUSTOM_DEV; }

void CustomROSCommsDevice::SetVariableBitRate(double mean, double sd) {
  _bitRateMean = mean;
  _bitRateSd = sd;
  auto _ttMean = 1 / (_bitRateMean / 8) * 1000;
  auto _ttSd = _bitRateSd > 0 ? 1 / (_bitRateSd / 8) * 1000 : 0;
  _ttDist = NormalDist(_ttMean, _ttSd);
}

void CustomROSCommsDevice::GetBitRate(double &mean, double &sd) {
  mean = _bitRateMean;
  sd = _bitRateSd;
}

void CustomROSCommsDevice::SetMinPktErrorRate(double minPktErrorRate) {
  _minPktErrorRate = minPktErrorRate;
}

double CustomROSCommsDevice::GetMinPktErrorRate() { return _minPktErrorRate; }

void CustomROSCommsDevice::SetPktErrorRateInc(double inc) {
  _pktErrorRateIncPerMeter = inc;
}

double CustomROSCommsDevice::GetPktErrorRateInc() {
  return _pktErrorRateIncPerMeter;
}

double CustomROSCommsDevice::GetNextTt() {
  auto tt = _ttDist(_ttGenerator);
  if (tt < 0) {
    Log->warn("trRate < 0: {} . Changing to its abs({}) value", tt, tt);
    tt = -tt;
  }
  return tt;
}

bool CustomROSCommsDevice::ErrOnNextPkt(double errRate) {
  auto rand = _erDist(_erGenerator);
  return rand < errRate;
}

void CustomROSCommsDevice::SetMaxDistance(uint32_t d) { _maxDistance = d; }

uint32_t CustomROSCommsDevice::GetMaxDistance() { return _maxDistance; }

void CustomROSCommsDevice::SetMinDistance(uint32_t d) { _minDistance = d; }

uint32_t CustomROSCommsDevice::GetMinDistance() { return _minDistance; }

void CustomROSCommsDevice::DoSetMac(uint32_t mac) { _mac = mac; }
void CustomROSCommsDevice::DoSend(PacketPtr dlf) {
  static_pointer_cast<CustomCommsChannel>(_channel)->SendPacket(_ownPtr, dlf);
}
void CustomROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel) {
  if (channel->GetType() == CHANNEL_TYPE::CUSTOM_CHANNEL) {
    //_channel = static_pointer_cast<CustomCommsChannel>(channel);
    _channel = channel;
    static_pointer_cast<CustomCommsChannel>(_channel)->AddDevice(_ownPtr);
  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}
void CustomROSCommsDevice::DoStart() {}
bool CustomROSCommsDevice::DoStarted() { return true; }
void CustomROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  _position = position;
}
}
