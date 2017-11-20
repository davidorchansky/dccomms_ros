#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>

namespace dccomms_ros {

CustomROSCommsDevice::CustomROSCommsDevice(ROSCommsSimulatorPtr sim,
                                           PacketBuilderPtr pb)
    : ROSCommsDevice(sim, pb), _ownPtr(this) {}

DEV_TYPE CustomROSCommsDevice::GetDevType() { return DEV_TYPE::CUSTOM_DEV; }

void CustomROSCommsDevice::SetTrTime(float mean, float sd) {
  _trTimeMean = mean;
  _trTimeSd = sd;
}

void CustomROSCommsDevice::GetTrTime(float &mean, float &sd) {
  mean = _trTimeMean;
  sd = _trTimeSd;
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
  auto trRate = _ttDist(_ttGenerator);
  if (trRate < 0) {
    Log->warn("trRate < 0: {} . Changing to its abs({}) value", trRate, trRate);
    trRate = -trRate;
  }
  return trRate;
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
void CustomROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  _position = position;
}
}
