#include <dccomms_ros/simulator/CustomCommsChannel.h>

namespace dccomms_ros {
CustomCommsChannelPtr CustomCommsChannel::BuildCommsChannelState() {
  return CustomCommsChannelPtr(new CustomCommsChannel());
}

CustomCommsChannel::CustomCommsChannel() {}

CustomCommsChannel::CustomCommsChannel(int delay)
    : _delay(delay), _erDist(0.0, 1.0) {}

int CustomCommsChannel::SetDelay(int delay) {
  _delayMutex.lock();
  _delay = delay;
  _delayMutex.unlock();
}

int CustomCommsChannel::GetDelay() {
  int res;

  _delayMutex.lock();
  res = _delay;
  _delayMutex.unlock();

  return res;
}

void CustomCommsChannel::SetLinkOk(bool ok) { _linkOk = ok; }

bool CustomCommsChannel::LinkOk() { return _linkOk; }

void CustomCommsChannel::Lock() { _channelMutex.lock(); }

void CustomCommsChannel::Unlock() { _channelMutex.unlock(); }

/*
void CommsChannelState::ChannelFree(bool channelFree)
{
    _channelFree = channelFree;
    if(channelFree) _channelFreeCond.notify_one ();
}

bool CommsChannelState::ChannelFree()
{
    return _channelFree;
}

bool CommsChannelState::WaitForChannelFree()
{
    std::unique_lock<std::mutex> lock(_channelMutex);
    while(_channelFree)
    {
            _channelFreeCond.wait(lock);
    }
    return true;
}
*/

double CustomCommsChannel::GetNextTt() { return _ttDist(_ttGenerator); }

bool CustomCommsChannel::ErrOnNextPkt() {
  auto rand = _erDist(_erGenerator);
  return rand < _errRate;
}

CustomCommsChannel::NormalDist CustomCommsChannel::GetTtDist() {
  return _ttDist;
}

void CustomCommsChannel::SetTtDist(double mean, double sd) {
  _ttDist = NormalDist(mean, sd);
}

void CustomCommsChannel::SetTxNode(ROSCommsDevicePtr node) { _txDev = node; }

void CustomCommsChannel::SetErrRate(double rate) {
  _errRateMutex.lock();
  _errRate = rate;
  _errRateMutex.unlock();
}

double CustomCommsChannel::GetErrRate() {
  float res;
  _errRateMutex.lock();
  res = _errRate;
  _errRateMutex.unlock();

  return res;
}

ROSCommsDevicePtr CustomCommsChannel::GetTxNode() { return _txDev; }

void CustomCommsChannel::SetRxNode(ROSCommsDevicePtr node) { _rxDev = node; }
ROSCommsDevicePtr CustomCommsChannel::GetRxNode() { return _rxDev; }
}
