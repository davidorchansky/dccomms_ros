#include <dccomms_ros/simulator/ROSCommsChannelState.h>

namespace dccomms_ros {
CommsChannelStatePtr CommsChannelState::BuildCommsChannelState() {
  return CommsChannelStatePtr(new CommsChannelState());
}

CommsChannelState::CommsChannelState() {}

CommsChannelState::CommsChannelState(int delay)
    : _delay(delay), _erDist(0.0, 1.0) {}

int CommsChannelState::SetDelay(int delay) {
  _delayMutex.lock();
  _delay = delay;
  _delayMutex.unlock();
}

int CommsChannelState::GetDelay() {
  int res;

  _delayMutex.lock();
  res = _delay;
  _delayMutex.unlock();

  return res;
}

void CommsChannelState::SetLinkOk(bool ok) { _linkOk = ok; }

bool CommsChannelState::LinkOk() { return _linkOk; }

void CommsChannelState::Lock() { _channelMutex.lock(); }

void CommsChannelState::Unlock() { _channelMutex.unlock(); }

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

double CommsChannelState::GetNextTt() { return _ttDist(_ttGenerator); }

bool CommsChannelState::ErrOnNextPkt() {
  auto rand = _erDist(_erGenerator);
  return rand < _errRate;
}

CommsChannelState::NormalDist CommsChannelState::GetTtDist() { return _ttDist; }

void CommsChannelState::SetTtDist(double mean, double sd) {
  _ttDist = NormalDist(mean, sd);
}

void CommsChannelState::SetTxNode(ROSCommsDevicePtr node) { _txDev = node; }

void CommsChannelState::SetErrRate(double rate) {
  _errRateMutex.lock();
  _errRate = rate;
  _errRateMutex.unlock();
}

double CommsChannelState::GetErrRate() {
  float res;
  _errRateMutex.lock();
  res = _errRate;
  _errRateMutex.unlock();

  return res;
}

ROSCommsDevicePtr CommsChannelState::GetTxNode() { return _txDev; }

void CommsChannelState::SetRxNode(ROSCommsDevicePtr node) { _rxDev = node; }
ROSCommsDevicePtr CommsChannelState::GetRxNode() { return _rxDev; }
}
