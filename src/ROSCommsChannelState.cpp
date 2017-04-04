#include <dccomms_ros/ROSCommsChannelState.h>

namespace dccomms_ros
{
CommsChannelStatePtr CommsChannelState::BuildCommsChannelState()
{
    return CommsChannelStatePtr(new CommsChannelState());
}

CommsChannelState::CommsChannelState()
{}

CommsChannelState::CommsChannelState(
        int maxBitRate,
        int delay
        ): _maxBitRate(maxBitRate), _delay(delay), _erDist(0.0,1.0)
{

}

void CommsChannelState::SetMaxBitRate(int maxBitRate)
{
    _maxBitRate = maxBitRate;
}

int CommsChannelState::GetMaxBitRate()
{
    return _maxBitRate;
}

int CommsChannelState::SetDelay(int delay)
{
    _delayMutex.lock();
    _delay = delay;
    _delayMutex.unlock();
}

int CommsChannelState::GetDelay()
{
  int res;

  _delayMutex.lock();
  res = _delay;
  _delayMutex.unlock();

  return res;

}

void CommsChannelState::SetLinkOk(bool ok)
{
    _linkOk = ok;
}

bool CommsChannelState::LinkOk()
{
    return _linkOk;
}

double CommsChannelState::GetNextTt()
{
    return _ttDist(_ttGenerator);
}

bool CommsChannelState::ErrOnNextPkt ()
{
    auto rand =  _erDist(_erGenerator);
    return rand < _errRate;
}

CommsChannelState::ttDist CommsChannelState::GetTtDist()
{
    return _ttDist;
}

void CommsChannelState::SetTtDist(double mean, double sd)
{
    _ttDist = ttDist(mean, sd);
}

void CommsChannelState::SetTxNode(CommsDevicePtr node)
{
    _txDev = node;
}

void CommsChannelState::SetErrRate(double rate)
{
    _errRateMutex.lock();
    _errRate = rate;
    _errRateMutex.unlock();
}

double CommsChannelState::GetErrRate()
{
  float res;
  _errRateMutex.lock();
  res = _errRate;
  _errRateMutex.unlock();

  return res;
}

CommsDevicePtr CommsChannelState::GetTxNode()
{
    return _txDev;
}

void CommsChannelState::SetRxNode(CommsDevicePtr node)
{
    _rxDev = node;
}
CommsDevicePtr CommsChannelState::GetRxNode()
{
    return _rxDev;
}
}
