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
        ): _maxBitRate(maxBitRate), _delay(delay){}

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
    _delay = delay;
}

int CommsChannelState::GetDelay()
{
    return _delay;
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

CommsChannelState::ttDist CommsChannelState::GetTtDist()
{
    return _ttDist;
}

void CommsChannelState::SetTtDist(double mean, double sd)
{
    _ttDist = ttDist(mean, sd);
}
}
