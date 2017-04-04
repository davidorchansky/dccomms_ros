#ifndef COMMSCHANNELPROPERTIES_H
#define COMMSCHANNELPROPERTIES_H

#include <memory>
#include <random>
#include <dccomms_ros/ROSCommsDevice.h>

using namespace std;

namespace dccomms_ros
{

class CommsChannelState;
typedef std::shared_ptr<CommsChannelState> CommsChannelStatePtr;

class CommsChannelState
{
    typedef std::normal_distribution<double> NormalDist;
    typedef std::default_random_engine RandEngGen;

    typedef std::uniform_real_distribution<double> erDist;

public:

    static CommsChannelStatePtr BuildCommsChannelState();

    CommsChannelState();
    CommsChannelState(
            int maxBitRate,
            int delay
            );

    void SetMaxBitRate(int maxBitRate);
    int GetMaxBitRate();
    int SetDelay(int delay);
    int GetDelay();
    void SetLinkOk(bool ok);
    bool LinkOk();
    double GetNextTt();
    void SetErrRate(double);
    double GetErrRate();
    NormalDist GetTtDist();
    void SetTtDist(double mean, double sd);
    bool ErrOnNextPkt();

    void SetTxNode(CommsDevicePtr dev);
    void SetRxNode(CommsDevicePtr dev);

    CommsDevicePtr GetTxNode();
    CommsDevicePtr GetRxNode();

private:
    int _maxBitRate;
    int _delay; //ms
    bool _linkOk;
    NormalDist _ttDist;
    erDist _erDist;
    RandEngGen _ttGenerator,_erGenerator;

    CommsDevicePtr _txDev, _rxDev;
    double _errRate;

    std::mutex _errRateMutex, _delayMutex;
};
}

#endif // COMMSCHANNELPROPERTIES_H
