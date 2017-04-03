#ifndef COMMSCHANNELPROPERTIES_H
#define COMMSCHANNELPROPERTIES_H

#include <memory>
#include <random>
#include <dccomms_ros/ROSCommsNode.h>

using namespace std;

namespace dccomms_ros
{

class CommsChannelState;
typedef std::shared_ptr<CommsChannelState> CommsChannelStatePtr;

class CommsChannelState
{
    typedef std::normal_distribution<double> ttDist;
    typedef std::default_random_engine ttGenerator;
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
    ttDist GetTtDist();
    void SetTtDist(double mean, double sd);

    void SetTxNode(CommsNodePtr dev);
    void SetRxNode(CommsNodePtr dev);

    CommsNodePtr GetTxNode();
    CommsNodePtr GetRxNode();

private:
    int _maxBitRate;
    int _delay; //ms
    bool _linkOk;
    ttDist _ttDist;
    ttGenerator _ttGenerator;
    CommsNodePtr _txDev, _rxDev;
};
}

#endif // COMMSCHANNELPROPERTIES_H
