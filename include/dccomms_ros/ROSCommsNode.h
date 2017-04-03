#ifndef COMMSNODE_H
#define COMMSNODE_H

#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <Loggable.h>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros
{

class ROSCommsDevice;
typedef boost::shared_ptr<ROSCommsDevice> CommsNodePtr;

class ROSCommsSimulator;
typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsDevice : public virtual Loggable
{
public:
    static CommsNodePtr BuildCommsNode(
            ROSCommsSimulator * sim, CommsDeviceServicePtr dev
            )
    {
            return CommsNodePtr(new ROSCommsDevice(
                                    sim, dev
                                     ));
    }
    ROSCommsDevice(ROSCommsSimulator *, CommsDeviceServicePtr);

    CommsDeviceServicePtr GetService();
    void ReceiveFrame(DataLinkFramePtr);
    void StartDeviceService();
    void StartNodeWorker();
    std::string GetName();
    void SetName(const std::string name);

    virtual void SetLogName(std::string name);
    virtual void SetLogLevel(Loggable::LogLevel);
    virtual void FlushLog();
    virtual void FlushLogOn(LogLevel);
    virtual void LogToConsole(bool);
    virtual void LogToFile(const string &filename);

    void SetMaxBitRate(int maxBitRate);
    void SetTrTime(float trTimeMean, float trTimeSd = 0);
    void SetMinPrTime(float prTime);
    void SetPrTimeInc(float inc);
    void SetMinPktErrorRate(float minPktErrorRate);
    void SetPktErrorRateInc(float pktErrorRateInc);
    void SetMac(int mac);

    int GetMaxBitRate();
    void GetTrTime(float & trTimeMean, float & trTimeSd);
    float GetMinPrTime();
    float GetPrTimeInc();
    float GetMinPktErrorRate();
    float GetPktErrorRateInc();
    int GetMac();

private:
    std::mutex _receiveFrameMutex;
    CommsDeviceServicePtr _device;
    ROSCommsSimulatorPtr _sim;
    ServiceThread<ROSCommsDevice> _txserv;
    DataLinkFramePtr _txdlf;
    std::string _name;
    int _mac;
    int _maxBitRate;
    float _trTimeMean, _trTimeSd,
      _minPrTime, _prTimeIncPerMeter,
      _minPktErrorRate,
      _pktErrorRateIncPerMeter;

    void _TxWork();
};


}
#endif // COMMSNODE_H
