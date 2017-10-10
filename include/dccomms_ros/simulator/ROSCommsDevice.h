#ifndef COMMSNODE_H
#define COMMSNODE_H

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

class ROSCommsDevice;
typedef std::shared_ptr<ROSCommsDevice> ROSCommsDevicePtr;

class ROSCommsSimulator;
typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsDevice : public virtual Loggable {
public:
  static ROSCommsDevicePtr BuildCommsDevice(ROSCommsSimulator *sim,
                                            CommsDeviceServicePtr dev) {
    return ROSCommsDevicePtr(new ROSCommsDevice(sim, dev));
  }
  ROSCommsDevice(ROSCommsSimulator *, CommsDeviceServicePtr);

  CommsDeviceServicePtr GetService();
  void ReceiveFrame(PacketPtr);
  void StartDeviceService();
  void StartNodeWorker();
  std::string GetName();
  void SetName(const std::string name);

  virtual void SetLogName(std::string name);
  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const std::string &filename);

  void SetTrTime(float trTimeMean, float trTimeSd = 0);
  void SetMinPrTime(float prTime);
  void SetPrTimeInc(float inc);
  void SetMinPktErrorRate(double minPktErrorRate);
  void SetPktErrorRateInc(double pktErrorRateInc);
  void SetMac(int mac);
  void SetDevType(int type);
  void SetTfFrameId(const std::string &);
  void SetMaxDistance(uint32_t d);
  void SetMinDistance(uint32_t d);

  void GetTrTime(float &trTimeMean, float &trTimeSd);
  float GetMinPrTime();
  float GetPrTimeInc();
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  int GetMac();
  int GetDevType();
  uint32_t GetMaxDistance();
  uint32_t GetMinDistance();
  std::string GetTfFrameId();

  std::string ToString();

private:
  std::mutex _receiveFrameMutex;
  CommsDeviceServicePtr _device;
  ROSCommsSimulatorPtr _sim;
  ServiceThread<ROSCommsDevice> _txserv;
  PacketPtr _txdlf;
  std::string _name, _tfFrameId;
  int _mac, _devType;
  float _trTimeMean, _trTimeSd, _minPrTime, _prTimeIncPerMeter;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;

  uint32_t _maxDistance, _minDistance;

  void _TxWork();
};
}
#endif // COMMSNODE_H
