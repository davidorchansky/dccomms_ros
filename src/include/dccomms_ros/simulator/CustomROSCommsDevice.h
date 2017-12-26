#ifndef DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_

#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <random>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

class CustomROSCommsDevice;
typedef std::shared_ptr<CustomROSCommsDevice> CustomROSCommsDevicePtr;

class CustomROSCommsDevice : public ROSCommsDevice {
public:
  CustomROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr txpb, PacketBuilderPtr rxpb);

  void SetVariableBitRate(double trTimeMean, double trTimeSd = 0); //as bps
  void SetMinPktErrorRate(double minPktErrorRate);
  void SetPktErrorRateInc(double pktErrorRateInc);
  void SetMaxDistance(uint32_t d);
  void SetMinDistance(uint32_t d);

  void GetBitRate(double &trTimeMean, double &trTimeSd); //as bps
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  uint32_t GetMaxDistance();
  uint32_t GetMinDistance();

  bool ErrOnNextPkt(double errRate);
  double GetNextTt(); // get next ms/byte

  virtual DEV_TYPE GetDevType();

  typedef std::normal_distribution<double> NormalDist;
  typedef std::default_random_engine RandEngGen;
  typedef std::uniform_real_distribution<double> UniformRealDist;

protected:
  virtual void DoSetMac(uint32_t mac);
  virtual void DoSend(PacketPtr dlf);
  virtual void DoLinkToChannel(CommsChannelPtr channel);
  virtual void DoStart();
  virtual void DoSetPosition(const tf::Vector3 &position);
  virtual bool DoStarted();

private:
  uint32_t _mac;
  double _bitRateMean, _bitRateSd;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;

  uint32_t _maxDistance, _minDistance;
  CommsChannelPtr _channel;
  tf::Vector3 _position;
  CustomROSCommsDevicePtr _ownPtr;

  NormalDist _ttDist;
  UniformRealDist _erDist;
  RandEngGen _ttGenerator, _erGenerator;
};
}
#endif // COMMSNODE_H
