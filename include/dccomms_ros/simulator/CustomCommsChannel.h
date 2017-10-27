#ifndef COMMSCHANNELPROPERTIES_H
#define COMMSCHANNELPROPERTIES_H

#include <condition_variable>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <memory>
#include <random>

using namespace std;

namespace dccomms_ros {

class CustomCommsChannel;
typedef std::shared_ptr<CustomCommsChannel> CustomCommsChannelPtr;

class CustomCommsChannel {
  typedef std::normal_distribution<double> NormalDist;
  typedef std::default_random_engine RandEngGen;

  typedef std::uniform_real_distribution<double> UniformRealDist;

public:
  static CustomCommsChannelPtr BuildCommsChannelState();

  CustomCommsChannel();
  CustomCommsChannel(int delay);

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

  void SetTxNode(ROSCommsDevicePtr dev);
  void SetRxNode(ROSCommsDevicePtr dev);

  ROSCommsDevicePtr GetTxNode();
  ROSCommsDevicePtr GetRxNode();

  // void ChannelFree(bool channelFree);
  // bool ChannelFree();

  // bool WaitForChannelFree();

  void Lock();
  void Unlock();

private:
  int _delay; // ms
  bool _linkOk;
  bool _channelFree;
  NormalDist _ttDist;
  UniformRealDist _erDist;
  RandEngGen _ttGenerator, _erGenerator;

  ROSCommsDevicePtr _txDev, _rxDev;
  double _errRate;

  std::mutex _errRateMutex, _delayMutex, _channelMutex;
  // condition_variable _channelFreeCond;
};
}

#endif // COMMSCHANNELPROPERTIES_H
