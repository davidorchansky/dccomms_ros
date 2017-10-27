#ifndef COMMSCHANNELPROPERTIES_H
#define COMMSCHANNELPROPERTIES_H

#include <condition_variable>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <memory>
#include <random>
#include <list>

using namespace std;

namespace dccomms_ros {

class CommsChannel
{

};
typedef std::shared_ptr<CommsChannel> CommsChannelPtr;

class VirtualDeviceLink;
typedef std::shared_ptr<VirtualDeviceLink> VirtualDeviceLinkPtr;

class VirtualDeviceLink {

public:
  VirtualDeviceLink(ROSCommsDevicePtr dev0, ROSCommsDevicePtr dev1, CommsChannel);
  int GetDelay();
  bool LinkOk();
  double GetNextTt();
  double GetErrRate();
  bool ErrOnNextPkt();

  ROSCommsDevicePtr GetDevice0();
  ROSCommsDevicePtr GetDevice1();

};

typedef std::list<VirtualDeviceLinkPtr> VirtualDevicesLinks;

}

#endif // COMMSCHANNELPROPERTIES_H
