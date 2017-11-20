#ifndef DCCOMMS_ROS_CUSTOM_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_CUSTOM_COMMS_CHANNEL_H_

#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros_msgs/types.h>
#include <list>

using namespace std;
namespace dccomms_ros {


class CustomCommsChannel : public CommsChannel, public virtual cpplogging::Logger {
public:
  CustomCommsChannel(uint32_t id);
  void SetMinPrTime(double prTime);
  void SetPrTimeInc(double inc);
  void AddDevice(CustomROSCommsDevicePtr dev);
  void SendPacket(CustomROSCommsDevicePtr dev, dccomms::PacketPtr pkt);
  uint32_t GetId() { return _rosChannelId; }
  CHANNEL_TYPE GetType() { return CUSTOM_CHANNEL; }

private:
  uint32_t _rosChannelId;
  double _prTimeIncPerMeter, _minPrTime;
  std::list<CustomROSCommsDevicePtr> _devices;

};

typedef dccomms::Ptr<CustomCommsChannel> CustomCommsChannelPtr;
}

#endif // COMMSCHANNELPROPERTIES_H
