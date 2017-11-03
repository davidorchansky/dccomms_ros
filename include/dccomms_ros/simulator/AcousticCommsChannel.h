#ifndef DCCOMMS_ROS_ACOUSTIC_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_ACOUSTIC_COMMS_CHANNEL_H_

#include <condition_variable>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros_msgs/types.h>
#include <list>
#include <memory>
#include <ns3/aqua-sim-channel.h>
#include <ns3/internet-module.h>
#include <random>

using namespace std;
using namespace ns3;
namespace dccomms_ros {

class AcousticCommsChannel : public CommsChannel {
  typedef ns3::Ptr<ns3::AquaSimChannel> AquaSimChannelPtr;
  typedef std::unordered_map<int, AquaSimChannelPtr> AquaSimChannelSet;

public:
  AcousticCommsChannel(uint32_t id);
  uint32_t GetId() { return _rosChannelId; }
  CHANNEL_TYPE GetType() { return ACOUSTIC_UNDERWATER_CHANNEL; }
  uint32_t GetPropDelay(ns3::Ptr<AquaSimNetDevice> tx,
                        ns3::Ptr<AquaSimNetDevice> rx) {
    return _aquaSimChannel->GetPropDelay(tx, rx);
  }

private:
  int _rosChannelId;
  int _ns3ChannelId;
  AquaSimChannelPtr _aquaSimChannel;
  static AquaSimChannelSet _aquaSimChannels;
  InternetStackHelper _internet;
};
}

#endif // COMMSCHANNELPROPERTIES_H
