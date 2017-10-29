#include <dccomms_ros/simulator/AcousticCommsChannel.h>

#include <ns3/aqua-sim-channel.h>
#include <ns3/channel-list.h>

namespace dccomms_ros {

AcousticCommsChannel::AcousticCommsChannel(uint32_t id) {
  _rosChannelId = id;
  _ns3ChannelId = ns3::ChannelList::GetNChannels();
  _aquaSimChannel = ns3::CreateObject<AquaSimChannel>();
}
}
