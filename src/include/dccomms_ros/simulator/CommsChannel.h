#ifndef DCCOMMS_ROS_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_COMMS_CHANNEL_H_

#include <dccomms/dccomms.h>
#include <dccomms_ros_msgs/types.h>

namespace dccomms_ros {

class CommsChannel {
public:
  virtual uint32_t GetId() = 0;
  virtual CHANNEL_TYPE GetType() = 0;
};

typedef std::shared_ptr<CommsChannel> CommsChannelPtr;
}
#endif // COMMSCHANNELPROPERTIES_H
