#ifndef DCCOMMS_ROS_ROSCOMMSSIMULATOR_H_
#define DCCOMMS_ROS_ROSCOMMSSIMULATOR_H_

#include <cpplogging/Loggable.h>
#include <dccomms/dccomms.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <dccomms_ros/simulator/VirtualDeviceLink.h>
#include <functional>
#include <memory>
#include <random>
#include <unordered_map>

// ROS
#include <dccomms_ros_msgs/AddChannel.h>
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

typedef std::unordered_map<int, ROSCommsDevicePtr> Mac2DevMap;
typedef std::shared_ptr<Mac2DevMap> Mac2DevMapPtr;
typedef std::unordered_map<int, Mac2DevMapPtr> Type2DevMapMap;

typedef std::unordered_map<std::string, ROSCommsDevicePtr> DccommsDevMap;

typedef std::unordered_map<int, CommsChannelPtr> Id2ChannelMap;
typedef std::shared_ptr<Id2ChannelMap> Id2ChannelMapPtr;
typedef std::unordered_map<int, Id2ChannelMapPtr> Type2ChannelMapMap;

class ROSCommsSimulator;

typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Loggable {
public:
  ROSCommsSimulator(ros::NodeHandle &rosnode, PacketBuilderPtr packetBuilder);
  void TransmitFrame(ROSCommsDevicePtr dev, PacketPtr dlf);
  void Start();

  PacketBuilderPtr GetPacketBuilder();

  void
  SetTransmitPDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);
  void
  SetReceivePDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);
  void SetErrorPDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);

  void SetGetSrcAddrFunc(std::function<int(dccomms::PacketPtr)> cb);
  void SetGetDstAddrFunc(std::function<int(dccomms::PacketPtr)> cb);

  void SetIsBroadcastFunc(std::function<bool(int)> cb);

private:
  void _Init();
  std::function<void(int, dccomms::PacketPtr)> _TransmitPDUCb, _ReceivePDUCb,
      _ErrorPDUCb;

  std::function<int(dccomms::PacketPtr)> _getSrcAddr, _getDstAddr;
  std::function<bool(int)> _isBroadcast;

  int _GetSrcAddr(PacketPtr pkt);
  int _GetDstAddr(PacketPtr pkt);

  bool _IsBroadcast(int addr);

  bool _AddDevice(dccomms_ros_msgs::AddDevice::Request &req,
                  dccomms_ros_msgs::AddDevice::Response &res);
  bool _CheckDevice(dccomms_ros_msgs::CheckDevice::Request &req,
                    dccomms_ros_msgs::CheckDevice::Response &res);
  bool _RemoveDevice(dccomms_ros_msgs::RemoveDevice::Request &req,
                     dccomms_ros_msgs::RemoveDevice::Response &res);
  bool _LinkDevToChannel(dccomms_ros_msgs::LinkDeviceToChannel::Request &req,
                         dccomms_ros_msgs::LinkDeviceToChannel::Response &res);
  bool _AddChannel(dccomms_ros_msgs::AddChannel::Request &req,
                   dccomms_ros_msgs::AddChannel::Response &res);
  void _PropagateFrame(PacketPtr dlf, int delay, ROSCommsDevicePtr dst);

  void _DeliverFrame(PacketPtr dlf, ROSCommsDevicePtr dst);

  void _AddDeviceToSet(std::string iddev, ROSCommsDevicePtr dev);
  bool _DeviceExists(std::string iddev);
  void _RemoveDeviceFromSet(std::string iddev);

  ROSCommsDevicePtr _GetDevice(std::string iddev);

  ros::ServiceServer _addDevService, _checkDevService, _removeDevService,
      _linkDeviceToChannelService;
  ros::NodeHandle &_rosNode;

  std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;

  PacketBuilderPtr _packetBuilder;
  ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
  void _LinkUpdaterWork();
  void _UpdateDevLinkFromRange(VirtualDeviceLinkPtr chn, double range,
                               bool log = true);

  ////////////////////
  VirtualDevicesLinks _devLinks;
  Type2DevMapMap _type2DevMap;
  DccommsDevMap _dccommsDevMap;
  Type2ChannelMapMap _type2ChannelMapMap;

  //////////
  ROSCommsSimulatorPtr _this;
};
}
#endif // WHROVSIMULATOR_H
