#ifndef WHROVSIMULATOR_H
#define WHROVSIMULATOR_H

#include <cpplogging/Loggable.h>
#include <dccomms/dccomms.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <functional>
#include <memory>
#include <random>
#include <unordered_map>

// ROS
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

typedef std::unordered_map<int, ROSCommsDevicePtr> NodeMap;

typedef std::shared_ptr<NodeMap> NodeMapPtr;

typedef std::unordered_map<int, NodeMapPtr> NodeTypeMap;

typedef std::unordered_map<std::string, //"class_txdir_rxdir"
                           VirtualDeviceLinkPtr>
    ChannelMap;

typedef std::unordered_map<std::string, ROSCommsDevicePtr> IdDevMap;

class ROSCommsSimulator;

typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Loggable {
public:
  ROSCommsSimulator(ros::NodeHandle &rosnode, PacketBuilderPtr packetBuilder);
  void TransmitFrame(ROSCommsDevicePtr dev, PacketPtr dlf);
  void Start();

  virtual void SetLogName(std::string name);
  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

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
  void _PropagateFrame(PacketPtr dlf, int delay, VirtualDeviceLinkPtr channel);
  void _DeliverFrame(PacketPtr dlf, VirtualDeviceLinkPtr channel);

  void _AddDeviceToSet(std::string iddev, ROSCommsDevicePtr dev);
  bool _DeviceExists(std::string iddev);
  void _RemoveDeviceFromSet(std::string iddev);

  void _RemoveChannel(std::string channelKey);

  ROSCommsDevicePtr _GetDevice(std::string iddev);
  VirtualDeviceLinkPtr _GetChannel(std::string channelKey);

  ros::ServiceServer _addDevService, _checkDevService, _removeDevService;
  ros::NodeHandle &_rosNode;
  NodeTypeMap _nodeTypeMap;
  ChannelMap _channels;
  IdDevMap _idDevMap;

  std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;

  PacketBuilderPtr _packetBuilder;
  ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
  void _LinkUpdaterWork();
  void _UpdateChannelStateFromRange(VirtualDeviceLinkPtr chn, double range,
                                    bool log = true);

  ////////////////////
  VirtualDevicesLinks _devLinks;
};
}
#endif // WHROVSIMULATOR_H
