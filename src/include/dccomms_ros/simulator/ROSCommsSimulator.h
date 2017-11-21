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
#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/AddCustomDevice.h>
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <dccomms_ros_msgs/StartSimulation.h>
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

typedef std::unordered_map<uint32_t, ROSCommsDevicePtr> Mac2DevMap;
typedef std::shared_ptr<Mac2DevMap> Mac2DevMapPtr;
typedef std::unordered_map<uint32_t, Mac2DevMapPtr> Type2DevMapMap;

typedef std::unordered_map<std::string, ROSCommsDevicePtr> DccommsDevMap;

typedef std::unordered_map<uint32_t, CommsChannelPtr> Id2ChannelMap;
typedef std::shared_ptr<Id2ChannelMap> Id2ChannelMapPtr;
typedef std::unordered_map<uint32_t, Id2ChannelMapPtr> Type2ChannelMapMap;

class ROSCommsSimulator;

typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Logger {
public:
  ROSCommsSimulator(ros::NodeHandle &rosnode, PacketBuilderPtr packetBuilder);
  void StartROSInterface();
  PacketBuilderPtr GetPacketBuilder();

  void
  SetTransmitPDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);
  void
  SetReceivePDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);
  void SetErrorPDUCb(std::function<void(int linkType, dccomms::PacketPtr)> cb);

  void GetSimTime(std::string &datetime, double &secsFromStart);

  bool Ready();
private:
  const char _timeFormat[100] = "%Y-%m-%d %H:%M:%S";
  void _Run();
  void _Init();

  std::chrono::high_resolution_clock::time_point _startPoint;
  void _SetSimulationStartDateTime();

  std::function<void(int, dccomms::PacketPtr)> _TransmitPDUCb, _ReceivePDUCb,
      _ErrorPDUCb;

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
  bool _AddCustomChannel(dccomms_ros_msgs::AddCustomChannel::Request &req,
                         dccomms_ros_msgs::AddCustomChannel::Response &res);
  bool _AddCustomDevice(dccomms_ros_msgs::AddCustomDevice::Request &req,
                        dccomms_ros_msgs::AddCustomDevice::Response &res);

  void _AddDeviceToSet(std::string iddev, ROSCommsDevicePtr dev);
  bool _DeviceExists(std::string iddev);
  void _RemoveDeviceFromSet(std::string iddev);

  bool _StartSimulation(dccomms_ros_msgs::StartSimulation::Request &req,
                        dccomms_ros_msgs::StartSimulation::Response &res);

  bool _CommonPreAddDev(const std::string &dccommsId, DEV_TYPE deviceType,
                        uint32_t mac);
  ROSCommsDevicePtr _GetDevice(std::string iddev);

  CommsChannelPtr _GetChannel(int id);

  ros::ServiceServer _addDevService, _checkDevService, _addChannelService,
      _removeDevService, _linkDeviceToChannelService, _startSimulationService,
      _addCustomDeviceService, _addCustomChannelService;
  ros::NodeHandle &_rosNode;

  std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;

  PacketBuilderPtr _packetBuilder;
  ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
  void _LinkUpdaterWork();
  void _IsAliveWork();

  ////////////////////
  VirtualDevicesLinks _devLinks;
  Type2DevMapMap _type2DevMap;
  DccommsDevMap _dccommsDevMap;
  // Type2ChannelMapMap _type2ChannelMapMap;
  Id2ChannelMap _channelMap;
  //////////
  ROSCommsSimulatorPtr _this;
  bool _started;
};
}
#endif // WHROVSIMULATOR_H
