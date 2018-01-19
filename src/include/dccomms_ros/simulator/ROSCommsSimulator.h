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
#include <dccomms_ros_msgs/AddAcousticChannel.h>
#include <dccomms_ros_msgs/AddAcousticDevice.h>
#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/AddCustomDevice.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <dccomms_ros_msgs/StartSimulation.h>
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

enum PACKET_TYPE { TX_PACKET, RX_PACKET };

struct DevicePacketBuilder {
  PacketBuilderPtr txpb, rxpb;
};
typedef std::unordered_map<std::string, DevicePacketBuilder> PacketBuilderMap;
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
  ROSCommsSimulator(ros::NodeHandle &rosnode);
  void StartROSInterface();

  void
  SetTransmitPDUCb(std::function<void(ROSCommsDevice* txdev, dccomms::PacketPtr)> cb);
  void
  SetReceivePDUCb(std::function<void(ROSCommsDevice* rxdev, dccomms::PacketPtr)> cb);
  void SetErrorPDUCb(std::function<void(ROSCommsDevice* rxdev, dccomms::PacketPtr)> cb);
  void SetPositionUpdatedCb(std::function<void(ROSCommsDevicePtr dev, tf::Vector3)> cb, double cbMinPeriod, uint32_t positionUpdateRate = 10); //callback period = ms ; update rate = Hz

  void GetSimTime(std::string &datetime, double &secsFromStart);

  PacketBuilderPtr GetPacketBuilder(const std::string &dccommsId,
                                    PACKET_TYPE type);
  void SetPacketBuilder(const std::string &dccommsId, PACKET_TYPE type,
                        PacketBuilderPtr pb);
  void SetDefaultPacketBuilder(PacketBuilderPtr pb) {
    _defaultPacketBuilder = pb;
  }
  PacketBuilderPtr GetDefaultPacketBuilder() { return _defaultPacketBuilder; }
  bool Ready(DEV_TYPE);

  bool AddAcousticDevice(dccomms_ros_msgs::AddAcousticDevice::Request &req);
  bool LinkDevToChannel(dccomms_ros_msgs::LinkDeviceToChannel::Request &req);
  bool AddAcousticChannel(dccomms_ros_msgs::AddAcousticChannel::Request &req);
  bool AddCustomChannel(dccomms_ros_msgs::AddCustomChannel::Request &req);
  bool AddCustomDevice(dccomms_ros_msgs::AddCustomDevice::Request &req);
  bool StartSimulation();

  friend class ROSCommsDevice;
private:
  const char _timeFormat[100] = "%Y-%m-%d %H:%M:%S";
  void _Run();
  void _Init();

  std::chrono::high_resolution_clock::time_point _startPoint;
  void _SetSimulationStartDateTime();

  std::function<void(ROSCommsDevice* dev, dccomms::PacketPtr)> TransmitPDUCb, ReceivePDUCb,
      ErrorPDUCb;
  std::function<void(ROSCommsDevicePtr dev, tf::Vector3)> PositionUpdatedCb;

  bool _AddAcousticDevice(dccomms_ros_msgs::AddAcousticDevice::Request &req,
                          dccomms_ros_msgs::AddAcousticDevice::Response &res);
  bool _CheckDevice(dccomms_ros_msgs::CheckDevice::Request &req,
                    dccomms_ros_msgs::CheckDevice::Response &res);
  bool _CheckChannel(dccomms_ros_msgs::CheckChannel::Request &req,
                     dccomms_ros_msgs::CheckChannel::Response &res);
  bool _RemoveDevice(dccomms_ros_msgs::RemoveDevice::Request &req,
                     dccomms_ros_msgs::RemoveDevice::Response &res);
  bool _LinkDevToChannel(dccomms_ros_msgs::LinkDeviceToChannel::Request &req,
                         dccomms_ros_msgs::LinkDeviceToChannel::Response &res);
  bool _AddAcousticChannel(dccomms_ros_msgs::AddAcousticChannel::Request &req,
                           dccomms_ros_msgs::AddAcousticChannel::Response &res);
  bool _AddCustomChannel(dccomms_ros_msgs::AddCustomChannel::Request &req,
                         dccomms_ros_msgs::AddCustomChannel::Response &res);
  bool _AddCustomDevice(dccomms_ros_msgs::AddCustomDevice::Request &req,
                        dccomms_ros_msgs::AddCustomDevice::Response &res);

  void _AddDeviceToSet(std::string iddev, ROSCommsDevicePtr dev);
  bool _DeviceExists(std::string iddev);
  bool _ChannelExists(uint32_t id);
  void _RemoveDeviceFromSet(std::string iddev);

  bool _StartSimulation(dccomms_ros_msgs::StartSimulation::Request &req,
                        dccomms_ros_msgs::StartSimulation::Response &res);

  bool _CommonPreAddDev(const std::string &dccommsId, DEV_TYPE deviceType,
                        uint32_t mac);
  ROSCommsDevicePtr _GetDevice(std::string iddev);

  CommsChannelPtr _GetChannel(int id);

  ros::ServiceServer _addDevService, _checkDevService, _addChannelService,
      _removeDevService, _linkDeviceToChannelService, _startSimulationService,
      _addCustomDeviceService, _addCustomChannelService, _checkChannelService;
  ros::NodeHandle &_rosNode;

  std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;

  PacketBuilderPtr _defaultPacketBuilder;
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
  double _positionUpdatedCbMinPeriod;
  uint32_t _updatePositionRate;

  PacketBuilderMap _packetBuilderMap;
};
}
#endif // WHROVSIMULATOR_H
