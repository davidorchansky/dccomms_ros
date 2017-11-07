#include <dccomms/CommsDeviceService.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/types.h>
#include <iostream>
#include <list>
#include <regex>
#include <tf/transform_listener.h>

#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;
using namespace dccomms;
using namespace dccomms_ros_msgs;

namespace dccomms_ros {
ROSCommsSimulator::ROSCommsSimulator(ros::NodeHandle &rosNode,
                                     PacketBuilderPtr packetBuilder)
    : _rosNode(rosNode), _linkUpdaterWorker(this),
      _this(ROSCommsSimulatorPtr(this)) {
  _packetBuilder = packetBuilder;
  SetLogName("CommsSimulator");
  LogToConsole(true);
  FlushLogOn(cpplogging::LogLevel::info);
  _linkUpdaterWorker.SetWork(&ROSCommsSimulator::_LinkUpdaterWork);
  _Init();
}

PacketBuilderPtr ROSCommsSimulator::GetPacketBuilder() {
  return _packetBuilder;
}

void ROSCommsSimulator::SetTransmitPDUCb(
    std::function<void(int, PacketPtr)> cb) {
  _TransmitPDUCb = cb;
}

void ROSCommsSimulator::SetReceivePDUCb(
    std::function<void(int, PacketPtr)> cb) {
  _ReceivePDUCb = cb;
}

void ROSCommsSimulator::SetErrorPDUCb(std::function<void(int, PacketPtr)> cb) {
  _ErrorPDUCb = cb;
}

void ROSCommsSimulator::_Init() {
  _TransmitPDUCb = [](int linkType, PacketPtr pdu) {};
  _ReceivePDUCb = [](int linkType, PacketPtr pdu) {};
  _ErrorPDUCb = [](int linkType, PacketPtr pdu) {};
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
}

void ROSCommsSimulator::_AddDeviceToSet(std::string iddev,
                                        ROSCommsDevicePtr dev) {
  _idDevMapMutex.lock();
  _dccommsDevMap[iddev] = dev;
  _idDevMapMutex.unlock();
}

void ROSCommsSimulator::_RemoveDeviceFromSet(std::string iddev) {
  _idDevMapMutex.lock();
  auto iter = _dccommsDevMap.find(iddev);
  if (iter != _dccommsDevMap.end()) {
    _dccommsDevMap.erase(iter);
  }
  _idDevMapMutex.unlock();
}

bool ROSCommsSimulator::_CheckDevice(CheckDevice::Request &req,
                                     CheckDevice::Response &res) {
  auto iddev = req.iddev;
  res.exists = _DeviceExists(iddev);
  return true;
}

bool ROSCommsSimulator::_DeviceExists(std::string iddev) {
  bool exists;
  auto devIt = _dccommsDevMap.find(iddev);
  if (devIt != _dccommsDevMap.end()) {
    exists = true;
  }
  return exists;
}

ROSCommsDevicePtr ROSCommsSimulator::_GetDevice(std::string iddev) {
  ROSCommsDevicePtr dev;
  auto devIt = _dccommsDevMap.find(iddev);
  if (devIt != _dccommsDevMap.end()) {
    dev = devIt->second;
  }
  return dev;
}

bool ROSCommsSimulator::_RemoveDevice(RemoveDevice::Request &req,
                                      RemoveDevice::Response &res) {
  return true;
}

bool ROSCommsSimulator::_AddDevice(AddDevice::Request &req,
                                   AddDevice::Response &res) {
  auto dccommsId = req.dccommsId;
  DEV_TYPE deviceType = (DEV_TYPE)req.type;
  auto mac = req.mac;
  auto frameId = req.frameId;
  auto maxBitRate = req.maxBitRate;
  // auto energyModel = req.energyModel;

  Log->info("Add device request received");

  bool exists = false;
  auto mac2DevMapIt = _type2DevMap.find(deviceType);
  if (mac2DevMapIt != _type2DevMap.end()) {
    Mac2DevMapPtr mac2DevMap = mac2DevMapIt->second;
    auto devIt = mac2DevMap->find(mac);
    if (devIt != mac2DevMap->end()) {
      exists = true;
      Log->error("Unable to add the device. A net device with the same MAC "
                 "already exists: '{}'",
                 devIt->second->GetDccommsId());
    }
  } else {
    Mac2DevMapPtr mac2DevMap(new Mac2DevMap());
    _type2DevMap[deviceType] = mac2DevMap;
  }

  if (!exists) {
    auto devIt = _dccommsDevMap.find(dccommsId);
    if (devIt != _dccommsDevMap.end()) {
      exists = true;
      Log->error(
          "Unable to add the device. A net device with the same dccommsId "
          "already exists: '{}'",
          devIt->second->GetDccommsId());
    }
  }

  if (!exists) {
    //    ROSCommsDevicePtr dev =
    //        dccomms::CreateObject<ROSCommsDevice>(_this, _packetBuilder);
    //    dev->SetDccommsId(dccommsId);
    //    dev->SetMac(mac);
    //    dev->SetTfFrameId(frameId);
    //    dev->SetMaxBitRate(maxBitRate);

    //    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    //    (*mac2DevMap)[mac] = dev;
    //    _dccommsDevMap[dev->GetDccommsId()] = dev;

    //    Log->info("\nAdding device:\n{}", dev->ToString());
    //    auto starterWork = [dev]() {
    //      dev->StartDeviceService();
    //      dev->StartNodeWorker();
    //    };

    //    std::thread starter(starterWork);
    //    starter.detach();
    //    res.res = true;

  } else {
    res.res = false;
  }

  return res.res;
}

CommsChannelPtr ROSCommsSimulator::_GetChannel(int id) {
  CommsChannelPtr channel;
  auto it = _channelMap.find(id);
  if (it != _channelMap.end()) {
    channel = it->second;
  }
  return channel;
}
bool ROSCommsSimulator::_LinkDevToChannel(LinkDeviceToChannel::Request &req,
                                          LinkDeviceToChannel::Response &res) {
  ROSCommsDevicePtr dev = _GetDevice(req.dccommsId);
  CommsChannelPtr channel = _GetChannel(req.channelId);
  DEV_TYPE devType = dev->GetDevType();
  CHANNEL_TYPE chnType = channel->GetType();
  switch (devType) {
  case ACOUSTIC_UNDERWATER_DEV: {
    if (chnType == ACOUSTIC_UNDERWATER_CHANNEL) {

    } else {
      res.res = false;
    }
    break;
  }
  case CUSTOM_DEV: {
    break;
  }
  }
  if (res.res) {
    // Link dev to channel
  }
  return res.res;
}

bool ROSCommsSimulator::_AddChannel(AddChannel::Request &req,
                                    AddChannel::Response &res) {
  CommsChannelPtr channel;
  CHANNEL_TYPE type = (CHANNEL_TYPE)req.type;
  uint32_t id = req.id;
  switch (type) {
  case ACOUSTIC_UNDERWATER_CHANNEL: {
    auto acousticChannel =
        dccomms::CreateObject<dccomms_ros::AcousticCommsChannel>(id);
    channel = acousticChannel;
    res.res = true;
    break;
  }
  case CUSTOM_CHANNEL:
    break;
  default:
    Log->error("unknown channel type");
    res.res = false;
    return false;
    break;
  }
  _channelMap[id] = channel;
  return res.res;
}

void ROSCommsSimulator::StartROSInterface() {
  /*
   * http://www.boost.org/doc/libs/1_63_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_functions_and_fu
   */
  _addDevService = _rosNode.advertiseService(
      "add_net_device", &ROSCommsSimulator::_AddDevice, this);
  _addDevService = _rosNode.advertiseService(
      "add_channel", &ROSCommsSimulator::_AddChannel, this);
  _checkDevService = _rosNode.advertiseService(
      "check_net_device", &ROSCommsSimulator::_CheckDevice, this);
  _removeDevService = _rosNode.advertiseService(
      "remove_net_device", &ROSCommsSimulator::_RemoveDevice, this);
  _linkDeviceToChannelService = _rosNode.advertiseService(
      "link_dev_to_channel", &ROSCommsSimulator::_LinkDevToChannel, this);
  _linkUpdaterWorker.Start();
}

void ROSCommsSimulator::_Run() {
  std::thread task([]() { Simulator::Run(); });
  task.detach();
}

void ROSCommsSimulator::_LinkUpdaterWork() {
  tf::TransformListener listener;

  dccomms::Timer timer;
  bool showLog = false;
  unsigned int showLogInterval = 1500;
  timer.Reset();

  ros::Rate loop_rate(20);

  std::string frameId0, frameId1;
  while (1) {

    if (timer.Elapsed() > showLogInterval) {
      showLog = true;
    }

    _devLinksMutex.lock();
    tf::StampedTransform transform;
    for (VirtualDeviceLinkPtr link : _devLinks) {
      double distance = 0;
      try {
        frameId0 = link->GetDevice0()->GetTfFrameId();
        frameId1 = link->GetDevice1()->GetTfFrameId();

        ros::Time now = ros::Time::now();

        listener.lookupTransform(frameId0, frameId1, ros::Time(0), transform);
        distance = transform.getOrigin().distance(tf::Vector3(0, 0, 0));
        if (showLog)
          Log->debug("Range between frame '{}' and '{}': {}", frameId0,
                     frameId1, distance);
      } catch (std::exception &e) {
        distance = 0;
        if (showLog)
          Log->warn("An exception has ocurred in the link updater work: frames "
                    "{}-{}: {}",
                    frameId0, frameId1, std::string(e.what()));
      }
      //_UpdateDevLinkFromRange(link, distance, showLog);
    }
    _devLinksMutex.unlock();
    loop_rate.sleep();

    if (showLog) {
      showLog = false;
      timer.Reset();
    }
  }
}
}
