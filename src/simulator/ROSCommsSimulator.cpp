#include <dccomms/CommsDeviceService.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
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

bool ROSCommsSimulator::_CheckChannel(CheckChannel::Request &req,
                                      CheckChannel::Response &res) {
  auto id = req.id;
  res.exists = _ChannelExists(id);
  return true;
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

bool ROSCommsSimulator::_AddAcousticDevice(AddAcousticDevice::Request &req,
                                           AddAcousticDevice::Response &res) {
  auto dccommsId = req.dccommsId;
  DEV_TYPE deviceType = static_cast<DEV_TYPE>(req.type);
  auto mac = req.mac;
  auto frameId = req.frameId;
  auto maxBitRate = req.maxBitRate;
  // auto energyModel = req.energyModel;

  Log->info("Add device request received");

  bool exists = _CommonPreAddDev(dccommsId, deviceType, mac);

  if (!exists) {
    ROSCommsDevicePtr dev =
        dccomms::CreateObject<AcousticROSCommsDevice>(_this, _packetBuilder);
    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetMaxBitRate(maxBitRate);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = dev;
    _dccommsDevMap[dev->GetDccommsId()] = dev;

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev.get()));
    res.res = true;

  } else {
    res.res = false;
  }

  return res.res;
}

bool ROSCommsSimulator::_ChannelExists(uint32_t id) {
  return _GetChannel(id) ? true : false;
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
  if (!dev || dev->GetLinkedChannel()) {
    res.res = false;
    return res.res;
  }
  DEV_TYPE devType = dev->GetDevType();
  res.res = true;
  if (!channel) {
    res.res = false;
    return res.res;
  }

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
  default: { res.res = false; }
  }
  }
  if (res.res) {
    dev->LinkToChannel(channel);
    Log->info("dev {} linked to channel {}", dev->GetDccommsId(),
              channel->GetId());
  } else {
    Log->error("error linking dev {} to channel {}", dev->GetDccommsId(),
               channel->GetId());
  }
  return res.res;
}

bool ROSCommsSimulator::_AddAcousticChannel(AddAcousticChannel::Request &req,
                                            AddAcousticChannel::Response &res) {
  CommsChannelPtr channel;
  CHANNEL_TYPE type = (CHANNEL_TYPE)req.type;
  uint32_t id = req.id;
  res.res = false;
  switch (type) {
  case ACOUSTIC_UNDERWATER_CHANNEL: {
    if (!_channelMap[id]) {
      auto acousticChannel =
          dccomms::CreateObject<dccomms_ros::AcousticCommsChannel>(id);
      channel = acousticChannel;
      _channelMap[id] = channel;
      res.res = true;
      Log->info("acoustic channel {} added", id);
    }
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
  if (!res.res) {
    Log->error("error adding acoustic channel {}", id);
  }
  return res.res;
}

bool ROSCommsSimulator::_AddCustomChannel(AddCustomChannel::Request &req,
                                          AddCustomChannel::Response &res) {
  uint32_t id = req.id;
  if (!_channelMap[id]) {
    CommsChannelPtr channel = dccomms::CreateObject<CustomCommsChannel>(id);
    _channelMap[id] = channel;
    res.res = true;
    Log->info("custom channel {} added", id);
  } else {
    Log->error("error adding custom channel {}", id);
    res.res = false;
  }
  return res.res;
}

bool ROSCommsSimulator::_CommonPreAddDev(const std::string &dccommsId,
                                         DEV_TYPE deviceType, uint32_t mac) {
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
  return exists;
}
bool ROSCommsSimulator::_AddCustomDevice(AddCustomDevice::Request &req,
                                         AddCustomDevice::Response &res) {

  auto dccommsId = req.dccommsId;
  auto mac = req.mac;
  auto frameId = req.frameId;
  auto maxBitRate = req.maxBitRate;
  DEV_TYPE deviceType = DEV_TYPE::CUSTOM_DEV;

  bool exists = _CommonPreAddDev(dccommsId, deviceType, mac);

  Log->info("Add device request received");

  if (!exists) {
    CustomROSCommsDevicePtr dev =
        dccomms::CreateObject<CustomROSCommsDevice>(_this, _packetBuilder);
    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetMaxBitRate(maxBitRate);
    dev->SetMaxDistance(req.maxDistance);
    dev->SetMinDistance(req.minDistance);
    dev->SetMinPktErrorRate(req.minPktErrorRate);
    dev->SetPktErrorRateInc(req.pktErrorRateIncPerMeter);
    dev->SetBitRate(req.bitrate, req.bitrateSd);
    dev->SetMaxBitRate(req.maxBitRate);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = dev;
    _dccommsDevMap[dev->GetDccommsId()] = dev;

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev.get()));
    res.res = true;

  } else {
    res.res = false;
  }

  return res.res;
}

void ROSCommsSimulator::StartROSInterface() {
  /*
   * http://www.boost.org/doc/libs/1_63_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_functions_and_fu
   */
  _addDevService = _rosNode.advertiseService(
      "add_acoustic_net_device", &ROSCommsSimulator::_AddAcousticDevice, this);
  _addChannelService = _rosNode.advertiseService(
      "add_acoustic_channel", &ROSCommsSimulator::_AddAcousticChannel, this);
  _checkDevService = _rosNode.advertiseService(
      "check_net_device", &ROSCommsSimulator::_CheckDevice, this);
  _checkChannelService = _rosNode.advertiseService(
      "check_channel", &ROSCommsSimulator::_CheckChannel, this);
  _removeDevService = _rosNode.advertiseService(
      "remove_net_device", &ROSCommsSimulator::_RemoveDevice, this);
  _linkDeviceToChannelService = _rosNode.advertiseService(
      "link_dev_to_channel", &ROSCommsSimulator::_LinkDevToChannel, this);
  _startSimulationService = _rosNode.advertiseService(
      "start_simulation", &ROSCommsSimulator::_StartSimulation, this);
  _addCustomChannelService = _rosNode.advertiseService(
      "add_custom_channel", &ROSCommsSimulator::_AddCustomChannel, this);
  _addCustomDeviceService = _rosNode.advertiseService(
      "add_custom_net_device", &ROSCommsSimulator::_AddCustomDevice, this);
  _linkUpdaterWorker.Start();
}

bool ROSCommsSimulator::_StartSimulation(
    dccomms_ros_msgs::StartSimulation::Request &req,
    dccomms_ros_msgs::StartSimulation::Response &res) {
  if (!_started) {
    _Run();
    res.res = true;
    _started = true;
  } else
    res.res = false;
  return res.res;
}

void ROSCommsSimulator::_SetSimulationStartDateTime() {
  _startPoint = std::chrono::high_resolution_clock::now();
}

void ROSCommsSimulator::GetSimTime(std::string &datetime,
                                   double &secsFromStart) {
  auto simTime = ns3::Simulator::Now();
  secsFromStart = simTime.GetSeconds();
  auto tstamp = simTime.GetTimeStep(); // nanoseconds

  auto simNanos = std::chrono::nanoseconds(tstamp);
  auto curpoint = _startPoint + simNanos;
  auto t = std::chrono::high_resolution_clock::to_time_t(curpoint);
  auto localEventTime = std::localtime(&t);
  char mbstr[100];
  auto count = std::strftime(mbstr, sizeof(mbstr), _timeFormat, localEventTime);
  char *mp = mbstr + count;

  auto durationFromEpoch = curpoint.time_since_epoch();
  auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(durationFromEpoch) -
      std::chrono::duration_cast<std::chrono::seconds>(durationFromEpoch);
  sprintf(mp, ".%ld", millis.count());
  datetime = mbstr;
}

void ROSCommsSimulator::_IsAliveWork() {
  Info("Is alive...");
  Simulator::Schedule(Seconds(1),
                      MakeEvent(&ROSCommsSimulator::_IsAliveWork, this));
}

bool ROSCommsSimulator::Ready() {
  bool ready = true;
  for (auto dpair : _dccommsDevMap) {
    if (!dpair.second->Started()) {
      ready = false;
      break;
    }
  }
  return ready;
}

void ROSCommsSimulator::_Run() {
  std::thread task([this]() {
    Simulator::Schedule(
        Seconds(0),
        MakeEvent(&ROSCommsSimulator::_SetSimulationStartDateTime, this));
    //    Simulator::Schedule(Seconds(1),
    //                        MakeEvent(&ROSCommsSimulator::_IsAliveWork,
    //                        this));
    Simulator::Run();
  });
  task.detach();
}

void ROSCommsSimulator::_LinkUpdaterWork() {
  tf::TransformListener listener;

  dccomms::Timer timer;
  bool showLog = false;
  unsigned int showLogInterval = 10000;
  timer.Reset();

  ros::Rate loop_rate(5);

  std::string frameId0, frameId1;
  while (1) {

    if (timer.Elapsed() > showLogInterval) {
      showLog = true;
    }

    _devLinksMutex.lock();
    tf::StampedTransform transform;
    for (std::pair<const uint32_t, Mac2DevMapPtr> type2Devs : _type2DevMap) {
      Mac2DevMapPtr mac2DevMap = type2Devs.second;
      for (std::pair<const uint32_t, ROSCommsDevicePtr> mac2Dev : *mac2DevMap) {
        ROSCommsDevicePtr dev = mac2Dev.second;
        std::string tfFrameId = dev->GetTfFrameId();
        try {
          ros::Time now = ros::Time::now();
          listener.lookupTransform(tfFrameId, "/world", ros::Time(0),
                                   transform);
          tf::Vector3 position = transform.getOrigin();
          dev->SetPosition(position);
        } catch (std::exception &e) {
          if (showLog)
            Log->warn("An exception has ocurred in the link updater work: {}",
                      std::string(e.what()));
        }
      }
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
