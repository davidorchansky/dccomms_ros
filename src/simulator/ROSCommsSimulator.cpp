#include <dccomms/CommsDeviceService.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/PacketBuilderLoader.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/types.h>
#include <iostream>
#include <list>
#include <ns3/core-module.h>
#include <ns3/simulator.h>
#include <regex>
#include <tf/transform_listener.h>

using namespace ns3;
using namespace dccomms;
using namespace dccomms_ros_msgs;

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(ROSCommsSimulator);
NS_LOG_COMPONENT_DEFINE("ROSCommsSimulator");

TypeId ROSCommsSimulator::GetTypeId(void) {
  static TypeId tid =
      TypeId("ROSCommsSimulator")
          .SetParent<Object>()
          .AddAttribute("ROSDeviceList",
                        "The list of devices associated to the simulator.",
                        ObjectVectorValue(),
                        MakeObjectVectorAccessor(&ROSCommsSimulator::_devices),
                        MakeObjectVectorChecker<ROSCommsDevice>());

  return tid;
}

ROSCommsSimulator::ROSCommsSimulator() : _linkUpdaterWorker(this), _this(this) {
  SetLogName("CommsSimulator");
  LogToConsole(true);
  FlushLogOn(cpplogging::LogLevel::info);
  _linkUpdaterWorker.SetWork(&ROSCommsSimulator::_LinkUpdaterWork);
  _Init();
}

ROSCommsSimulator::~ROSCommsSimulator() {}

PacketBuilderPtr
ROSCommsSimulator::GetPacketBuilder(const std::string &dccommsId,
                                    PACKET_TYPE type) {
  DevicePacketBuilder dpb;
  auto it = _packetBuilderMap.find(dccommsId);
  if (it != _packetBuilderMap.end())
    dpb = it->second;
  if (type == RX_PACKET)
    return dpb.rxpb;
  else if (type == TX_PACKET)
    return dpb.txpb;
}

void ROSCommsSimulator::SetPacketBuilder(const string &dccommsId,
                                         PACKET_TYPE type,
                                         PacketBuilderPtr pb) {
  DevicePacketBuilder dpb = _packetBuilderMap[dccommsId];
  if (type == RX_PACKET)
    dpb.rxpb = pb;
  else
    dpb.txpb = pb;
  _packetBuilderMap[dccommsId] = dpb;
}

void ROSCommsSimulator::SetPacketBuilder(const std::string &dccommsId,
                                         PACKET_TYPE type,
                                         const std::string &libName,
                                         const std::string &className) {
  dccomms::PacketBuilderPtr pb =
      PacketBuilderLoader::LoadPacketBuilder(libName, className);
  SetPacketBuilder(dccommsId, type, pb);
}
void ROSCommsSimulator::SetDefaultPacketBuilder(const std::string &libName,
                                                const std::string &className) {
  dccomms::PacketBuilderPtr pb =
      PacketBuilderLoader::LoadPacketBuilder(libName, className);
  SetDefaultPacketBuilder(pb);
}

void ROSCommsSimulator::SetTransmitPDUCb(
    std::function<void(ROSCommsDevice *, dccomms::PacketPtr)> cb) {
  TransmitPDUCb = cb;
}

void ROSCommsSimulator::SetReceivePDUCb(
    std::function<void(ROSCommsDevice *, dccomms::PacketPtr)> cb) {
  ReceivePDUCb = cb;
}

void ROSCommsSimulator::SetPositionUpdatedCb(
    std::function<void(ROSCommsDevicePtr dev, tf::Vector3)> cb,
    double cbMinPeriod, uint32_t positionUpdateRate) {
  PositionUpdatedCb = cb;
  _positionUpdatedCbMinPeriod = cbMinPeriod;
  _updatePositionRate = positionUpdateRate;
}

void ROSCommsSimulator::_Init() {
  SetTransmitPDUCb([](ROSCommsDevice *dev, PacketPtr pdu) {});
  SetReceivePDUCb([](ROSCommsDevice *dev, PacketPtr pdu) {});
  SetPositionUpdatedCb([](ROSCommsDevicePtr dev, tf::Vector3 pos) {}, 1000);
  _publish_rate = 10;
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
}

void ROSCommsSimulator::_AddDeviceToSet(std::string iddev,
                                        ROSCommsDevicePtr dev) {
  _idDevMapMutex.lock();
  _dccommsDevMap[iddev] = dev;
  _devices.push_back(dev);

  static ns3::Ptr<ROSCommsSimulator> ptr = 0;
  if (ptr == 0) {
    ptr = ns3::Ptr<ROSCommsSimulator>(this);
    Config::RegisterRootNamespaceObject(ptr);
  }
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
  // auto energyModel = req.energyModel;

  Log->info("Add device request received");

  bool exists = _CommonPreAddDev(dccommsId, deviceType, mac);

  if (!exists) {
    auto txpb = GetPacketBuilder(dccommsId, TX_PACKET);
    if (!txpb)
      txpb = GetDefaultPacketBuilder();
    auto rxpb = GetPacketBuilder(dccommsId, RX_PACKET);
    if (!rxpb)
      rxpb = GetDefaultPacketBuilder();
    ns3::Ptr<AcousticROSCommsDevice> dev =
        ns3::CreateObject<AcousticROSCommsDevice>(_this, txpb, rxpb);

    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetCodingEff(req.codingEff);
    dev->SetInitialEnergy(req.batteryEnergy);
    dev->SetSymbolsPerSecond(req.symbolsPerSecond);
    dev->SetBitErrorRate(req.bitErrorRate);
    dev->SetBitRate(req.symbolsPerSecond / req.codingEff);
    dev->SetMaxTxFifoSize(req.maxTxFifoSize);
    dev->SetMACProtocol(req.macProtocol);
    dev->SetRange(req.range);
    dev->SetPT(req.PT);
    dev->SetFreq(req.frequency);
    dev->SetL(req.L);
    dev->SetK(req.K);
    dev->SetTurnOnEnergy(req.turnOnEnergy);
    dev->SetTurnOffEnergy(req.turnOffEnergy);
    dev->SetPreamble(req.preamble);
    dev->SetTxPower(req.PTConsume);
    dev->SetRxPower(req.PRConsume);
    dev->SetIdlePower(req.PIdle);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = dev;
    _AddDeviceToSet(dev->GetDccommsId(), dev);

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev));
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
  if (!dev) {
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
  }
  default: { res.res = false; }
  }
  if (res.res) {
    dev->LinkToChannel(channel, (CHANNEL_LINK_TYPE)req.linkType);
    Log->info("dev {} linked to channel {}:\n{}", dev->GetDccommsId(),
              channel->GetId(), dev->ToString());
  } else {
    Log->error("error linking dev {} to channel {}", dev->GetDccommsId(),
               channel->GetId());
  }
  return res.res;
}

bool ROSCommsSimulator::_AddAcousticChannel(AddAcousticChannel::Request &req,
                                            AddAcousticChannel::Response &res) {

  uint32_t id = req.id;
  res.res = false;
  if (!_channelMap[id]) {
    auto acousticChannel =
        dccomms::CreateObject<dccomms_ros::AcousticCommsChannel>(id);

    acousticChannel->SetBandwidth(req.bandwidth);
    acousticChannel->SetNoiseLevel(req.noiseLvl);
    acousticChannel->SetSalinity(req.salinity);
    acousticChannel->SetTemperature(req.temperature);

    _channelMap[id] = acousticChannel;
    res.res = true;
    Log->info("acoustic channel {} added", id);
  }
  return res.res;
}

bool ROSCommsSimulator::_AddCustomChannel(AddCustomChannel::Request &req,
                                          AddCustomChannel::Response &res) {
  uint32_t id = req.id;
  if (!_channelMap[id]) {
    CustomCommsChannelPtr channel =
        dccomms::CreateObject<CustomCommsChannel>(id);
    channel->SetMinPrTime(req.minPrTime);
    channel->SetPrTimeInc(req.prTimeIncPerMeter);
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
  DEV_TYPE deviceType = DEV_TYPE::CUSTOM_DEV;

  bool exists = _CommonPreAddDev(dccommsId, deviceType, mac);

  Log->info("Add device request received");

  if (!exists) {
    auto txpb = GetPacketBuilder(dccommsId, TX_PACKET);
    if (!txpb)
      txpb = GetDefaultPacketBuilder();
    auto rxpb = GetPacketBuilder(dccommsId, RX_PACKET);
    if (!rxpb)
      rxpb = GetDefaultPacketBuilder();
    CustomROSCommsDevicePtr dev =
        ns3::CreateObject<CustomROSCommsDevice>(_this, txpb, rxpb);
    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetBitRate(req.bitrate);
    dev->SetMaxDistance(req.maxDistance);
    dev->SetMinDistance(req.minDistance);
    dev->SetMinPktErrorRate(req.minPktErrorRate);
    dev->SetPktErrorRateInc(req.pktErrorRateIncPerMeter);
    dev->SetVariableBitRate(req.bitrate, req.bitrateSd);
    dev->SetMaxTxFifoSize(req.maxTxFifoSize);
    dev->SetRateErrorModel(req.errorRateExpr, req.errorUnit);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = dev;
    _AddDeviceToSet(dev->GetDccommsId(), dev);

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev));
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
  _rosNode = ros::NodeHandle("~");
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

  std::thread rosLoop([this]() {
    ros::Rate rate(_publish_rate);
    while (ros::ok()) {

      rate.sleep();
    }
  });
  rosLoop.detach();
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

bool ROSCommsSimulator::Ready(DEV_TYPE devType) {
  bool ready = true;
  for (auto dpair : _dccommsDevMap) {
    if (dpair.second->GetDevType() == devType && !dpair.second->Started()) {
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

  dccomms::Timer showLogTimer;
  bool callPositionUpdatedCb = false;
  unsigned int positionUpdatedCbInterval = _positionUpdatedCbMinPeriod;
  showLogTimer.Reset();

  ros::Rate loop_rate(_updatePositionRate);

  std::string frameId0, frameId1;
  while (1) {

    if (showLogTimer.Elapsed() > positionUpdatedCbInterval) {
      callPositionUpdatedCb = true;
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
          if (callPositionUpdatedCb)
            PositionUpdatedCb(dev, position);
        } catch (std::exception &e) {
          if (callPositionUpdatedCb)
            Log->warn("An exception has ocurred in the link updater work: {}",
                      std::string(e.what()));
        }
      }
    }
    _devLinksMutex.unlock();
    loop_rate.sleep();

    if (callPositionUpdatedCb) {
      callPositionUpdatedCb = false;
      showLogTimer.Reset();
    }
  }
}

bool ROSCommsSimulator::AddAcousticDevice(
    dccomms_ros_msgs::AddAcousticDevice::Request &req) {
  dccomms_ros_msgs::AddAcousticDevice::Response res;
  return _AddAcousticDevice(req, res);
}
bool ROSCommsSimulator::LinkDevToChannel(
    dccomms_ros_msgs::LinkDeviceToChannel::Request &req) {
  dccomms_ros_msgs::LinkDeviceToChannel::Response res;
  return _LinkDevToChannel(req, res);
}
bool ROSCommsSimulator::AddAcousticChannel(
    dccomms_ros_msgs::AddAcousticChannel::Request &req) {
  dccomms_ros_msgs::AddAcousticChannel::Response res;
  return _AddAcousticChannel(req, res);
}
bool ROSCommsSimulator::AddCustomChannel(
    dccomms_ros_msgs::AddCustomChannel::Request &req) {
  dccomms_ros_msgs::AddCustomChannel::Response res;
  return _AddCustomChannel(req, res);
}
bool ROSCommsSimulator::AddCustomDevice(
    dccomms_ros_msgs::AddCustomDevice::Request &req) {
  dccomms_ros_msgs::AddCustomDevice::Response res;
  return _AddCustomDevice(req, res);
}
bool ROSCommsSimulator::StartSimulation() {
  dccomms_ros_msgs::StartSimulation::Request req;
  dccomms_ros_msgs::StartSimulation::Response res;
  return _StartSimulation(req, res);
}
}
