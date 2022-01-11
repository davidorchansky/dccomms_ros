#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/AcousticROSCommsDevice.h>
#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros/simulator/NetsimTime.h>
#include <dccomms_ros/simulator/PacketBuilderLoader.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/srv/check_device.hpp>
#include <dccomms_ros_msgs/types.h>
#include <iostream>
#include <list>
#include <ns3/core-module.h>
#include <ns3/simulator.h>
#include <regex>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

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
                        "The list of all devices associated to the simulator.",
                        ObjectVectorValue(),
                        MakeObjectVectorAccessor(&ROSCommsSimulator::_devices),
                        MakeObjectVectorChecker<ROSCommsDevice>())
          .AddAttribute(
              "AcousticROSDeviceList",
              "The list of acoustic devices associated to the simulator.",
              ObjectVectorValue(),
              MakeObjectVectorAccessor(&ROSCommsSimulator::_acousticDevices),
              MakeObjectVectorChecker<AcousticROSCommsDevice>())
          .AddAttribute(
              "CustomROSDeviceList",
              "The list of custom devices associated to the simulator.",
              ObjectVectorValue(),
              MakeObjectVectorAccessor(&ROSCommsSimulator::_customDevices),
              MakeObjectVectorChecker<CustomROSCommsDevice>())
          .AddAttribute("ROSChannelList",
                        "The list of channels associated to the simulator.",
                        ObjectVectorValue(),
                        MakeObjectVectorAccessor(&ROSCommsSimulator::_channels),
                        MakeObjectVectorChecker<CommsChannel>())
          .AddAttribute(
              "CustomROSChannelList",
              "The list of custom channels associated to the simulator.",
              ObjectVectorValue(),
              MakeObjectVectorAccessor(&ROSCommsSimulator::_customChannels),
              MakeObjectVectorChecker<CustomCommsChannel>())
          .AddAttribute(
              "AcousticROSChannelList",
              "The list of acoustic channels associated to the simulator.",
              ObjectVectorValue(),
              MakeObjectVectorAccessor(&ROSCommsSimulator::_acousticChannels),
              MakeObjectVectorChecker<AcousticCommsChannel>());

  return tid;
}

ROSCommsSimulator::ROSCommsSimulator()
    : _linkUpdaterWorker(this) {
  _linkUpdaterLoopRate.reset(new rclcpp::Rate(std::chrono::milliseconds(10)));
  SetLogName("CommsSimulator");
  LogToConsole(true);
  // FlushLogOn(cpplogging::LogLevel::info);
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
    std::function<void(ROSCommsDeviceNs3Ptr dev, tf2::Vector3)> cb,
    double cbMinPeriod, uint32_t positionUpdateRate) {
  PositionUpdatedCb = cb;
  _positionUpdatedCbMinPeriod = cbMinPeriod;
  _updatePositionRate = positionUpdateRate;
}

void ROSCommsSimulator::_Init() {
  SetTransmitPDUCb([](ROSCommsDevice *dev, PacketPtr pdu) {});
  SetReceivePDUCb([](ROSCommsDevice *dev, PacketPtr pdu) {});
  SetPositionUpdatedCb([](ROSCommsDeviceNs3Ptr dev, tf2::Vector3 pos) {}, 1000);
  _started = false;
  _publish_rate = 10;
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
}

void ROSCommsSimulator::_AddDeviceToSet(std::string iddev,
                                        ROSCommsDeviceNs3Ptr dev) {
  _idDevMapMutex.lock();
  _dccommsDevMap[iddev] = PeekPointer(dev);
  _InsertDeviceAsc<ROSCommsDeviceNs3Ptr>(_devices, dev);

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

bool ROSCommsSimulator::_CheckDevice(const dccomms_ros_msgs::srv::CheckDevice::Request::SharedPtr req,
                                    dccomms_ros_msgs::srv::CheckDevice::Response::SharedPtr res) {
  auto iddev = req->iddev;
  res->exists = _DeviceExists(iddev);
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

bool ROSCommsSimulator::_CheckChannel(const dccomms_ros_msgs::srv::CheckChannel::Request::SharedPtr req,
                                     dccomms_ros_msgs::srv::CheckChannel::Response::SharedPtr res) {
  auto id = req->id;
  res->exists = _ChannelExists(id);
  return true;
}

ROSCommsDeviceNs3Ptr ROSCommsSimulator::_GetDevice(std::string iddev) {
  ROSCommsDeviceNs3Ptr dev;
  auto devIt = _dccommsDevMap.find(iddev);
  if (devIt != _dccommsDevMap.end()) {
    dev = devIt->second;
  }
  return dev;
}

bool ROSCommsSimulator::_RemoveDevice(const dccomms_ros_msgs::srv::RemoveDevice::Request::SharedPtr req,
                                     dccomms_ros_msgs::srv::RemoveDevice::Response::SharedPtr res) {
  return true;
}

bool ROSCommsSimulator::_AddAcousticDevice(const dccomms_ros_msgs::srv::AddAcousticDevice::Request::SharedPtr req,
                                          dccomms_ros_msgs::srv::AddAcousticDevice::Response::SharedPtr res) {
  auto dccommsId = req->dccomms_id;
  DEV_TYPE deviceType = static_cast<DEV_TYPE>(req->type);
  auto mac = req->mac;
  auto frameId = req->frame_id;
  auto refFrame = req->relative_tf_id;
  // auto energyModel = req->energy_model;

  Log->info("Add device request received");

  bool exists = _CommonPreAddDev(dccommsId, deviceType, mac);

  if (!exists) {
    auto txpb = GetPacketBuilder(dccommsId, TX_PACKET);
    if (!txpb)
      txpb = GetDefaultPacketBuilder();
    auto rxpb = GetPacketBuilder(dccommsId, RX_PACKET);
    if (!rxpb)
      rxpb = GetDefaultPacketBuilder();
    auto dev = AcousticROSCommsDevice::Build(this, txpb, rxpb);

    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetRefFrame(refFrame);
    dev->SetCodingEff(req->coding_eff);
    dev->SetInitialEnergy(req->battery_energy);
    dev->SetSymbolsPerSecond(req->symbols_per_second);
    dev->SetBitErrorRate(req->bit_error_rate);
    dev->SetBitRate(req->symbols_per_second / req->coding_eff);
    dev->SetMaxTxFifoSize(req->max_tx_fifo_size);
    dev->SetMACProtocol(req->mac_protocol);
    dev->SetRange(req->range);
    dev->SetPT(req->pt);
    dev->SetFreq(req->frequency);
    dev->SetL(req->l);
    dev->SetK(req->k);
    dev->SetTurnOnEnergy(req->turn_on_energy);
    dev->SetTurnOffEnergy(req->turn_off_energy);
    dev->SetPreamble(req->preamble);
    dev->SetTxPower(req->pt_consume);
    dev->SetRxPower(req->pr_consume);
    dev->SetIdlePower(req->p_idle);
    auto errorLevel = cpplogging::GetLevelFromString(req->log_level);
    dev->SetLogLevel(errorLevel);

    _InsertDeviceAsc<AcousticROSCommsDeviceNs3Ptr>(_acousticDevices, dev);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = PeekPointer(dev);
    _AddDeviceToSet(dev->GetDccommsId(), dev);

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev));
    res->res = true;

  } else {
    res->res = false;
  }

  return res->res;
}

bool ROSCommsSimulator::_ChannelExists(uint32_t id) {
  return _GetChannel(id) ? true : false;
}

CommsChannelNs3Ptr ROSCommsSimulator::_GetChannel(int id) {
  CommsChannelNs3Ptr channel;
  auto it = _channelMap.find(id);
  if (it != _channelMap.end()) {
    channel = it->second;
  }
  return channel;
}
bool ROSCommsSimulator::_LinkDevToChannel(const dccomms_ros_msgs::srv::LinkDeviceToChannel::Request::SharedPtr req,
                                        dccomms_ros_msgs::srv::LinkDeviceToChannel::Response::SharedPtr res) {
  ROSCommsDeviceNs3Ptr dev = _GetDevice(req->dccomms_id);
  CommsChannelNs3Ptr channel = _GetChannel(req->channel_id);
  if (!dev) {
    res->res = false;
    return res->res;
  }
  DEV_TYPE devType = dev->GetDevType();
  res->res = true;
  if (!channel) {
    res->res = false;
    return res->res;
  }

  CHANNEL_TYPE chnType = channel->GetType();
  switch (devType) {
  case ACOUSTIC_UNDERWATER_DEV: {
    if (chnType == ACOUSTIC_UNDERWATER_CHANNEL) {
    } else {
      res->res = false;
    }
    break;
  }
  case CUSTOM_DEV: {
    break;
  }
  default: { res->res = false; }
  }
  if (res->res) {
    dev->LinkToChannel(channel, (CHANNEL_LINK_TYPE)req->link_type);
    Log->info("dev {} linked to channel {}:\n{}", dev->GetDccommsId(),
              channel->GetId(), dev->ToString());
  } else {
    Log->error("error linking dev {} to channel {}", dev->GetDccommsId(),
               channel->GetId());
  }
  return res->res;
}

bool ROSCommsSimulator::_AddAcousticChannel(const dccomms_ros_msgs::srv::AddAcousticChannel::Request::SharedPtr req,
                                          dccomms_ros_msgs::srv::AddAcousticChannel::Response::SharedPtr res) {

  uint32_t id = req->id;
  res->res = false;
  if (!_channelMap[id]) {
    auto acousticChannel =
        ns3::CreateObject<dccomms_ros::AcousticCommsChannel>(id);

    acousticChannel->SetBandwidth(req->bandwidth);
    acousticChannel->SetNoiseLevel(req->noise_lvl);
    acousticChannel->SetSalinity(req->salinity);
    acousticChannel->SetTemperature(req->temperature);
    // TODO: implement acoustic channel logging
    // auto errorLevel = cpplogging::GetLevelFromString(req->logLevel);
    // acousticChannel->SetLogLevel(errorLevel);

    _channelMap[id] = PeekPointer(acousticChannel);
    _InsertChannelAsc<CommsChannelNs3Ptr>(_channels, acousticChannel);
    _InsertChannelAsc<AcousticCommsChannelNs3Ptr>(_acousticChannels,
                                                  acousticChannel);
    res->res = true;
    Log->info("acoustic channel {} added", id);
  }
  return res->res;
}

bool ROSCommsSimulator::_AddCustomChannel(const dccomms_ros_msgs::srv::AddCustomChannel::Request::SharedPtr req,
                                        dccomms_ros_msgs::srv::AddCustomChannel::Response::SharedPtr res) {
  uint32_t id = req->id;
  if (!_channelMap[id]) {
    CustomCommsChannelNs3Ptr channel =
        ns3::CreateObject<CustomCommsChannel>(id);
    channel->SetMinPrTime(req->min_pr_time);
    channel->SetPrTimeInc(req->pr_time_inc_per_meter);
    auto errorLevel = cpplogging::GetLevelFromString(req->log_level);
    channel->SetLogLevel(errorLevel);
    _InsertChannelAsc<CommsChannelNs3Ptr>(_channels, channel);
    _InsertChannelAsc<CustomCommsChannelNs3Ptr>(_customChannels, channel);
    _channelMap[id] = PeekPointer(channel);
    res->res = true;
    Log->info("custom channel {} added", id);
  } else {
    Log->error("error adding custom channel {}", id);
    res->res = false;
  }
  return res->res;
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

bool ROSCommsSimulator::_AddCustomDevice(const dccomms_ros_msgs::srv::AddCustomDevice::Request::SharedPtr req,
                                        dccomms_ros_msgs::srv::AddCustomDevice::Response::SharedPtr res) {

  auto dccommsId = req->dccomms_id;
  auto mac = req->mac;
  auto frameId = req->frame_id;
  auto refFrame = req->relative_tf_id;
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

    auto dev = CustomROSCommsDevice::Build(this, txpb, rxpb);
    dev->SetDccommsId(dccommsId);
    dev->SetMac(mac);
    dev->SetTfFrameId(frameId);
    dev->SetRefFrame(refFrame);
    dev->SetBitRate(req->bitrate);
    dev->SetMaxDistance(req->max_distance);
    dev->SetMinDistance(req->min_distance);
    dev->SetMinPktErrorRate(req->min_pkt_error_rate);
    dev->SetPktErrorRateInc(req->pkt_error_rate_inc_per_meter);
    dev->SetJitter(req->tx_jitter, req->rx_jitter);
    dev->SetMaxTxFifoSize(req->max_tx_fifo_size);
    dev->SetRateErrorModel(req->error_rate_expr, req->error_unit);
    dev->SetIntrinsicDelay(req->intrinsic_delay);

    if (req->mac_protocol != "") {
      ObjectFactory factory;
      std::string macProtocolName = GetMACPType(req->mac_protocol);
      factory.SetTypeId(macProtocolName);
      factory.Set("BitRate", DoubleValue(req->bitrate));
      factory.Set("EncodingEfficiency", DoubleValue(1));

      double macDistance =
          req->max_distance != 0 ? req->max_distance : req->max_distance;

      uint maxBackoffSlots = req->max_backoff_slots >= 4 ? req->max_backoff_slots : 4;

      dev->SetMacMaxTransmitDistance(macDistance);

      if (macProtocolName == "ns3::AquaSimSFama") {
        factory.Set("MaxBackoffSlots", IntegerValue(maxBackoffSlots));
      } else if (macProtocolName == "ns3::AquaSimFama") {
        factory.Set("RTSToNextHop", BooleanValue(true));
        factory.Set("DataPacketSize", IntegerValue(0));
        factory.Set("MaxTransmitDistance", DoubleValue(macDistance));
      } else if (macProtocolName == "ns3::AquaSimBroadcastMac") {
      } else if (macProtocolName == "ns3::AquaSimAloha") {
        factory.Set("MaxTransmitDistance", DoubleValue(macDistance));
      }
      ns3::Ptr<AquaSimMac> macLayer = factory.Create<AquaSimMac>();
      dev->SetMacLayer(macLayer);
      dev->EnableMac(true);
    } else
      dev->EnableMac(false);
    auto errorLevel = cpplogging::GetLevelFromString(req->log_level);
    dev->SetLogLevel(errorLevel);
    // dev->LogToFile(dccommsId);

    _InsertDeviceAsc<CustomROSCommsDeviceNs3Ptr>(_customDevices, dev);

    Mac2DevMapPtr mac2DevMap = _type2DevMap.find(deviceType)->second;
    (*mac2DevMap)[mac] = PeekPointer(dev);
    _AddDeviceToSet(dev->GetDccommsId(), dev);

    Log->info("\nAdding device:\n{}", dev->ToString());
    Simulator::Schedule(Seconds(0 + 0.01 * mac),
                        MakeEvent(&ROSCommsDevice::Start, dev));
    res->res = true;
  } else {
    res->res = false;
  }

  return res->res;
}

void ROSCommsSimulator::Stop() {
  auto level = Log->level();
  Log->set_level(spdlog::level::info);
  Info("Stopping ns3...");
  Simulator::Stop();
  try {
    Info("Stopping TF worker...");
    _linkUpdaterWorker.Stop();
    Info("Stopping devices...");
    for (auto dev : _devices) {
      dev->Stop();
    }
  } catch (CommsException e) {
    if (e.code != COMMS_EXCEPTION_STOPPED)
      Warn("CommsException trying to stop the network simulator: {}", e.what());
  } catch (exception e) {
    Error("Something failed when trying to stop the network simulator: {}",
          e.what());
  } catch (int e) {
    Error(
        "Something failed when trying to stop the network simulator. Code: {}",
        e);
  }
  Info("Network simulator stopped");
  Log->set_level(level);
}
void ROSCommsSimulator::StartROSInterface() {
  _rosNode = rclcpp::Node::make_shared("ros_comms_simulator");

  StartROSInterface(_rosNode);
}
void ROSCommsSimulator::StartROSInterface(std::shared_ptr<rclcpp::Node> node) {
  using std::placeholders::_1;
  using std::placeholders::_2;

  /*
   * http://www.boost.org/doc/libs/1_63_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_functions_and_fu
   */
  _rosNode = node;

  buffer.reset(new tf2_ros::Buffer(_rosNode->get_clock()));
  listener.reset(new tf2_ros::TransformListener(*buffer, _rosNode, false));

  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    _rosNode->get_node_base_interface(),
    _rosNode->get_node_timers_interface()
  );
  buffer->setCreateTimerInterface(timer_interface);
  buffer->setUsingDedicatedThread(true);

  _addDevService = _rosNode->create_service<dccomms_ros_msgs::srv::AddAcousticDevice>(
      "add_acoustic_net_device",
      std::bind(&ROSCommsSimulator::_AddAcousticDevice, this, _1, _2));
  _addChannelService = _rosNode->create_service<dccomms_ros_msgs::srv::AddAcousticChannel>(
      "add_acoustic_channel",
      std::bind(&ROSCommsSimulator::_AddAcousticChannel, this, _1, _2));
  _checkDevService = _rosNode->create_service<dccomms_ros_msgs::srv::CheckDevice>(
      "check_net_device",
      std::bind(&ROSCommsSimulator::_CheckDevice, this, _1, _2));
  _checkChannelService = _rosNode->create_service<dccomms_ros_msgs::srv::CheckChannel>(
      "check_channel",
      std::bind(&ROSCommsSimulator::_CheckChannel, this, _1, _2));
  _removeDevService = _rosNode->create_service<dccomms_ros_msgs::srv::RemoveDevice>(
      "remove_net_device",
      std::bind(&ROSCommsSimulator::_RemoveDevice, this, _1, _2));
  _linkDeviceToChannelService = _rosNode->create_service<dccomms_ros_msgs::srv::LinkDeviceToChannel>(
      "link_dev_to_channel",
      std::bind(&ROSCommsSimulator::_LinkDevToChannel, this, _1, _2));
  _startSimulationService = _rosNode->create_service<dccomms_ros_msgs::srv::StartSimulation>(
      "start_simulation",
      std::bind(&ROSCommsSimulator::_StartSimulation, this, _1, _2));
  _addCustomChannelService = _rosNode->create_service<dccomms_ros_msgs::srv::AddCustomChannel>(
      "add_custom_channel",
      std::bind(&ROSCommsSimulator::_AddCustomChannel, this, _1, _2));
  _addCustomDeviceService = _rosNode->create_service<dccomms_ros_msgs::srv::AddCustomDevice>(
      "add_custom_net_device",
      std::bind(&ROSCommsSimulator::_AddCustomDevice, this, _1, _2));
  _StartLinkUpdaterWork();
}

bool ROSCommsSimulator::_StartSimulation(const dccomms_ros_msgs::srv::StartSimulation::Request::SharedPtr req,
                                        dccomms_ros_msgs::srv::StartSimulation::Response::SharedPtr res) {
  if (!_started) {
    _Run();
    res->res = true;
    _started = true;
  } else
    res->res = false;
  return res->res;
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
  NetsimTime::Reset();
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

void ROSCommsSimulator::_StartLinkUpdaterWork() {
  _callPositionUpdatedCb = false;
  _showLinkUpdaterLogTimer.Reset();
  _linkUpdaterLoopRate.reset(new rclcpp::Rate(std::chrono::milliseconds(_updatePositionRate)));
  _linkUpdaterWorker.Start();
}

void ROSCommsSimulator::_LinkUpdaterWork() {
  if (_showLinkUpdaterLogTimer.Elapsed() > _positionUpdatedCbMinPeriod) {
    _callPositionUpdatedCb = true;
  }
  _devLinksMutex.lock();
  geometry_msgs::msg::TransformStamped transform;
  for (std::pair<const uint32_t, Mac2DevMapPtr> type2Devs : _type2DevMap) {
    Mac2DevMapPtr mac2DevMap = type2Devs.second;
    for (std::pair<const uint32_t, ROSCommsDevicePtr> mac2Dev : *mac2DevMap) {
      ROSCommsDevicePtr dev = mac2Dev.second;
      std::string tfFrameId = dev->GetTfFrameId();
      std::string refFrame = dev->GetRefFrame();
      try {
        std::string warning_msg;

        if (!buffer->canTransform(refFrame, tfFrameId, tf2::TimePoint(), &warning_msg)) {
          Log->warn("Waiting for transform {} -> {}: {}", refFrame, tfFrameId, warning_msg);
        } else {
          transform = buffer->lookupTransform(refFrame, tfFrameId, rclcpp::Time(0));

          tf2::Vector3 position(transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z); 
          dev->SetPosition(position);
          if (_callPositionUpdatedCb)
            PositionUpdatedCb(dev, position);
        }
      } catch (std::exception &e) {
        if (_callPositionUpdatedCb)
          Log->warn("An exception has ocurred in the link updater work: {}",
                    std::string(e.what()));
      }
    }
  }
  _devLinksMutex.unlock();
  _linkUpdaterLoopRate->sleep();

  if (_callPositionUpdatedCb) {
    _callPositionUpdatedCb = false;
    _showLinkUpdaterLogTimer.Reset();
  }
}

bool ROSCommsSimulator::AddAcousticDevice(const dccomms_ros_msgs::srv::AddAcousticDevice::Request::SharedPtr req) {
  dccomms_ros_msgs::srv::AddAcousticDevice::Response::SharedPtr res(new dccomms_ros_msgs::srv::AddAcousticDevice::Response);
  return _AddAcousticDevice(req, res);
}
bool ROSCommsSimulator::LinkDevToChannel(const dccomms_ros_msgs::srv::LinkDeviceToChannel::Request::SharedPtr req) {
  dccomms_ros_msgs::srv::LinkDeviceToChannel::Response::SharedPtr res(new dccomms_ros_msgs::srv::LinkDeviceToChannel::Response);
  return _LinkDevToChannel(req, res);
}
bool ROSCommsSimulator::AddAcousticChannel(const dccomms_ros_msgs::srv::AddAcousticChannel::Request::SharedPtr req) {
  dccomms_ros_msgs::srv::AddAcousticChannel::Response::SharedPtr res(new dccomms_ros_msgs::srv::AddAcousticChannel::Response);
  return _AddAcousticChannel(req, res);
}
bool ROSCommsSimulator::AddCustomChannel(const dccomms_ros_msgs::srv::AddCustomChannel::Request::SharedPtr req) {
  dccomms_ros_msgs::srv::AddCustomChannel::Response::SharedPtr res(new dccomms_ros_msgs::srv::AddCustomChannel::Response);
  return _AddCustomChannel(req, res);
}
bool ROSCommsSimulator::AddCustomDevice(const dccomms_ros_msgs::srv::AddCustomDevice::Request::SharedPtr req) {
  dccomms_ros_msgs::srv::AddCustomDevice::Response::SharedPtr res(new dccomms_ros_msgs::srv::AddCustomDevice::Response);
  return _AddCustomDevice(req, res);
}
bool ROSCommsSimulator::StartSimulation() {
  dccomms_ros_msgs::srv::StartSimulation::Request::SharedPtr req(new dccomms_ros_msgs::srv::StartSimulation::Request);
  dccomms_ros_msgs::srv::StartSimulation::Response::SharedPtr res(new dccomms_ros_msgs::srv::StartSimulation::Response);
  return _StartSimulation(req, res);
}
} // namespace dccomms_ros
