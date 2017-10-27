#include <dccomms/CommsDeviceService.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
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
    : _rosNode(rosNode), _linkUpdaterWorker(this) {
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
void ROSCommsSimulator::SetGetSrcAddrFunc(
    std::function<int(dccomms::PacketPtr)> cb) {
  _getSrcAddr = cb;
}

void ROSCommsSimulator::SetGetDstAddrFunc(
    std::function<int(dccomms::PacketPtr)> cb) {
  _getDstAddr = cb;
}

void ROSCommsSimulator::SetIsBroadcastFunc(std::function<bool(int)> fc) {
  _isBroadcast = fc;
}

bool ROSCommsSimulator::_IsBroadcast(int addr) { return _isBroadcast(addr); }

void ROSCommsSimulator::SetErrorPDUCb(std::function<void(int, PacketPtr)> cb) {
  _ErrorPDUCb = cb;
}

void ROSCommsSimulator::_Init() {
  _TransmitPDUCb = [](int linkType, PacketPtr pdu) {};
  _ReceivePDUCb = [](int linkType, PacketPtr pdu) {};
  _ErrorPDUCb = [](int linkType, PacketPtr pdu) {};
  _getDstAddr = [](PacketPtr pkt) { return 0; };
  _getDstAddr = [](PacketPtr pkt) { return 0; };
  _isBroadcast = [](int addr) { return true; };
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
}

void ROSCommsSimulator::_AddDeviceToSet(std::string iddev,
                                        ROSCommsDevicePtr dev) {
  _idDevMapMutex.lock();
  _idDevMap[iddev] = dev;
  _idDevMapMutex.unlock();
}

void ROSCommsSimulator::_RemoveDeviceFromSet(std::string iddev) {
  _idDevMapMutex.lock();
  auto iter = _idDevMap.find(iddev);
  if (iter != _idDevMap.end()) {
    _idDevMap.erase(iter);
  }
  _idDevMapMutex.unlock();
}

VirtualDeviceLinkPtr ROSCommsSimulator::_GetChannel(std::string channelKey) {
  VirtualDeviceLinkPtr res;
  _channelsMutex.lock();
  res = _channels[channelKey];
  _channelsMutex.unlock();
  return res;
}

int ROSCommsSimulator::_GetSrcAddr(PacketPtr pkt) { return _getSrcAddr(pkt); }

int ROSCommsSimulator::_GetDstAddr(PacketPtr pkt) { return _getDstAddr(pkt); }

void ROSCommsSimulator::TransmitFrame(ROSCommsDevicePtr dev, PacketPtr dlf) {}

void ROSCommsSimulator::_PropagateFrame(PacketPtr dlf, int delay,
                                        VirtualDeviceLinkPtr channel) {

  Simulator::Schedule(
      MilliSeconds(delay),
      MakeEvent(&ROSCommsSimulator::_DeliverFrame, this, dlf, channel));
}

void ROSCommsSimulator::_DeliverFrame(PacketPtr dlf,
                                      VirtualDeviceLinkPtr channel) {
  auto dstdir = channel->GetDevice0()->GetMac();

  auto rxNode = channel->GetDevice1();
  auto devType = rxNode->GetDevType();
  if (dlf->PacketIsOk()) {
    ROSCommsDevicePtr dstNode = (*_nodeTypeMap[devType])[dstdir];
    _ReceivePDUCb(devType, dlf);
    dstNode->ReceiveFrame(dlf);
  } else {
    _ErrorPDUCb(devType, dlf);
  }
}

bool ROSCommsSimulator::_CheckDevice(CheckDevice::Request &req,
                                     CheckDevice::Response &res) {
  auto iddev = req.iddev;
  res.exists = _DeviceExists(iddev);
  return true;
}

bool ROSCommsSimulator::_DeviceExists(std::string iddev) {
  bool exists;
  return exists;
}

ROSCommsDevicePtr ROSCommsSimulator::_GetDevice(std::string iddev) {
  ROSCommsDevicePtr dev;
  return dev;
}

bool ROSCommsSimulator::_RemoveDevice(RemoveDevice::Request &req,
                                      RemoveDevice::Response &res) {

  return true;
}

void ROSCommsSimulator::_RemoveChannel(std::string channelKey) {
  _channelsMutex.lock();
  _channels.erase(channelKey);
  _channelsMutex.unlock();
}

bool ROSCommsSimulator::_AddDevice(AddDevice::Request &req,
                                   AddDevice::Response &res) {}

void ROSCommsSimulator::Start() {
  /*
   * http://www.boost.org/doc/libs/1_63_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_functions_and_fu
   */
  _addDevService = _rosNode.advertiseService(
      "add_net_device", &ROSCommsSimulator::_AddDevice, this);
  _checkDevService = _rosNode.advertiseService(
      "check_net_device", &ROSCommsSimulator::_CheckDevice, this);
  _removeDevService = _rosNode.advertiseService(
      "remove_net_device", &ROSCommsSimulator::_RemoveDevice, this);
  _linkUpdaterWorker.Start();
  std::thread task([]() { Simulator::Run(); });
  task.detach();
}

void ROSCommsSimulator::SetLogName(std::string name) {
  Loggable::SetLogName(name);
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->SetLogName(name + ":Node(" + node->GetName() + ")");
    }
  }
}

void ROSCommsSimulator::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->SetLogLevel(_level);
    }
  }
}

void ROSCommsSimulator::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->LogToConsole(c);
    }
  }
}

void ROSCommsSimulator::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->LogToFile(filename + "_" + node->GetName() + "_node");
    }
  }
}

void ROSCommsSimulator::FlushLog() {
  Loggable::FlushLog();
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->FlushLog();
    }
  }
}

void ROSCommsSimulator::FlushLogOn(cpplogging::LogLevel level) {
  Loggable::FlushLogOn(level);
  for (pair<int, NodeMapPtr> tPair : _nodeTypeMap) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->FlushLogOn(level);
    }
  }
}

void ROSCommsSimulator::_UpdateChannelStateFromRange(VirtualDeviceLinkPtr chn,
                                                     double distance,
                                                     bool log) {
  auto dev0 = chn->GetDevice0();
  auto dev1 = chn->GetDevice1();

  auto dev0FrameId = dev0->GetTfFrameId();
  auto dev1FrameId = dev1->GetTfFrameId();

  if (log) {
    // TODO: show log
  }
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
      _UpdateChannelStateFromRange(link, distance, showLog);
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
