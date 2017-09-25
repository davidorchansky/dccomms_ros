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

using namespace dccomms;
using namespace dccomms_ros_msgs;

namespace dccomms_ros {
ROSCommsSimulator::ROSCommsSimulator(ros::NodeHandle &rosNode)
    : _rosNode(rosNode), _linkUpdaterWorker(this) {
  SetLogName("CommsSimulator");
  LogToConsole(true);
  FlushLogOn(cpplogging::LogLevel::info);
  _linkUpdaterWorker.SetWork(&ROSCommsSimulator::_LinkUpdaterWork);
  _Init();
}

void ROSCommsSimulator::SetTransmitPDUCb(
    std::function<void(int, DataLinkFramePtr)> cb) {
  _TransmitPDUCb = cb;
}

void ROSCommsSimulator::SetReceivePDUCb(
    std::function<void(int, DataLinkFramePtr)> cb) {
  _ReceivePDUCb = cb;
}

void ROSCommsSimulator::SetErrorPDUCb(
    std::function<void(int, DataLinkFramePtr)> cb) {
  _ErrorPDUCb = cb;
}

void ROSCommsSimulator::_Init() {
  _TransmitPDUCb = [](int linkType, DataLinkFramePtr pdu) {};
  _ReceivePDUCb = [](int linkType, DataLinkFramePtr pdu) {};
  _ErrorPDUCb = [](int linkType, DataLinkFramePtr pdu) {};
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

CommsChannelStatePtr ROSCommsSimulator::_GetChannel(std::string channelKey) {
  CommsChannelStatePtr res;
  _channelsMutex.lock();
  res = _channels[channelKey];
  _channelsMutex.unlock();
  return res;
}

void ROSCommsSimulator::TransmitFrame(int linkType, DataLinkFramePtr dlf) {
  auto srcdir = dlf->GetSrcDir();
  auto dstdir = dlf->GetDesDir();

  // Build channel key
  std::stringstream ss;
  ss << (int)linkType << "_" << (int)srcdir << "_" << (int)dstdir;
  std::string channelKey = ss.str();

  auto channelState = _GetChannel(channelKey);
  if (channelState) {
    auto frameSize = dlf->GetFrameSize();

    auto txtrp = TransportPDU::BuildTransportPDU(dlf->GetPayloadBuffer());
    auto delay = channelState->GetDelay();

    // Simulate total delay (transmission + propagation + reception)
    auto trRate = channelState->GetNextTt();
    if (trRate < 0) {
      Log->warn("trRate < 0: {} . Changing to its abs({}) value", trRate,
                trRate);
      trRate = -trRate;
    }
    auto trTime = trRate * frameSize;
    int totalTime = ceil(delay + trTime);
    _TransmitPDUCb(linkType, dlf);
    Log->debug(
        "TX {}->{}: R: {} ms/byte ; TT: {} ms ; D: {} ms (Seq: {}) (FS: {}).",
        srcdir, dstdir, trRate, totalTime, delay, txtrp->GetSeqNum(),
        frameSize);

    if (channelState->LinkOk()) {
      if (channelState->ErrOnNextPkt()) {
        auto pBuffer = dlf->GetPayloadBuffer();
        *pBuffer = ~*pBuffer;
      }
      _PropagateFrame(dlf, totalTime, channelState);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds((int)round(trTime)));
  }
}

void ROSCommsSimulator::_PropagateFrame(DataLinkFramePtr dlf, int delay,
                                        CommsChannelStatePtr channel) {
  std::thread task([this, delay, dlf, channel]() {
    dccomms::Timer timer;
    timer.Reset();
    channel->Lock();

    if (delay < 0)
      Log->critical("BUG! delay < 0!!");

    auto elapsed = timer.Elapsed();

    // We don't want to make a pdu arrives before a previous pdu that is still
    // propagating
    auto delay2 = elapsed < delay ? delay - elapsed : 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(delay2));
    this->_DeliverFrame(dlf, channel);
    channel->Unlock();
  });
  task.detach();
}

void ROSCommsSimulator::_DeliverFrame(DataLinkFramePtr dlf,
                                      CommsChannelStatePtr channel) {
  auto dstdir = dlf->GetDesDir();

  auto trs = TransportPDU::BuildTransportPDU(dlf->GetPayloadBuffer());
  auto rxNode = channel->GetRxNode();
  auto devType = rxNode->GetDevType();
  if (dlf->checkFrame()) {
    ROSCommsDevicePtr dstNode = (*_nodes[devType])[dstdir];
    _ReceivePDUCb(devType, dlf);
    Log->debug("RX {}<-{}: received frame without errors (Seq: {}) (FS: {}).",
               dlf->GetDesDir(), dlf->GetSrcDir(), trs->GetSeqNum(),
               dlf->GetFrameSize());
    dstNode->ReceiveFrame(dlf);
  } else {
    _ErrorPDUCb(devType, dlf);
    Log->warn("RX {}<-{}: received frame with errors. Frame will be discarted "
              "(Seq: {}) (FS: {}).",
              dlf->GetDesDir(), dlf->GetSrcDir(), trs->GetSeqNum(),
              dlf->GetFrameSize());
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
  _idDevMapMutex.lock();
  exists = _idDevMap.find(iddev) != _idDevMap.end();
  _idDevMapMutex.unlock();
  return exists;
}

ROSCommsDevicePtr ROSCommsSimulator::_GetDevice(std::string iddev) {
  ROSCommsDevicePtr dev;
  _idDevMapMutex.lock();
  auto iterator = _idDevMap.find(iddev);
  if (iterator != _idDevMap.end()) {
    dev = iterator->second;
    _idDevMap.erase(iterator);
  }
  _idDevMapMutex.unlock();
  return dev;
}

bool ROSCommsSimulator::_RemoveDevice(RemoveDevice::Request &req,
                                      RemoveDevice::Response &res) {
  auto iddev = req.iddev;
  auto exists = _DeviceExists(iddev);

  if (exists) {
    auto dev = _GetDevice(iddev);
    auto mac = dev->GetMac();
    auto devType = dev->GetDevType();

    _RemoveDeviceFromSet(iddev);

    std::stringstream ss;
    ss << (int)devType << "_" << (int)mac << "_(\\d*)";
    std::string txChannelPattern = ss.str();

    ss.str(std::string());
    ss << (int)devType << "_(\\d*)_" << (int)mac;
    std::string rxChannelPattern = ss.str();

    ss.str(std::string());
    ss << "^((" << txChannelPattern << ")|(" << rxChannelPattern << "))$";
    std::string channelPattern = ss.str();

    std::regex channelReg(channelPattern);

    std::list<std::string> channelKeys;

    for (auto pair : _channels) {
      auto key = pair.first;
      bool match = std::regex_match(key, channelReg);
      if (match) {
        channelKeys.push_back(key);
      }
    }

    for (auto key : channelKeys) {
      _RemoveChannel(key);
    }

    _devLinksMutex.lock();
    _devLinks.remove_if([iddev](DevicesLink devLink) {
      return devLink.device0->GetName() == iddev ||
             devLink.device1->GetName() == iddev;
    });
    _devLinksMutex.unlock();

    _nodes[devType]->erase(mac);

    res.removed = true;
  } else {
    res.removed = false;
  }
  return true;
}

void ROSCommsSimulator::_RemoveChannel(std::string channelKey) {
  _channelsMutex.lock();
  _channels.erase(channelKey);
  _channelsMutex.unlock();
}

bool ROSCommsSimulator::_AddDevice(AddDevice::Request &req,
                                   AddDevice::Response &res) {
  auto newDevice = req.iddev;
  auto mac = req.mac;
  auto trTimeMean = req.trTimeMean;
  auto trTimeSd = req.trTimeSd;
  auto prTime = req.minPrTime;
  auto prTimeIncPerMeter = req.prTimeIncPerMeter;
  auto minPktErrorRate = req.minPktErrorRate;
  auto pktErrorRateIncPerMeter = req.pktErrorRateIncPerMeter;
  auto deviceType = req.devType;
  auto frameId = req.frameId;
  auto maxDistance = req.maxDistance;
  auto minDistance = req.minDistance;

  Log->info("Add device request received");

  bool exists = false;
  auto nodeMapIt = _nodes.find(deviceType);
  if (nodeMapIt != _nodes.end()) {
    NodeMapPtr nodeMap = nodeMapIt->second;
    auto nodeIt = nodeMap->find(mac);
    if (nodeIt != nodeMap->end()) {
      exists = true;
    }
  } else {
    NodeMapPtr nodeMap(new NodeMap());
    _nodes[deviceType] = nodeMap;
  }

  if (!exists) {
    auto commsDeviceService =
        dccomms::CommsDeviceService::BuildCommsDeviceService(
            IPHY_TYPE_PHY, dccomms::DataLinkFrame::crc16);

    commsDeviceService->SetNamespace(newDevice);
    auto node = ROSCommsDevice::BuildCommsDevice(this, commsDeviceService);
    node->SetName(newDevice);
    node->SetDevType(deviceType);
    node->SetMac(mac);
    node->SetTrTime(trTimeMean, trTimeSd);
    node->SetMinPrTime(prTime);
    node->SetPrTimeInc(prTimeIncPerMeter);
    node->SetMinPktErrorRate(minPktErrorRate);
    node->SetPktErrorRateInc(pktErrorRateIncPerMeter);
    node->SetTfFrameId(frameId);
    node->SetMaxDistance(maxDistance);
    node->SetMinDistance(minDistance);

    Log->info("\nAdding new device...:\n{}", node->ToString());

    NodeMapPtr nodes = _nodes.find(deviceType)->second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto rxNode = pair.second;

      auto remoteMac = pair.first;

      Log->info("Creating a link from MAC {} to MAC {}", mac, remoteMac);

      // Build tx channel
      std::stringstream ss;
      ss << (int)deviceType << "_" << (int)mac << "_" << (int)remoteMac;
      std::string txChannelKey = ss.str();

      auto txChannel = CommsChannelState::BuildCommsChannelState();
      txChannel->SetDelay(prTime);
      txChannel->SetLinkOk(true);
      txChannel->SetTtDist(trTimeMean, trTimeSd);

      txChannel->SetTxNode(node);
      txChannel->SetRxNode(pair.second);

      // Build rx channel
      ss.str(std::string());
      ss << (int)deviceType << "_" << (int)remoteMac << "_" << (int)mac;
      std::string rxChannelKey = ss.str();

      auto rxChannel = CommsChannelState::BuildCommsChannelState();
      rxChannel->SetDelay(rxNode->GetMinPrTime());
      rxChannel->SetLinkOk(true);
      float rxTrTimeMean, rxTrTimeSd;
      rxNode->GetTrTime(rxTrTimeMean, rxTrTimeSd);
      rxChannel->SetTtDist(rxTrTimeMean, rxTrTimeSd);

      rxChannel->SetTxNode(rxNode);
      rxChannel->SetRxNode(node);

      _channels[txChannelKey] = txChannel;
      _channels[rxChannelKey] = rxChannel;

      DevicesLink devLink(node, rxNode, txChannel, rxChannel);

      _devLinksMutex.lock();
      _devLinks.push_back(devLink);
      _devLinksMutex.unlock();
    }
    (*nodes)[mac] = node;

    auto starterWork = [](ROSCommsDevicePtr _node) {
      _node->StartDeviceService();
      _node->StartNodeWorker();
    };

    _AddDeviceToSet(newDevice, node);

    std::thread starter(starterWork, node);
    starter.detach();
    res.res = true;

  } else {
    res.res = false;
    // ROSCommsDevicePtr node = _nodes[mac];
    // Log->error("Unable to add the device. A net device with the same MAC
    // already exists: '{}'", node->GetName ());
  }

  return res.res;
}

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
}

void ROSCommsSimulator::SetLogName(std::string name) {
  Loggable::SetLogName(name);
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->SetLogName(name + ":Node(" + node->GetName() + ")");
    }
  }
}

void ROSCommsSimulator::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->SetLogLevel(_level);
    }
  }
}

void ROSCommsSimulator::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->LogToConsole(c);
    }
  }
}

void ROSCommsSimulator::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->LogToFile(filename + "_" + node->GetName() + "_node");
    }
  }
}

void ROSCommsSimulator::FlushLog() {
  Loggable::FlushLog();
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->FlushLog();
    }
  }
}

void ROSCommsSimulator::FlushLogOn(LogLevel level) {
  Loggable::FlushLogOn(level);
  for (pair<int, NodeMapPtr> tPair : _nodes) {
    NodeMapPtr nodes = tPair.second;
    for (pair<int, ROSCommsDevicePtr> pair : *nodes) {
      auto node = pair.second;
      node->FlushLogOn(level);
    }
  }
}

void ROSCommsSimulator::_UpdateChannelStateFromRange(CommsChannelStatePtr chn,
                                                     double distance,
                                                     bool log) {
  auto txDev = chn->GetTxNode();
  auto rxDev = chn->GetRxNode();

  auto txFrameId = txDev->GetTfFrameId();
  auto rxFrameId = rxDev->GetTfFrameId();

  auto minPrTime = txDev->GetMinPrTime();
  auto prTimeInc = txDev->GetPrTimeInc() * distance;
  auto newDelay = minPrTime + prTimeInc;
  chn->SetDelay(newDelay);

  auto linkType = txDev->GetDevType();

  auto txMac = txDev->GetMac();
  auto rxMac = rxDev->GetMac();

  if (log)
    Log->debug(
        "class {}: mac({})frame('{}') ==> mac({})frame('{}') delay: {} ms",
        linkType, txMac, txFrameId, rxMac, rxFrameId, newDelay);

  auto minErrRate = txDev->GetMinPktErrorRate();
  auto errRateInc = txDev->GetPktErrorRateInc();
  auto newErrRate = minErrRate + errRateInc * distance;
  chn->SetErrRate(newErrRate);

  if (log)
    Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') packet "
               "error rate: {}%",
               linkType, txMac, txFrameId, rxMac, rxFrameId, newErrRate * 100);

  double minDistance = txDev->GetMinDistance() / 100.; // cm to meters
  double maxDistance = txDev->GetMaxDistance() / 100.; // cm to meters
  if (distance <= maxDistance && distance >= minDistance) {
    chn->SetLinkOk(true);
    if (log)
      Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') link ok",
                 linkType, txMac, txFrameId, rxMac, rxFrameId);
  } else {
    chn->SetLinkOk(false);
    if (log)
      Log->debug(
          "class {}: mac({})frame('{}') ==> mac({})frame('{}') link down",
          linkType, txMac, txFrameId, rxMac, rxFrameId);
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
    for (DevicesLink link : _devLinks) {
      auto chn0 = link.channel0;
      auto chn1 = link.channel1;
      double distance = 0;
      try {
        frameId0 = link.device0->GetTfFrameId();
        frameId1 = link.device1->GetTfFrameId();

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

      if (chn0) {
        _UpdateChannelStateFromRange(chn0, distance, showLog);
      }
      if (chn1) {
        _UpdateChannelStateFromRange(chn1, distance, showLog);
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
