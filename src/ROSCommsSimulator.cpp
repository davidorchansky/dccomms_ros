#include <dccomms_ros/ROSCommsSimulator.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms_ros/AddDevice.h>
#include <iostream>
#include <tf/transform_listener.h>

using namespace dccomms;

namespace dccomms_ros
{
ROSCommsSimulator::ROSCommsSimulator(ros::NodeHandle & rosNode): _rosNode(rosNode), _linkUpdaterWorker(this)
{
    SetLogName ("CommsSimulator");
    LogToConsole (true);
    FlushLogOn (cpplogging::Loggable::info);
    _linkUpdaterWorker.SetWork (&ROSCommsSimulator::_LinkUpdaterWork);
}

void ROSCommsSimulator::TransmitFrame(DataLinkFramePtr dlf)
{
    auto srcdir = dlf->GetSrcDir ();
    auto dstdir = dlf->GetDesDir ();

    std::string channelKey =
            std::to_string (srcdir) + std::string("_") + std::to_string (dstdir);

    auto channelState = _channels[channelKey];
    if(channelState)
    {
      auto bitRate = channelState->GetMaxBitRate ();

      //Simulate transmission time
      auto byteTransmissionTime =  1000./( bitRate/ 8.);
      auto frameSize = dlf->GetFrameSize ();
      unsigned int frameTransmissionTime = ceil(frameSize * byteTransmissionTime);
      auto txtrp = TransportPDU::BuildTransportPDU (0);
      txtrp->UpdateBuffer (dlf->GetPayloadBuffer ());
      auto delay = channelState->GetDelay ();

      //Simulate total delay (transmission + propagation + reception)
      auto trRate = channelState->GetNextTt();
      auto trTime = trRate * frameSize;
      int totalTime = ceil(delay + trTime);
      Log->debug("TX {}->{}: R: {} ms/byte ; TT: {} ms ; D: {} ms (Seq: {}) (FS: {}).",
                 srcdir, dstdir,
                 trRate,
                 totalTime,
                 delay,
                 txtrp->GetSeqNum (),
                 frameSize);

      if(channelState->LinkOk())
      {
        if(channelState->ErrOnNextPkt())
        {
          auto pBuffer = dlf->GetPayloadBuffer();
          *pBuffer = ~*pBuffer;
        }
        _PropagateFrame (dlf, totalTime);
      }

      std::this_thread::sleep_for(
                  std::chrono::milliseconds(frameTransmissionTime));
   }
}

void ROSCommsSimulator::_PropagateFrame (DataLinkFramePtr dlf, int delay)
{
    std::thread task(
                [this, delay, dlf]()
    {
        std::this_thread::sleep_for(
                    std::chrono::milliseconds(delay)
                    );
        this->_DeliverFrame (dlf);
    });
    task.detach ();
}

void ROSCommsSimulator::_DeliverFrame (DataLinkFramePtr dlf)
{
    auto dstdir = dlf->GetDesDir ();

    auto trs = TransportPDU::BuildTransportPDU (0);
    trs->UpdateBuffer (dlf->GetPayloadBuffer ());
    if(dlf->checkFrame ())
    {
        CommsDevicePtr dstNode = _nodes[dstdir];
        Log->debug("RX {}<-{}: received frame without errors (Seq: {}) (FS: {}).",
               dlf->GetDesDir (),
               dlf->GetSrcDir (),
               trs->GetSeqNum (),
               dlf->GetFrameSize());
        dstNode->ReceiveFrame (dlf);
    }
    else
    {
        Log->debug("RX {}<-{}: received frame with errors. Frame will be discarted (Seq: {}) (FS: {}).",
               dlf->GetDesDir (),
               dlf->GetSrcDir (),
               trs->GetSeqNum (),
               dlf->GetFrameSize());
    }
}

bool ROSCommsSimulator::_AddDevice (AddDevice::Request &req, AddDevice::Response &res)
{
    auto newDevice = req.iddev;
    auto mac = req.mac;
    auto maxBitRate              = req.maxBitRate;
    auto trTimeMean              = req.trTimeMean;
    auto trTimeSd                = req.trTimeSd;
    auto prTime                  = req.minPrTime;
    auto prTimeIncPerMeter       = req.prTimeIncPerMeter;
    auto minPktErrorRate         = req.minPktErrorRate;
    auto pktErrorRateIncPerMeter = req.pktErrorRateIncPerMeter;
    auto deviceType              = req.devType;
    auto frameId                 = req.frameId;
    auto maxDistance             = req.maxDistance;
    auto minDistance             = req.minDistance;

    Log->info("Add device request received");

    if(_nodes.find(mac) == _nodes.end())
    {
        auto commsDeviceService = dccomms::CommsDeviceService::BuildCommsDeviceService (
                    IPHY_TYPE_PHY,
                    dccomms::DataLinkFrame::crc16
                    );

        commsDeviceService->SetNamespace (newDevice);
        auto node =  ROSCommsDevice::BuildCommsDevice(this, commsDeviceService);
        node->SetName(newDevice);
        node->SetDevType (deviceType);
        node->SetMac (mac);
        node->SetMaxBitRate (maxBitRate);
        node->SetTrTime (trTimeMean, trTimeSd);
        node->SetMinPrTime (prTime);
        node->SetPrTimeInc (prTimeIncPerMeter);
        node->SetMinPktErrorRate (minPktErrorRate);
        node->SetPktErrorRateInc (pktErrorRateIncPerMeter);
        node->SetTfFrameId (frameId);
        node->SetMaxDistance (maxDistance);
        node->SetMinDistance (minDistance);

        Log->info("\nAdding new device...:\n{}", node->ToString ());

        for(pair<int, CommsDevicePtr> pair: _nodes)
        {
            auto rxNode = pair.second;

            if(rxNode->GetDevType () == deviceType)
            {
                auto remoteMac = pair.first;

                Log->info("Creating a link from MAC {} to MAC {}",
                          mac, remoteMac);

                //Build tx channel
                std::stringstream ss;
                ss <<  (int)mac << "_" <<  (int)remoteMac;
                std::string txChannelKey = ss.str();

                auto txChannel = CommsChannelState::BuildCommsChannelState ();
                txChannel->SetDelay (prTime);
                txChannel->SetLinkOk (true);
                txChannel->SetMaxBitRate (maxBitRate);
                txChannel->SetTtDist (trTimeMean, trTimeSd);

                txChannel->SetTxNode(node);
                txChannel->SetRxNode(pair.second);

                //Build rx channel
                ss.str(std::string());
                ss << (int)remoteMac << "_" << (int)mac;
                std::string rxChannelKey = ss.str();

                auto rxChannel = CommsChannelState::BuildCommsChannelState ();
                rxChannel->SetDelay (rxNode->GetMinPrTime ());
                rxChannel->SetLinkOk (true);
                rxChannel->SetMaxBitRate (rxNode->GetMaxBitRate ());
                float rxTrTimeMean, rxTrTimeSd;
                rxNode->GetTrTime (rxTrTimeMean, rxTrTimeSd);
                rxChannel->SetTtDist (rxTrTimeMean, rxTrTimeSd);

                rxChannel->SetTxNode(rxNode);
                rxChannel->SetRxNode(node);

                _channels[txChannelKey] = txChannel;
                _channels[rxChannelKey] = rxChannel;

                DevicesLink devLink(node, rxNode, txChannel, rxChannel);

                _devLinksMutex.lock();
                _devLinks.push_back (devLink);
                _devLinksMutex.unlock();
            }

        }
        _nodes[mac] = node;

        node->StartDeviceService ();
        node->StartNodeWorker ();
    }
    else
    {
        CommsDevicePtr node = _nodes[mac];
        Log->error("Unable to add the device. A net device with the same MAC already exists: '{}'", node->GetName ());
    }

    res.res = true;
    return true;
}

void ROSCommsSimulator::Start()
{
    /*
     * http://www.boost.org/doc/libs/1_63_0/libs/bind/doc/html/bind.html#bind.purpose.using_bind_with_functions_and_fu
     */
    _addDevService = _rosNode.advertiseService(
                "add_net_device",
                &ROSCommsSimulator::_AddDevice, this
                );
    _linkUpdaterWorker.Start();

    /*
    //Start device services first
    for(pair<int, CommsNodePtr> pair: nodes)
    {
        auto node = pair.second;
        node->StartDeviceService ();
    }
    //Start simulated comms
    for(pair<int, CommsNodePtr> pair: nodes)
    {
        auto node = pair.second;
        node->StartNodeWorker ();
    }
    */
}
void ROSCommsSimulator::SetLogName(std::string name)
{
    Loggable::SetLogName(name);
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->SetLogName(name + ":Node(" + node->GetName()+")");
    }
}
void ROSCommsSimulator::SetLogLevel(Loggable::LogLevel _level)
{
    Loggable::SetLogLevel(_level);
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->SetLogLevel(_level);
    }
}

void ROSCommsSimulator::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->LogToConsole(c);
    }

}

void ROSCommsSimulator::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->LogToFile(filename + "_" + node->GetName () + "_node");
    }
}

void ROSCommsSimulator::FlushLog()
{
    Loggable::FlushLog();
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->FlushLog ();
    }
}

void ROSCommsSimulator::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    for(pair<int, CommsDevicePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->FlushLogOn(level);
    }
}

void ROSCommsSimulator::_UpdateChannelStateFromRange(CommsChannelStatePtr chn, double distance, bool log)
{
  auto txDev = chn->GetTxNode ();
  auto rxDev = chn->GetRxNode ();

  auto txFrameId = txDev->GetTfFrameId ();
  auto rxFrameId = rxDev->GetTfFrameId ();

  auto minPrTime = txDev->GetMinPrTime ();
  auto prTimeInc = txDev->GetPrTimeInc () * distance;
  auto newDelay = minPrTime + prTimeInc;
  chn->SetDelay (newDelay);

  auto linkType = txDev->GetDevType ();

  auto txMac = txDev->GetMac();
  auto rxMac = rxDev->GetMac();

  if(log)
  Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') delay: {} ms",
           linkType,
           txMac,
           txFrameId,
           rxMac,
           rxFrameId,
           newDelay);

  auto minErrRate = txDev->GetMinPktErrorRate ();
  auto errRateInc = txDev->GetPktErrorRateInc ();
  auto newErrRate = minErrRate + errRateInc * distance;
  chn->SetErrRate (newErrRate);

  if(log)
  Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') packet error rate: {}%",
           linkType,
           txMac,
           txFrameId,
           rxMac,
           rxFrameId,
           newErrRate*100);

  double minDistance = txDev->GetMinDistance () / 100.; //cm to meters
  double maxDistance = txDev->GetMaxDistance () / 100.; //cm to meters
  if(distance <= maxDistance && distance >= minDistance)
  {
     chn->SetLinkOk (true);
     if(log)Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') link ok",
              linkType,
              txMac,
              txFrameId,
              rxMac,
              rxFrameId);
  }
  else
  {
    chn->SetLinkOk(false);
    if(log)Log->debug("class {}: mac({})frame('{}') ==> mac({})frame('{}') link down",
             linkType,
             txMac,
             txFrameId,
             rxMac,
             rxFrameId);
  }

}

void ROSCommsSimulator::_LinkUpdaterWork()
{
  tf::TransformListener listener;

  dccomms::Timer  timer;
  bool showLog = false;
  unsigned int showLogInterval = 1500;
  timer.Reset();

  ros::Rate loop_rate(20);
  while(1)
  {
    try
    {
      if(timer.Elapsed () > showLogInterval)
      {
        showLog = true;
      }

      _devLinksMutex.lock();

      tf::StampedTransform transform;
      for(DevicesLink link : _devLinks)
      {
        auto frameId0 = link.device0->GetTfFrameId ();
        auto frameId1 = link.device1->GetTfFrameId ();

        ros::Time now = ros::Time::now();

        listener.lookupTransform (frameId0, frameId1,
                                  ros::Time(0),
                                  transform);
        auto distance = transform.getOrigin ().distance(tf::Vector3(0,0,0));
        if(showLog)
        Log->debug("Range between frame '{}' and '{}': {}",
                  frameId0,
                  frameId1,
                  distance);

        auto chn = link.channel0;

        if(chn)
        {
           _UpdateChannelStateFromRange (chn, distance, showLog);
        }

        chn = link.channel1;

        if(chn)
        {
           _UpdateChannelStateFromRange (chn, distance, showLog);
        }

        if(showLog)
        {
          showLog = false;
          timer.Reset();
        }
      }

      _devLinksMutex.unlock();
      loop_rate.sleep();
    }
    catch(std::exception & e)
    {
      Log->critical("Han exception has ocurred in the link updater work");
    }
  }

}
}
