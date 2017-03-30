#include <dccomms_ros/ROSCommsSimulator.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms_ros/AddDevice.h>
#include <iostream>

using namespace dccomms;

namespace dccomms_ros
{
ROSCommsSimulator::ROSCommsSimulator(ros::NodeHandle & rosNode): _rosNode(rosNode)
{
    SetLogName ("CommsSimulator");
    LogToConsole (true);
    FlushLogOn (cpplogging::Loggable::info);
}

void ROSCommsSimulator::TransmitFrame(DataLinkFramePtr dlf)
{
    auto srcdir = dlf->GetSrcDir ();
    auto dstdir = dlf->GetDesDir ();

    std::string channelKey =
            std::to_string (srcdir) + std::string("_") + std::to_string (dstdir);

    auto channelProperties = _channels[channelKey];
    auto bitRate = channelProperties->GetMaxBitRate ();

    //Simulate transmission time
    auto byteTransmissionTime =  1000./( bitRate/ 8.);
    auto frameSize = dlf->GetFrameSize ();
    unsigned int frameTransmissionTime = ceil(frameSize * byteTransmissionTime);
    auto txtrp = TransportPDU::BuildTransportPDU (0);
    txtrp->UpdateBuffer (dlf->GetPayloadBuffer ());
    auto delay = channelProperties->GetDelay ();

    //Simulate total delay (transmission + propagation + reception)
    auto trRate = channelProperties->GetNextTt();
    auto trTime = trRate * frameSize;
    int totalTime = ceil(delay + trTime);
    Log->debug("TX {}->{}: R: {} ms/byte ; TT: {} ms ; D: {} ms (Seq: {}) (FS: {}).",
               srcdir, dstdir,
               trRate,
               totalTime,
               delay,
               txtrp->GetSeqNum (),
               frameSize);

    _PropagateFrame (dlf, totalTime);

    std::this_thread::sleep_for(
                std::chrono::milliseconds(frameTransmissionTime));
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
        CommsNodePtr dstNode = _nodes[dstdir];
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
    auto prTime                  = req.prTime;
    auto prTimeIncPerMeter       = req.prTimeIncPerMeter;
    auto minPktErrorRate         = req.minPktErrorRate;
    auto pktErrorRateIncPerMeter = req.pktErrorRateIncPerMeter;

    Log->info("\nAdding new net device:\n"
              "\tID ........................ {}\n"
              "\tMAC ....................... {}\n"
              "\tMax. bits/s .................. {}\n"
              "\tTransmission time ......... {} ms/byte (std = {} ms/byte)\n"
              "\tMin. propagation time ..... {} ms/m\n"
              "\tPropagation time inc. ..... {} ms/m\n"
              "\tPacket Error Rate ......... {}%\n"
              "\tPacket Error Rate inc. .... {}% per meter\n",
              newDevice,
              mac,
              maxBitRate,
              trTimeMean, trTimeSd,
              prTime,
              prTimeIncPerMeter,
              minPktErrorRate,
              pktErrorRateIncPerMeter);

    if(_nodes.find(mac) == _nodes.end())
    {
        auto commsDeviceService = dccomms::CommsDeviceService::BuildCommsDeviceService (
                    IPHY_TYPE_PHY,
                    dccomms::DataLinkFrame::crc16
                    );

        commsDeviceService->SetNamespace (newDevice);
        auto node =  ROSCommsDevice::BuildCommsNode(this, commsDeviceService);
        node->SetName(newDevice);

        for(pair<int, CommsNodePtr> pair: _nodes)
        {
            auto remoteMac = pair.first;

            //Build tx channel
            std::stringstream ss;
            ss << mac << "_" << remoteMac;
            std::string txChannelKey = ss.str();

            auto txChannel = CommsChannelState::BuildCommsChannelState ();
            txChannel->SetDelay (prTime);
            txChannel->SetLinkOk (true);
            txChannel->SetMaxBitRate (maxBitRate);
            txChannel->SetTtDist (trTimeMean, trTimeSd);

            _channels[txChannelKey] = txChannel;

            //Build rx channel
            ss.str(std::string());
            ss << remoteMac << "_" << mac;
            std::string rxChannelKey = ss.str();

            auto rxChannel = CommsChannelState::BuildCommsChannelState ();
            rxChannel->SetDelay (prTime);
            rxChannel->SetLinkOk (true);
            rxChannel->SetMaxBitRate (maxBitRate);
            rxChannel->SetTtDist (trTimeMean, trTimeSd);

        }


        _nodes[mac] = node;
    }
    else
    {
        CommsNodePtr node = _nodes[mac];
        Log->error("Unable to add the device. A net device with the same MAC already exists: '{}'", node->GetName ());
    }

    /*
    //make operator comms interface
    auto operatorComms = dccomms::CommsDeviceService::BuildCommsDeviceService (
                IPHY_TYPE_PHY,
                dccomms::DataLinkFrame::crc16
                );
    operatorComms->SetNamespace ("operator");
    nodes[1] = ROSCommsDevice::BuildCommsNode(this, operatorComms);
    nodes[1]->SetName("operator");

    //make Girona500 comms interface
    auto g500Comms = dccomms::CommsDeviceService::BuildCommsDeviceService (
                IPHY_TYPE_PHY,
                dccomms::DataLinkFrame::crc16
                );
    g500Comms->SetNamespace ("camera");
    nodes[2] = ROSCommsDevice::BuildCommsNode(this, g500Comms);
    nodes[2]->SetName("g500");

    //Operator -> Girona500
    auto channel1_2 = CommsChannelState::BuildCommsChannelState ();
    channel1_2->SetDelay(0);
    channel1_2->SetMaxBitRate (100);

    //Girona500 -> Operator
    auto channel2_1 = CommsChannelState::BuildCommsChannelState ();
    channel2_1->SetDelay(0);
    channel2_1->SetMaxBitRate (1600);

    channel2_1->SetTtDist(12, 0.1);
    channel1_2->SetTtDist(30, 0.1);
    channels["1_2"] = channel1_2;
    channels["2_1"] = channel2_1;
    */

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
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->SetLogName(name + ":Node(" + node->GetName()+")");
    }
}
void ROSCommsSimulator::SetLogLevel(Loggable::LogLevel _level)
{
    Loggable::SetLogLevel(_level);
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->SetLogLevel(_level);
    }
}

void ROSCommsSimulator::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->LogToConsole(c);
    }

}

void ROSCommsSimulator::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->LogToFile(filename + "_" + node->GetName () + "_node");
    }
}

void ROSCommsSimulator::FlushLog()
{
    Loggable::FlushLog();
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->FlushLog ();
    }
}

void ROSCommsSimulator::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    for(pair<int, CommsNodePtr> pair: _nodes)
    {
        auto node = pair.second;
        node->FlushLogOn(level);
    }
}
}
