#ifndef WHROVSIMULATOR_H
#define WHROVSIMULATOR_H

#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <unordered_map>
#include <Loggable.h>
#include <random>
#include <dccomms_ros/ROSCommsChannelState.h>
#include <dccomms_ros/ROSCommsDevice.h>
#include <memory>

//ROS
#include <ros/ros.h>
#include <dccomms_ros/AddDevice.h>
//end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros
{

class DevicesLink
{
    public:
        DevicesLink(CommsDevicePtr dev0, CommsDevicePtr dev1,
                    CommsChannelStatePtr chn0, CommsChannelStatePtr chn1)
            : device0(dev0), device1(dev1), channel0(chn0), channel1(chn1){}

        CommsDevicePtr device0, device1;
        CommsChannelStatePtr channel0, channel1;
};

typedef std::list<DevicesLink> DevicesLinks;

typedef std::unordered_map<
    int,
    CommsDevicePtr>
    NodeMap;


typedef std::unordered_map<
    std::string, //"txdir rxdir"
    CommsChannelStatePtr>
    ChannelMap;

class ROSCommsSimulator;

typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Loggable
{
public:
    ROSCommsSimulator(ros::NodeHandle & rosnode);
    void TransmitFrame(DataLinkFramePtr dlf);
    void Start();

    virtual void SetLogName(std::string name);
    virtual void SetLogLevel(Loggable::LogLevel);
    virtual void FlushLog();
    virtual void FlushLogOn(LogLevel);
    virtual void LogToConsole(bool);
    virtual void LogToFile(const string &filename);

private:
    bool _AddDevice(dccomms_ros::AddDevice::Request & req,
                    dccomms_ros::AddDevice::Response & res);
    void _PropagateFrame(DataLinkFramePtr dlf, int delay);
    void _DeliverFrame(DataLinkFramePtr dlf);

    ros::ServiceServer _addDevService;
    ros::NodeHandle & _rosNode;
    NodeMap _nodes;
    ChannelMap _channels;

    std::mutex _devLinksMutex;
    DevicesLinks _devLinks;


    ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
    void _LinkUpdaterWork();
};

}
#endif // WHROVSIMULATOR_H
