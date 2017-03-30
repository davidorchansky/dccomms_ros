#ifndef WHROVSIMULATOR_H
#define WHROVSIMULATOR_H

#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <unordered_map>
#include <Loggable.h>
#include <random>
#include <dccomms_ros/ROSCommsChannelState.h>
#include <dccomms_ros/ROSCommsNode.h>
#include <memory>

//ROS
#include <ros/ros.h>
#include <dccomms_ros/AddDevice.h>
//end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros
{

typedef std::unordered_map<
    int,
    CommsNodePtr>
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
};

}
#endif // WHROVSIMULATOR_H
