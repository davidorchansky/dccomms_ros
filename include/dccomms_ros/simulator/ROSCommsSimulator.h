#ifndef WHROVSIMULATOR_H
#define WHROVSIMULATOR_H

#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <unordered_map>
#include <Loggable.h>
#include <random>
#include <dccomms_ros/simulator/ROSCommsChannelState.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <memory>
#include <functional>

//ROS
#include <ros/ros.h>
#include <dccomms_ros_msgs/AddDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
//end ROS

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros
{

class DevicesLink
{
    public:
        DevicesLink(ROSCommsDevicePtr dev0, ROSCommsDevicePtr dev1,
                    CommsChannelStatePtr chn0, CommsChannelStatePtr chn1)
            : device0(dev0), device1(dev1), channel0(chn0), channel1(chn1){}

        ROSCommsDevicePtr device0, device1;
        CommsChannelStatePtr channel0, channel1;
};

typedef std::list<DevicesLink> DevicesLinks;

typedef std::unordered_map<
    int,
    ROSCommsDevicePtr>
    NodeMap;

typedef std::shared_ptr<NodeMap> NodeMapPtr;

typedef std::unordered_map<
    int,
    NodeMapPtr>
    NodeTypeMap;

typedef std::unordered_map<
    std::string, //"class_txdir_rxdir"
    CommsChannelStatePtr>
    ChannelMap;

typedef std::unordered_map<
  std::string,
  ROSCommsDevicePtr>
  IdDevMap;


class ROSCommsSimulator;

typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsSimulator : public virtual Loggable
{
public:
    ROSCommsSimulator(ros::NodeHandle & rosnode);
    void TransmitFrame(int linkType, DataLinkFramePtr dlf);
    void Start();

    virtual void SetLogName(std::string name);
    virtual void SetLogLevel(Loggable::LogLevel);
    virtual void FlushLog();
    virtual void FlushLogOn(LogLevel);
    virtual void LogToConsole(bool);
    virtual void LogToFile(const string &filename);

    void SetTransmitPDUCb(std::function<void(int linkType, dccomms::DataLinkFramePtr)> cb);
    void SetReceivePDUCb(std::function<void(int linkType, dccomms::DataLinkFramePtr)> cb);
    void SetErrorPDUCb(std::function<void(int linkType, dccomms::DataLinkFramePtr)> cb);

private:
    void _Init();
    std::function<void(int, dccomms::DataLinkFramePtr)> _TransmitPDUCb, _ReceivePDUCb, _ErrorPDUCb;
    bool _AddDevice(dccomms_ros_msgs::AddDevice::Request & req,
                    dccomms_ros_msgs::AddDevice::Response & res);
    bool _CheckDevice(dccomms_ros_msgs::CheckDevice::Request & req,
                      dccomms_ros_msgs::CheckDevice::Response & res);
    bool _RemoveDevice(dccomms_ros_msgs::RemoveDevice::Request & req,
                       dccomms_ros_msgs::RemoveDevice::Response & res);
    void _PropagateFrame(DataLinkFramePtr dlf, int delay, CommsChannelStatePtr channel);
    void _DeliverFrame(DataLinkFramePtr dlf, CommsChannelStatePtr channel);

    void _AddDeviceToSet(std::string iddev, ROSCommsDevicePtr dev);
    bool _DeviceExists(std::string iddev);
    void _RemoveDeviceFromSet(std::string iddev);

    void _RemoveChannel(std::string channelKey);

    ROSCommsDevicePtr _GetDevice(std::string iddev);
    CommsChannelStatePtr _GetChannel(std::string channelKey);

    ros::ServiceServer _addDevService, _checkDevService, _removeDevService;
    ros::NodeHandle & _rosNode;
    NodeTypeMap _nodes;
    ChannelMap _channels;
    IdDevMap _idDevMap;

    std::mutex _devLinksMutex, _idDevMapMutex, _channelsMutex;
    DevicesLinks _devLinks;


    ServiceThread<ROSCommsSimulator> _linkUpdaterWorker;
    void _LinkUpdaterWork();
    void _UpdateChannelStateFromRange(CommsChannelStatePtr chn,  double range, bool log = true);
};

}
#endif // WHROVSIMULATOR_H
