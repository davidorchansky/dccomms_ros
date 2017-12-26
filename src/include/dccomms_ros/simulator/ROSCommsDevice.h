#ifndef DCCOMMS_ROS_ROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_ROSCOMMSDEVICE_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <tf/transform_listener.h>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

class ROSCommsDevice;
typedef std::shared_ptr<ROSCommsDevice> ROSCommsDevicePtr;

class ROSCommsSimulator;
typedef std::shared_ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;

class ROSCommsDevice : public virtual Logger {
public:
  ROSCommsDevice(ROSCommsSimulatorPtr,  PacketBuilderPtr txpb, PacketBuilderPtr rxpb);

  CommsDeviceServicePtr GetService();
  void ReceiveFrame(PacketPtr);
  std::string GetDccommsId();
  void SetDccommsId(const std::string name);

  void SetBitRate(uint32_t bps);

  void SetPosition(const tf::Vector3 &position);

  virtual void SetLogName(std::string name);
  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void FlushLog();
  virtual void FlushLogOn(cpplogging::LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const std::string &filename);

  void SetMac(uint32_t mac);
  void SetTfFrameId(const std::string &);

  uint32_t GetMac();
  std::string GetTfFrameId();

  std::string ToString();

  void Start();

  void LinkToChannel(CommsChannelPtr channel);
  CommsChannelPtr GetLinkedChannel();

  tf::Vector3 GetPosition();

  virtual DEV_TYPE GetDevType() = 0;
  bool Started();

protected:
  virtual void DoSetMac(uint32_t mac) = 0;
  virtual void DoSend(PacketPtr dlf) = 0;
  virtual void DoLinkToChannel(CommsChannelPtr channel) = 0;
  virtual void DoStart() = 0;
  virtual void DoSetPosition(const tf::Vector3 &position) = 0;
  virtual bool DoStarted() = 0;

  ROSCommsSimulatorPtr _sim;
  PacketBuilderPtr _txpb, _rxpb;

private:
  void _StartDeviceService();
  void _StartNodeWorker();

  std::mutex _receiveFrameMutex;
  CommsDeviceServicePtr _device;
  CommsChannelPtr _channel;
  ServiceThread<ROSCommsDevice> _txserv;
  PacketPtr _txdlf;
  std::string _name, _tfFrameId;
  uint32_t _mac;
  uint32_t _bitRate;
  uint32_t _millisPerByte;
  tf::Vector3 _position;

  void _TxWork();

  bool _commonStarted;
};
}
#endif // COMMSNODE_H
