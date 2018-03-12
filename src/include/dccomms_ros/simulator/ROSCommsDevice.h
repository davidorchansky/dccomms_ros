#ifndef DCCOMMS_ROS_ROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_ROSCOMMSDEVICE_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <dccomms_ros/simulator/CommsChannel.h>
#include <ns3/config-store-module.h>
#include <ns3/core-module.h>
#include <tf/transform_listener.h>

using namespace dccomms;
using namespace cpplogging;
namespace dccomms_ros {

enum PacketErrorType { PE_PROP, PE_COL };
class ROSCommsDevice;
typedef ns3::Ptr<ROSCommsDevice> ROSCommsDevicePtr;

class ROSCommsSimulator;
//typedef dccomms::Ptr<ROSCommsSimulator> ROSCommsSimulatorPtr;
typedef ROSCommsSimulator* ROSCommsSimulatorPtr;

//why inheritance for std::enable_shared_from_this??:
//  https://stackoverflow.com/questions/11711034/stdshared-ptr-of-this
//  https://stackoverflow.com/questions/16082785/use-of-enable-shared-from-this-with-multiple-inheritance
class ROSCommsDevice : public virtual Logger, public ns3::Object, public std::enable_shared_from_this<ROSCommsDevice> {
public:
  ROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr txpb,
                 PacketBuilderPtr rxpb);
  ~ROSCommsDevice();

  CommsDeviceServicePtr GetService();
  void ReceiveFrame(PacketPtr);
  std::string GetDccommsId();
  void SetDccommsId(const std::string name);

  void SetBitRate(uint32_t bps);
  double GetNanosPerByte() { return _nanosPerByte; }

  void SetPosition(const tf::Vector3 &position);
  void SetMaxTxFifoSize(uint32_t size);
  uint32_t GetMaxTxFifoSize();

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

  void LinkToChannel(CommsChannelPtr channel,
                     CHANNEL_LINK_TYPE linkType = CHANNEL_TXRX);
  CommsChannelPtr GetLinkedTxChannel();
  CommsChannelPtr GetLinkedRxChannel();

  tf::Vector3 GetPosition();

  virtual DEV_TYPE GetDevType() = 0;
  bool Started();

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

  typedef void (*PacketReceivedCallback)(std::string path, ROSCommsDevicePtr, PacketPtr);
  typedef void (*PacketTransmittingCallback)(std::string path, ROSCommsDevicePtr, PacketPtr);

protected:
  virtual std::string DoToString() = 0;
  virtual void DoSetMac(uint32_t mac) = 0;
  virtual void DoSend(PacketPtr dlf) = 0;
  virtual void DoLinkToChannel(CommsChannelPtr channel,
                               CHANNEL_LINK_TYPE linkType = CHANNEL_TXRX) = 0;
  virtual void DoStart() = 0;
  virtual void DoSetPosition(const tf::Vector3 &position) = 0;
  virtual bool DoStarted() = 0;

  ROSCommsSimulatorPtr _sim;
  PacketBuilderPtr _txpb, _rxpb;

  ns3::TracedCallback<ROSCommsDevicePtr, PacketPtr> _rxCbTrace;
  ns3::TracedCallback<ROSCommsDevicePtr, PacketPtr> _txCbTrace;

private:
  void _StartDeviceService();
  void _StartNodeWorker();
  void _TxWork();

protected:
  std::mutex _receiveFrameMutex;
  CommsDeviceServicePtr _device;
  CommsChannelPtr _txChannel, _rxChannel;
  ServiceThread<ROSCommsDevice> _txserv;
  PacketPtr _txdlf;
  std::string _name, _tfFrameId;
  uint32_t _mac;
  uint32_t _bitRate;
  uint64_t _nanosPerByte;
  tf::Vector3 _position;

  ROSCommsDevicePtr _ownPtr;

  bool _commonStarted;
};
}
#endif // COMMSNODE_H
