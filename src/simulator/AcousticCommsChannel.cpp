#include <dccomms_ros/simulator/AcousticCommsChannel.h>

#include <ns3/aqua-sim-channel.h>
#include <ns3/channel-list.h>

namespace dccomms_ros {

AcousticCommsChannel::AcousticCommsChannel(uint32_t id) {
  _rosChannelId = id;
  _ns3ChannelId = ns3::ChannelList::GetNChannels();
  _channelHelper = ns3::AquaSimChannelHelper::Default();
  _channelHelper.SetPropagation("ns3::AquaSimRangePropagation");
  _aquaSimChannel = _channelHelper.Create();
  SetBandwidth(4096);
  SetTemperature(25);
  SetSalinity(35);
  SetNoiseLevel(0);
}

void AcousticCommsChannel::SetBandwidth(double value) {
  _aquaSimChannel->SetAttribute("Bandwith", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetTemperature(double value) {
  _aquaSimChannel->SetAttribute("Temperature", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetSalinity(double value) {
  _aquaSimChannel->SetAttribute("Salinity", ns3::DoubleValue(value));
}
void AcousticCommsChannel::SetNoiseLevel(double value) {
  _aquaSimChannel->SetAttribute("NoiseLvl", ns3::DoubleValue(value));
}

double AcousticCommsChannel::GetBandwidth() {
  ns3::DoubleValue value;
  _aquaSimChannel->GetAttribute("Bandwidth", value);
  return value.Get();
}
double AcousticCommsChannel::GetTemperature() {
  ns3::DoubleValue value;
  _aquaSimChannel->GetAttribute("Temperature", value);
  return value.Get();
}
double AcousticCommsChannel::GetSalinity() {
  ns3::DoubleValue value;
  _aquaSimChannel->GetAttribute("Salinity", value);
  return value.Get();
}
double AcousticCommsChannel::GetNoiseLevel() {
  ns3::DoubleValue value;
  _aquaSimChannel->GetAttribute("NoiseLvl", value);
  return value.Get();
}
}
