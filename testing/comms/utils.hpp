#ifndef TESTING_COMMS_UTILS_HPP
#define TESTING_COMMS_UTILS_HPP

#include <dccomms/dccomms.h>

using namespace dccomms;
namespace teleop1 {
class MasterPacket : public Packet {
public:
  MasterPacket();
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(IStream *comms);

  bool PacketIsOk();

private:
  uint8_t *_order;
  int _packetSize;
  void _Init();
};

MasterPacket::MasterPacket() {
  _AllocBuffer(1);
  _Init();
}

void MasterPacket::_Init() {
  _order = GetBuffer();
  _packetSize = 1;
}

void MasterPacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *MasterPacket::GetPayloadBuffer() { return _order; }

inline uint32_t MasterPacket::GetPayloadSize() { return _packetSize; }

inline int MasterPacket::GetPacketSize() { return _packetSize; }

void MasterPacket::Read(IStream *stream) {
  stream->Read(_order, _packetSize);
}

bool MasterPacket::PacketIsOk() { return true; }

class SlavePacket : public Packet {
public:
  SlavePacket();
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(IStream *comms);

  bool PacketIsOk();

  void GetPayload(void *copy);
  void SetPayload(void *data, int size = PAYLOAD_SIZE);

private:
  static const int PRE_SIZE = 1, PAYLOAD_SIZE = 20, FCS_SIZE = 2;

  uint8_t *_pre;
  uint8_t *_payload;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
  void _SetFCS();
  bool _CheckFCS();
};

SlavePacket::SlavePacket() {
  _packetSize = PRE_SIZE + PAYLOAD_SIZE + FCS_SIZE;
  _AllocBuffer(_packetSize);
  _Init();
}

void SlavePacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payload = _pre + PRE_SIZE;
  _fcs = _payload + PAYLOAD_SIZE;
}

void SlavePacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *SlavePacket::GetPayloadBuffer() { return _payload; }

inline uint32_t SlavePacket::GetPayloadSize() { return PAYLOAD_SIZE; }

inline int SlavePacket::GetPacketSize() { return _packetSize; }

void SlavePacket::Read(IStream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payload, PAYLOAD_SIZE + FCS_SIZE);
}

void SlavePacket::GetPayload(void *copy) { memcpy(copy, _payload, PAYLOAD_SIZE); }

void SlavePacket::SetPayload(void *data, int size) {
  memset(_payload, 0, PAYLOAD_SIZE);
  memcpy(_payload, data, size);
}

void SlavePacket::_SetFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool SlavePacket::_CheckFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE + FCS_SIZE);
  return crc == 0;
}
bool SlavePacket::PacketIsOk() { return _CheckFCS(); }

class ROVPacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<SlavePacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create(){
      return CreateObject<SlavePacket>();
  }
};

class MasterPacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<MasterPacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create(){
      return CreateObject<MasterPacket>();
  }
};
}
#endif
