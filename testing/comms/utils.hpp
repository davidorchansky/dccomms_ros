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

  void UpdateFCS();

private:
  uint8_t *_pre;
  uint8_t *_order;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
  const static int PRE_SIZE = 1, CODE_SIZE = 4, FCS_SIZE = 1;

  bool _CheckFCS();
};

MasterPacket::MasterPacket() {
  _packetSize = PRE_SIZE + CODE_SIZE + FCS_SIZE;
  _AllocBuffer(_packetSize);
  _Init();
}

void MasterPacket::_Init() {
  _pre = GetBuffer();
  _order = _pre + PRE_SIZE;
  _fcs = _order + CODE_SIZE;
  memset(_pre, 0x55, PRE_SIZE);
}

void MasterPacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *MasterPacket::GetPayloadBuffer() { return _order; }

inline uint32_t MasterPacket::GetPayloadSize() { return _packetSize; }

inline int MasterPacket::GetPacketSize() { return _packetSize; }

void MasterPacket::Read(IStream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_order, CODE_SIZE);
  stream->Read(_fcs, FCS_SIZE);
}

void MasterPacket::UpdateFCS() { *_fcs = ~*_order; }
bool MasterPacket::_CheckFCS() { return (uint8_t)*_order == (uint8_t) ~*_fcs; }
bool MasterPacket::PacketIsOk() { return _CheckFCS(); }

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
  void SetPayload(const void *data, int size = PAYLOAD_SIZE);

  void UpdateFCS();

private:
  static const int PRE_SIZE = 1, PAYLOAD_SIZE = 40, FCS_SIZE = 2;

  uint8_t *_pre;
  uint8_t *_payload;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
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

void SlavePacket::GetPayload(void *copy) {
  memcpy(copy, _payload, PAYLOAD_SIZE);
}

void SlavePacket::SetPayload(const void *data, int size) {
  memset(_payload, '#', PAYLOAD_SIZE);
  memcpy(_payload, data, size);
  *(_payload + PAYLOAD_SIZE - 1) = 0;
}

void SlavePacket::UpdateFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool SlavePacket::_CheckFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE + FCS_SIZE);
  return crc == 0;
}
bool SlavePacket::PacketIsOk() { return _CheckFCS(); }

class SlavePacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<SlavePacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create() { return CreateObject<SlavePacket>(); }
};

class MasterPacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<MasterPacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create() { return CreateObject<MasterPacket>(); }
};
}
#endif
