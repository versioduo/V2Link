// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#include "V2Link.h"

V2Link::Packet::Type V2Link::Packet::getType() {
  return static_cast<Type>(_data[0] & 0x0f);
}

uint8_t V2Link::Packet::getAddress() {
  return _data[0] >> 4;
}

bool V2Link::Packet::receive(V2MIDI::Packet *midi) {
  _data[0] = (uint8_t)Packet::Type::MIDI;
  midi->setData(_data + 1);
  return true;
}

bool V2Link::Packet::send(V2MIDI::Packet *midi) {
  memcpy(_data + 1, midi->getData(), 4);
  return true;
}

void V2Link::Packet::getPulse(Pulse *pulse) {
  pulse->port     = _data[1] & 0x0f;
  pulse->fade_in  = _data[1] & (1 << 4);
  pulse->fade_out = _data[1] & (1 << 5);

  {
    uint16_t map = (_data[2] >> 4) << 8;
    map |= _data[3];
    const float fraction = (float)map / 4095.f;
    pulse->watts         = 100.f * powf(fraction, 3);
  }
  {
    uint16_t map = (_data[2] & 0x0f) << 8;
    map |= _data[4];
    const float fraction = (float)map / 4095.f;
    pulse->seconds       = 10.f * powf(fraction, 3);
  }
}

void V2Link::Packet::setPulse(const Packet::Pulse *pulse) {
  _data[0] = (uint8_t)Packet::Type::Pulse;
  _data[1] = pulse->port & 0x0f;
  if (pulse->fade_in)
    _data[1] |= 1 << 4;
  if (pulse->fade_out)
    _data[1] |= 1 << 5;

  {
    float watts = pulse->watts;
    if (watts > 100.f)
      watts = 100;

    const float fraction = watts / 100.f;
    const uint16_t map   = powf(fraction, 1.f / 3.f) * 4095.f;
    _data[2]             = (map >> 8) << 4;
    _data[3]             = map & 0xff;
  }
  {
    float seconds = pulse->seconds;
    if (seconds > 10.f)
      seconds = 10;

    const float fraction = seconds / 10.f;
    const uint16_t map   = powf(fraction, 1.f / 3.f) * 4095.f;
    _data[2] |= map >> 8;
    _data[4] = map & 0xff;
  }
}

void V2Link::Port::begin() {
  _uart->begin(3000000);
  _uart->setTimeout(1);
}

bool V2Link::Port::idle() {
  if ((unsigned long)(micros() - _usec) < 1000)
    return false;

  if (_uart->available() > 0)
    return false;

  return true;
}

bool V2Link::Port::receive(Packet *packet) {
  if (_uart->available() == 0)
    return false;

  _usec = micros();

  // Drop partial messages which don't complete in time.
  if (_uart->available() < 5) {
    if (_timeout_usec == 0)
      _timeout_usec = micros();

    if ((unsigned long)(micros() - _timeout_usec) > 100) {
      while (_uart->available())
        _uart->read();

      _timeout_usec = 0;
    }

    return false;
  }

  _timeout_usec = 0;
  _uart->readBytes(packet->_data, 5);
  return true;
}

bool V2Link::Port::send(uint8_t address, Packet *packet) {
  _usec = micros();

  if (_uart->availableForWrite() < 5)
    return false;

  uint8_t header = address << 4;
  header |= packet->_data[0] & 0x0f;
  _uart->write(header);
  _uart->write(packet->_data + 1, 4);
  return true;
}

bool V2Link::Port::send(V2MIDI::Packet *midi) {
  Packet packet;
  packet._data[0] = (uint8_t)Packet::Type::MIDI;
  memcpy(packet._data + 1, midi->getData(), 4);
  return send(midi->getPort(), &packet);
}

void V2Link::loop() {
  Packet packet;

  if (_plug && _plug->receive(&packet)) {
    if (packet.getAddress() > 0) {
      // Forward message from a parent device to a child device.
      if (_socket)
        _socket->send(packet.getAddress() - 1, &packet);

    } else
      receivePlug(&packet);
  }

  if (_socket && _socket->receive(&packet)) {
    // Forward message from a child device towards the parent device, stop after
    // too many hops.
    if (_plug && packet.getAddress() < 0x0f)
      _plug->send(packet.getAddress() + 1, &packet);

    receiveSocket(&packet);
  }
}

bool V2Link::idle() {
  if (_plug && !_plug->idle())
    return false;

  if (_socket && !_socket->idle())
    return false;

  return true;
}
