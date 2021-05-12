// © Kay Sievers <kay@vrfy.org>, 2020-2021
// SPDX-License-Identifier: Apache-2.0

#pragma once

// V2 Link devices are full-duplex RS422 serial lines, a differential signal,
// impedance is 120 Ohm, terminated at both endpoints. The baud rate is 3 Mhz.
//
// The 'plug' connects the device to the parent device, the 'socket' connects
// the children devices. Up to 16 devices can be daisy-chained.

#include <Arduino.h>
#include <V2MIDI.h>

class V2Link {
public:
  // Header:
  //   4 bit: target/child address,
  //   4 bit: message type
  class Packet : public V2MIDI::Transport {
  public:
    enum class Type : uint8_t { MIDI, Pulse };
    Type getType();
    uint8_t getAddress();

    // Transport: MIDI packets can be received from a Link packet.
    bool receive(V2MIDI::Packet *midi);
    bool send(V2MIDI::Packet *midi);

    // Solenoid pulse:
    //   12 bit: watts
    //   12 bit: seconds
    //    1 bit: fade in
    //    1 bit: fade out
    struct Pulse {
      uint8_t port;
      float watts;
      float seconds;
      bool fade_in;
      bool fade_out;
    };
    void getPulse(Pulse *pulse);
    void setPulse(const Pulse *pulse);

  private:
    friend class V2Link;
    uint8_t _data[5];
  };

  class Port : public V2MIDI::Transport {
  public:
    constexpr Port(Uart *uart) : _uart(uart) {}
    void begin();
    bool idle();

    bool receive(Packet *packet);
    bool send(uint8_t address, Packet *packet);

    // Transport: MIDI packets can be sent over the serial line.
    bool receive(V2MIDI::Packet *midi) {
      return false;
    }
    bool send(V2MIDI::Packet *midi);

  private:
    friend class V2Link;
    Uart *_uart;
    unsigned long _timeout_usec{};
    unsigned long _usec{};
  };

  constexpr V2Link(Port *plug, Port *socket) : _plug(plug), _socket(socket) {}
  void loop();
  bool idle();

private:
  virtual void receivePlug(Packet *packet) {}
  virtual void receiveSocket(Packet *packet) {}
  Port *_plug;
  Port *_socket;
};
