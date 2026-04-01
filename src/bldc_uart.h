#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <functional>

namespace BLDC {

static constexpr uint8_t  FRAME_LEN   = 7;
static constexpr uint8_t  START_BYTE  = 0x07;   // byte[0]
static constexpr uint32_t DEFAULT_BAUD = 9600;  // match your controller

// Commands (from your Python UI)
enum : uint8_t {
  CMD_HEARTBEAT   = 0xA5,
  CMD_SPEED       = 0x5A,   // P1 = step 0..60
  CMD_RAMP_STEP   = 0xAA,   // P1 = target step
  CMD_VIBRATE     = 0x8A,   // P2 = mode 1..4
  CMD_START_STOP  = 0x69
};

using Frame = uint8_t[FRAME_LEN];
using OnFrameFn = std::function<void(const uint8_t* frame)>;

class Driver {
public:
  Driver(HardwareSerial& port, int rxPin, int txPin,
         uint32_t baud = DEFAULT_BAUD)
  : _ser(port), _rx(rxPin), _tx(txPin), _baud(baud) {}

  // Call once (from setup or your MotorTask init)
  void begin() {
    // 8N1, non-inverted
    _ser.begin(_baud, SERIAL_8N1, _rx, _tx);
    _rxBufLen = 0;
  }

  // ------------- TX helpers -------------
  void sendHeartbeat()                  { sendRaw(CMD_HEARTBEAT, 0, 0, 0); }
  void sendStartStop()                  { sendRaw(CMD_START_STOP, 0, 0, 0); }

  void sendSpeed(uint8_t step /*0..60*/) {
    if (step > 60) step = 60;
    sendRaw(CMD_SPEED, step, 0, 0);
  }

  void sendRamp(uint8_t targetStep /*0..60*/) {
    if (targetStep > 60) targetStep = 60;
    sendRaw(CMD_RAMP_STEP, targetStep, 0, 0);
  }

  void sendVibrate(uint8_t mode /*1..4*/) {
    if (mode < 1) mode = 1;
    if (mode > 4) mode = 4;
    sendRaw(CMD_VIBRATE, 0, mode, 0);
  }

  void sendRaw(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3) {
    uint8_t f[FRAME_LEN];
    f[0] = START_BYTE;
    f[1] = 0x00;
    f[2] = cmd;
    f[3] = p1;
    f[4] = p2;
    f[5] = p3;
    f[6] = xor6(f);
    _ser.write(f, FRAME_LEN);
    _ser.flush(false);
  }

  // ------------- RX (non-blocking) -------------
  // Call this often (e.g., in loop() or your MotorTask).
  // When a valid frame is parsed, onFrame (if set) is called.
  void poll() {
    while (_ser.available() > 0) {
      if (_rxBufLen >= sizeof(_rxBuf)) _rxBufLen = 0; // safety
      int b = _ser.read();
      if (b < 0) break;
      _rxBuf[_rxBufLen++] = (uint8_t)b;

      // Re-sync to START_BYTE
      while (_rxBufLen > 0 && _rxBuf[0] != START_BYTE) {
        shiftLeft(1);
      }

      // Have a full candidate frame?
      if (_rxBufLen >= FRAME_LEN && _rxBuf[0] == START_BYTE) {
        // Copy out
        uint8_t f[FRAME_LEN];
        memcpy(f, _rxBuf, FRAME_LEN);
        // Pop it from buffer
        shiftLeft(FRAME_LEN);

        // Validate checksum
        if (xor6(f) != f[6]) {
          // bad checksum → keep looking; try to resync by discarding one byte next loop
          continue;
        }

        // Valid frame → callback
        if (onFrame) onFrame(f);
      }
    }
  }

  // User assigns this to observe RX frames
  OnFrameFn onFrame;

private:
  HardwareSerial& _ser;
  int _rx, _tx;
  uint32_t _baud;

  uint8_t  _rxBuf[128];
  size_t   _rxBufLen{0};

  static uint8_t xor6(const uint8_t* f) {
    uint8_t c = 0;
    // XOR bytes 0..5
    for (int i = 0; i < 6; ++i) c ^= f[i];
    return c;
  }

  void shiftLeft(size_t n) {
    if (n >= _rxBufLen) { _rxBufLen = 0; return; }
    memmove(_rxBuf, _rxBuf + n, _rxBufLen - n);
    _rxBufLen -= n;
  }
};

} // namespace BLDC
