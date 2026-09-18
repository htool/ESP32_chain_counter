#ifndef N2K_CHAIN_H_
#define N2K_CHAIN_H_

#include <N2kMessages.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>

#include "chain_counter.h"

using sensesp::ChainCounter;

/**
 * B&G proprietary PGN 130824 chain-length output on the SH-ESP32 CAN
 * transceiver (TX GPIO 32, RX GPIO 34). TX starts after the first received
 * N2K frame, same as the original Arduino sketch. Rate-limited to 5 Hz.
 */
class N2kChainOutput {
 public:
  N2kChainOutput(gpio_num_t tx_pin, gpio_num_t rx_pin, ChainCounter* counter);

 private:
  static void HandleNMEA2000Msg(const tN2kMsg& msg);
  void request_send();
  void tick();
  void send_now();

  tNMEA2000_esp32 nmea2000_;
  ChainCounter* counter_;
  static bool tx_enabled_;
  bool pending_ = false;
  unsigned long last_tx_ms_ = 0;
};

#endif
