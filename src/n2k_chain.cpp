#include "n2k_chain.h"

#include "sensesp.h"
#include "sensesp_base_app.h"

using namespace sensesp;

bool N2kChainOutput::tx_enabled_ = false;

static const int kDeviceId = 51;
static const int kDevChain = 0;
static constexpr unsigned long kMinTxMs = 200;       // 5 Hz
static constexpr unsigned long kHeartbeatMs = 2000;

N2kChainOutput::N2kChainOutput(gpio_num_t tx_pin, gpio_num_t rx_pin,
                               ChainCounter* counter)
    : nmea2000_(tx_pin, rx_pin), counter_{counter} {
  nmea2000_.SetN2kCANSendFrameBufSize(250);
  nmea2000_.SetN2kCANReceiveFrameBufSize(250);
  nmea2000_.SetProductInformation(
      "107018103", 13233, "Chain counter", "0.1.0", "", 1, 0xffff, 0xff,
      kDevChain);
  nmea2000_.SetDeviceInformation(1048278, 140, 60, 275, 4, kDevChain);
  nmea2000_.SetMode(tNMEA2000::N2km_NodeOnly, kDeviceId);
  nmea2000_.EnableForward(false);
  nmea2000_.SetN2kCANMsgBufSize(20);
  nmea2000_.SetMsgHandler(HandleNMEA2000Msg);
  nmea2000_.Open();

  event_loop()->onTick([this]() { nmea2000_.ParseMessages(); });
  event_loop()->onRepeat(50, [this]() { this->tick(); });

  counter_->rode_producer().attach([this]() { this->request_send(); });
}

void N2kChainOutput::HandleNMEA2000Msg(const tN2kMsg& /*msg*/) {
  if (!tx_enabled_) {
    tx_enabled_ = true;
    ESP_LOGI("n2k", "Enabling NMEA2000 chain length TX");
  }
}

void N2kChainOutput::request_send() { pending_ = true; }

void N2kChainOutput::tick() {
  if (!tx_enabled_ || counter_ == nullptr) {
    return;
  }
  const unsigned long now = millis();
  if (pending_ && (now - last_tx_ms_ >= kMinTxMs)) {
    send_now();
    pending_ = false;
    return;
  }
  if (now - last_tx_ms_ >= kHeartbeatMs) {
    send_now();
  }
}

void N2kChainOutput::send_now() {
  tN2kMsg msg;
  msg.SetPGN(130824L);
  msg.Priority = 3;
  msg.AddByte(0x7d);
  msg.AddByte(0x99);
  msg.AddByte(0x1c);
  msg.AddByte(0x21);
  msg.Add2ByteDouble(counter_->rode(), 0.01);
  nmea2000_.SendMsg(msg, kDevChain);
  last_tx_ms_ = millis();
}
