// SensESP bidirectional windlass chain counter for Sailor Hat ESP32.
//
// Replaces the Arduino IDE sketch: Signal K websocket + B&G PGN 130824,
// with SensESP Wi-Fi onboarding, web config, and OTA firmware updates.

#include "sensesp.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"

#include "chain_counter.h"
#include "n2k_chain.h"
#include "sk_watchdog.h"

// WIFI_SSID / WIFI_PASSWORD / OTA_PASSWORD: gitignored wifi_secrets.h at the
// project root, injected by include_secrets.py (-include). Not under src/.
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "thisisfine"
#endif

using namespace sensesp;

// SH-ESP32 optocoupler inputs (input-only GPIOs, external pull-ups).
static const uint8_t kSensorPin = 35;
static const uint8_t kUpPin = 39;
static const uint8_t kDownPin = 36;

// SH-ESP32 onboard CAN transceiver.
static const gpio_num_t kCanTxPin = GPIO_NUM_32;
static const gpio_num_t kCanRxPin = GPIO_NUM_34;

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto* app_builder =
      (&builder)
          ->set_hostname("chain-counter")
          ->set_sk_server("192.168.3.1", 3000)
          ->enable_ota(OTA_PASSWORD);

#ifdef WIFI_SSID
  app_builder->set_wifi_client(WIFI_SSID, WIFI_PASSWORD);
#endif

  sensesp_app = app_builder->get_app();

  auto counter = std::make_shared<ChainCounter>(
      kSensorPin, kUpPin, kDownPin, "/chain/counter");

  ConfigItem(counter)
      ->set_title("Chain counter")
      ->set_description(
          "Pulse count and metres of rode per gypsy pulse. Survives reboot. "
          "Set pulse count or rode to 0 when the anchor is fully retrieved.")
      ->set_sort_order(100);

  auto* rode_meta =
      new SKMetadata("m", "Rode", "Anchor rode deployed", "Rode", 10.0, true);
  auto rode_sk = std::make_shared<SKOutputFloat>(
      "winches.windlass.rode", "/chain/sk/rode", rode_meta);
  ConfigItem(rode_sk)
      ->set_title("Rode SK path")
      ->set_sort_order(200);
  counter->rode_producer().connect_to(rode_sk);

  auto* pulse_meta = new SKMetadata(
      "", "Pulse count", "Windlass gypsy pulse count", "Pulses", 10.0, true);
  auto pulse_sk = std::make_shared<SKOutputInt>(
      "winches.windlass.pulseCount", "/chain/sk/pulse", pulse_meta);
  ConfigItem(pulse_sk)
      ->set_title("Pulse count SK path")
      ->set_sort_order(210);
  counter->pulse_producer().connect_to(pulse_sk);

  auto* dpp_meta = new SKMetadata(
      "m", "Distance per pulse", "Rode metres per gypsy pulse", "dpp", 10.0,
      true);
  auto dpp_sk = std::make_shared<SKOutputFloat>(
      "winches.windlass.distanceperpulse", "/chain/sk/dpp", dpp_meta);
  ConfigItem(dpp_sk)
      ->set_title("Distance-per-pulse SK path")
      ->set_sort_order(220);
  counter->dpp_producer().connect_to(dpp_sk);

  auto pulse_put = std::make_shared<SKPutRequestListener<float>>(
      "winches.windlass.pulseCount");
  pulse_put->connect_to(std::make_shared<LambdaConsumer<float>>(
      [counter](float value) { counter->set_pulse_count((int)value); }));

  auto dpp_put = std::make_shared<SKPutRequestListener<float>>(
      "winches.windlass.distanceperpulse");
  dpp_put->connect_to(std::make_shared<LambdaConsumer<float>>(
      [counter](float value) { counter->set_distance_per_pulse(value); }));

  auto rode_put = std::make_shared<SKPutRequestListener<float>>(
      "winches.windlass.rode");
  rode_put->connect_to(std::make_shared<LambdaConsumer<float>>(
      [counter](float value) { counter->set_rode(value); }));

  auto n2k = std::make_shared<N2kChainOutput>(kCanTxPin, kCanRxPin,
                                              counter.get());
  auto watchdog = std::make_shared<SKReconnectWatchdog>(
      sensesp_app->get_ws_client(), counter);
  (void)n2k;
  (void)watchdog;

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
