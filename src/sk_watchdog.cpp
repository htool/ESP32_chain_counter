#include "sk_watchdog.h"

#include <WiFi.h>

#include "sensesp.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app.h"
#include "sensesp_base_app.h"

using namespace sensesp;

static constexpr uint32_t kProbeIntervalMs = 10000;
static constexpr uint32_t kHttpTimeoutMs = 2000;
static constexpr int kFailuresBeforeRestart = 2;

SKReconnectWatchdog::SKReconnectWatchdog(std::shared_ptr<SKWSClient> ws,
                                         std::shared_ptr<ChainCounter> counter)
    : ws_{ws}, counter_{counter} {
  ws_->connect_to(std::make_shared<LambdaConsumer<SKWSConnectionState>>(
      [this](SKWSConnectionState state) { this->on_ws_state(state); }));

  // restart() must run on the SensESP event loop, not the probe task.
  event_loop()->onRepeat(200, [this]() {
    if (restart_requested_.exchange(false)) {
      ESP_LOGW("sk_watchdog",
               "Signal K HTTP is down; dropping stale websocket so it can "
               "reconnect");
      ws_->restart();
    }
  });

  xTaskCreate(&SKReconnectWatchdog::probe_task, "sk_probe", 4096, this, 1,
              nullptr);
}

void SKReconnectWatchdog::on_ws_state(SKWSConnectionState state) {
  if (state == SKWSConnectionState::kSKWSConnected) {
    consecutive_http_failures_.store(0);
    ESP_LOGI("sk_watchdog",
             "Signal K websocket connected; republishing chain state");
    if (counter_) {
      counter_->publish();
    }
  } else if (state == SKWSConnectionState::kSKWSDisconnected) {
    ESP_LOGW("sk_watchdog",
             "Signal K websocket disconnected; counting continues locally");
  }
}

bool SKReconnectWatchdog::probe_http() const {
  if (ws_ == nullptr) {
    return false;
  }
  String host = ws_->get_server_address();
  uint16_t port = ws_->get_server_port();
  if (host.isEmpty() || port == 0) {
    host = "192.168.3.1";
    port = 3000;
  }

  WiFiClient client;
  if (!client.connect(host.c_str(), port, kHttpTimeoutMs)) {
    return false;
  }

  client.printf(
      "GET /signalk HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
      host.c_str());

  const unsigned long start = millis();
  while (client.connected() && (millis() - start) < kHttpTimeoutMs) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      client.stop();
      return line.indexOf("HTTP/") >= 0;
    }
    delay(20);
  }
  client.stop();
  return false;
}

void SKReconnectWatchdog::probe_task(void* arg) {
  auto* self = static_cast<SKReconnectWatchdog*>(arg);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(kProbeIntervalMs));
    auto provisioner = SensESPApp::get()->get_network_provisioner();
    if (!provisioner || !provisioner->is_connected()) {
      continue;
    }
    if (!self->ws_ || !self->ws_->is_connected()) {
      continue;
    }
    const bool http_ok = self->probe_http();
    if (http_ok) {
      self->consecutive_http_failures_.store(0);
      continue;
    }
    const int fails = self->consecutive_http_failures_.fetch_add(1) + 1;
    ESP_LOGW("sk_watchdog", "Signal K HTTP probe failed (%d)", fails);
    if (fails >= kFailuresBeforeRestart) {
      self->restart_requested_.store(true);
    }
  }
}
