#ifndef SK_WATCHDOG_H_
#define SK_WATCHDOG_H_

#include <atomic>
#include <memory>

#include "chain_counter.h"
#include "sensesp/signalk/signalk_ws_client.h"

/**
 * Keep the chain counter alive across Signal K restarts.
 *
 * SensESP reconnects the websocket on DISCONNECT, but a Docker/SK bounce
 * often leaves a half-open TCP socket that never fires that event. This
 * watchdog probes SK over HTTP from a worker task (does not block pulse
 * counting). If HTTP is down while the websocket still claims connected,
 * the websocket is torn down so SensESP will reconnect. On a successful
 * reconnect the current rode is republished so SK is not left empty.
 */
class SKReconnectWatchdog {
 public:
  SKReconnectWatchdog(std::shared_ptr<sensesp::SKWSClient> ws,
                      std::shared_ptr<sensesp::ChainCounter> counter);

 private:
  static void probe_task(void* arg);
  bool probe_http() const;
  void on_ws_state(sensesp::SKWSConnectionState state);

  std::shared_ptr<sensesp::SKWSClient> ws_;
  std::shared_ptr<sensesp::ChainCounter> counter_;
  std::atomic<bool> restart_requested_{false};
  std::atomic<int> consecutive_http_failures_{0};
};

#endif
