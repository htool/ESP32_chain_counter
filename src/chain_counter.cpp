#include "chain_counter.h"

#include <cmath>

#include "sensesp.h"
#include "sensesp_base_app.h"

namespace sensesp {

static constexpr unsigned long kSaveIdleMs = 5000;

ChainCounter::ChainCounter(uint8_t sensor_pin, uint8_t up_pin, uint8_t down_pin,
                           const String& config_path)
    : FileSystemSaveable(config_path),
      sensor_pin_{sensor_pin},
      up_pin_{up_pin},
      down_pin_{down_pin},
      up_item_{"Windlass up", false, "Chain", 110},
      down_item_{"Windlass down", false, "Chain", 111},
      sensor_item_{"Gypsy sensor", false, "Chain", 112},
      direction_item_{"Direction", String("out"), "Chain", 113},
      rode_item_{"Rode (m)", 0.0f, "Chain", 114} {
  pinMode(sensor_pin_, INPUT);
  pinMode(up_pin_, INPUT);
  pinMode(down_pin_, INPUT);

  load();
  apply_boot_auto_zero();
  if (dpp_ <= 0.0f || isnan(dpp_)) {
    dpp_ = 0.1675f;
  }
  if (debounce_ms_ < 0) {
    debounce_ms_ = 0;
  }
  if (auto_zero_pulses_ < 0) {
    auto_zero_pulses_ = 0;
  }

  sensor_last_ = digitalRead(sensor_pin_);
  sensor_candidate_ = sensor_last_;
  candidate_since_ms_ = millis();

  ESP_LOGI("chain",
           "Restored pulse_count=%d dpp=%.4f m  rode=%.1f m  debounce=%d ms  "
           "auto_zero=<%d",
           pulse_count_, dpp_, rode(), debounce_ms_, auto_zero_pulses_);

  emit_outputs();

  event_loop()->onRepeat(10, [this]() { this->sample(); });
  event_loop()->onRepeat(2000, [this]() { this->emit_outputs(); });
  event_loop()->onRepeat(1000, [this]() { this->maybe_save(); });
}

void ChainCounter::apply_boot_auto_zero() {
  if (auto_zero_pulses_ > 0 && pulse_count_ < auto_zero_pulses_) {
    ESP_LOGI("chain", "Boot auto-zero: pulse_count %d < %d", pulse_count_,
             auto_zero_pulses_);
    pulse_count_ = 0;
  }
}

void ChainCounter::sample() {
  const bool chain_up = !digitalRead(up_pin_);
  const bool chain_down = !digitalRead(down_pin_);
  const bool sensor = digitalRead(sensor_pin_);

  if (chain_up) {
    direction_ = -1;
  } else if (chain_down) {
    direction_ = 1;
  }

  up_item_.set(chain_up);
  down_item_.set(chain_down);
  sensor_item_.set(sensor);
  direction_item_.set(String(direction_ < 0 ? "in" : "out"));

  const unsigned long now = millis();
  if (sensor != sensor_candidate_) {
    sensor_candidate_ = sensor;
    candidate_since_ms_ = now;
  }

  const unsigned long stable_ms = now - candidate_since_ms_;
  const bool stable =
      debounce_ms_ <= 0 || stable_ms >= static_cast<unsigned long>(debounce_ms_);

  if (stable && sensor_candidate_ != sensor_last_) {
    sensor_last_ = sensor_candidate_;
    pulse_count_ += direction_;
    if (pulse_count_ < 0) {
      pulse_count_ = 0;
    }
    mark_dirty();
    emit_outputs();
    ESP_LOGD("chain", "pulse=%d dir=%d rode=%.2f m", pulse_count_, direction_,
             rode());
  }
}

void ChainCounter::set_pulse_count(int count) {
  if (count < 0) {
    count = 0;
  }
  if (count == pulse_count_) {
    return;
  }
  pulse_count_ = count;
  mark_dirty();
  emit_outputs();
  ESP_LOGI("chain", "pulse_count set to %d (rode=%.2f m)", pulse_count_,
           rode());
}

void ChainCounter::set_distance_per_pulse(float meters) {
  if (meters <= 0.0f || isnan(meters)) {
    return;
  }
  if (fabsf(meters - dpp_) < 1e-6f) {
    return;
  }
  dpp_ = meters;
  mark_dirty();
  emit_outputs();
  ESP_LOGI("chain", "distance_per_pulse set to %.5f m", dpp_);
}

void ChainCounter::set_rode(float meters) {
  if (isnan(meters) || meters < 0.0f) {
    meters = 0.0f;
  }
  if (dpp_ <= 0.0f) {
    return;
  }
  set_pulse_count(static_cast<int>(lroundf(meters / dpp_)));
}

void ChainCounter::emit_outputs() {
  const float length = rode();
  rode_producer_.emit_value(length);
  pulse_producer_.emit_value(pulse_count_);
  dpp_producer_.emit_value(dpp_);
  rode_item_.set(length);
}

void ChainCounter::mark_dirty() {
  dirty_ = true;
  last_mutation_ms_ = millis();
}

void ChainCounter::maybe_save() {
  if (!dirty_) {
    return;
  }
  if (millis() - last_mutation_ms_ < kSaveIdleMs) {
    return;
  }
  save();
  dirty_ = false;
  ESP_LOGI("chain", "Saved pulse_count=%d dpp=%.4f after idle", pulse_count_,
           dpp_);
}

bool ChainCounter::to_json(JsonObject& root) {
  root["pulse_count"] = pulse_count_;
  root["distance_per_pulse"] = dpp_;
  root["debounce_ms"] = debounce_ms_;
  root["auto_zero_pulses"] = auto_zero_pulses_;
  return true;
}

bool ChainCounter::from_json(const JsonObject& config) {
  if (!config["pulse_count"].isNull()) {
    pulse_count_ = config["pulse_count"].as<int>();
    if (pulse_count_ < 0) {
      pulse_count_ = 0;
    }
  }
  if (!config["distance_per_pulse"].isNull()) {
    float dpp = config["distance_per_pulse"].as<float>();
    if (dpp > 0.0f) {
      dpp_ = dpp;
    }
  }
  if (!config["debounce_ms"].isNull()) {
    int ms = config["debounce_ms"].as<int>();
    debounce_ms_ = ms < 0 ? 0 : ms;
  }
  if (!config["auto_zero_pulses"].isNull()) {
    int z = config["auto_zero_pulses"].as<int>();
    auto_zero_pulses_ = z < 0 ? 0 : z;
  }
  emit_outputs();
  return true;
}

const String ConfigSchema(const ChainCounter& obj) {
  return R"json({
    "type": "object",
    "properties": {
      "pulse_count": {
        "title": "Pulse count",
        "type": "integer",
        "description": "Gypsy pulses since chain fully retrieved. PUT 0 to zero the counter."
      },
      "distance_per_pulse": {
        "title": "Distance per pulse in metres",
        "type": "number",
        "description": "Metres of rode per gypsy pulse."
      },
      "debounce_ms": {
        "title": "Sensor debounce (ms)",
        "type": "integer",
        "description": "Ignore gypsy edges until the pin is stable this long. 0 = no debounce."
      },
      "auto_zero_pulses": {
        "title": "Boot auto-zero below N pulses",
        "type": "integer",
        "description": "On boot, treat a stored count below this as fully retrieved (0). 0 disables. Only applied at startup, not when saving this form."
      }
    }
  })json";
}

}  // namespace sensesp
