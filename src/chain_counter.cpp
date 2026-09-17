#include "chain_counter.h"

#include <cmath>

#include "sensesp.h"
#include "sensesp_base_app.h"

namespace sensesp {

ChainCounter::ChainCounter(uint8_t sensor_pin, uint8_t up_pin, uint8_t down_pin,
                           const String& config_path)
    : FileSystemSaveable(config_path),
      sensor_pin_{sensor_pin},
      up_pin_{up_pin},
      down_pin_{down_pin} {
  pinMode(sensor_pin_, INPUT);
  pinMode(up_pin_, INPUT);
  pinMode(down_pin_, INPUT);

  load();

  // Original sketch: treat a nearly-home count as fully retrieved.
  if (pulse_count_ < 10) {
    pulse_count_ = 0;
  }
  if (dpp_ <= 0.0f || isnan(dpp_)) {
    dpp_ = 0.1675f;
  }

  sensor_last_ = digitalRead(sensor_pin_);

  ESP_LOGI("chain", "Restored pulse_count=%d dpp=%.4f m  rode=%.1f m",
           pulse_count_, dpp_, rode());

  emit_outputs();

  event_loop()->onRepeat(10, [this]() { this->sample(); });

  event_loop()->onRepeat(2000, [this]() { this->emit_outputs(); });

  event_loop()->onRepeat(5000, [this]() {
    if (dirty_) {
      save();
      dirty_ = false;
    }
  });
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

  if (sensor != sensor_last_) {
    sensor_last_ = sensor;
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

void ChainCounter::emit_outputs() {
  rode_producer_.emit_value(rode());
  pulse_producer_.emit_value(pulse_count_);
  dpp_producer_.emit_value(dpp_);
}

void ChainCounter::mark_dirty() { dirty_ = true; }

bool ChainCounter::to_json(JsonObject& root) {
  root["pulse_count"] = pulse_count_;
  root["distance_per_pulse"] = dpp_;
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
        "description": "Metres of rode per sensor pulse."
      }
    }
  })json";
}

}  // namespace sensesp
