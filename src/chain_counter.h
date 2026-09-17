#ifndef CHAIN_COUNTER_H_
#define CHAIN_COUNTER_H_

#include "sensesp/system/saveable.h"
#include "sensesp/system/valueproducer.h"

namespace sensesp {

/**
 * Bidirectional windlass chain counter.
 *
 * UP/DOWN optocouplers set direction; a gypsy sensor pulse increments or
 * decrements pulse_count. Rode length = pulse_count * distance_per_pulse.
 * Pulse count and metres-per-pulse are persisted on the filesystem.
 */
class ChainCounter : public FileSystemSaveable {
 public:
  ChainCounter(uint8_t sensor_pin, uint8_t up_pin, uint8_t down_pin,
               const String& config_path);

  void set_pulse_count(int count);
  void set_distance_per_pulse(float meters);

  int pulse_count() const { return pulse_count_; }
  float distance_per_pulse() const { return dpp_; }
  float rode() const { return pulse_count_ * dpp_; }
  int direction() const { return direction_; }

  ValueProducer<float>& rode_producer() { return rode_producer_; }
  ValueProducer<int>& pulse_producer() { return pulse_producer_; }
  ValueProducer<float>& dpp_producer() { return dpp_producer_; }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;

 private:
  class IntProducer : public ValueProducer<int> {
   public:
    void emit_value(int value) { this->emit(value); }
  };
  class FloatProducer : public ValueProducer<float> {
   public:
    void emit_value(float value) { this->emit(value); }
  };

  void sample();
  void emit_outputs();
  void mark_dirty();

  uint8_t sensor_pin_;
  uint8_t up_pin_;
  uint8_t down_pin_;

  int pulse_count_ = 0;
  float dpp_ = 0.1675f;
  int direction_ = 1;  // 1 = down (out), -1 = up (in)
  bool sensor_last_ = false;
  bool dirty_ = false;

  FloatProducer rode_producer_;
  IntProducer pulse_producer_;
  FloatProducer dpp_producer_;
};

const String ConfigSchema(const ChainCounter& obj);

}  // namespace sensesp

#endif
