#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/ds248x/ds248x.h"

using namespace esphome::ds248x;

namespace esphome {
namespace ds18x {

class DS18xTemperatureSensor : public DS248xSensor {
 public:
  /// Get the set resolution for this sensor.
  uint8_t get_resolution() const;
  /// Set the resolution for this sensor.
  void set_resolution(uint8_t resolution);
  /// Get the number of milliseconds we have to wait for the conversion phase.
  uint16_t millis_to_wait_for_conversion() const;
  
  float get_temp_c();
  float get_value();

  bool setup_sensor();

  bool update();

 protected:
   uint8_t resolution_;
};

}
}