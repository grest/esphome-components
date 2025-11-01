#pragma once

#include "transceiver.h"

#include <vector>

namespace esphome {
namespace wmbus_radio {

class CC1101 : public RadioTransceiver {
public:
  void setup() override;
  optional<uint8_t> read() override;
  bool read_in_task(uint8_t *buffer, size_t length) override;
  void restart_rx() override;
  int8_t get_rssi() override;
  const char *get_name() override;

  void set_frequency(float frequency_mhz);
  void set_sync_mode(bool sync_mode);
  void set_gdo2_pin(InternalGPIOPin *pin);

private:
  bool capture_packet();
  void read_fifo_into_buffer(size_t to_read);
  size_t calculate_expected_length();
  void configure_length_registers(size_t length);
  void apply_rf_settings();
  void apply_frequency();
  void handle_fifo_overflow();
  int8_t read_rssi_register();

  uint8_t spi_read_config(uint8_t address);
  uint8_t spi_read_status(uint8_t address);
  void spi_write_register(uint8_t address, uint8_t value);
  void spi_write_burst(uint8_t address, const uint8_t *data, size_t length);
  void spi_strobe(uint8_t command);

  static size_t packet_size(uint8_t l_field);
  static size_t byte_size(size_t packet_size);

  std::vector<uint8_t> packet_buffer_;
  size_t bytes_delivered_{0};
  size_t expected_bytes_{0};

  bool packet_ready_{false};

  float frequency_mhz_{868.950f};
  bool sync_mode_{false};

  int8_t last_rssi_{0};
  InternalGPIOPin *gdo2_pin_{nullptr};

  enum class LengthMode : uint8_t { INFINITE, FIXED };
  LengthMode length_mode_{LengthMode::INFINITE};
};

}  // namespace wmbus_radio
}  // namespace esphome
