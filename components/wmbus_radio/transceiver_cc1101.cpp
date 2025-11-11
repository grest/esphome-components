#include "transceiver_cc1101.h"

#include <algorithm>
#include <cstring>

#include "cc1101_rf_settings.h"
#include "decode3of6.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wmbus_radio {

static const char *TAG = "wmbus.cc1101";

static constexpr size_t MAX_PACKET_BYTES = 512;
static constexpr uint8_t RX_FIFO_START_THRESHOLD = 0;
static constexpr uint8_t RX_FIFO_THRESHOLD = 10;
static constexpr uint8_t FIXED_PACKET_LENGTH = 0x00;
static constexpr uint8_t INFINITE_PACKET_LENGTH = 0x02;

static constexpr uint32_t INITIAL_WAIT_TIMEOUT_MS = 200;
static constexpr uint32_t DATA_TIMEOUT_MS = 120;

void CC1101::setup() {
  this->common_setup();

  if (this->gdo2_pin_ != nullptr) {
    this->gdo2_pin_->setup();
  }

  ESP_LOGD(TAG, "Resetting CC1101");
  this->reset();
  delay(5);

  this->spi_strobe(CC1101_SRES);
  delay(1);

  this->apply_rf_settings();
  this->apply_frequency();

  this->spi_strobe(CC1101_SCAL);
  delay(1);

  uint8_t version = this->spi_read_status(CC1101_VERSION);
  ESP_LOGCONFIG(TAG, "CC1101 silicon revision: 0x%02X", version);

  this->restart_rx();
}

void CC1101::apply_rf_settings() {
  for (size_t i = 0; i < TMODE_RF_SETTINGS_BYTES.size(); i += 2)
    this->spi_write_register(TMODE_RF_SETTINGS_BYTES[i],
                             TMODE_RF_SETTINGS_BYTES[i + 1]);
}

void CC1101::apply_frequency() {
  // CC1101 expects crystal frequency of 26 MHz
  uint32_t freq_reg =
      static_cast<uint32_t>((this->frequency_mhz_ * 65536.0f) / 26.0f);
  uint8_t freq2 = (freq_reg >> 16) & 0xFF;
  uint8_t freq1 = (freq_reg >> 8) & 0xFF;
  uint8_t freq0 = freq_reg & 0xFF;

  ESP_LOGD(TAG, "Setting CC1101 frequency to %.3f MHz [%02X %02X %02X]",
           this->frequency_mhz_, freq2, freq1, freq0);

  this->spi_write_register(CC1101_FREQ2, freq2);
  this->spi_write_register(CC1101_FREQ1, freq1);
  this->spi_write_register(CC1101_FREQ0, freq0);
}

void CC1101::set_frequency(float frequency_mhz) {
  this->frequency_mhz_ = frequency_mhz;
}

void CC1101::set_sync_mode(bool sync_mode) { this->sync_mode_ = sync_mode; }

void CC1101::set_gdo2_pin(InternalGPIOPin *pin) { this->gdo2_pin_ = pin; }

void CC1101::restart_rx() {
  this->packet_ready_ = false;
  this->packet_buffer_.clear();
  this->bytes_delivered_ = 0;
  this->expected_bytes_ = 0;
  this->length_mode_ = LengthMode::INFINITE;

  this->spi_strobe(CC1101_SIDLE);
  auto start = millis();
  while ((this->spi_read_status(CC1101_MARCSTATE) & 0x1F) != MARCSTATE_IDLE) {
    if (millis() - start > 20)
      break;
    delayMicroseconds(50);
  }

  this->spi_strobe(CC1101_SFTX);
  this->spi_strobe(CC1101_SFRX);

  this->spi_write_register(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
  this->spi_write_register(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

  this->spi_strobe(CC1101_SRX);
}

optional<uint8_t> CC1101::read() { return {}; }

bool CC1101::capture_packet() {
  const uint32_t initial_timeout =
      this->sync_mode_ ? INITIAL_WAIT_TIMEOUT_MS * 4 : INITIAL_WAIT_TIMEOUT_MS;
  const uint32_t data_timeout =
      this->sync_mode_ ? DATA_TIMEOUT_MS * 4 : DATA_TIMEOUT_MS;

  bool sync_asserted = this->wait_for_gdo2_assert(initial_timeout);
  if (!sync_asserted) {
    ESP_LOGW(TAG, "Sync timeout, checking FIFO directly");
    if (!this->wait_for_fifo_data(initial_timeout))
      return false;
  }

  auto wait_start = millis();
  while (true) {
    uint8_t status = this->spi_read_status(CC1101_RXBYTES);
    if (status & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow while waiting for data");
      this->handle_fifo_overflow();
      return false;
    }
    if ((status & 0x7F) > 0)
      break;
    if ((millis() - wait_start) > initial_timeout) {
      if (this->wait_for_fifo_data(5))
        break;
      return false;
    }
    delayMicroseconds(100);
  }

  this->packet_buffer_.clear();
  this->packet_buffer_.reserve(MAX_PACKET_BYTES);
  this->expected_bytes_ = 0;
  this->length_mode_ = LengthMode::INFINITE;

  auto last_data_time = millis();

  while (true) {
    uint8_t status = this->spi_read_status(CC1101_RXBYTES);
    if (status & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow during reception");
      this->handle_fifo_overflow();
      return false;
    }

    uint8_t rxbytes = status & 0x7F;
    if (rxbytes == 0) {
      if (this->expected_bytes_ &&
          this->packet_buffer_.size() >= this->expected_bytes_)
        break;
      if ((millis() - last_data_time) > data_timeout)
        return false;
      delayMicroseconds(100);
      continue;
    }

    size_t to_read = rxbytes;

    if (this->expected_bytes_) {
      size_t remaining = this->expected_bytes_ - this->packet_buffer_.size();
      if (remaining <= rxbytes)
        to_read = remaining;
      else if (rxbytes > 1)
        to_read = rxbytes - 1;
      else
        to_read = 1;
    } else {
      size_t header_remaining =
          this->packet_buffer_.size() >= 3 ? 0 : 3 - this->packet_buffer_.size();
      if (header_remaining > 0)
        to_read = std::min<size_t>(rxbytes, header_remaining);
      else if (rxbytes > 1)
        to_read = rxbytes - 1;
      else
        to_read = 1;
    }

    this->read_fifo_into_buffer(to_read);
    last_data_time = millis();

    if (!this->expected_bytes_ && this->packet_buffer_.size() >= 3) {
      size_t length = this->calculate_expected_length();
      if (length == 0 || length > MAX_PACKET_BYTES) {
        ESP_LOGW(TAG, "Unsupported packet length: %zu", length);
        this->handle_fifo_overflow();
        return false;
      }
      this->expected_bytes_ = length;
      this->configure_length_registers(length);
      ESP_LOGV(TAG, "Expecting %zu bytes from CC1101", length);

      if (this->packet_buffer_.size() >= this->expected_bytes_)
        break;
    }
  }

  while (this->expected_bytes_ &&
         this->packet_buffer_.size() < this->expected_bytes_) {
    uint8_t status = this->spi_read_status(CC1101_RXBYTES);
    if (status & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow while draining");
      this->handle_fifo_overflow();
      return false;
    }
    uint8_t rxbytes = status & 0x7F;
    if (rxbytes == 0) {
      if ((millis() - last_data_time) > data_timeout)
        break;
      delayMicroseconds(100);
      continue;
    }
    size_t to_read = std::min<size_t>(
        rxbytes, this->expected_bytes_ - this->packet_buffer_.size());
    this->read_fifo_into_buffer(to_read);
    last_data_time = millis();
  }

  if (this->expected_bytes_ &&
      this->packet_buffer_.size() != this->expected_bytes_) {
    ESP_LOGW(TAG, "Incomplete packet (%zu/%zu)", this->packet_buffer_.size(),
             this->expected_bytes_);
    return false;
  }

  this->last_rssi_ = this->read_rssi_register();

  this->wait_for_irq_level(false, data_timeout);

  this->packet_ready_ = true;
  this->bytes_delivered_ = 0;

  return true;
}

bool CC1101::read_in_task(uint8_t *buffer, size_t length) {
  if (!this->packet_ready_) {
    if (!this->capture_packet())
      return false;
  }

  size_t available = this->packet_buffer_.size() - this->bytes_delivered_;
  if (available < length)
    return false;

  std::memcpy(buffer, this->packet_buffer_.data() + this->bytes_delivered_,
              length);
  this->bytes_delivered_ += length;
  return true;
}

void CC1101::read_fifo_into_buffer(size_t to_read) {
  if (to_read == 0 || this->packet_buffer_.size() >= MAX_PACKET_BYTES)
    return;

  to_read = std::min(to_read, MAX_PACKET_BYTES - this->packet_buffer_.size());

  this->delegate_->begin_transaction();
  this->delegate_->transfer(0xC0 | CC1101_RXFIFO);
  for (size_t i = 0; i < to_read; i++)
    this->packet_buffer_.push_back(this->delegate_->transfer(0x00));
  this->delegate_->end_transaction();
}

size_t CC1101::calculate_expected_length() {
  if (this->packet_buffer_.size() < 3)
    return 0;

  const uint8_t mode = this->packet_buffer_[0];
  const uint8_t block = this->packet_buffer_[1];
  const uint8_t length_field = this->packet_buffer_[2];

  constexpr uint8_t WMBUS_MODE_C_PREAMBLE = 0x54;
  constexpr uint8_t WMBUS_BLOCK_A_PREAMBLE = 0xCD;
  constexpr uint8_t WMBUS_BLOCK_B_PREAMBLE = 0x3D;

  if (mode == WMBUS_MODE_C_PREAMBLE) {
    if (block == WMBUS_BLOCK_A_PREAMBLE)
      return 2 + packet_size(length_field);
    if (block == WMBUS_BLOCK_B_PREAMBLE)
      return 3 + length_field;
    return 0;
  }

  std::vector<uint8_t> encoded(this->packet_buffer_.begin(),
                               this->packet_buffer_.begin() + 3);
  auto decoded = decode3of6(encoded);
  if (!decoded || decoded->empty())
    return 0;

  return byte_size(packet_size(decoded->at(0)));
}

void CC1101::configure_length_registers(size_t length) {
  if (length < MAX_PACKET_BYTES && length < 0x100) {
    this->spi_write_register(CC1101_PKTLEN, static_cast<uint8_t>(length));
    this->spi_write_register(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
    this->length_mode_ = LengthMode::FIXED;
  } else {
    this->spi_write_register(CC1101_PKTLEN,
                             static_cast<uint8_t>(length % 0x100));
    this->spi_write_register(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);
    this->length_mode_ = LengthMode::INFINITE;
  }

  this->spi_write_register(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
}

void CC1101::handle_fifo_overflow() {
  this->spi_strobe(CC1101_SIDLE);
  this->spi_strobe(CC1101_SFRX);
  this->spi_strobe(CC1101_SFTX);
  this->spi_strobe(CC1101_SRX);

  this->packet_ready_ = false;
  this->packet_buffer_.clear();
  this->expected_bytes_ = 0;
  this->bytes_delivered_ = 0;
}

int8_t CC1101::read_rssi_register() {
  uint8_t rssi = this->spi_read_status(CC1101_RSSI);
  if (rssi >= 128)
    return static_cast<int8_t>((static_cast<int>(rssi) - 256) / 2 - 74);
  return static_cast<int8_t>((rssi / 2) - 74);
}

int8_t CC1101::get_rssi() { return this->last_rssi_; }

const char *CC1101::get_name() { return TAG; }

bool CC1101::wait_for_irq_level(bool level, uint32_t timeout_ms) {
  if (this->irq_pin_ == nullptr)
    return true;
  auto start = millis();
  while (this->irq_pin_->digital_read() != level) {
    if (millis() - start > timeout_ms)
      return false;
    delayMicroseconds(50);
  }
  return true;
}

bool CC1101::wait_for_gdo2_assert(uint32_t timeout_ms) {
  if (this->gdo2_pin_ == nullptr)
    return true;
  auto start = millis();
  while (!this->gdo2_pin_->digital_read()) {
    if (millis() - start > timeout_ms)
      return false;
    delayMicroseconds(50);
  }
  return true;
}

bool CC1101::wait_for_fifo_data(uint32_t timeout_ms) {
  auto start = millis();
  while (millis() - start <= timeout_ms) {
    uint8_t status = this->spi_read_status(CC1101_RXBYTES);
    if (status & 0x80) {
      ESP_LOGW(TAG, "RX FIFO overflow while probing data");
      this->handle_fifo_overflow();
      return false;
    }
    if (status & 0x7F)
      return true;
    delayMicroseconds(50);
  }
  return false;
}

uint8_t CC1101::spi_read_config(uint8_t address) {
  return this->spi_transaction(0x80, address, {0});
}

uint8_t CC1101::spi_read_status(uint8_t address) {
  return this->spi_transaction(0xC0, address, {0});
}

void CC1101::spi_write_register(uint8_t address, uint8_t value) {
  this->spi_transaction(0x00, address, {value});
}

void CC1101::spi_write_burst(uint8_t address, const uint8_t *data,
                             size_t length) {
  if (length == 0)
    return;
  this->delegate_->begin_transaction();
  this->delegate_->transfer(0x40 | address);
  for (size_t i = 0; i < length; i++)
    this->delegate_->transfer(data[i]);
  this->delegate_->end_transaction();
}

void CC1101::spi_strobe(uint8_t command) {
  this->delegate_->begin_transaction();
  this->delegate_->transfer(command);
  this->delegate_->end_transaction();
}

size_t CC1101::packet_size(uint8_t l_field) {
  uint8_t nr_blocks = (l_field < 26) ? 2 : static_cast<uint8_t>((l_field - 26) / 16 + 3);
  size_t nr_bytes = l_field + 1; // include the L-field target byte
  nr_bytes += 2 * nr_blocks;     // add CRC bytes for every block
  return nr_bytes;
}

size_t CC1101::byte_size(size_t packet_size_value) {
  size_t size = (3 * packet_size_value) / 2;
  if (packet_size_value % 2)
    size += 1;
  return size;
}

}  // namespace wmbus_radio
}  // namespace esphome
