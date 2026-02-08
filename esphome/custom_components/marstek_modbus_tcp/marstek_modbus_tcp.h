#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

#ifdef USE_ETHERNET
#include <Ethernet.h>
using NetClient = EthernetClient;
#else
#include <WiFi.h>
using NetClient = WiFiClient;
#endif

namespace esphome {
namespace marstek_modbus_tcp {

class ModbusTcpClient {
 public:
  ModbusTcpClient(const std::string &host, uint16_t port, uint8_t unit_id, uint32_t timeout_ms = 1000);

  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }

  bool read_holding_registers(uint16_t start_address, uint16_t count, std::vector<uint16_t> &out);
  bool write_single_register(uint16_t address, uint16_t value);

 private:
  bool connect_();
  bool send_request_(const std::vector<uint8_t> &pdu);
  bool read_response_(std::vector<uint8_t> &pdu_out);

  std::string host_;
  uint16_t port_;
  uint8_t unit_id_;
  uint16_t transaction_id_{0};
  uint32_t timeout_ms_{1000};

  NetClient client_;
};

static const char *const TAG = "marstek_modbus_tcp";

inline ModbusTcpClient::ModbusTcpClient(const std::string &host, uint16_t port, uint8_t unit_id,
                                        uint32_t timeout_ms)
    : host_(host), port_(port), unit_id_(unit_id), timeout_ms_(timeout_ms) {}

inline bool ModbusTcpClient::connect_() {
  if (this->client_.connected())
    return true;

  this->client_.stop();
  this->client_.setTimeout(this->timeout_ms_ / 1000.0f);
  if (!this->client_.connect(this->host_.c_str(), this->port_)) {
    ESP_LOGW(TAG, "Connect failed to %s:%u", this->host_.c_str(), this->port_);
    return false;
  }
  return true;
}

inline bool ModbusTcpClient::send_request_(const std::vector<uint8_t> &pdu) {
  if (!this->connect_())
    return false;

  uint16_t tx_id = ++this->transaction_id_;
  uint16_t protocol_id = 0;
  uint16_t length = static_cast<uint16_t>(pdu.size() + 1);

  uint8_t mbap[7];
  mbap[0] = tx_id >> 8;
  mbap[1] = tx_id & 0xFF;
  mbap[2] = protocol_id >> 8;
  mbap[3] = protocol_id & 0xFF;
  mbap[4] = length >> 8;
  mbap[5] = length & 0xFF;
  mbap[6] = this->unit_id_;

  size_t written = this->client_.write(mbap, sizeof(mbap));
  written += this->client_.write(pdu.data(), pdu.size());

  if (written != sizeof(mbap) + pdu.size()) {
    ESP_LOGW(TAG, "Write incomplete (%u/%u)", (unsigned) written, (unsigned) (sizeof(mbap) + pdu.size()));
    this->client_.stop();
    return false;
  }
  return true;
}

inline bool ModbusTcpClient::read_response_(std::vector<uint8_t> &pdu_out) {
  uint8_t mbap[7];
  size_t got = this->client_.readBytes(mbap, sizeof(mbap));
  if (got != sizeof(mbap)) {
    ESP_LOGW(TAG, "MBAP read timeout (%u/%u)", (unsigned) got, (unsigned) sizeof(mbap));
    this->client_.stop();
    return false;
  }

  uint16_t length = (uint16_t(mbap[4]) << 8) | mbap[5];
  if (length < 2) {
    ESP_LOGW(TAG, "Invalid length %u", (unsigned) length);
    return false;
  }

  uint16_t pdu_len = length - 1;
  pdu_out.resize(pdu_len);
  got = this->client_.readBytes(pdu_out.data(), pdu_len);
  if (got != pdu_len) {
    ESP_LOGW(TAG, "PDU read timeout (%u/%u)", (unsigned) got, (unsigned) pdu_len);
    this->client_.stop();
    return false;
  }

  return true;
}

inline bool ModbusTcpClient::read_holding_registers(uint16_t start_address, uint16_t count,
                                                    std::vector<uint16_t> &out) {
  if (count == 0)
    return false;

  std::vector<uint8_t> pdu;
  pdu.reserve(5);
  pdu.push_back(0x03);
  pdu.push_back(start_address >> 8);
  pdu.push_back(start_address & 0xFF);
  pdu.push_back(count >> 8);
  pdu.push_back(count & 0xFF);

  if (!this->send_request_(pdu))
    return false;

  std::vector<uint8_t> resp;
  if (!this->read_response_(resp))
    return false;

  if (resp.size() < 2 || resp[0] != 0x03) {
    ESP_LOGW(TAG, "Read response invalid");
    return false;
  }

  uint8_t byte_count = resp[1];
  if (byte_count != count * 2 || resp.size() < 2 + byte_count) {
    ESP_LOGW(TAG, "Read response byte count mismatch (%u)", (unsigned) byte_count);
    return false;
  }

  out.resize(count);
  for (uint16_t i = 0; i < count; i++) {
    uint8_t hi = resp[2 + (i * 2)];
    uint8_t lo = resp[2 + (i * 2) + 1];
    out[i] = (uint16_t(hi) << 8) | lo;
  }

  return true;
}

inline bool ModbusTcpClient::write_single_register(uint16_t address, uint16_t value) {
  std::vector<uint8_t> pdu;
  pdu.reserve(5);
  pdu.push_back(0x06);
  pdu.push_back(address >> 8);
  pdu.push_back(address & 0xFF);
  pdu.push_back(value >> 8);
  pdu.push_back(value & 0xFF);

  if (!this->send_request_(pdu))
    return false;

  std::vector<uint8_t> resp;
  if (!this->read_response_(resp))
    return false;

  if (resp.size() < 5 || resp[0] != 0x06) {
    ESP_LOGW(TAG, "Write response invalid");
    return false;
  }

  return true;
}

}  // namespace marstek_modbus_tcp
}  // namespace esphome
