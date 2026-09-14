#include "protocol_megatec.h"
#include "protocol_factory.h"
#include "constants_ups.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace esphome {
namespace ups_hid {

static const char *const MEGATEC_TAG = "ups_hid.megatec";

// ==================== armac transport framing ====================

namespace {

// Space-separated hex, for wire-level debug logs
std::string hex_dump(const uint8_t *data, size_t len) {
  static const char *const DIGITS = "0123456789abcdef";
  std::string out;
  out.reserve(len * 3);
  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      out += ' ';
    }
    out += DIGITS[data[i] >> 4];
    out += DIGITS[data[i] & 0x0f];
  }
  return out;
}

}  // namespace

bool MegatecProtocol::send_command_raw(const std::string &command) {
  // A length byte (0xa0 + len) followed by the command bytes, with no NUL.
  // Matches NUT armac_command_internal() on its interrupt path: qx_process()
  // passes cmdlen = strlen(cmd), so "Q1\r" goes out as a3 51 31 0d.
  const size_t payload_len = command.size();
  if (payload_len == 0 || payload_len + 1 > megatec::PACKET_SIZE) {
    ESP_LOGE(MEGATEC_TAG, "Command length %zu does not fit one packet", payload_len);
    return false;
  }

  uint8_t packet[megatec::PACKET_SIZE] = {0};
  packet[0] = static_cast<uint8_t>(megatec::PACKET_LENGTH_PREFIX + payload_len);
  memcpy(packet + 1, command.data(), payload_len);

  ESP_LOGD(MEGATEC_TAG, "TX %s", hex_dump(packet, payload_len + 1).c_str());
  esp_err_t ret = parent_->interrupt_write(packet, payload_len + 1, megatec::WRITE_TIMEOUT_MS);
  if (ret != ESP_OK) {
    ESP_LOGW(MEGATEC_TAG, "Failed to send command: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}

bool MegatecProtocol::read_response(std::string &response) {
  response.clear();

  for (size_t chunk = 0; chunk < megatec::MAX_READ_CHUNKS; chunk++) {
    uint8_t buffer[megatec::PACKET_SIZE] = {0};
    size_t buffer_len = sizeof(buffer);

    // Only the first read waits for the UPS to turn the command around; any
    // further packets are already queued behind it
    const uint32_t timeout_ms =
        (chunk == 0) ? megatec::FIRST_READ_TIMEOUT_MS : megatec::CHUNK_READ_TIMEOUT_MS;

    esp_err_t ret = parent_->interrupt_read(buffer, &buffer_len, timeout_ms);
    if (ret != ESP_OK || buffer_len == 0) {
      ESP_LOGD(MEGATEC_TAG, "RX chunk %zu: nothing within %ums (%s)", chunk,
               static_cast<unsigned>(timeout_ms), esp_err_to_name(ret));
      // A timeout after we already have data means the reply simply ended
      // without a carriage return, which some units do
      return !response.empty();
    }

    size_t available = buffer[0] & megatec::PACKET_LENGTH_MASK;
    // Only the bytes the control byte claims, not a fixed-size report's padding
    ESP_LOGD(MEGATEC_TAG, "RX chunk %zu: %zu bytes, %s", chunk, buffer_len,
             hex_dump(buffer, std::min(buffer_len, available + 1)).c_str());
    if (available == 0) {
      // Control byte reports an empty buffer - end of transfer
      break;
    }
    // Some units report one more byte than they actually transferred
    if (available > buffer_len - 1) {
      available = buffer_len - 1;
    }

    for (size_t i = 0; i < available; i++) {
      const char c = static_cast<char>(buffer[i + 1]);

      if (c == '\r') {
        return !response.empty();
      }
      if (c == '\0') {
        // A leading NUL means the UPS is powered off; mid-reply it marks the end
        if (response.empty()) {
          ESP_LOGW(MEGATEC_TAG, "Received NUL byte - is the UPS switched off?");
          return false;
        }
        return true;
      }
      if (response.size() >= megatec::MAX_RESPONSE_LENGTH) {
        ESP_LOGW(MEGATEC_TAG, "Response exceeded %zu bytes, truncating",
                 megatec::MAX_RESPONSE_LENGTH);
        return true;
      }
      response += c;
    }
  }

  return !response.empty();
}

bool MegatecProtocol::transact(const std::string &command, std::string &response) {
  // No input drain here on purpose. NUT only drains on armac's control-transfer
  // path, not this interrupt path, and every drained read is a deliberate
  // endpoint timeout - which is exactly the situation worth not provoking.
  if (!send_command_raw(command)) {
    return false;
  }

  if (!read_response(response)) {
    ESP_LOGD(MEGATEC_TAG, "No reply to command '%s'", trim(command).c_str());
    return false;
  }

  ESP_LOGD(MEGATEC_TAG, "Command '%s' -> '%s'", trim(command).c_str(), response.c_str());
  return true;
}

bool MegatecProtocol::transact_no_reply(const std::string &command) {
  // Control commands are not acknowledged by the UPS
  return send_command_raw(command);
}

// ==================== String helpers ====================

std::vector<std::string> MegatecProtocol::split_fields(const std::string &input) {
  std::vector<std::string> fields;
  size_t pos = 0;

  while (pos < input.size()) {
    while (pos < input.size() && input[pos] == ' ') {
      pos++;
    }
    if (pos >= input.size()) {
      break;
    }
    size_t end = input.find(' ', pos);
    if (end == std::string::npos) {
      end = input.size();
    }
    fields.push_back(input.substr(pos, end - pos));
    pos = end;
  }

  return fields;
}

std::string MegatecProtocol::trim(const std::string &input) {
  size_t start = 0;
  size_t end = input.size();

  while (start < end && (input[start] == ' ' || input[start] == '\r' || input[start] == '\n')) {
    start++;
  }
  while (end > start && (input[end - 1] == ' ' || input[end - 1] == '\r' || input[end - 1] == '\n')) {
    end--;
  }

  return input.substr(start, end - start);
}

std::string MegatecProtocol::extract_fixed_field(const std::string &input, size_t offset,
                                                 size_t length) {
  if (offset >= input.size()) {
    return "";
  }
  return trim(input.substr(offset, std::min(length, input.size() - offset)));
}

// ==================== Protocol interface ====================

bool MegatecProtocol::detect() {
  if (!parent_->supports_interrupt_transfer()) {
    ESP_LOGD(MEGATEC_TAG, "Transport has no interrupt endpoint pair - not a Megatec bridge");
    return false;
  }

  // Logged before the exchange because this is the first interrupt transfer the
  // firmware ever performs - if the USB stack faults, this is the last line out
  ESP_LOGI(MEGATEC_TAG, "Probing with Q1 over interrupt endpoints");

  // A valid Q1 reply is the only reliable signal; these units ignore anything
  // they do not understand rather than returning an error
  std::string response;
  if (!transact(megatec::CMD_STATUS, response)) {
    ESP_LOGD(MEGATEC_TAG, "No response to Q1 - not a Megatec device");
    return false;
  }

  if (response.empty() || response[0] != megatec::STATUS_REPLY_PREFIX) {
    ESP_LOGD(MEGATEC_TAG, "Unexpected Q1 reply: '%s'", response.c_str());
    return false;
  }

  const auto fields = split_fields(response.substr(1));
  if (fields.size() < megatec::STATUS_FIELD_COUNT) {
    ESP_LOGD(MEGATEC_TAG, "Q1 reply has %zu fields, expected %zu", fields.size(),
             megatec::STATUS_FIELD_COUNT);
    return false;
  }

  ESP_LOGI(MEGATEC_TAG, "Detected Megatec/Q1 UPS behind Richcomm framing");
  return true;
}

bool MegatecProtocol::initialize() {
  // Ratings and identity never change, so query them once
  std::string response;

  if (transact(megatec::CMD_RATING, response)) {
    parse_rating(response);
  } else {
    ESP_LOGW(MEGATEC_TAG, "Rating query (F) failed - nominal values unavailable");
  }

  if (transact(megatec::CMD_INFO, response)) {
    parse_info(response);
  } else {
    ESP_LOGW(MEGATEC_TAG, "Info query (I) failed - device identity unavailable");
  }

  return true;
}

bool MegatecProtocol::read_data(UpsData &data) {
  std::string response;
  if (!transact(megatec::CMD_STATUS, response)) {
    return false;
  }

  if (!parse_status(response, data)) {
    return false;
  }

  populate_device_info(data);
  return true;
}

// ==================== Reply parsing ====================

bool MegatecProtocol::parse_status(const std::string &response, UpsData &data) {
  // (MMM.M NNN.N PPP.P QQQ RR.R S.SS TT.T b7b6b5b4b3b2b1b0
  if (response.empty() || response[0] != megatec::STATUS_REPLY_PREFIX) {
    ESP_LOGW(MEGATEC_TAG, "Malformed Q1 reply: '%s'", response.c_str());
    return false;
  }

  const auto fields = split_fields(response.substr(1));
  if (fields.size() < megatec::STATUS_FIELD_COUNT) {
    ESP_LOGW(MEGATEC_TAG, "Q1 reply has %zu of %zu fields: '%s'", fields.size(),
             megatec::STATUS_FIELD_COUNT, response.c_str());
    return false;
  }

  data.power.input_voltage = strtof(fields[0].c_str(), nullptr);
  data.power.output_voltage = strtof(fields[2].c_str(), nullptr);
  data.power.load_percent = strtof(fields[3].c_str(), nullptr);
  data.power.frequency = strtof(fields[4].c_str(), nullptr);
  data.battery.voltage = strtof(fields[5].c_str(), nullptr);

  data.power.input_voltage_fault = strtof(fields[1].c_str(), nullptr);
  data.device.temperature = strtof(fields[6].c_str(), nullptr);

  if (!std::isnan(input_voltage_nominal_)) {
    data.power.input_voltage_nominal = input_voltage_nominal_;
    data.power.output_voltage_nominal = input_voltage_nominal_;
  }
  if (!std::isnan(input_current_nominal_)) {
    data.power.input_current_nominal = input_current_nominal_;
  }
  if (!std::isnan(frequency_nominal_)) {
    data.power.frequency_nominal = frequency_nominal_;
  }
  if (!std::isnan(input_voltage_nominal_) && !std::isnan(input_current_nominal_)) {
    // F carries no VA rating; rated voltage x rated current approximates it
    data.power.apparent_power_nominal = input_voltage_nominal_ * input_current_nominal_;
  }
  if (!std::isnan(battery_voltage_nominal_)) {
    data.battery.voltage_nominal = battery_voltage_nominal_;
  }

  apply_status_flags(fields[7], data);
  estimate_battery_charge(data);

  return true;
}

void MegatecProtocol::apply_status_flags(const std::string &flags, UpsData &data) {
  if (flags.size() < megatec::STATUS_FLAG_COUNT) {
    ESP_LOGW(MEGATEC_TAG, "Q1 status field too short: '%s'", flags.c_str());
    return;
  }

  // Megatec status bits, in reply order
  const bool utility_fail = flags[0] == '1';
  const bool battery_low = flags[1] == '1';
  const bool bypass_active = flags[2] == '1';
  const bool ups_failed = flags[3] == '1';
  const bool test_in_progress = flags[5] == '1';
  const bool shutdown_active = flags[6] == '1';
  const bool beeper_on = flags[7] == '1';
  const bool line_interactive = flags[4] == '1';

  // The UPS reports these directly, so they are authoritative - the charge
  // level here is only an estimate and must not be what decides "low battery"
  data.power.status_flags_valid = true;
  data.power.flag_on_battery = utility_fail;
  data.power.flag_fault = ups_failed;
  data.power.flag_shutdown_active = shutdown_active;
  data.battery.status_flags_valid = true;
  data.battery.flag_low_battery = battery_low;

  data.device.ups_type =
      line_interactive ? megatec::UPS_TYPE_LINE_INTERACTIVE : megatec::UPS_TYPE_ONLINE;

  data.power.status = utility_fail ? status::ON_BATTERY : status::ONLINE;
  if (bypass_active) {
    classify_line_regulation(data.power);
    if (data.power.flag_boost) {
      data.power.status += " - Boost";
    } else if (data.power.flag_trim) {
      data.power.status += " - Buck";
    } else if (data.power.flag_bypass) {
      data.power.status += " - Bypass";
    }
  }
  if (ups_failed) {
    data.power.status += " - Fault";
  }
  if (data.power.is_overloaded()) {
    data.power.status += " - Overload";
  }

  if (battery_low) {
    data.battery.status = battery_status::LOW;
  } else if (utility_fail) {
    data.battery.status = battery_status::DISCHARGING;
  } else {
    data.battery.status = battery_status::CHARGING;
  }

  data.test.ups_test_result =
      test_in_progress ? test::RESULT_IN_PROGRESS : test::RESULT_NO_TEST;
  data.test.current_test_state = test_in_progress
                                     ? TestStatus::TEST_STATE_BATTERY_QUICK_RUNNING
                                     : TestStatus::TEST_STATE_IDLE;

  data.config.parse_beeper_status(beeper_on ? "enabled" : "disabled");
  beeper_enabled_ = beeper_on;
  beeper_state_known_ = true;

  if (shutdown_active) {
    ESP_LOGW(MEGATEC_TAG, "UPS reports a shutdown sequence in progress");
  }

  // The delays that the next shutdown command will actually use
  data.config.delay_shutdown = static_cast<int16_t>(shutdown_delay_or_default());
  data.config.delay_start = static_cast<int16_t>(start_delay_or_default());
}

void MegatecProtocol::classify_line_regulation(PowerData &power) {
  if (std::isnan(power.input_voltage) || std::isnan(power.output_voltage) ||
      power.input_voltage <= 0.0f) {
    return;
  }

  // NUT blazer_process_status_bits(): the flag alone does not say which mode
  const float ratio = power.output_voltage / power.input_voltage;
  if (ratio < megatec::LINE_RATIO_MIN || ratio >= megatec::LINE_RATIO_MAX) {
    return;
  }
  if (ratio < megatec::LINE_RATIO_TRIM_BELOW) {
    power.flag_trim = true;
  } else if (ratio < megatec::LINE_RATIO_BOOST_FROM) {
    power.flag_bypass = true;
  } else {
    power.flag_boost = true;
  }
}

float MegatecProtocol::infer_nominal_battery_voltage(float measured_voltage) {
  // Standard pack sizes, picking the smallest one the reading could plausibly
  // belong to. Same approach as NUT, and equally ambiguous at the boundaries -
  // a deeply discharged pack can look like the size below it.
  static constexpr float STANDARD_PACKS[] = {12.0f, 24.0f, 36.0f, 48.0f, 72.0f, 96.0f};
  static constexpr float UPPER_BOUND_FACTOR = 1.25f;

  for (float pack : STANDARD_PACKS) {
    if (measured_voltage < pack * UPPER_BOUND_FACTOR) {
      return pack;
    }
  }
  return NAN;
}

void MegatecProtocol::estimate_battery_charge(UpsData &data) {
  // Megatec reports battery voltage only. NUT derives a charge estimate from
  // per-12V-block bounds, and this reproduces that so the level sensor and the
  // NUT server have a value to publish.
  if (std::isnan(data.battery.voltage) || data.battery.voltage <= 0.0f) {
    return;
  }

  // Without a nominal from the F query there would be no level at all, which
  // would also silence the low-battery flag below, so fall back to a guess
  if (std::isnan(battery_voltage_nominal_) || battery_voltage_nominal_ <= 0.0f) {
    battery_voltage_nominal_ = infer_nominal_battery_voltage(data.battery.voltage);
    if (std::isnan(battery_voltage_nominal_)) {
      return;
    }
    ESP_LOGW(MEGATEC_TAG,
             "No nominal battery voltage from the UPS - inferred %.0fV from a %.1fV reading",
             battery_voltage_nominal_, data.battery.voltage);
    data.battery.voltage_nominal = battery_voltage_nominal_;
  }

  const float blocks =
      roundf(battery_voltage_nominal_ / megatec::BLOCK_VOLTAGE_NOMINAL);
  if (blocks < 1.0f) {
    return;
  }

  const float empty_voltage = blocks * megatec::BLOCK_VOLTAGE_EMPTY;
  const float full_voltage = blocks * megatec::BLOCK_VOLTAGE_FULL;
  if (full_voltage <= empty_voltage) {
    return;
  }

  const float ratio = (data.battery.voltage - empty_voltage) / (full_voltage - empty_voltage);
  data.battery.level =
      std::max(0.0f, std::min(battery::MAX_LEVEL_PERCENT, ratio * battery::MAX_LEVEL_PERCENT));

  ESP_LOGV(MEGATEC_TAG, "Battery %.1fV against %.1f-%.1fV -> %.0f%% (estimated)",
           data.battery.voltage, empty_voltage, full_voltage, data.battery.level);
}

bool MegatecProtocol::parse_rating(const std::string &response) {
  // #MMM.M QQQ SS.SS RR.R - rating voltage, current, battery voltage, frequency
  if (response.empty() || response[0] != megatec::INFO_REPLY_PREFIX) {
    ESP_LOGW(MEGATEC_TAG, "Malformed F reply: '%s'", response.c_str());
    return false;
  }

  const auto fields = split_fields(response.substr(1));
  if (fields.size() < 4) {
    ESP_LOGW(MEGATEC_TAG, "F reply has %zu of 4 fields: '%s'", fields.size(), response.c_str());
    return false;
  }

  input_voltage_nominal_ = strtof(fields[0].c_str(), nullptr);
  input_current_nominal_ = strtof(fields[1].c_str(), nullptr);
  battery_voltage_nominal_ = strtof(fields[2].c_str(), nullptr);
  frequency_nominal_ = strtof(fields[3].c_str(), nullptr);

  ESP_LOGI(MEGATEC_TAG, "Ratings: %.1fV %.1fA, battery %.1fV, %.1fHz",
           input_voltage_nominal_, input_current_nominal_, battery_voltage_nominal_,
           frequency_nominal_);
  return true;
}

bool MegatecProtocol::parse_info(const std::string &response) {
  // Fixed-width fields, frequently blank apart from the firmware version
  if (response.empty() || response[0] != megatec::INFO_REPLY_PREFIX) {
    ESP_LOGW(MEGATEC_TAG, "Malformed I reply: '%s'", response.c_str());
    return false;
  }

  manufacturer_ = extract_fixed_field(response, megatec::INFO_MFR_OFFSET, megatec::INFO_MFR_LENGTH);
  model_ = extract_fixed_field(response, megatec::INFO_MODEL_OFFSET, megatec::INFO_MODEL_LENGTH);
  firmware_ =
      extract_fixed_field(response, megatec::INFO_FIRMWARE_OFFSET, megatec::INFO_FIRMWARE_LENGTH);

  ESP_LOGI(MEGATEC_TAG, "Identity: mfr='%s' model='%s' firmware='%s'", manufacturer_.c_str(),
           model_.c_str(), firmware_.c_str());
  return true;
}

void MegatecProtocol::populate_device_info(UpsData &data) {
  data.device.usb_vendor_id = parent_->get_vendor_id();
  data.device.usb_product_id = parent_->get_product_id();
  data.device.firmware_version = firmware_;

  // Many of these bridges return blank identity fields, so fall back to the USB
  // string descriptors, which carry the actual vendor and product names
  if (!manufacturer_.empty()) {
    data.device.manufacturer = manufacturer_;
  } else {
    std::string descriptor;
    if (parent_->get_string_descriptor(1, descriptor) == ESP_OK) {
      data.device.manufacturer = trim(descriptor);
    }
  }

  if (!model_.empty()) {
    data.device.model = model_;
  } else {
    std::string descriptor;
    if (parent_->get_string_descriptor(2, descriptor) == ESP_OK) {
      data.device.model = trim(descriptor);
    }
  }

  data.device.capabilities.supports_beeper_control = true;
  data.device.capabilities.supports_battery_test = true;
  data.device.capabilities.supports_runtime_estimation = false;
  data.device.capabilities.supports_configuration_queries = false;
}

// ==================== Beeper control ====================

bool MegatecProtocol::beeper_mute() {
  // Megatec only exposes a toggle, which is the closest thing to a mute
  return transact_no_reply(megatec::CMD_BEEPER_TOGGLE);
}

bool MegatecProtocol::beeper_enable() {
  if (!beeper_state_known_) {
    ESP_LOGW(MEGATEC_TAG, "Beeper state unknown - poll the UPS before setting it");
    return false;
  }
  if (beeper_enabled_) {
    ESP_LOGD(MEGATEC_TAG, "Beeper already enabled");
    return true;
  }
  return transact_no_reply(megatec::CMD_BEEPER_TOGGLE);
}

bool MegatecProtocol::beeper_disable() {
  if (!beeper_state_known_) {
    ESP_LOGW(MEGATEC_TAG, "Beeper state unknown - poll the UPS before setting it");
    return false;
  }
  if (!beeper_enabled_) {
    ESP_LOGD(MEGATEC_TAG, "Beeper already disabled");
    return true;
  }
  return transact_no_reply(megatec::CMD_BEEPER_TOGGLE);
}

// ==================== Battery test control ====================

bool MegatecProtocol::start_battery_test_quick() {
  ESP_LOGI(MEGATEC_TAG, "Starting quick battery test");
  return transact_no_reply(megatec::CMD_TEST_QUICK);
}

bool MegatecProtocol::start_battery_test_deep() {
  ESP_LOGI(MEGATEC_TAG, "Starting deep battery test (runs until the battery is low)");
  return transact_no_reply(megatec::CMD_TEST_DEEP);
}

bool MegatecProtocol::stop_battery_test() {
  ESP_LOGI(MEGATEC_TAG, "Stopping battery test");
  return transact_no_reply(megatec::CMD_TEST_STOP);
}

bool MegatecProtocol::start_battery_test_timed(int minutes) {
  if (minutes < megatec::TEST_MINUTES_MIN || minutes > megatec::TEST_MINUTES_MAX) {
    ESP_LOGW(MEGATEC_TAG, "Battery test length %d min is outside %d-%d", minutes,
             megatec::TEST_MINUTES_MIN, megatec::TEST_MINUTES_MAX);
    return false;
  }
  char command[megatec::COMMAND_BUFFER_SIZE];
  snprintf(command, sizeof(command), "T%02d\r", minutes);
  ESP_LOGI(MEGATEC_TAG, "Starting %d-minute battery test", minutes);
  return transact_no_reply(command);
}

// ==================== Output control ====================

std::string MegatecProtocol::format_shutdown_delay(int seconds) {
  // Same encoding as NUT blazer_process_command(), clamped to the accepted range
  const int clamped =
      std::max(megatec::SHUTDOWN_DELAY_MIN_S, std::min(megatec::SHUTDOWN_DELAY_MAX_S, seconds));
  char buffer[megatec::COMMAND_BUFFER_SIZE];
  if (clamped < 60) {
    snprintf(buffer, sizeof(buffer), ".%d", clamped / 6);
  } else {
    snprintf(buffer, sizeof(buffer), "%02d", clamped / 60);
  }
  return buffer;
}

int MegatecProtocol::shutdown_delay_or_default() const {
  return shutdown_delay_seconds_ >= 0 ? shutdown_delay_seconds_ : megatec::DEFAULT_SHUTDOWN_DELAY_S;
}

int MegatecProtocol::start_delay_or_default() const {
  return start_delay_seconds_ >= 0 ? start_delay_seconds_ : megatec::DEFAULT_START_DELAY_S;
}

bool MegatecProtocol::shutdown_return() {
  // S<n> alone restores the output when mains returns; R<m> restores it after
  // m minutes instead
  std::string command = "S" + format_shutdown_delay(shutdown_delay_or_default());
  const int restore_minutes = std::min(megatec::RESTORE_MINUTES_MAX, start_delay_or_default() / 60);
  if (restore_minutes > 0) {
    char restore[megatec::COMMAND_BUFFER_SIZE];
    snprintf(restore, sizeof(restore), "R%04d", restore_minutes);
    command += restore;
  }
  command += "\r";

  ESP_LOGW(MEGATEC_TAG, "Shutting down UPS output (%s)", trim(command).c_str());
  return transact_no_reply(command);
}

bool MegatecProtocol::shutdown_stayoff() {
  const std::string command = "S" + format_shutdown_delay(shutdown_delay_or_default()) + "R0000\r";
  ESP_LOGW(MEGATEC_TAG, "Shutting down UPS output until switched back on (%s)",
           trim(command).c_str());
  return transact_no_reply(command);
}

bool MegatecProtocol::shutdown_cancel() {
  ESP_LOGI(MEGATEC_TAG, "Cancelling UPS shutdown");
  return transact_no_reply(megatec::CMD_CANCEL_SHUTDOWN);
}

bool MegatecProtocol::load_off() {
  ESP_LOGW(MEGATEC_TAG, "Switching UPS output off now");
  return transact_no_reply(megatec::CMD_LOAD_OFF);
}

bool MegatecProtocol::load_on() {
  ESP_LOGI(MEGATEC_TAG, "Switching UPS output on");
  return transact_no_reply(megatec::CMD_CANCEL_SHUTDOWN);
}

// ==================== Delay configuration ====================

bool MegatecProtocol::set_shutdown_delay(int seconds) {
  // On Megatec the delay is an argument to the shutdown command rather than a
  // stored setting, so keep it locally for whoever issues that command
  shutdown_delay_seconds_ = seconds;
  ESP_LOGI(MEGATEC_TAG, "Shutdown delay set to %d s (applied when shutting down)", seconds);
  return true;
}

bool MegatecProtocol::set_start_delay(int seconds) {
  start_delay_seconds_ = seconds;
  ESP_LOGI(MEGATEC_TAG, "Start delay set to %d s (applied when shutting down)", seconds);
  return true;
}

// ==================== Registration ====================

std::unique_ptr<UpsProtocolBase> create_megatec_protocol(UpsHidComponent *parent) {
  return std::make_unique<MegatecProtocol>(parent);
}

}  // namespace ups_hid
}  // namespace esphome
