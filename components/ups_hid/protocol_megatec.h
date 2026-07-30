#pragma once

#include "ups_hid.h"
#include <cmath>
#include <string>
#include <vector>

namespace esphome {
namespace ups_hid {

/**
 * @brief Megatec / Q* Protocol Implementation (Richcomm "armac" USB framing)
 *
 * Based on NUT nutdrv_qx.c - megatec subdriver over the armac USB subdriver.
 *
 * Unlike the APC and CyberPower protocols, this is not a HID Power Device at
 * all: the UPS exposes a HID-class interface that merely tunnels the classic
 * Megatec ASCII command set. Commands go out on the interrupt OUT endpoint and
 * replies come back on the interrupt IN endpoint, both wrapped in a one-byte
 * length header.
 *
 * Verified against a Lakeview/Richcomm bridge (0925:1234, "UPS USB Mon V2.0"):
 *   Q1 -> "(231.0 000.0 232.0 014 50.0 27.0 20.8 00001001"
 *   F  -> "#220.0 005 24.00 50.0"
 *   I  -> "#                           V3.65     "
 *
 * Protocol limitations, inherent to Megatec rather than this implementation:
 * - No runtime estimate is reported at all.
 * - No battery charge percentage; it is derived from battery voltage against
 *   per-12V-block bounds, exactly as NUT does.
 * - The beeper can only be toggled, never set to an absolute state.
 * - Shutdown/start delays are arguments to the shutdown command rather than
 *   stored device settings, so they are held locally.
 */
class MegatecProtocol : public UpsProtocolBase {
 public:
  explicit MegatecProtocol(UpsHidComponent *parent) : UpsProtocolBase(parent) {}
  ~MegatecProtocol() override = default;

  bool detect() override;
  bool initialize() override;
  bool read_data(UpsData &data) override;

  DeviceInfo::DetectedProtocol get_protocol_type() const override {
    return DeviceInfo::PROTOCOL_MEGATEC;
  }
  std::string get_protocol_name() const override { return "Megatec Q1"; }

  // Beeper control - hardware only offers a toggle, so absolute requests are
  // resolved against the state reported by Q1
  bool beeper_enable() override;
  bool beeper_disable() override;
  bool beeper_mute() override;

  // Battery test control
  bool start_battery_test_quick() override;
  bool start_battery_test_deep() override;
  bool stop_battery_test() override;

  // Delay configuration (held locally, applied to shutdown commands)
  bool set_shutdown_delay(int seconds) override;
  bool set_start_delay(int seconds) override;

 private:
  // armac transport framing
  bool send_command_raw(const std::string &command);
  bool read_response(std::string &response);
  bool transact(const std::string &command, std::string &response);
  bool transact_no_reply(const std::string &command);

  // Reply parsers
  bool parse_status(const std::string &response, UpsData &data);
  bool parse_rating(const std::string &response);
  bool parse_info(const std::string &response);
  void apply_status_flags(const std::string &flags, UpsData &data);
  void estimate_battery_charge(UpsData &data);
  void populate_device_info(UpsData &data);

  // Guesses the pack size from a measured voltage when the F query gave us no
  // nominal value. Returns NAN if no standard pack size fits.
  static float infer_nominal_battery_voltage(float measured_voltage);

  static std::vector<std::string> split_fields(const std::string &input);
  static std::string extract_fixed_field(const std::string &input, size_t offset, size_t length);
  static std::string trim(const std::string &input);

  // Cached values from the one-shot F and I queries
  float input_voltage_nominal_{NAN};
  float input_current_nominal_{NAN};
  float battery_voltage_nominal_{NAN};
  float frequency_nominal_{NAN};
  std::string manufacturer_{};
  std::string model_{};
  std::string firmware_{};

  // Beeper state as last reported by Q1, needed to emulate absolute control
  bool beeper_enabled_{false};
  bool beeper_state_known_{false};

  // Delay settings are command arguments on this protocol, not device state
  int shutdown_delay_seconds_{-1};
  int start_delay_seconds_{-1};

  // Set when a transaction failed, triggering a buffer drain before the next one
  bool needs_drain_{false};

  void drain_stale_input();
};

}  // namespace ups_hid
}  // namespace esphome
