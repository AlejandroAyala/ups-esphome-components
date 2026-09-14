#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "constants_ups.h"
#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

enum UpsHidBeeperAction {
  UPS_HID_BEEPER_ENABLE,
  UPS_HID_BEEPER_DISABLE,
  UPS_HID_BEEPER_MUTE,
  UPS_HID_BEEPER_TEST
};

enum UpsHidTestAction {
  UPS_HID_TEST_BATTERY_QUICK,
  UPS_HID_TEST_BATTERY_DEEP,
  UPS_HID_TEST_BATTERY_STOP,
  UPS_HID_TEST_UPS_TEST,
  UPS_HID_TEST_UPS_STOP
};

class UpsHidButton : public button::Button, public Component {
 public:
  void set_ups_hid_parent(UpsHidComponent *parent) { parent_ = parent; }
  void set_beeper_action(const std::string &action) { beeper_action_ = action; button_type_ = BUTTON_TYPE_BEEPER; }
  void set_test_action(const std::string &action) { test_action_ = action; button_type_ = BUTTON_TYPE_TEST; }
  void set_shutdown_action(const std::string &action) { shutdown_action_ = action; button_type_ = BUTTON_TYPE_SHUTDOWN; }
  void set_test_duration_minutes(int minutes) { test_duration_minutes_ = minutes; }
  void set_require_confirmation(bool require) { require_confirmation_ = require; }

  void dump_config() override;

 protected:
  void press_action() override;

  enum ButtonType {
    BUTTON_TYPE_BEEPER,
    BUTTON_TYPE_TEST,
    BUTTON_TYPE_SHUTDOWN
  };

  const std::string &action_name() const;
  // True for actions that cut power to the protected load
  bool is_destructive() const;
  // Arms on the first press; true only for a second press inside the window
  bool confirm_press();

  UpsHidComponent *parent_{nullptr};
  std::string beeper_action_{};
  std::string test_action_{};
  std::string shutdown_action_{};
  ButtonType button_type_{BUTTON_TYPE_BEEPER};
  int test_duration_minutes_{test::DEFAULT_TIMED_TEST_MINUTES};
  bool require_confirmation_{true};
  bool armed_{false};
  uint32_t armed_at_ms_{0};
};

}  // namespace ups_hid
}  // namespace esphome
