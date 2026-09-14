#include "control_button.h"
#include "constants_ups.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ups_hid {

static const char *const BUTTON_TAG = "ups_hid.button";

void UpsHidButton::dump_config() {
  ESP_LOGCONFIG(BUTTON_TAG, "UPS HID Button:");
  if (button_type_ == BUTTON_TYPE_BEEPER) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Beeper action: %s", beeper_action_.c_str());
  } else if (button_type_ == BUTTON_TYPE_TEST) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Test action: %s", test_action_.c_str());
    if (test_action_ == test::ACTION_BATTERY_TIMED) {
      ESP_LOGCONFIG(BUTTON_TAG, "  Test duration: %d min", test_duration_minutes_);
    }
  } else if (button_type_ == BUTTON_TYPE_SHUTDOWN) {
    ESP_LOGCONFIG(BUTTON_TAG, "  Shutdown action: %s", shutdown_action_.c_str());
    if (is_destructive()) {
      ESP_LOGCONFIG(BUTTON_TAG, "  Requires confirmation: %s",
                    require_confirmation_ ? status::YES : status::NO);
    }
  }
}

const std::string &UpsHidButton::action_name() const {
  switch (button_type_) {
    case BUTTON_TYPE_TEST:
      return test_action_;
    case BUTTON_TYPE_SHUTDOWN:
      return shutdown_action_;
    default:
      return beeper_action_;
  }
}

bool UpsHidButton::is_destructive() const {
  return button_type_ == BUTTON_TYPE_SHUTDOWN &&
         (shutdown_action_ == shutdown::ACTION_RETURN || shutdown_action_ == shutdown::ACTION_STAYOFF ||
          shutdown_action_ == shutdown::ACTION_LOAD_OFF);
}

bool UpsHidButton::confirm_press() {
  const uint32_t now = millis();
  if (armed_ && now - armed_at_ms_ <= shutdown::CONFIRM_WINDOW_MS) {
    armed_ = false;
    return true;
  }

  armed_ = true;
  armed_at_ms_ = now;
  ESP_LOGW(BUTTON_TAG, "'%s' cuts power to everything on the UPS - press again within %us to confirm",
           shutdown_action_.c_str(), static_cast<unsigned>(shutdown::CONFIRM_WINDOW_MS / 1000));
  return false;
}

void UpsHidButton::press_action() {
  if (!parent_) {
    ESP_LOGE(BUTTON_TAG, log_messages::NO_PARENT_COMPONENT);
    return;
  }

  if (!parent_->is_connected()) {
    ESP_LOGW(BUTTON_TAG, "UPS not connected, cannot execute action: %s", action_name().c_str());
    return;
  }

  // Get the current protocol from the parent component
  UpsData ups_data = parent_->get_ups_data();
  if (ups_data.device.detected_protocol == DeviceInfo::PROTOCOL_UNKNOWN) {
    ESP_LOGW(BUTTON_TAG, "UPS protocol not detected, cannot execute button action");
    return;
  }

  // Get the active protocol
  auto active_protocol = parent_->get_active_protocol();
  if (!active_protocol) {
    ESP_LOGE(BUTTON_TAG, "No active protocol available");
    return;
  }

  if (is_destructive() && require_confirmation_ && !confirm_press()) {
    return;
  }

  bool success = false;
  const std::string &action = action_name();
  ESP_LOGI(BUTTON_TAG, "Executing action: %s", action.c_str());

  if (button_type_ == BUTTON_TYPE_BEEPER) {
    if (action == beeper::ACTION_ENABLE) {
      success = active_protocol->beeper_enable();
    } else if (action == beeper::ACTION_DISABLE) {
      success = active_protocol->beeper_disable();
    } else if (action == beeper::ACTION_MUTE) {
      success = active_protocol->beeper_mute();
    } else if (action == beeper::ACTION_TEST) {
      success = active_protocol->beeper_test();
    } else {
      ESP_LOGE(BUTTON_TAG, "Unknown beeper action: %s", action.c_str());
      return;
    }
  } else if (button_type_ == BUTTON_TYPE_TEST) {
    if (action == test::ACTION_BATTERY_QUICK) {
      success = active_protocol->start_battery_test_quick();
    } else if (action == test::ACTION_BATTERY_DEEP) {
      success = active_protocol->start_battery_test_deep();
    } else if (action == test::ACTION_BATTERY_TIMED) {
      success = active_protocol->start_battery_test_timed(test_duration_minutes_);
    } else if (action == test::ACTION_BATTERY_STOP) {
      success = active_protocol->stop_battery_test();
    } else if (action == test::ACTION_UPS_TEST) {
      success = active_protocol->start_ups_test();
    } else if (action == test::ACTION_UPS_STOP) {
      success = active_protocol->stop_ups_test();
    } else {
      ESP_LOGE(BUTTON_TAG, "Unknown test action: %s", action.c_str());
      return;
    }
  } else if (button_type_ == BUTTON_TYPE_SHUTDOWN) {
    if (action == shutdown::ACTION_RETURN) {
      success = active_protocol->shutdown_return();
    } else if (action == shutdown::ACTION_STAYOFF) {
      success = active_protocol->shutdown_stayoff();
    } else if (action == shutdown::ACTION_CANCEL) {
      success = active_protocol->shutdown_cancel();
    } else if (action == shutdown::ACTION_LOAD_OFF) {
      success = active_protocol->load_off();
    } else if (action == shutdown::ACTION_LOAD_ON) {
      success = active_protocol->load_on();
    } else {
      ESP_LOGE(BUTTON_TAG, "Unknown shutdown action: %s", action.c_str());
      return;
    }
  }

  if (success) {
    ESP_LOGI(BUTTON_TAG, "Action '%s' executed successfully", action.c_str());
  } else {
    ESP_LOGW(BUTTON_TAG, "Failed to execute action '%s' (not supported by this UPS?)", action.c_str());
  }
}

}  // namespace ups_hid
}  // namespace esphome
