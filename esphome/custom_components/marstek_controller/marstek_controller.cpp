#include "marstek_controller.h"

#include "esphome/core/log.h"

namespace esphome {
namespace marstek_controller {

static const char *const TAG = "marstek_controller";

MarstekBatteryComponent::MarstekBatteryComponent(const std::string &host, uint16_t port, uint8_t unit_id,
                                                 uint32_t timeout_ms)
    : PollingComponent(5000), client_(host, port, unit_id, timeout_ms) {}

bool MarstekBatteryComponent::set_rs485_control(bool enable) {
  uint16_t value = enable ? 21930 : 21947;
  bool ok = this->client_.write_single_register(42000, value);
  if (ok)
    this->rs485_enabled_ = enable;
  return ok;
}

bool MarstekBatteryComponent::set_work_mode(uint16_t mode) {
  return this->client_.write_single_register(43000, mode);
}

bool MarstekBatteryComponent::set_max_charge_power(uint16_t power_w) {
  bool ok = this->client_.write_single_register(44002, power_w);
  if (ok)
    this->max_charge_power_ = power_w;
  return ok;
}

bool MarstekBatteryComponent::set_max_discharge_power(uint16_t power_w) {
  bool ok = this->client_.write_single_register(44003, power_w);
  if (ok)
    this->max_discharge_power_ = power_w;
  return ok;
}

bool MarstekBatteryComponent::apply_command(Mode mode, uint16_t power_w) {
  if (!this->rs485_enabled_) {
    if (!this->set_rs485_control(true))
      return false;
  }

  if (mode == this->last_mode_ && power_w == this->last_power_)
    return true;

  if (mode == Mode::STOP) {
    if (!this->client_.write_single_register(42010, 0))
      return false;
    this->last_mode_ = mode;
    this->last_power_ = 0;
    return true;
  }

  if (mode == Mode::CHARGE) {
    if (!this->client_.write_single_register(42020, power_w))
      return false;
    if (!this->client_.write_single_register(42010, 1))
      return false;
  } else if (mode == Mode::DISCHARGE) {
    if (!this->client_.write_single_register(42021, power_w))
      return false;
    if (!this->client_.write_single_register(42010, 2))
      return false;
  }

  this->last_mode_ = mode;
  this->last_power_ = power_w;
  return true;
}

static int16_t to_s16(uint16_t v) { return static_cast<int16_t>(v); }

static int32_t to_s32(uint16_t hi, uint16_t lo) {
  uint32_t u = (uint32_t(hi) << 16) | lo;
  return static_cast<int32_t>(u);
}

void MarstekBatteryComponent::update() {
  std::vector<uint16_t> regs;

  if (this->client_.read_holding_registers(30001, 1, regs)) {
    int16_t raw = to_s16(regs[0]);
    this->power_sensor_->publish_state(raw);
  }

  if (this->client_.read_holding_registers(34002, 1, regs)) {
    float soc = regs[0] * 0.1f;
    this->soc_ = soc;
    this->soc_sensor_->publish_state(soc);
  }

  if (this->client_.read_holding_registers(32105, 1, regs)) {
    float total = regs[0] * 0.001f;
    this->total_energy_kwh_ = total;
    this->total_energy_sensor_->publish_state(total);
  }

  if (this->client_.read_holding_registers(32202, 2, regs)) {
    int32_t raw = to_s32(regs[0], regs[1]);
    this->ac_power_sensor_->publish_state(raw);
  }
}

float MarstekController::get_number_(number::Number *num, float fallback) const {
  if (num == nullptr)
    return fallback;
  if (std::isnan(num->state))
    return fallback;
  return num->state;
}

int MarstekController::get_minutes_(number::Number *num, int fallback) const {
  float v = this->get_number_(num, fallback);
  return static_cast<int>(v);
}

Strategy MarstekController::parse_strategy_(const std::string &name) const {
  if (name == "Full stop")
    return Strategy::FULL_STOP;
  if (name == "Self-consumption")
    return Strategy::SELF_CONSUMPTION;
  if (name == "Timed")
    return Strategy::TIMED;
  if (name == "Dynamic")
    return Strategy::DYNAMIC;
  if (name == "Charge")
    return Strategy::CHARGE;
  if (name == "Charge PV")
    return Strategy::CHARGE_PV;
  if (name == "Sell")
    return Strategy::SELL;
  return Strategy::UNKNOWN;
}

Strategy MarstekController::parse_sub_strategy_(const std::string &name) const {
  if (name == "Full stop")
    return Strategy::FULL_STOP;
  if (name == "Self-consumption")
    return Strategy::SELF_CONSUMPTION;
  if (name == "Charge")
    return Strategy::CHARGE;
  if (name == "Charge PV")
    return Strategy::CHARGE_PV;
  if (name == "Sell")
    return Strategy::SELL;
  return Strategy::UNKNOWN;
}

MasterMode MarstekController::parse_master_mode_(const std::string &name) const {
  if (name == "Manual control")
    return MasterMode::MANUAL;
  if (name == "Marstek control")
    return MasterMode::MARSTEK;
  if (name == "Full control")
    return MasterMode::FULL;
  return MasterMode::UNKNOWN;
}

Strategy MarstekController::resolve_timed_() {
  if (this->time_ == nullptr || !this->time_->now().is_valid())
    return this->parse_sub_strategy_(this->timed_default_ ? this->timed_default_->state : "Self-consumption");

  auto now = this->time_->now();
  int minutes = now.hour * 60 + now.minute;

  int a_start = this->get_minutes_(this->period_a_start_, 0);
  int a_end = this->get_minutes_(this->period_a_end_, 0);

  bool has_b = this->timed_has_b_ && this->timed_has_b_->state == "on";
  bool has_c = this->timed_has_c_ && this->timed_has_c_->state == "on";

  int b_start = this->get_minutes_(this->period_b_start_, 0);
  int b_end = this->get_minutes_(this->period_b_end_, 0);
  int c_start = this->get_minutes_(this->period_c_start_, 0);
  int c_end = this->get_minutes_(this->period_c_end_, 0);

  auto in_range = [](int now_m, int start, int end) {
    if (start == end)
      return false;
    if (start < end)
      return now_m >= start && now_m < end;
    return now_m >= start || now_m < end;  // wraps midnight
  };

  if (in_range(minutes, a_start, a_end))
    return this->parse_sub_strategy_(this->timed_a_ ? this->timed_a_->state : "Self-consumption");

  if (has_b && in_range(minutes, b_start, b_end))
    return this->parse_sub_strategy_(this->timed_b_ ? this->timed_b_->state : "Self-consumption");

  if (has_c && in_range(minutes, c_start, c_end))
    return this->parse_sub_strategy_(this->timed_c_ ? this->timed_c_->state : "Self-consumption");

  return this->parse_sub_strategy_(this->timed_default_ ? this->timed_default_->state : "Self-consumption");
}

Strategy MarstekController::resolve_dynamic_() {
  float avg_cheapest = this->get_number_(this->dyn_avg_cheapest_, 0);
  float avg_expensive = this->get_number_(this->dyn_avg_expensive_, 0);
  float threshold_cheapest = this->get_number_(this->dyn_threshold_cheapest_, 0);
  float threshold_delta = this->get_number_(this->dyn_threshold_delta_, 0);

  float cheapest_start = this->get_number_(this->dyn_cheapest_start_, 0);
  float cheapest_end = this->get_number_(this->dyn_cheapest_end_, 0);
  float expensive_start = this->get_number_(this->dyn_expensive_start_, 0);
  float expensive_end = this->get_number_(this->dyn_expensive_end_, 0);

  bool now_in_cheapest = false;
  bool now_in_expensive = false;

  if (this->time_ && this->time_->now().is_valid()) {
    auto now = this->time_->now().timestamp;
    now_in_cheapest = now >= cheapest_start && now < cheapest_end;
    now_in_expensive = now >= expensive_start && now < expensive_end;
  }

  float delta = avg_expensive - avg_cheapest;

  if (now_in_cheapest && avg_cheapest <= threshold_cheapest)
    return this->parse_sub_strategy_(this->dyn_cheapest_ ? this->dyn_cheapest_->state : "Charge");

  if (now_in_expensive && delta >= threshold_delta)
    return this->parse_sub_strategy_(this->dyn_expensive_ ? this->dyn_expensive_->state : "Sell");

  return this->parse_sub_strategy_(this->dyn_default_ ? this->dyn_default_->state : "Self-consumption");
}

Strategy MarstekController::select_strategy_() {
  Strategy base = this->parse_strategy_(this->strategy_select_ ? this->strategy_select_->state : "Self-consumption");
  if (base == Strategy::TIMED)
    return this->resolve_timed_();
  if (base == Strategy::DYNAMIC)
    return this->resolve_dynamic_();
  return base;
}

BatteryCommand MarstekController::compute_command_(Strategy strategy, float grid_power_w, float total_energy_kwh,
                                                   float avg_soc) {
  BatteryCommand cmd;

  float target_grid = this->get_number_(this->target_grid_power_, 0.0f);
  float hysteresis = this->get_number_(this->hysteresis_, 0.0f);
  float kp = this->get_number_(this->pid_kp_, 0.0f);
  float ki = this->get_number_(this->pid_ki_, 0.0f);
  float kd = this->get_number_(this->pid_kd_, 0.0f);
  float out_damp = this->get_number_(this->output_dampening_, 0.0f) / 100.0f;
  float err_damp = this->get_number_(this->error_dampening_, 0.0f) / 100.0f;

  if (strategy == Strategy::FULL_STOP) {
    cmd.mode = Mode::STOP;
    cmd.power_w = 0;
    return cmd;
  }

  if (strategy == Strategy::CHARGE) {
    float target_power = this->get_number_(this->charge_target_power_, 0.0f);
    float target_soc = this->get_number_(this->charge_target_soc_, 100.0f);
    float target_energy = this->get_number_(this->charge_target_energy_, 0.0f);

    bool stop = false;
    if (this->charge_goal_ && this->charge_goal_->state == "state of charge")
      stop = avg_soc >= target_soc;
    else if (this->charge_goal_ && this->charge_goal_->state == "energy reserve")
      stop = total_energy_kwh >= target_energy;

    if (stop) {
      cmd.mode = Mode::STOP;
      cmd.power_w = 0;
    } else {
      cmd.mode = Mode::CHARGE;
      cmd.power_w = static_cast<uint16_t>(std::max(0.0f, target_power));
    }
    return cmd;
  }

  if (strategy == Strategy::SELL) {
    float target_power = this->get_number_(this->sell_target_power_, 0.0f);
    float target_soc = this->get_number_(this->sell_target_soc_, 12.0f);
    float target_energy = this->get_number_(this->sell_target_energy_, 0.0f);

    bool stop = false;
    if (this->sell_goal_ && this->sell_goal_->state == "state of charge")
      stop = avg_soc <= target_soc;
    else if (this->sell_goal_ && this->sell_goal_->state == "energy reserve")
      stop = total_energy_kwh <= target_energy;

    if (stop) {
      cmd.mode = Mode::STOP;
      cmd.power_w = 0;
    } else {
      cmd.mode = Mode::DISCHARGE;
      cmd.power_w = static_cast<uint16_t>(std::max(0.0f, target_power));
    }
    return cmd;
  }

  if (strategy == Strategy::CHARGE_PV) {
    // Charge only when exporting to grid (negative grid power)
    if (grid_power_w < -hysteresis) {
      cmd.mode = Mode::CHARGE;
      cmd.power_w = static_cast<uint16_t>(std::min(-grid_power_w, 10000.0f));
    } else {
      cmd.mode = Mode::STOP;
      cmd.power_w = 0;
    }
    return cmd;
  }

  // Self-consumption PID control (also used as baseline for timed/dynamic when selected)
  float error = grid_power_w - target_grid;
  float damped_error = error * (1.0f - err_damp) + this->pid_prev_error_ * err_damp;

  this->pid_integral_ += damped_error;
  float derivative = damped_error - this->pid_prev_error_;

  float output = (kp * damped_error) + (ki * this->pid_integral_) + (kd * derivative);
  float damped_output = output * (1.0f - out_damp) + this->pid_prev_output_ * out_damp;

  this->pid_prev_error_ = damped_error;
  this->pid_prev_output_ = damped_output;

  if (damped_output > hysteresis) {
    cmd.mode = Mode::DISCHARGE;
    cmd.power_w = static_cast<uint16_t>(std::max(0.0f, damped_output));
  } else if (damped_output < -hysteresis) {
    cmd.mode = Mode::CHARGE;
    cmd.power_w = static_cast<uint16_t>(std::max(0.0f, -damped_output));
  } else {
    cmd.mode = Mode::STOP;
    cmd.power_w = 0;
  }

  return cmd;
}

void MarstekController::dispatch_command_(const BatteryCommand &cmd) {
  if (this->batteries_.empty())
    return;

  float idle_minutes = this->get_number_(this->idle_minutes_, 0.0f);
  uint32_t now_ms = millis();

  if (cmd.mode == Mode::STOP || cmd.power_w == 0) {
    if (idle_minutes > 0 && this->last_active_ms_ != 0) {
      uint32_t idle_ms = static_cast<uint32_t>(idle_minutes * 60.0f * 1000.0f);
      if (now_ms - this->last_active_ms_ < idle_ms)
        return;  // keep last state until idle timeout reached
    }
  } else {
    this->last_active_ms_ = now_ms;
  }

  // total max limits
  uint32_t total_max = 0;
  for (auto *bat : this->batteries_) {
    total_max += (cmd.mode == Mode::CHARGE) ? bat->max_charge_power() : bat->max_discharge_power();
  }
  uint32_t target_power = std::min<uint32_t>(cmd.power_w, total_max);

  // priority order
  std::vector<MarstekBatteryComponent *> ordered = this->batteries_;
  int priority = static_cast<int>(this->get_number_(this->priority_battery_, 1));
  if (priority >= 1 && priority <= (int) ordered.size()) {
    std::rotate(ordered.begin(), ordered.begin() + (priority - 1), ordered.end());
  }

  uint32_t remaining = target_power;
  for (auto *bat : ordered) {
    if (cmd.mode == Mode::CHARGE && !bat->can_charge()) {
      bat->apply_command(Mode::STOP, 0);
      continue;
    }
    if (cmd.mode == Mode::DISCHARGE && !bat->can_discharge()) {
      bat->apply_command(Mode::STOP, 0);
      continue;
    }

    uint16_t limit = (cmd.mode == Mode::CHARGE) ? bat->max_charge_power() : bat->max_discharge_power();
    uint16_t assign = static_cast<uint16_t>(std::min<uint32_t>(remaining, limit));
    if (assign == 0) {
      bat->apply_command(Mode::STOP, 0);
      continue;
    }

    bat->apply_command(cmd.mode, assign);
    remaining -= assign;
    if (remaining == 0)
      break;
  }

  if (remaining > 0) {
    for (auto *bat : ordered) {
      if (remaining == 0)
        break;
      uint16_t limit = (cmd.mode == Mode::CHARGE) ? bat->max_charge_power() : bat->max_discharge_power();
      uint16_t assign = static_cast<uint16_t>(std::min<uint32_t>(remaining, limit));
      if (assign == 0)
        continue;
      bat->apply_command(cmd.mode, assign);
      remaining -= assign;
    }
  }
}

void MarstekController::update() {
  if (this->grid_power_w_ == nullptr)
    return;

  MasterMode master_mode = this->parse_master_mode_(this->master_mode_select_ ? this->master_mode_select_->state :
                                                                         "Full control");
  if (master_mode == MasterMode::MARSTEK || master_mode == MasterMode::MANUAL) {
    for (auto *bat : this->batteries_) {
      bat->set_rs485_control(false);
      if (master_mode == MasterMode::MANUAL)
        bat->set_work_mode(0);
    }
    return;
  }

  float grid_power_w = this->grid_power_w_->state;
  if (std::isnan(grid_power_w))
    return;

  float total_energy = 0.0f;
  float soc_sum = 0.0f;
  int soc_count = 0;
  for (auto *bat : this->batteries_) {
    if (!std::isnan(bat->total_energy_kwh()))
      total_energy += bat->total_energy_kwh();
    if (!std::isnan(bat->soc())) {
      soc_sum += bat->soc();
      soc_count++;
    }
  }
  float avg_soc = soc_count > 0 ? soc_sum / soc_count : 0.0f;

  Strategy strategy = this->select_strategy_();
  BatteryCommand cmd = this->compute_command_(strategy, grid_power_w, total_energy, avg_soc);

  this->dispatch_command_(cmd);
}

}  // namespace marstek_controller
}  // namespace esphome
