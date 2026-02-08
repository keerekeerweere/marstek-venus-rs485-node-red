#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/switch/switch.h"

#include "../marstek_modbus_tcp/marstek_modbus_tcp.h"

namespace esphome {
namespace marstek_controller {

static const char *const TAG = "marstek_controller";

enum class Strategy { FULL_STOP, SELF_CONSUMPTION, TIMED, DYNAMIC, CHARGE, CHARGE_PV, SELL, UNKNOWN };

enum class Mode { STOP, CHARGE, DISCHARGE };

enum class MasterMode { MANUAL, MARSTEK, FULL, UNKNOWN };

struct BatteryCommand {
  Mode mode{Mode::STOP};
  uint16_t power_w{0};
};

class MarstekBatteryComponent : public PollingComponent {
 public:
  MarstekBatteryComponent(const std::string &host, uint16_t port, uint8_t unit_id, uint32_t timeout_ms)
      : PollingComponent(5000), client_(host, port, unit_id, timeout_ms) {}

  void set_label(const std::string &label) { this->label_ = label; }

  float soc() const { return this->soc_; }
  float total_energy_kwh() const { return this->total_energy_kwh_; }
  float battery_power_w() const { return this->battery_power_w_; }
  float ac_power_w() const { return this->ac_power_w_; }

  bool set_rs485_control(bool enable) {
    uint16_t value = enable ? 21930 : 21947;
    bool ok = this->client_.write_single_register(42000, value);
    if (ok)
      this->rs485_enabled_ = enable;
    return ok;
  }

  bool set_work_mode(uint16_t mode) { return this->client_.write_single_register(43000, mode); }

  bool set_max_charge_power(uint16_t power_w) {
    bool ok = this->client_.write_single_register(44002, power_w);
    if (ok)
      this->max_charge_power_ = power_w;
    return ok;
  }

  bool set_max_discharge_power(uint16_t power_w) {
    bool ok = this->client_.write_single_register(44003, power_w);
    if (ok)
      this->max_discharge_power_ = power_w;
    return ok;
  }

  bool apply_command(Mode mode, uint16_t power_w) {
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

  void set_soc_cutoff_charge(float soc) { this->soc_charge_cutoff_ = soc; }
  void set_soc_cutoff_discharge(float soc) { this->soc_discharge_cutoff_ = soc; }
  void set_max_power_charge(uint16_t power_w) { this->max_charge_power_ = power_w; }
  void set_max_power_discharge(uint16_t power_w) { this->max_discharge_power_ = power_w; }

  bool can_charge() const { return this->soc_ < this->soc_charge_cutoff_; }
  bool can_discharge() const { return this->soc_ > this->soc_discharge_cutoff_; }

  uint16_t max_charge_power() const { return this->max_charge_power_; }
  uint16_t max_discharge_power() const { return this->max_discharge_power_; }

  void update() override {
    std::vector<uint16_t> regs;

    if (this->client_.read_holding_registers(30001, 1, regs)) {
      int16_t raw = static_cast<int16_t>(regs[0]);
      this->battery_power_w_ = raw;
    }

    if (this->client_.read_holding_registers(34002, 1, regs)) {
      float soc = regs[0] * 0.1f;
      this->soc_ = soc;
    }

    if (this->client_.read_holding_registers(32105, 1, regs)) {
      float total = regs[0] * 0.001f;
      this->total_energy_kwh_ = total;
    }

    if (this->client_.read_holding_registers(32202, 2, regs)) {
      int32_t raw = (int32_t(uint32_t(regs[0]) << 16) | regs[1]);
      this->ac_power_w_ = raw;
    }
  }

 protected:
  marstek_modbus_tcp::ModbusTcpClient client_;
  std::string label_;

  float soc_{NAN};
  float total_energy_kwh_{NAN};
  float battery_power_w_{NAN};
  float ac_power_w_{NAN};

  float soc_charge_cutoff_{100.0f};
  float soc_discharge_cutoff_{12.0f};
  uint16_t max_charge_power_{2500};
  uint16_t max_discharge_power_{2500};

  bool rs485_enabled_{false};
  Mode last_mode_{Mode::STOP};
  uint16_t last_power_{0};
};

class MarstekController : public PollingComponent {
 public:
  void set_time_source(time::RealTimeClock *time) { this->time_ = time; }
  void set_grid_power_sensor(sensor::Sensor *sensor) { this->grid_power_w_ = sensor; }

  void set_strategy_select(select::Select *sel) { this->strategy_select_ = sel; }
  void set_master_mode_select(select::Select *sel) { this->master_mode_select_ = sel; }

  void set_timed_default_strategy(select::Select *sel) { this->timed_default_ = sel; }
  void set_timed_strat_a(select::Select *sel) { this->timed_a_ = sel; }
  void set_timed_strat_b(select::Select *sel) { this->timed_b_ = sel; }
  void set_timed_strat_c(select::Select *sel) { this->timed_c_ = sel; }

  void set_timed_period_a_start(number::Number *num) { this->period_a_start_ = num; }
  void set_timed_period_a_end(number::Number *num) { this->period_a_end_ = num; }
  void set_timed_period_b_start(number::Number *num) { this->period_b_start_ = num; }
  void set_timed_period_b_end(number::Number *num) { this->period_b_end_ = num; }
  void set_timed_period_c_start(number::Number *num) { this->period_c_start_ = num; }
  void set_timed_period_c_end(number::Number *num) { this->period_c_end_ = num; }
  void set_timed_has_b(switch_::Switch *sw) { this->timed_has_b_ = sw; }
  void set_timed_has_c(switch_::Switch *sw) { this->timed_has_c_ = sw; }

  void set_dynamic_default_strategy(select::Select *sel) { this->dyn_default_ = sel; }
  void set_dynamic_cheapest_strategy(select::Select *sel) { this->dyn_cheapest_ = sel; }
  void set_dynamic_expensive_strategy(select::Select *sel) { this->dyn_expensive_ = sel; }
  void set_dynamic_threshold_cheapest(number::Number *num) { this->dyn_threshold_cheapest_ = num; }
  void set_dynamic_threshold_delta(number::Number *num) { this->dyn_threshold_delta_ = num; }
  void set_dynamic_cheapest_start(sensor::Sensor *sensor) { this->dyn_cheapest_start_ = sensor; }
  void set_dynamic_cheapest_end(sensor::Sensor *sensor) { this->dyn_cheapest_end_ = sensor; }
  void set_dynamic_expensive_start(sensor::Sensor *sensor) { this->dyn_expensive_start_ = sensor; }
  void set_dynamic_expensive_end(sensor::Sensor *sensor) { this->dyn_expensive_end_ = sensor; }
  void set_dynamic_avg_cheapest(sensor::Sensor *sensor) { this->dyn_avg_cheapest_ = sensor; }
  void set_dynamic_avg_expensive(sensor::Sensor *sensor) { this->dyn_avg_expensive_ = sensor; }

  void set_target_grid_power(number::Number *num) { this->target_grid_power_ = num; }
  void set_pid_kp(number::Number *num) { this->pid_kp_ = num; }
  void set_pid_ki(number::Number *num) { this->pid_ki_ = num; }
  void set_pid_kd(number::Number *num) { this->pid_kd_ = num; }
  void set_hysteresis(number::Number *num) { this->hysteresis_ = num; }
  void set_idle_time(number::Number *num) { this->idle_minutes_ = num; }
  void set_output_dampening(number::Number *num) { this->output_dampening_ = num; }
  void set_error_dampening(number::Number *num) { this->error_dampening_ = num; }

  void set_charge_target_power(number::Number *num) { this->charge_target_power_ = num; }
  void set_charge_target_soc(number::Number *num) { this->charge_target_soc_ = num; }
  void set_charge_target_energy(number::Number *num) { this->charge_target_energy_ = num; }
  void set_charge_goal(select::Select *sel) { this->charge_goal_ = sel; }

  void set_sell_target_power(number::Number *num) { this->sell_target_power_ = num; }
  void set_sell_target_soc(number::Number *num) { this->sell_target_soc_ = num; }
  void set_sell_target_energy(number::Number *num) { this->sell_target_energy_ = num; }
  void set_sell_goal(select::Select *sel) { this->sell_goal_ = sel; }

  void set_priority_battery(number::Number *num) { this->priority_battery_ = num; }

  void add_battery(MarstekBatteryComponent *battery) { this->batteries_.push_back(battery); }
  void add_battery_config(number::Number *charge_cutoff, number::Number *discharge_cutoff,
                          number::Number *max_charge, number::Number *max_discharge) {
    this->battery_charge_cutoffs_.push_back(charge_cutoff);
    this->battery_discharge_cutoffs_.push_back(discharge_cutoff);
    this->battery_max_charge_.push_back(max_charge);
    this->battery_max_discharge_.push_back(max_discharge);
  }

  void update() override {
    if (this->grid_power_w_ == nullptr)
      return;

    for (size_t i = 0; i < this->batteries_.size(); i++) {
      if (i < this->battery_charge_cutoffs_.size())
        this->batteries_[i]->set_soc_cutoff_charge(this->get_number_(this->battery_charge_cutoffs_[i], 100));
      if (i < this->battery_discharge_cutoffs_.size())
        this->batteries_[i]->set_soc_cutoff_discharge(this->get_number_(this->battery_discharge_cutoffs_[i], 12));
      if (i < this->battery_max_charge_.size())
        this->batteries_[i]->set_max_power_charge((uint16_t) this->get_number_(this->battery_max_charge_[i], 2500));
      if (i < this->battery_max_discharge_.size())
        this->batteries_[i]->set_max_power_discharge((uint16_t) this->get_number_(this->battery_max_discharge_[i], 2500));
    }

    MasterMode master_mode =
        this->parse_master_mode_(this->master_mode_select_ ? this->master_mode_select_->state : "Full control");
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
    this->last_strategy_ = strategy;
    this->last_cmd_ = cmd;

    this->dispatch_command_(cmd);
  }

  Strategy last_strategy() const { return this->last_strategy_; }
  BatteryCommand last_command() const { return this->last_cmd_; }

 protected:
  Strategy parse_strategy_(const std::string &name) const {
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

  Strategy parse_sub_strategy_(const std::string &name) const {
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

  MasterMode parse_master_mode_(const std::string &name) const {
    if (name == "Manual control")
      return MasterMode::MANUAL;
    if (name == "Marstek control")
      return MasterMode::MARSTEK;
    if (name == "Full control")
      return MasterMode::FULL;
    return MasterMode::UNKNOWN;
  }

  Strategy resolve_timed_() {
    if (this->time_ == nullptr || !this->time_->now().is_valid())
      return this->parse_sub_strategy_(this->timed_default_ ? this->timed_default_->state : "Self-consumption");

    auto now = this->time_->now();
    int minutes = now.hour * 60 + now.minute;

    int a_start = this->get_minutes_(this->period_a_start_, 0);
    int a_end = this->get_minutes_(this->period_a_end_, 0);

    bool has_b = this->timed_has_b_ ? this->timed_has_b_->state : false;
    bool has_c = this->timed_has_c_ ? this->timed_has_c_->state : false;

    int b_start = this->get_minutes_(this->period_b_start_, 0);
    int b_end = this->get_minutes_(this->period_b_end_, 0);
    int c_start = this->get_minutes_(this->period_c_start_, 0);
    int c_end = this->get_minutes_(this->period_c_end_, 0);

    auto in_range = [](int now_m, int start, int end) {
      if (start == end)
        return false;
      if (start < end)
        return now_m >= start && now_m < end;
      return now_m >= start || now_m < end;
    };

    if (in_range(minutes, a_start, a_end))
      return this->parse_sub_strategy_(this->timed_a_ ? this->timed_a_->state : "Self-consumption");

    if (has_b && in_range(minutes, b_start, b_end))
      return this->parse_sub_strategy_(this->timed_b_ ? this->timed_b_->state : "Self-consumption");

    if (has_c && in_range(minutes, c_start, c_end))
      return this->parse_sub_strategy_(this->timed_c_ ? this->timed_c_->state : "Self-consumption");

    return this->parse_sub_strategy_(this->timed_default_ ? this->timed_default_->state : "Self-consumption");
  }

  Strategy resolve_dynamic_() {
    float avg_cheapest = this->get_sensor_(this->dyn_avg_cheapest_, 0.0f);
    float avg_expensive = this->get_sensor_(this->dyn_avg_expensive_, 0.0f);
    float threshold_cheapest = this->get_number_(this->dyn_threshold_cheapest_, 0.0f);
    float threshold_delta = this->get_number_(this->dyn_threshold_delta_, 0.0f);

    float cheapest_start = this->get_sensor_(this->dyn_cheapest_start_, 0.0f);
    float cheapest_end = this->get_sensor_(this->dyn_cheapest_end_, 0.0f);
    float expensive_start = this->get_sensor_(this->dyn_expensive_start_, 0.0f);
    float expensive_end = this->get_sensor_(this->dyn_expensive_end_, 0.0f);

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

  Strategy select_strategy_() {
    Strategy base = this->parse_strategy_(this->strategy_select_ ? this->strategy_select_->state : "Self-consumption");
    if (base == Strategy::TIMED)
      return this->resolve_timed_();
    if (base == Strategy::DYNAMIC)
      return this->resolve_dynamic_();
    return base;
  }

  BatteryCommand compute_command_(Strategy strategy, float grid_power_w, float total_energy_kwh, float avg_soc) {
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
      if (grid_power_w < -hysteresis) {
        cmd.mode = Mode::CHARGE;
        cmd.power_w = static_cast<uint16_t>(std::min(-grid_power_w, 10000.0f));
      } else {
        cmd.mode = Mode::STOP;
        cmd.power_w = 0;
      }
      return cmd;
    }

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

  void dispatch_command_(const BatteryCommand &cmd) {
    if (this->batteries_.empty())
      return;

    float idle_minutes = this->get_number_(this->idle_minutes_, 0.0f);
    uint32_t now_ms = millis();

    if (cmd.mode == Mode::STOP || cmd.power_w == 0) {
      if (idle_minutes > 0 && this->last_active_ms_ != 0) {
        uint32_t idle_ms = static_cast<uint32_t>(idle_minutes * 60.0f * 1000.0f);
        if (now_ms - this->last_active_ms_ < idle_ms)
          return;
      }
    } else {
      this->last_active_ms_ = now_ms;
    }

    uint32_t total_max = 0;
    for (auto *bat : this->batteries_) {
      total_max += (cmd.mode == Mode::CHARGE) ? bat->max_charge_power() : bat->max_discharge_power();
    }
    uint32_t target_power = std::min<uint32_t>(cmd.power_w, total_max);

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

  float get_number_(number::Number *num, float fallback) const {
    if (num == nullptr)
      return fallback;
    if (std::isnan(num->state))
      return fallback;
    return num->state;
  }

  float get_sensor_(sensor::Sensor *sensor, float fallback) const {
    if (sensor == nullptr)
      return fallback;
    if (std::isnan(sensor->state))
      return fallback;
    return sensor->state;
  }

  int get_minutes_(number::Number *num, int fallback) const {
    float v = this->get_number_(num, fallback);
    return static_cast<int>(v);
  }

  time::RealTimeClock *time_{nullptr};
  sensor::Sensor *grid_power_w_{nullptr};

  select::Select *strategy_select_{nullptr};
  select::Select *master_mode_select_{nullptr};

  select::Select *timed_default_{nullptr};
  select::Select *timed_a_{nullptr};
  select::Select *timed_b_{nullptr};
  select::Select *timed_c_{nullptr};
  number::Number *period_a_start_{nullptr};
  number::Number *period_a_end_{nullptr};
  number::Number *period_b_start_{nullptr};
  number::Number *period_b_end_{nullptr};
  number::Number *period_c_start_{nullptr};
  number::Number *period_c_end_{nullptr};
  switch_::Switch *timed_has_b_{nullptr};
  switch_::Switch *timed_has_c_{nullptr};

  select::Select *dyn_default_{nullptr};
  select::Select *dyn_cheapest_{nullptr};
  select::Select *dyn_expensive_{nullptr};
  number::Number *dyn_threshold_cheapest_{nullptr};
  number::Number *dyn_threshold_delta_{nullptr};
  sensor::Sensor *dyn_cheapest_start_{nullptr};
  sensor::Sensor *dyn_cheapest_end_{nullptr};
  sensor::Sensor *dyn_expensive_start_{nullptr};
  sensor::Sensor *dyn_expensive_end_{nullptr};
  sensor::Sensor *dyn_avg_cheapest_{nullptr};
  sensor::Sensor *dyn_avg_expensive_{nullptr};

  number::Number *target_grid_power_{nullptr};
  number::Number *pid_kp_{nullptr};
  number::Number *pid_ki_{nullptr};
  number::Number *pid_kd_{nullptr};
  number::Number *hysteresis_{nullptr};
  number::Number *idle_minutes_{nullptr};
  number::Number *output_dampening_{nullptr};
  number::Number *error_dampening_{nullptr};

  number::Number *charge_target_power_{nullptr};
  number::Number *charge_target_soc_{nullptr};
  number::Number *charge_target_energy_{nullptr};
  select::Select *charge_goal_{nullptr};

  number::Number *sell_target_power_{nullptr};
  number::Number *sell_target_soc_{nullptr};
  number::Number *sell_target_energy_{nullptr};
  select::Select *sell_goal_{nullptr};

  number::Number *priority_battery_{nullptr};

  std::vector<MarstekBatteryComponent *> batteries_{};
  std::vector<number::Number *> battery_charge_cutoffs_{};
  std::vector<number::Number *> battery_discharge_cutoffs_{};
  std::vector<number::Number *> battery_max_charge_{};
  std::vector<number::Number *> battery_max_discharge_{};

  float pid_integral_{0.0f};
  float pid_prev_error_{0.0f};
  float pid_prev_output_{0.0f};
  uint32_t last_active_ms_{0};
  Strategy last_strategy_{Strategy::UNKNOWN};
  BatteryCommand last_cmd_{};
};

}  // namespace marstek_controller
}  // namespace esphome
