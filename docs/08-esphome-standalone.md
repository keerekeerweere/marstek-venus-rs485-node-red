# ESPHome Standalone Controller (ESP32-S3)

This repo now includes an ESPHome-based standalone controller that runs the full battery strategy logic on an ESP32-S3 and talks directly to Marstek Venus V3 batteries via Modbus TCP. It exposes all controls and sensors to Home Assistant via the ESPHome API.

## Files
- `esphome/marstek_controller.yaml`
- `esphome/custom_components/marstek_modbus_tcp/marstek_modbus_tcp.h`
- `esphome/custom_components/marstek_controller/marstek_controller.h`

## What it does
- Reads P1 grid power via DSMR (UART)
- Runs all strategies locally: Full stop, Self-consumption (PID), Timed, Dynamic, Charge, Charge PV, Sell
- Commands up to 3 batteries via Modbus TCP (one IP per battery)
- Exposes configuration (numbers, selects, switches) and telemetry to Home Assistant

## Quick setup
1. Configure Wi-Fi and UART pins in `esphome/marstek_controller.yaml`.
2. Set battery IPs in `substitutions`:
   - `marstek_m1_ip` (required)
   - `marstek_m2_ip` and `marstek_m3_ip` (optional; keep `0.0.0.0` to disable)
3. Flash the ESP32-S3.
4. Add the device to Home Assistant via ESPHome.

## Dynamic pricing inputs
The controller expects timestamp and average price sensors from Home Assistant:
- `sensor.house_battery_strategy_dynamic_cheapest_start_ts`
- `sensor.house_battery_strategy_dynamic_cheapest_end_ts`
- `sensor.house_battery_strategy_dynamic_expensive_start_ts`
- `sensor.house_battery_strategy_dynamic_expensive_end_ts`
- `sensor.house_battery_strategy_dynamic_cheapest_avg_tariff`
- `sensor.house_battery_strategy_dynamic_expensive_avg_tariff`

If you already have the original Node-RED setup, you can create HA template sensors that expose these values.

## Notes
- Modbus TCP registers are based on the Marstek Venus V3 map (EMS V144+).
- The controller uses the built-in force-charge/discharge registers (42010/42020/42021).
