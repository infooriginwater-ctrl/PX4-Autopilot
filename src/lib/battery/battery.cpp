/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_platform_common/defines.h>

// ── ADDED: include file I/O for cycle count persistence on SD card ──────────
#include <stdio.h>

using namespace time_literals;
using namespace matrix;

// ── ADDED: SD card file path to save cycle count across reboots ─────────────
// PX4_STORAGEDIR expands to /fs/microsd on real hardware
static constexpr const char *CYCLE_COUNT_FILE = PX4_STORAGEDIR "/battery_cycles.txt";

// ── ADDED: Charge cycle detection thresholds ─────────────────────────────────
// A full cycle = SoC drops below LOW_THR then recovers above HIGH_THR
// These match real LiPo charge/discharge behaviour
static constexpr float CYCLE_LOW_THRESHOLD  = 0.20f; // 20% — considered discharged
static constexpr float CYCLE_HIGH_THRESHOLD = 0.95f; // 95% — considered fully charged


// ════════════════════════════════════════════════════════════════════════════
// CONSTRUCTOR
// Original code: sets up filters and finds all BAT%d_* parameters
// ADDED: loads saved cycle count from SD card on boot
// ════════════════════════════════════════════════════════════════════════════

Battery::Battery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source) :
	ModuleParams(parent),
	_index(index < 1 || index > 9 ? 1 : index),
	_source(source)
{
	// ORIGINAL: set low-pass filter time constants for current averaging and voltage
	const float expected_filter_dt = static_cast<float>(sample_interval_us) / 1_s;
	_current_average_filter_a.setParameters(expected_filter_dt, 50.f);
	_ocv_filter_v.setParameters(expected_filter_dt, 1.f);
	_cell_voltage_filter_v.setParameters(expected_filter_dt, 1.f);

	if (index > 9 || index < 1) {
		PX4_ERR("Battery index must be between 1 and 9 (inclusive). Received %d. Defaulting to 1.", index);
	}

	// ORIGINAL: build parameter names like "BAT1_V_EMPTY", "BAT2_V_EMPTY" etc.
	char param_name[17];

	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMPTY", _index);
	_param_handles.v_empty = param_find(param_name);

	if (_param_handles.v_empty == PARAM_INVALID) {
		PX4_ERR("Could not find parameter with name %s", param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHARGED", _index);
	_param_handles.v_charged = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_N_CELLS", _index);
	_param_handles.n_cells = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_CAPACITY", _index);
	_param_handles.capacity = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_R_INTERNAL", _index);
	_param_handles.r_internal = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOURCE", _index);
	_param_handles.source = param_find(param_name);

	_param_handles.low_thr    = param_find("BAT_LOW_THR");
	_param_handles.crit_thr   = param_find("BAT_CRIT_THR");
	_param_handles.emergen_thr = param_find("BAT_EMERGEN_THR");
	_param_handles.bat_avrg_current = param_find("BAT_AVRG_CURRENT");

	updateParams();

	// ── ADDED: Load saved charge cycle count from SD card ────────────────
	// Called here so cycle count is restored immediately on boot before
	// any battery messages are processed
	loadCycleCount();
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: simple voltage/current/temperature setters
// These are called by the SMBUS or ADC driver with raw sensor readings
// ════════════════════════════════════════════════════════════════════════════

void Battery::updateVoltage(const float voltage_v)
{
	_voltage_v = voltage_v;
}

void Battery::updateCurrent(const float current_a)
{
	_current_a = current_a;
}

void Battery::updateTemperature(const float temperature_c)
{
	_temperature_c = temperature_c;
}


// ════════════════════════════════════════════════════════════════════════════
// updateBatteryStatus()
// ORIGINAL: main update function — called every loop from the battery driver
// Checks connection, runs filters, estimates SoC, computes warnings
// ADDED: call trackChargeCycle() here so cycle tracking runs every loop
// ════════════════════════════════════════════════════════════════════════════

void Battery::updateBatteryStatus(const hrt_abstime &timestamp)
{
	// ORIGINAL: calculate dt since last call, capped at 2 seconds
	updateDt(timestamp);

	// ORIGINAL: if voltage is below minimum lithium recognition voltage,
	// mark battery as disconnected
	if (_voltage_v < LITHIUM_BATTERY_RECOGNITION_VOLTAGE) {
		_connected = false;
	}

	// ORIGINAL: record timestamp of last disconnection for filter warm-up
	if (!_connected || (_last_unconnected_timestamp == 0)) {
		_last_unconnected_timestamp = timestamp;
	}

	// ORIGINAL: battery is only considered "initialized" if it has been
	// connected for at least 2 seconds — avoids bad readings on plug-in spike
	_battery_initialized = _connected && (timestamp > _last_unconnected_timestamp + 2_s);

	// ORIGINAL: reset internal resistance estimation when battery just connected
	if (_connected && !_battery_initialized && _internal_resistance_initialized && _params.n_cells > 0) {
		resetInternalResistanceEstimation(_voltage_v, _current_a);
	}

	// ORIGINAL: accumulate mAh discharged since boot
	sumDischarged(_current_a);

	// ORIGINAL: compute voltage-based SoC using cell voltage vs empty/charged params
	_state_of_charge_volt_based =
		calculateStateOfChargeVoltageBased(_voltage_v, _current_a);

	// ORIGINAL: fuse voltage-based and current-based SoC estimates
	if (!_external_state_of_charge) {
		estimateStateOfCharge();
	}

	// ORIGINAL: compute throttle scale factor for power compensation
	computeScale();

	// ORIGINAL: determine warning level (NONE / LOW / CRITICAL / EMERGENCY)
	if (_connected && _battery_initialized) {
		_warning = determineWarning(_state_of_charge);
	}

	// ── ADDED: track charge cycles using the final estimated SoC ─────────
	// Only track when battery is fully initialized to avoid false counts
	// during plug-in voltage spikes
	if (_connected && _battery_initialized) {
		trackChargeCycle(_state_of_charge);
	}
}


// ════════════════════════════════════════════════════════════════════════════
// getBatteryStatus()
// ORIGINAL: packages all computed values into battery_status_s struct
// ADDED: populate cycle_count field from our local tracker
// ════════════════════════════════════════════════════════════════════════════

battery_status_s Battery::getBatteryStatus()
{
	battery_status_s battery_status{};

	// ORIGINAL fields — all from PX4 battery.cpp unchanged
	battery_status.voltage_v                 = _voltage_v;
	battery_status.current_a                 = _current_a;
	battery_status.current_average_a         = _current_average_filter_a.getState();
	battery_status.discharged_mah            = _discharged_mah;
	battery_status.remaining                 = _state_of_charge;
	battery_status.scale                     = _scale;
	battery_status.time_remaining_s          = computeRemainingTime(_current_a);
	battery_status.temperature               = _temperature_c;
	battery_status.cell_count                = _params.n_cells;
	battery_status.connected                 = _connected;
	battery_status.source                    = _source;
	battery_status.priority                  = _priority;
	battery_status.capacity                  = static_cast<uint16_t>(_capacity_mah);
	battery_status.id                        = static_cast<uint8_t>(_index);
	battery_status.warning                   = _warning;
	battery_status.timestamp                 = hrt_absolute_time();
	battery_status.faults                    = determineFaults();
	battery_status.internal_resistance_estimate = _internal_resistance_estimate;
	battery_status.ocv_estimate              = _voltage_v + _internal_resistance_estimate * _params.n_cells * _current_a;
	battery_status.ocv_estimate_filtered     = _ocv_filter_v.getState();
	battery_status.volt_based_soc_estimate   = _params.n_cells > 0
		? math::interpolate(_ocv_filter_v.getState() / _params.n_cells,
				    _params.v_empty, _params.v_charged, 0.f, 1.f)
		: -1.f;
	battery_status.voltage_prediction        = _voltage_prediction;
	battery_status.prediction_error          = _prediction_error;
	battery_status.estimation_covariance_norm = _estimation_covariance_norm;

	// ── ADDED: publish local cycle count via MAVLink battery_status ──────
	// If SMBUS smart battery provided a hardware cycle count, that value
	// was already loaded into _cycle_count inside loadCycleCount() or
	// can be set externally via setCycleCount().
	// This sends our tracked value out over MAVLink so the Android app
	// receives it in msg_battery_status.cycle_count
	battery_status.cycle_count = _cycle_count;

	return battery_status;
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: publishBatteryStatus() and updateAndPublishBatteryStatus()
// publish to uORB only if this battery instance matches the configured source
// ════════════════════════════════════════════════════════════════════════════

void Battery::publishBatteryStatus(const battery_status_s &battery_status)
{
	if (_source == _params.source) {
		_battery_status_pub.publish(battery_status);
	}
}

void Battery::updateAndPublishBatteryStatus(const hrt_abstime &timestamp)
{
	updateBatteryStatus(timestamp);
	publishBatteryStatus(getBatteryStatus());
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: updateDt()
// calculates time delta since last call
// capped at 2 seconds to prevent huge jumps after pauses
// ════════════════════════════════════════════════════════════════════════════

void Battery::updateDt(const hrt_abstime &timestamp)
{
	if (_last_timestamp != 0) {
		_dt = math::min((timestamp - _last_timestamp) / 1e6f, 2.f);
	}

	_last_timestamp = timestamp;
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: sumDischarged()
// integrates current over time to get total mAh consumed since boot
// formula: mAh = (current_A * 1000) * (dt_seconds / 3600)
// ════════════════════════════════════════════════════════════════════════════

float Battery::sumDischarged(float current_a)
{
	if (_dt > FLT_EPSILON) {
		_discharged_mah_loop = (current_a * 1e3f) * (_dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	return _discharged_mah;
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: calculateStateOfChargeVoltageBased()
// estimates SoC by mapping cell voltage to empty/charged range
// corrects for voltage sag under load using internal resistance
// ════════════════════════════════════════════════════════════════════════════

float Battery::calculateStateOfChargeVoltageBased(const float voltage_v, const float current_a)
{
	if (_params.n_cells == 0) {
		return -1.0f;
	}

	// divide pack voltage by cell count to get per-cell voltage
	float cell_voltage = voltage_v / _params.n_cells;

	// correct for internal resistance voltage drop when current is flowing
	if (current_a > FLT_EPSILON) {
		updateInternalResistanceEstimation(voltage_v, current_a);

		if (_params.r_internal >= 0.f) {
			cell_voltage += _params.r_internal * current_a;
		} else {
			cell_voltage += _internal_resistance_estimate * current_a;
		}
	}

	// low-pass filter the cell voltage to remove noise
	_cell_voltage_filter_v.update(cell_voltage);

	// map filtered cell voltage linearly: v_empty=0%, v_charged=100%
	return math::interpolate(_cell_voltage_filter_v.getState(), _params.v_empty, _params.v_charged, 0.f, 1.f);
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: updateInternalResistanceEstimation()
// uses Recursive Least Squares (RLS) algorithm to estimate battery
// internal resistance in real time from voltage and current measurements
// ════════════════════════════════════════════════════════════════════════════

void Battery::updateInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	// RLS regressor vector: [1, -current_a]
	// Battery model: V = OCV - I * R  →  x = [1, -I], params = [OCV, R]
	Vector2f x{1, -current_a};

	// predicted voltage using current RLS estimate
	_voltage_prediction = (x.transpose() * _RLS_est)(0, 0);

	// error between actual and predicted voltage
	_prediction_error = voltage_v - _voltage_prediction;

	// RLS gain vector — determines how much to update estimate
	const Vector2f gamma = _estimation_covariance * x / (LAMBDA + (x.transpose() * _estimation_covariance * x)(0, 0));

	// updated parameter estimate
	const Vector2f RSL_est_temp = _RLS_est + gamma * _prediction_error;

	// updated covariance matrix (divided by forgetting factor LAMBDA)
	const Matrix2f estimation_covariance_temp = (_estimation_covariance
			- Matrix<float, 2, 1>(gamma) * (x.transpose() * _estimation_covariance)) / LAMBDA;

	// Frobenius-like norm of the covariance matrix to check if estimation improved
	const float estimation_covariance_temp_norm =
		sqrtf(powf(estimation_covariance_temp(0, 0), 2.f)
		      + 2.f * powf(estimation_covariance_temp(1, 0), 2.f)
		      + powf(estimation_covariance_temp(1, 1), 2.f));

	// only accept the update if it improves (reduces) the covariance norm
	if (estimation_covariance_temp_norm < _estimation_covariance_norm) {
		_RLS_est                  = RSL_est_temp;
		_estimation_covariance    = estimation_covariance_temp;
		_estimation_covariance_norm = estimation_covariance_temp_norm;
		// internal resistance = RLS param(1) divided by cell count, clamped >= 0
		_internal_resistance_estimate = math::max(_RLS_est(1) / _params.n_cells, 0.f);
	} else {
		// if estimation didn't improve, only update the OCV estimate
		_RLS_est(0) = voltage_v + _RLS_est(1) * current_a;
	}

	// update open-circuit voltage filter with IR-compensated voltage
	_ocv_filter_v.update(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: resetInternalResistanceEstimation()
// called when battery just connects — resets RLS estimator to defaults
// ════════════════════════════════════════════════════════════════════════════

void Battery::resetInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	_RLS_est(0) = voltage_v;
	_RLS_est(1) = R_DEFAULT * _params.n_cells;
	_estimation_covariance.setZero();
	_estimation_covariance(0, 0) = OCV_COVARIANCE * _params.n_cells;
	_estimation_covariance(1, 1) = R_COVARIANCE * _params.n_cells;
	_estimation_covariance_norm  = sqrtf(powf(_estimation_covariance(0, 0), 2.f)
					     + 2.f * powf(_estimation_covariance(1, 0), 2.f)
					     + powf(_estimation_covariance(1, 1), 2.f));
	_internal_resistance_estimate = R_DEFAULT;
	_ocv_filter_v.reset(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);

	if (_params.r_internal >= 0.f) {
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _params.r_internal * current_a);
	} else {
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _internal_resistance_estimate * current_a);
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: estimateStateOfCharge()
// fuses voltage-based and current-based SoC estimates
// when capacity is known: uses coulomb counting + voltage correction
// when capacity is unknown: falls back to pure voltage-based estimate
// ════════════════════════════════════════════════════════════════════════════

void Battery::estimateStateOfCharge()
{
	if ((_capacity_mah > 0.f) && _battery_initialized) {
		// weight for voltage correction — increases as battery gets more empty
		// at full charge: weight ~0.03 (trust current integration more)
		// at empty: weight ~0.06 (trust voltage more to avoid deep discharge)
		const float weight_v = 3e-2f * (1 - _state_of_charge_volt_based);

		// blend old SoC estimate with voltage-based estimate
		_state_of_charge = (1 - weight_v) * _state_of_charge + weight_v * _state_of_charge_volt_based;

		// subtract mAh used this loop tick from SoC
		_state_of_charge -= _discharged_mah_loop / _capacity_mah;

		// clamp to 0 — can never go negative
		_state_of_charge = math::max(_state_of_charge, 0.f);

		// also compute pure current-based SoC for cross-check
		const float state_of_charge_current_based = math::max(1.f - _discharged_mah / _capacity_mah, 0.f);

		// take the lower of the two estimates — conservative approach
		_state_of_charge = math::min(state_of_charge_current_based, _state_of_charge);

	} else {
		// no capacity known — use voltage only
		_state_of_charge = _state_of_charge_volt_based;
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: determineWarning()
// compares SoC against three configurable thresholds
// thresholds come from BAT_LOW_THR, BAT_CRIT_THR, BAT_EMERGEN_THR params
// ════════════════════════════════════════════════════════════════════════════

uint8_t Battery::determineWarning(float state_of_charge)
{
	if (state_of_charge < _params.emergen_thr) {
		return battery_status_s::WARNING_EMERGENCY; // typically < 5%

	} else if (state_of_charge < _params.crit_thr) {
		return battery_status_s::WARNING_CRITICAL;  // typically < 7%

	} else if (state_of_charge < _params.low_thr) {
		return battery_status_s::WARNING_LOW;       // typically < 15%

	} else {
		return battery_status_s::WARNING_NONE;
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: determineFaults()
// checks for hardware fault conditions and returns a bitmask
// ADDED: added cycle-count based fault warning
// ════════════════════════════════════════════════════════════════════════════

uint16_t Battery::determineFaults()
{
	uint16_t faults{0};

	// ORIGINAL: overvoltage check — voltage more than 5% above fully charged level
	// called FAULT_SPIKES because MAVLink has no "overvoltage" fault type
	if ((_params.n_cells > 0)
	    && (_voltage_v > (_params.n_cells * _params.v_charged * 1.05f))) {
		faults |= (1 << battery_status_s::FAULT_SPIKES);
	}

	// ORIGINAL: over-temperature check using BAT_TEMP_MAX (45°C default)
	// PX4_ISFINITE check ensures we don't fault on invalid temperature readings
	if (PX4_ISFINITE(_temperature_c) && _temperature_c > BAT_TEMP_MAX) {
		faults |= (1 << battery_status_s::FAULT_OVER_TEMPERATURE);
	}

	// ── ADDED: cycle count fault — warn when battery is worn out ─────────
	// LiPo batteries typically degrade significantly after 300 charge cycles
	// We reuse FAULT_DEEP_DISCHARGE as the closest available MAVLink fault bit
	// since MAVLink has no dedicated "end of life" fault
	if (_cycle_count > 300) {
		faults |= (1 << battery_status_s::FAULT_DEEP_DISCHARGE);
		PX4_WARN_ONCE("Battery has %u charge cycles — consider replacement!", _cycle_count);
	}

	return faults;
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: computeScale()
// calculates throttle compensation scale factor
// allows flight controller to compensate for lower voltage at end of charge
// clamped to max 1.3 (30% compensation) to prevent over-throttling
// ════════════════════════════════════════════════════════════════════════════

void Battery::computeScale()
{
	_scale = _params.v_charged / _cell_voltage_filter_v.getState();

	if (PX4_ISFINITE(_scale)) {
		_scale = math::constrain(_scale, 1.f, 1.3f);
	} else {
		_scale = 1.f;
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: computeRemainingTime()
// estimates minutes of flight time remaining based on average current draw
// for fixed-wing: only updates average current during level flight
// for multirotor: updates average current whenever armed
// ════════════════════════════════════════════════════════════════════════════

float Battery::computeRemainingTime(float current_a)
{
	float time_remaining_s        = NAN;
	bool  reset_current_avg_filter = false;

	// check if vehicle status changed (armed state, vehicle type)
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// if transitioning TO fixed-wing mode, reset the current average filter
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING && !_vehicle_status_is_fw) {
				reset_current_avg_filter = true;
			}

			_vehicle_status_is_fw = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
		}
	}

	_flight_phase_estimation_sub.update();

	// reset filter if state is invalid or we just switched to FW mode
	if (!PX4_ISFINITE(_current_average_filter_a.getState())
	    || _current_average_filter_a.getState() < FLT_EPSILON
	    || reset_current_avg_filter) {
		_current_average_filter_a.reset(_params.bat_avrg_current);
	}

	if (_armed && PX4_ISFINITE(current_a)) {
		// for fixed-wing: only update current average during stable level flight
		// for multirotor: always update
		if (!_vehicle_status_is_fw
		    || ((hrt_absolute_time() - _flight_phase_estimation_sub.get().timestamp) < 2_s
			&& _flight_phase_estimation_sub.get().flight_phase == flight_phase_estimation_s::FLIGHT_PHASE_LEVEL)) {

			if (_dt > FLT_EPSILON) {
				_current_average_filter_a.update(fmaxf(current_a, 0.f), _dt);
			} else {
				_current_average_filter_a.update(fmaxf(current_a, 0.f));
			}
		}
	}

	// time remaining = remaining mAh / average current in mA * 3600 seconds/hour
	if (_capacity_mah > 0.f) {
		const float remaining_capacity_mah = _state_of_charge * _capacity_mah;
		const float current_ma             = fmaxf(_current_average_filter_a.getState() * 1e3f, FLT_EPSILON);
		time_remaining_s                   = remaining_capacity_mah / current_ma * 3600.f;
	}

	return time_remaining_s;
}


// ════════════════════════════════════════════════════════════════════════════
// ORIGINAL: updateParams()
// reads all BAT%d_* parameters from PX4 parameter system
// re-initializes RLS estimator if cell count changed
// ADDED: no changes needed here — cycle count is independent of params
// ════════════════════════════════════════════════════════════════════════════

void Battery::updateParams()
{
	const int n_cells = _first_parameter_update ? 0 : _params.n_cells;
	param_get(_param_handles.v_empty,         &_params.v_empty);
	param_get(_param_handles.v_charged,       &_params.v_charged);
	param_get(_param_handles.n_cells,         &_params.n_cells);
	param_get(_param_handles.r_internal,      &_params.r_internal);
	param_get(_param_handles.source,          &_params.source);
	param_get(_param_handles.low_thr,         &_params.low_thr);
	param_get(_param_handles.crit_thr,        &_params.crit_thr);
	param_get(_param_handles.emergen_thr,     &_params.emergen_thr);
	param_get(_param_handles.bat_avrg_current, &_params.bat_avrg_current);

	float capacity{0.f};
	param_get(_param_handles.capacity, &capacity);
	setCapacityMah(capacity);

	// re-initialize RLS estimator if cell count changed
	if (n_cells != _params.n_cells) {
		_internal_resistance_initialized = false;
	}

	if (!_internal_resistance_initialized && _params.n_cells > 0) {
		_RLS_est(0)                   = OCV_DEFAULT * _params.n_cells;
		_RLS_est(1)                   = R_DEFAULT * _params.n_cells;
		_estimation_covariance(0, 0)  = OCV_COVARIANCE * _params.n_cells;
		_estimation_covariance(0, 1)  = 0.f;
		_estimation_covariance(1, 0)  = 0.f;
		_estimation_covariance(1, 1)  = R_COVARIANCE * _params.n_cells;
		_estimation_covariance_norm   = sqrtf(powf(_estimation_covariance(0, 0), 2.f)
						      + 2.f * powf(_estimation_covariance(1, 0), 2.f)
						      + powf(_estimation_covariance(1, 1), 2.f));
		_internal_resistance_initialized = true;
	}

	ModuleParams::updateParams();
	_first_parameter_update = false;
}


// ════════════════════════════════════════════════════════════════════════════
// ADDED: trackChargeCycle()
// detects when a full charge cycle completes and increments _cycle_count
// called from updateBatteryStatus() every loop
//
// LOGIC:
//   Step 1: SoC drops to or below 20%  →  _battery_was_low = true
//   Step 2: SoC rises to or above 95%  →  _cycle_count++, save to SD card
//
// Why 20% and 95%?
//   - 20% matches real-world "depleted" usage (not waiting for 0%)
//   - 95% accounts for batteries that never quite report 100%
// ════════════════════════════════════════════════════════════════════════════

void Battery::trackChargeCycle(const float state_of_charge)
{
	// Step 1: battery went low — arm the cycle detector
	if (state_of_charge <= CYCLE_LOW_THRESHOLD) {
		if (!_battery_was_low) {
			_battery_was_low = true;
			PX4_INFO("Battery cycle tracker: battery discharged (%.1f%%), waiting for recharge",
				 (double)(state_of_charge * 100.f));
		}
	}

	// Step 2: battery was low and is now fully charged — complete one cycle
	if (_battery_was_low && state_of_charge >= CYCLE_HIGH_THRESHOLD) {
		_cycle_count++;
		_battery_was_low = false;
		saveCycleCount(); // immediately write to SD so count survives power loss

		PX4_INFO("Battery cycle tracker: charge cycle #%u complete", _cycle_count);

		// warn pilot if battery has accumulated too many cycles
		if (_cycle_count > 300) {
			PX4_WARN("Battery has %u charge cycles — degraded capacity, consider replacement", _cycle_count);
		}
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ADDED: saveCycleCount()
// writes current cycle count and low-flag to SD card file
// called every time a cycle completes so data survives sudden power loss
// ════════════════════════════════════════════════════════════════════════════

void Battery::saveCycleCount()
{
	// open for writing — "w" creates file if missing, overwrites if exists
	FILE *f = fopen(CYCLE_COUNT_FILE, "w");

	if (f != nullptr) {
		// write two values: cycle count and whether battery is currently low
		// two separate lines so file is human-readable
		fprintf(f, "%u\n%d\n", _cycle_count, (int)_battery_was_low);
		fclose(f); // close flushes buffer — important before power loss
		PX4_DEBUG("Battery cycle count %u saved to SD card", _cycle_count);
	} else {
		PX4_ERR("Failed to save battery cycle count to %s — SD card missing?", CYCLE_COUNT_FILE);
	}
}


// ════════════════════════════════════════════════════════════════════════════
// ADDED: loadCycleCount()
// reads saved cycle count from SD card on boot
// called from constructor so count is restored before first battery update
// ════════════════════════════════════════════════════════════════════════════

void Battery::loadCycleCount()
{
	// open for reading — returns nullptr if file doesn't exist (first boot)
	FILE *f = fopen(CYCLE_COUNT_FILE, "r");

	if (f != nullptr) {
		unsigned int count = 0;
		int          was_low = 0;

		// read back the two values we saved — fscanf returns count of items parsed
		int parsed = fscanf(f, "%u\n%d\n", &count, &was_low);
		fclose(f);

		if (parsed >= 1) {
			// restore cycle count — cast to uint16_t (matches battery_status field type)
			_cycle_count     = static_cast<uint16_t>(count);
			// only restore low-flag if both values were successfully read
			_battery_was_low = (parsed >= 2) && (was_low != 0);
			PX4_INFO("Battery cycle count restored: %u cycles from SD card", _cycle_count);
		}

	} else {
		// normal on first boot or after SD card format
		PX4_INFO("No battery cycle count file found — starting from 0");
		_cycle_count     = 0;
		_battery_was_low = false;
	}
}
