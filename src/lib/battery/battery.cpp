/****************************************************************************
 *
 * battery_cycle_tracker.h / battery_cycle_tracker.cpp
 *
 * Battery charge cycle tracker for PX4.
 *
 * CYCLE LOGIC:
 *   A charge cycle is counted when:
 *     1. Battery SoC drops below CHARGE_START_THRESHOLD (20%)
 *     2. Battery SoC then rises above CHARGE_DONE_THRESHOLD (95%)
 *
 * Cycle count is:
 *   - Read from SMBUS smart battery (cycle_count field) if available
 *   - Otherwise tracked locally and saved to a file on SD card
 *
 * Integrates with PX4 battery.cpp:
 *   - Reads battery_status uORB topic
 *   - Publishes updated cycle count back to battery_status
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/battery_status.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <stdio.h>
#include <string.h>

using namespace time_literals;

// ── Thresholds ────────────────────────────────────────────────────────────────
static constexpr float CHARGE_START_THRESHOLD = 0.20f;  // 20% — considered discharged
static constexpr float CHARGE_DONE_THRESHOLD  = 0.95f;  // 95% — considered fully charged

// File on SD card to persist cycle count across reboots
static constexpr const char *CYCLE_COUNT_FILE = PX4_STORAGEDIR "/battery_cycles.txt";

// ── BatteryCycleTracker class ─────────────────────────────────────────────────
class BatteryCycleTracker : public ModuleParams
{
public:
    BatteryCycleTracker(ModuleParams *parent = nullptr)
        : ModuleParams(parent)
    {
        loadCycleCount();
    }

    ~BatteryCycleTracker() = default;

    /**
     * Call this every loop iteration.
     * Pass in the battery_status struct from uORB.
     * Returns current charge cycle count.
     *
     * @param battery   battery_status_s from uORB subscription
     * @return          current total charge cycle count
     */
    uint16_t update(const battery_status_s &battery)
    {
        // ── Priority 1: Use hardware cycle count from smart battery (SMBUS) ──
        // battery_status.cycle_count is populated by SMBUS driver if available
        if (battery.cycle_count > 0) {
            _smart_battery_available = true;
            _cycle_count = battery.cycle_count;
            return _cycle_count;
        }

        // ── Priority 2: Track locally if no smart battery ─────────────────
        if (!battery.connected || battery.remaining < 0.f) {
            return _cycle_count;
        }

        const float soc = battery.remaining; // 0.0 to 1.0

        // Step 1: Battery went low — mark as discharged
        if (soc <= CHARGE_START_THRESHOLD) {
            if (!_battery_was_low) {
                _battery_was_low = true;
                PX4_INFO("BatteryCycleTracker: battery low (%.1f%%), waiting for full charge",
                         (double)(soc * 100.f));
            }
        }

        // Step 2: Battery was low and is now fully charged — count cycle
        if (_battery_was_low && soc >= CHARGE_DONE_THRESHOLD) {
            _cycle_count++;
            _battery_was_low = false;
            saveCycleCount();

            PX4_INFO("BatteryCycleTracker: charge cycle complete! Total cycles: %u", _cycle_count);
        }

        return _cycle_count;
    }

    /**
     * Get current charge cycle count.
     */
    uint16_t getCycleCount() const { return _cycle_count; }

    /**
     * Returns true if a hardware smart battery is providing cycle data.
     */
    bool isSmartBattery() const { return _smart_battery_available; }

    /**
     * Reset cycle counter (call when replacing battery).
     * Saves reset to SD card.
     */
    void resetCycleCount()
    {
        _cycle_count     = 0;
        _battery_was_low = false;
        saveCycleCount();
        PX4_INFO("BatteryCycleTracker: cycle count reset to 0");
    }

    /**
     * Get battery health warning level based on cycle count.
     * Returns:
     *   0 = GOOD    (< 200 cycles)
     *   1 = WARNING (200–300 cycles)
     *   2 = CRITICAL (> 300 cycles — replace battery)
     */
    uint8_t getCycleHealthLevel() const
    {
        if (_cycle_count > 300) return 2; // CRITICAL
        if (_cycle_count > 200) return 1; // WARNING
        return 0;                         // GOOD
    }

private:

    // ── State ──────────────────────────────────────────────────────────────
    uint16_t _cycle_count            {0};
    bool     _battery_was_low        {false};
    bool     _smart_battery_available{false};

    // ── Save cycle count to SD card ────────────────────────────────────────
    void saveCycleCount()
    {
        FILE *f = fopen(CYCLE_COUNT_FILE, "w");

        if (f != nullptr) {
            fprintf(f, "%u\n%d\n", _cycle_count, (int)_battery_was_low);
            fclose(f);
            PX4_DEBUG("BatteryCycleTracker: saved cycle count %u to %s",
                      _cycle_count, CYCLE_COUNT_FILE);
        } else {
            PX4_ERR("BatteryCycleTracker: failed to save cycle count to %s", CYCLE_COUNT_FILE);
        }
    }

    // ── Load cycle count from SD card ──────────────────────────────────────
    void loadCycleCount()
    {
        FILE *f = fopen(CYCLE_COUNT_FILE, "r");

        if (f != nullptr) {
            unsigned int count = 0;
            int was_low        = 0;
            int parsed         = fscanf(f, "%u\n%d\n", &count, &was_low);
            fclose(f);

            if (parsed >= 1) {
                _cycle_count     = (uint16_t)count;
                _battery_was_low = (parsed >= 2) && (was_low != 0);
                PX4_INFO("BatteryCycleTracker: loaded cycle count %u from SD card", _cycle_count);
            }
        } else {
            PX4_INFO("BatteryCycleTracker: no saved cycle count found, starting at 0");
        }
    }
};


// ═══════════════════════════════════════════════════════════════════════════════
// HOW TO INTEGRATE INTO battery.cpp (PX4)
// ═══════════════════════════════════════════════════════════════════════════════
//
// 1. In your battery.h, add member:
//
//      #include "battery_cycle_tracker.h"
//      BatteryCycleTracker _cycle_tracker;
//
// 2. In Battery::getBatteryStatus(), after existing fields, add:
//
//      // Update cycle tracker and add to status
//      battery_status.cycle_count = _cycle_tracker.update(battery_status);
//
// 3. In Battery::determineFaults(), add cycle-based fault:
//
//      // Warn if battery has too many charge cycles
//      uint8_t cycleHealth = _cycle_tracker.getCycleHealthLevel();
//      if (cycleHealth == 2) {
//          faults |= (1 << battery_status_s::FAULT_DEEP_DISCHARGE); // reuse closest fault bit
//          PX4_WARN_ONCE("Battery has >300 charge cycles — consider replacement!");
//      }
//
// ═══════════════════════════════════════════════════════════════════════════════


// ═══════════════════════════════════════════════════════════════════════════════
// STANDALONE USAGE EXAMPLE (outside of battery.cpp)
// ═══════════════════════════════════════════════════════════════════════════════

/*

#include "battery_cycle_tracker.h"
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>

class MyBatteryModule : public ModuleBase<MyBatteryModule>
{
public:
    void run()
    {
        uORB::Subscription battery_sub{ORB_ID(battery_status)};
        BatteryCycleTracker cycle_tracker;

        while (!should_exit()) {

            battery_status_s battery{};

            if (battery_sub.update(&battery)) {

                uint16_t cycles = cycle_tracker.update(battery);

                PX4_INFO("Battery: %.1f%% | Cycles: %u | Health: %s",
                         (double)(battery.remaining * 100.f),
                         cycles,
                         cycle_tracker.getCycleHealthLevel() == 0 ? "GOOD" :
                         cycle_tracker.getCycleHealthLevel() == 1 ? "WARNING" : "CRITICAL");
            }

            px4_usleep(1_s);
        }
    }
};

*/
