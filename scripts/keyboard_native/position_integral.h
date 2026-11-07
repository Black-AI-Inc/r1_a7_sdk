#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// Slow joint-space outer loop around the motor's own PD. It removes the static
// error the PD leaves for gravity-model error and stiction. The integral is
// updated only while the reference is (nearly) stationary and is frozen, not
// reset, during motion: the load support it found stays applied. Inside the
// deadband it holds its value so stick-slip cannot make it hunt.
//
// A joint whose error changes sign during a hold has slipped past its
// reference: integrating on would wind up the other way and slip back, a
// limit cycle (field 2026-11-03: J2 jumped -0.004 <-> +0.005 rad every 3.3 s,
// ~1.2 mm at the TCP). Its integral is held until the next reference motion.
// Sign changes in the first settle_s of a hold only re-arm the side: a joint
// often slips as the reference comes to rest (field 2026-11-03 21:50: J2
// +0.0014 -> -0.0035 rad), which is not integral wind-up, and freezing there
// left J2 0.0038 rad (0.5 mm) off for the whole hold.
struct PositionIntegral {
    using Q = std::array<double, 7>;
    Q value{};
    std::array<int, 7> side{};        // sign of the error integrated in this hold
    std::array<bool, 7> crossed{};    // held after a sign change until the next motion
    Q limit{.03, .03, .03, .03, .03, .03, .03};   // per joint: equal torque authority at unequal kp
    double ki = 0, deadband = .0003, max_reference_speed = .02, settle_s = .5;
    double held_s = 0;                // time with a stationary reference, this hold

    bool enabled() const { return ki > 0; }
    void reset() { value.fill(0); release(); }
    void release() { side.fill(0); crossed.fill(false); held_s = 0; }

    void configure(double gain, double bound) {
        Q bounds; bounds.fill(bound);
        configure(gain, bounds);
    }
    void configure(double gain, const Q& bounds) {
        if (!std::isfinite(gain) || gain < 0 || gain > 5)
            throw std::runtime_error("integral settings out of range");
        for (double b : bounds)
            if (!std::isfinite(b) || b < 0 || b > .05) throw std::runtime_error("integral settings out of range");
        ki = gain; limit = bounds;
        for (size_t i = 0; i < 7; ++i) value[i] = std::clamp(value[i], -limit[i], limit[i]);
    }

    Q apply(const Q& reference) const {
        Q out = reference;
        for (size_t i = 0; i < 7; ++i) out[i] += value[i];
        return out;
    }

    void update(const Q& reference, const Q& measured, const Q& reference_velocity, double dt) {
        if (!enabled()) return;
        for (size_t i = 0; i < 7; ++i)
            if (!std::isfinite(reference_velocity[i]) || std::abs(reference_velocity[i]) > max_reference_speed) {
                release();   // a new hold starts after this motion
                return;
            }
        dt = std::clamp(dt, 0., .02);  // a missed cycle must not cause a catch-up step
        held_s += dt;
        for (size_t i = 0; i < 7; ++i) {
            const double error = reference[i] - measured[i];
            if (!std::isfinite(error)) throw std::runtime_error("integral_invalid_feedback");
            if (crossed[i] || std::abs(error) <= deadband) continue;
            const int sign = error > 0 ? 1 : -1;
            if (side[i] != 0 && sign != side[i] && held_s >= settle_s) { crossed[i] = true; continue; }
            side[i] = sign;
            value[i] = std::clamp(value[i] + ki * error * dt, -limit[i], limit[i]);
        }
    }
};
