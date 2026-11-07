#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <ruckig/ruckig.hpp>

// Jerk-limited, phase-synchronized joint trajectories from the vendored Ruckig,
// computed locally (no cloud waypoint API). Retargeting preserves
// velocity/acceleration. HOLD and protective gates instead reset at the last
// sent position so no unconsumed trajectory can run on.
//
// Each cycle: propose() computes the next step toward the target; the caller
// either accept()s it or reset()s at the position it actually sent.
class SmoothMotion {
public:
    using Q = std::array<double, 7>;
    ruckig::InputParameter<7> input;
    ruckig::OutputParameter<7> output;
    ruckig::Ruckig<7> otg{.01};   // the worker's 10 ms control period

    SmoothMotion(double speed, double acceleration, double jerk) {
        input.max_velocity.fill(speed);
        input.max_acceleration.fill(acceleration);
        input.max_jerk.fill(jerk);
        input.synchronization = ruckig::Synchronization::Phase;
        input.target_velocity.fill(0);
        input.target_acceleration.fill(0);
        reset(Q{});
    }

    void reset(const Q& position) {
        input.current_position = input.target_position = position;
        input.current_velocity.fill(0);
        input.current_acceleration.fill(0);
        otg.reset();
    }

    bool propose(const Q& target, const Q& lower, const Q& upper) {
        input.target_position = target;
        const auto result = otg.update(input, output);
        if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
            throw std::runtime_error("smooth_trajectory_failed");
        // A valid endpoint alone cannot exclude overshoot while retargeting.
        const auto extrema = output.trajectory.get_position_extrema();
        for (size_t i = 0; i < 7; ++i) {
            if (!std::isfinite(output.new_position[i]) ||
                extrema[i].min < lower[i] - 1e-10 || extrema[i].max > upper[i] + 1e-10)
                throw std::runtime_error("smooth_trajectory_outside_limits");
            // Remove roundoff at exact URDF boundaries; real overshoot is
            // rejected above, never repaired by clipping a trajectory.
            output.new_position[i] = std::clamp(output.new_position[i], lower[i], upper[i]);
        }
        return result == ruckig::Result::Finished;
    }

    void accept() { output.pass_to_input(input); }
};
