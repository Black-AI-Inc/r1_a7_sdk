// Limits, continuity, online retargeting, HOLD and overshoot rejection of SmoothMotion.
#include "smooth_motion.h"
#include <algorithm>
#include <iostream>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int main() {
    using Q = SmoothMotion::Q;
    Q lower{}, upper{};
    lower.fill(-2); upper.fill(2);
    SmoothMotion motion(.15, .5, 2);
    Q target{.6, .2, -.15, .01, 0, -.3, .02};
    bool finished = false;
    double peak_speed = 0;
    for (int k = 0; k < 1000; ++k) {
        const Q previous_v = motion.input.current_velocity;
        const Q previous_a = motion.input.current_acceleration;
        const Q previous_p = motion.input.current_position;
        // Retarget while in motion, including a reversal. No reset of v/a.
        if (k == 70) target[0] = .8;
        if (k == 130) target[1] = -.2;
        finished = motion.propose(target, lower, upper);
        for (size_t i = 0; i < 7; ++i) {
            check(std::abs(motion.output.new_velocity[i]) <= .15 + 1e-9, "speed limit");
            check(std::abs(motion.output.new_acceleration[i]) <= .5 + 1e-9, "acceleration limit");
            check(std::abs(motion.output.new_acceleration[i]-previous_a[i]) <= 2*.01+1e-9, "jerk discontinuity");
            check(std::abs(motion.output.new_velocity[i]-previous_v[i]) <= .5*.01+1e-9, "velocity discontinuity");
            check(std::abs(motion.output.new_position[i]-previous_p[i]) <= .15*.01+1e-9, "position discontinuity");
            peak_speed = std::max(peak_speed, std::abs(motion.output.new_velocity[i]));
        }
        motion.accept();
        if (finished) break;
    }
    check(finished && peak_speed > .14, "trajectory did not finish/use speed cap");
    for (size_t i = 0; i < 7; ++i) {
        check(std::abs(motion.input.current_position[i]-target[i]) < 1e-9, "endpoint error");
        check(std::abs(motion.input.current_velocity[i]) < 1e-9, "endpoint velocity");
        check(std::abs(motion.input.current_acceleration[i]) < 1e-9, "endpoint acceleration");
    }

    // A stream of nearby targets must keep moving through target handovers.
    motion.reset(Q{});
    target = Q{}; target[0] = .04;
    for (int k = 0; k < 300; ++k) {
        if (k > 0 && k % 10 == 0) target[0] += .015;
        motion.propose(target, lower, upper);
        if (k > 60) check(motion.output.new_velocity[0] > .08, "stream stuttered");
        motion.accept();
    }
    // Operator HOLD overrides normal smoothing, with no position tail.
    const Q frozen = motion.input.current_position;
    motion.reset(frozen);
    for (int k = 0; k < 20; ++k) {
        motion.propose(frozen, lower, upper);
        check(motion.output.new_position == frozen, "HOLD moved reference");
        motion.accept();
    }

    // Endpoint is in range, but stopping from current velocity would overshoot.
    motion.reset(Q{});
    motion.input.current_position[0] = .999;
    motion.input.current_velocity[0] = .15;
    target = motion.input.current_position;
    upper[0] = 1;
    bool rejected = false;
    try { motion.propose(target, lower, upper); }
    catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "trajectory extrema did not reject overshoot");
    std::cout << "Smooth trajectories, online retargeting, bounds and HOLD passed\n";
}
