#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

// Deliberately narrow, one-shot commissioning experiment (GRAVITY_DIAG,
// GRAVITY_Z_PREPARE in worker.cpp). Not a general runtime gravity setter.
// Cartesian monitoring is left to the external observer that sends
// GRAVITY_TICK; these joint guards and the observation watchdog stay in the
// 100 Hz worker.
//
// Raising feedforward is EXPECTED to move the arm back toward its unchanged
// command reference (less PD sag). The guard therefore allows motion along
// the initial command-minus-measured error, plus a small margin, and rejects
// moving further away or overshooting past the reference.
//
// status: idle -> ramping -> holding -> z_validation. HOLD during the ramp or
// hold cancels it ("cancelled"); the worker marks it "failed" or "stopped".
// Only one attempt per worker process.
struct GravityDiagnostic {
    using Q = std::array<double, 7>;
    // Joint displacement margin (rad), tracking-error cap (rad), ramp duration (s).
    static constexpr double kMargin = .01, kMaxTracking = .06, kRampSeconds = 10.;
    std::string status = "idle";
    Q reference{}, initial{};
    double elapsed = 0, start_scale = .25, target_scale = .5;
    bool armed() const { return status=="ramping" || status=="holding" || status=="cancelled" || status=="z_validation"; }
    void prepareZ(double scale, double gain) {
        check(status=="holding" && std::abs(scale-target_scale)<1e-9 && std::abs(gain-target_scale)<1e-9,
              "gravity_z_requires_completed_target_scale");
        status="z_validation";
    }
    static void check(bool ok, const char* error) { if(!ok) throw std::runtime_error(error); }
    // Starts the ramp from the settled scale to target. The experiment is
    // defined for uniform Kp 100 / Kd 1 only.
    void start(const Q& command, const Q& q, double scale, double gain, double target, double kp, double kd) {
        check(status=="idle", "gravity_diagnostic_already_attempted");
        check(std::isfinite(scale) && std::abs(scale-gain)<1e-9 && scale>0,
              "gravity_diagnostic_requires_settled_start_scale");
        check(std::isfinite(target) && target>scale+1e-9 && target<=1, "gravity_diagnostic_invalid_target");
        check(kp==100 && kd==1, "gravity_diagnostic_requires_100_1");
        reference=command; initial=q; elapsed=0; status="ramping";
        start_scale=scale; target_scale=target;
        guard(command, command, q, kp, kd);
    }
    // Throws when the gains, the reference (outside z validation) or the
    // measured motion leave the envelope described above.
    void guard(const Q& command, const Q& target, const Q& q, double kp, double kd) const {
        check(kp==100 && kd==1, "gravity_diagnostic_gains_changed");
        for(size_t i=0;i<7;++i) {
            check(std::isfinite(q[i]), "gravity_diagnostic_invalid_feedback");
            check(status=="z_validation" || (std::abs(command[i]-reference[i])<1e-9 && std::abs(target[i]-reference[i])<1e-9),
                  "gravity_diagnostic_reference_changed");
            const double sag=command[i]-initial[i], moved=q[i]-initial[i];
            check(moved>=std::min(0., sag)-kMargin && moved<=std::max(0., sag)+kMargin,
                  "gravity_diagnostic_joint_displacement");
            check(std::abs(command[i]-q[i])<=kMaxTracking, "gravity_diagnostic_tracking_error");
        }
    }
    // Next feedforward scale: linear over kRampSeconds; dt is capped so a missed
    // cycle cannot jump ahead.
    double advance(double current, double dt) {
        if(status!="ramping") return current;
        check(std::isfinite(dt) && dt>=0, "gravity_diagnostic_invalid_dt");
        elapsed=std::min(kRampSeconds, elapsed+std::min(dt,.02));
        if(elapsed>=kRampSeconds-1e-9) { elapsed=kRampSeconds; status="holding"; return target_scale; }
        return start_scale+(target_scale-start_scale)*elapsed/kRampSeconds;
    }
    void hold() { if(armed() && status!="z_validation") status="cancelled"; }
};
