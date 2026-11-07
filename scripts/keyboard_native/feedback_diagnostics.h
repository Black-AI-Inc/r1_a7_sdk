#pragma once
#include "arm_motor_bus.h"
#include <cmath>
#include <iomanip>
#include <sstream>

// Per-joint feedback diagnostics as JSON. Pure formatting: never sends packets
// or changes control thresholds.

// Feedback acceptance thresholds, shared with the worker's control loop so that
// the diagnostics report exactly the checks that latched a fault.
constexpr double kFeedbackMaxAge = .2;   // s since the joint's latest packet
constexpr double kLimitTolerance = .03;  // rad a reading may lie past a limit ("tolerance_rad")

// A finite number, or JSON null.
inline void feedbackNumber(std::ostream& out, double v) {
    if (std::isfinite(v)) out << v;
    else out << "null";
}

// One object per joint: raw and URDF-frame values, calibration, limits and the
// reasons this feedback would be rejected (no_packet, stale, nonfinite, ...).
inline std::string feedbackDiagnostics(const ArmFeedbackSnapshot& fb,
        const std::array<double, 7>& lower, const std::array<double, 7>& upper) {
    std::ostringstream out;
    out << std::setprecision(10) << "[";
    for (int i = 0; i < 7; ++i) {
        const auto& m = fb.motor[i]; const auto& u = fb.urdf[i];
        const auto& c = fb.calibration[i];
        const bool finite = std::isfinite(u.q) && std::isfinite(u.dq);
        const bool range = std::isfinite(u.q) && u.q >= lower[i]-kLimitTolerance
                           && u.q <= upper[i]+kLimitTolerance;
        out << (i ? "," : "") << "{\"joint\":\"J" << i+1 << "\",\"motor_id\":" << int(fb.motor_ids[i])
            << ",\"received\":" << (fb.ready[i] ? "true" : "false") << ",\"age_s\":";
        feedbackNumber(out, fb.age_s[i]);
        out << ",\"motor_q_rad\":"; feedbackNumber(out, m.q);
        out << ",\"q_rad\":"; feedbackNumber(out, u.q);
        out << ",\"motor_dq_rad_s\":"; feedbackNumber(out, m.dq);
        out << ",\"dq_rad_s\":"; feedbackNumber(out, u.dq);
        out << ",\"motor_tau_est_nm\":"; feedbackNumber(out, m.tau_est);
        out << ",\"mode\":" << int(u.mode) << ",\"timeout\":" << int(u.timeout)
            << ",\"error_code\":" << u.error_code << ",\"dir\":" << c.dir
            << ",\"scale\":" << c.scale << ",\"offset_rad\":" << c.offset
            << ",\"lower_rad\":" << lower[i] << ",\"upper_rad\":" << upper[i]
            << ",\"tolerance_rad\":0.03,\"fresh\":" << (fb.age_s[i]<kFeedbackMaxAge ? "true" : "false")
            << ",\"finite\":" << (finite ? "true" : "false")
            << ",\"within_limits\":" << (range ? "true" : "false") << ",\"reasons\":[";
        bool first = true;
        auto reason = [&](const char* text) { out << (first ? "" : ",") << '"' << text << '"'; first=false; };
        if (!fb.ready[i]) reason("no_packet");
        else if (!(fb.age_s[i]<kFeedbackMaxAge)) reason("stale");
        if (!finite) reason("nonfinite");
        if (u.error_code) reason("motor_error");
        if (!range) reason("outside_limits");
        out << "]}";
    }
    out << "]";
    return out.str();
}
