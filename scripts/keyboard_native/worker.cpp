// Persistent SDK position control. stdin protocol, stdout JSON status, stderr logs.
// No automatic reset/re-enable. STOP releases torque and is latched until the
// process is restarted. With --fault-kd > 0, soft faults instead enter a latched
// compliant "damping" hold at the measured pose (gravity feedforward kept);
// RECOVER resumes position hold there, STOP still releases torque.
#include "arm_motor_bus.h"
#include "smooth_motion.h"
#include "gravity_model.h"
#include "feedback_diagnostics.h"
#include "gravity_diagnostic.h"
#include "position_integral.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
using Q = std::array<double, 7>;
// Startup recovery never travels further than this toward --recover-q, and
// has its own time budget (it can be longer than a normal 5 s service move).
constexpr double kMaxRecoveryTravel = .8, kRecoveryTimeout = 30;
volatile std::sig_atomic_t running = 1;
void onSignal(int) { running = 0; }
double age(Clock::time_point t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
void require(bool condition, const char* why) { if (!condition) throw std::runtime_error(why); }

struct Options {
    bool mock = true, have_limits = false;
    bool have_gravity = false;
    gravity::Model gravity_model;
    double gravity_scale = 0;
    std::string port = "/dev/ttyUSB3", calib;
    double kp = 60, kd = .5, speed = .15, acceleration = .5, jerk = 2;
    double lead_limit = .08, tracking_limit = .2, move_timeout = 60;
    // Opt-in additions; the defaults keep the original behaviour.
    double ki = 0, integral_limit = .03;   // joint integral outer loop, 1/s and rad
    double fault_kp = 0, fault_kd = 0;     // fault_kd > 0 enables damping on soft faults
    // Per-joint gains/integral limits (--joint-kp/--joint-kd/--joint-integral-limit,
    // 7 values each) override the scalars. Status then reports kp/kd as arrays,
    // so consumers that require a uniform kp (gravity diagnostic, VLM gates)
    // reject them instead of reading the scalar.
    Q joint_kp{}, joint_kd{}, joint_integral_limit{};
    bool joint_gains = false, joint_limits = false;
    double recover_margin = 0;             // > 0: enable may bring out-of-limit joints back
    Q recover_q{}; bool have_recover_q = false;
    Q lower{}, upper{};
    Q mock_q{}, mock_load{};
    bool mock_configured = false;
    // Home translated -Y 5 cm, +Z 5 cm, preserving Home orientation.
    Q ready{0.006792101712, 0.605442587728, 0.003982975938,
            0.349301756762, -0.004018032920, 0.256020348855, 0.009633129608};
};

Q gains(const Options& o, double scalar, const Q& joint) {
    Q out; out.fill(scalar);
    return o.joint_gains ? joint : out;
}
Q integralLimits(const Options& o) {
    Q out; out.fill(o.integral_limit);
    return o.joint_limits ? o.joint_integral_limit : out;
}
// Status value: the scalar, or a JSON array when per-joint values are in use.
std::string jsonGains(double scalar, const Q& joint, bool per_joint) {
    std::ostringstream s; s<<std::setprecision(10);
    if (!per_joint) { s<<scalar; return s.str(); }
    s<<"[";
    for (size_t i=0;i<7;++i) s<<(i?",":"")<<joint[i];
    s<<"]";
    return s.str();
}
// The uniform value, or NaN when joints differ (fails any equality check).
double uniform(const Q& v) {
    for (double x : v) if (x != v[0]) return std::nan("");
    return v[0];
}

void validateSettings(const Options& o) {
    require(std::isfinite(o.kp)&&o.kp>=.01&&o.kp<=100, "kp must be in .01..100");
    require(std::isfinite(o.kd)&&o.kd>=.01&&o.kd<=5, "kd must be in .01..5");
    for (size_t i=0; i<7&&o.joint_gains; ++i) {
        require(std::isfinite(o.joint_kp[i])&&o.joint_kp[i]>=.01&&o.joint_kp[i]<=250, "joint-kp must be in .01..250");
        require(std::isfinite(o.joint_kd[i])&&o.joint_kd[i]>=.01&&o.joint_kd[i]<=5, "joint-kd must be in .01..5");
    }
    for (size_t i=0; i<7&&o.joint_limits; ++i)
        require(std::isfinite(o.joint_integral_limit[i])&&o.joint_integral_limit[i]>=0&&o.joint_integral_limit[i]<=.05,
                "joint-integral-limit must be 0..0.05 rad");
    require(std::isfinite(o.speed)&&o.speed>=.01&&o.speed<=1, "joint-speed must be .01..1 rad/s");
    require(std::isfinite(o.acceleration)&&o.acceleration>=.1&&o.acceleration<=5, "joint-accel must be .1..5 rad/s^2");
    require(std::isfinite(o.jerk)&&o.jerk>=.5&&o.jerk<=50, "joint-jerk must be .5..50 rad/s^3");
    require(std::isfinite(o.lead_limit)&&o.lead_limit>=.01&&o.lead_limit<=.3, "lead_limit_range");
    require(std::isfinite(o.tracking_limit)&&o.tracking_limit>=.02&&o.tracking_limit<=.5, "tracking_limit_range");
    require(o.lead_limit<o.tracking_limit, "lead_must_be_less_than_tracking_limit");
    require(std::isfinite(o.move_timeout)&&o.move_timeout>=5&&o.move_timeout<=300, "move_timeout_range");
    require(std::isfinite(o.ki)&&o.ki>=0&&o.ki<=5, "ki must be 0..5 1/s");
    require(std::isfinite(o.integral_limit)&&o.integral_limit>=0&&o.integral_limit<=.05, "integral-limit must be 0..0.05 rad");
    require(std::isfinite(o.fault_kd)&&o.fault_kd>=0&&o.fault_kd<=5, "fault-kd must be 0..5");
    const Q kp=gains(o,o.kp,o.joint_kp);
    require(std::isfinite(o.fault_kp)&&o.fault_kp>=0&&o.fault_kp<=*std::min_element(kp.begin(),kp.end()),
            "fault-kp must be 0..kp");
    require(std::isfinite(o.recover_margin)&&o.recover_margin>=0&&o.recover_margin<=.25, "recover-margin must be 0..0.25 rad");
}

Options parse(int argc, char** argv) {
    Options o;
    for (int i=1; i<argc; ++i) {
        std::string a=argv[i];
        auto value=[&]() { require(i+1<argc, "missing option value"); return std::string(argv[++i]); };
        if (a=="--hardware") o.mock=false;
        else if (a=="--mock") o.mock=true;
        else if (a=="--port") o.port=value();
        else if (a=="--calib") o.calib=value();
        else if (a=="--kp") o.kp=std::stod(value());
        else if (a=="--kd") o.kd=std::stod(value());
        else if (a=="--joint-speed") o.speed=std::stod(value());
        else if (a=="--joint-accel") o.acceleration=std::stod(value());
        else if (a=="--joint-jerk") o.jerk=std::stod(value());
        else if (a=="--lead-limit") o.lead_limit=std::stod(value());
        else if (a=="--tracking-limit") o.tracking_limit=std::stod(value());
        else if (a=="--move-timeout") o.move_timeout=std::stod(value());
        else if (a=="--ki") o.ki=std::stod(value());
        else if (a=="--integral-limit") o.integral_limit=std::stod(value());
        else if (a=="--fault-kp") o.fault_kp=std::stod(value());
        else if (a=="--fault-kd") o.fault_kd=std::stod(value());
        else if (a=="--recover-margin") o.recover_margin=std::stod(value());
        else if (a=="--joint-kp"||a=="--joint-kd") {
            for (double& v:(a=="--joint-kp"?o.joint_kp:o.joint_kd)) v=std::stod(value());
            o.joint_gains=true;
        }
        else if (a=="--joint-integral-limit") {
            for (double& v:o.joint_integral_limit) v=std::stod(value());
            o.joint_limits=true;
        }
        else if (a=="--recover-q") {
            for (double& v:o.recover_q) v=std::stod(value());
            o.have_recover_q=true;
        }
        else if (a=="--gravity-model") { o.gravity_model=gravity::Model::parse(value()); o.have_gravity=true; }
        else if (a=="--gravity-scale") o.gravity_scale=std::stod(value());
        else if (a=="--ready-q") {
            for (double& v:o.ready) v=std::stod(value());
        }
        else if (a=="--mock-q"||a=="--mock-load") {
            o.mock_configured=true;
            for(double& v:(a=="--mock-q"?o.mock_q:o.mock_load)) v=std::stod(value());
        }
        else if (a=="--limits") {
            for (double& v:o.lower) v=std::stod(value());
            for (double& v:o.upper) v=std::stod(value());
            o.have_limits=true;
        } else throw std::runtime_error("unknown option: "+a);
    }
    require(o.have_limits, "joint limits required");
    // Both or neither: a lone --joint-kp would pair stiff joints with default damping.
    require(!o.joint_gains||(o.joint_kp[0]>0&&o.joint_kd[0]>0), "--joint-kp and --joint-kd go together");
    for (size_t i=0;i<7;++i)
        require(std::isfinite(o.lower[i])&&std::isfinite(o.upper[i])&&
                o.lower[i]<o.upper[i]&&o.lower[i]<=0&&o.upper[i]>=0, "invalid limits");
    for (size_t i=0;i<7;++i)
        require(std::isfinite(o.ready[i])&&o.ready[i]>=o.lower[i]&&o.ready[i]<=o.upper[i],
                "ready-q outside joint limits");
    for (size_t i=0;i<7&&o.have_recover_q;++i)
        require(std::isfinite(o.recover_q[i])&&o.recover_q[i]>=o.lower[i]&&o.recover_q[i]<=o.upper[i],
                "recover-q outside joint limits");
    validateSettings(o);
    require(std::isfinite(o.gravity_scale)&&o.gravity_scale>=0&&o.gravity_scale<=1, "gravity scale must be 0..1");
    require(o.have_gravity||o.gravity_scale==0, "gravity scale requires model");
    require(o.mock||!o.calib.empty(), "hardware requires calibration path");
    require(o.mock||!o.mock_configured, "mock plant options cannot be used with hardware");
    for(size_t i=0;i<7;++i)
        require(std::isfinite(o.mock_q[i])&&o.mock_q[i]>=o.lower[i]-o.recover_margin
                &&o.mock_q[i]<=o.upper[i]+o.recover_margin
                &&std::isfinite(o.mock_load[i])&&std::abs(o.mock_load[i])<=.05, "invalid mock plant");
    return o;
}

int main(int argc, char** argv) {
    // SDK diagnostics must not contaminate machine-readable stdout.
    std::cout.rdbuf(std::cerr.rdbuf());
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal); std::signal(SIGPIPE,SIG_IGN);
    ArmMotorBus bus;
    try {
        auto o=parse(argc,argv);
        fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
        fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
        if (!o.mock && !bus.open(o.port,kArmDefaultBaudrate,o.calib,true)) return 2;
        Q q{}, command{}, target{};
        Q measured_dq{}, measured_tau{};
        // Effective joint limits: the URDF limits, widened only while a startup
        // recovery brings an out-of-limit joint back inside.
        Q lower=o.lower, upper=o.upper, damp_q{};
        ArmFeedbackSnapshot feedback;
        bool feedback_valid=o.mock, damp_feedback_ok=o.mock;
        std::string first_fault_diagnostics="null";
        std::string output_pending;
        size_t output_offset=0;
        gravity::Result gravity_prediction;
        gravity::Ramp gravity_ramp;
        GravityDiagnostic gravity_diag;
        PositionIntegral integral;
        integral.configure(o.ki,integralLimits(o));
        auto gravity_observation = Clock::now();
        auto outside=[&](size_t i, double v, double margin) { return v<o.lower[i]-margin||v>o.upper[i]+margin; };
        SmoothMotion motion(o.speed,o.acceleration,o.jerk);
        std::string mode=o.mock?"hold":"probing", reason, pending;
        auto begin_recovery=[&](const Q& start) {
            // Joints within the normal 0.03 rad tolerance are captured as usual;
            // the others start at their measured angle and move back inside:
            // to --recover-q when that is near, otherwise just inside the limit.
            for(size_t i=0;i<7;++i) {
                if(outside(i,start[i],.03)) {
                    command[i]=start[i];
                    target[i]=o.have_recover_q&&std::abs(o.recover_q[i]-start[i])<=kMaxRecoveryTravel
                        ?o.recover_q[i]:std::clamp(start[i],o.lower[i]+.02,o.upper[i]-.02);
                    lower[i]=std::min(o.lower[i],start[i]-.03); upper[i]=std::max(o.upper[i],start[i]+.03);
                } else command[i]=target[i]=std::clamp(start[i],o.lower[i],o.upper[i]);
            }
            motion.reset(command);
            integral.reset();
        };
        bool recovery_pending=false;
        if(o.mock) {
            for(size_t i=0;i<7;++i) q[i]=o.mock_q[i];
            bool out=false;
            for(size_t i=0;i<7;++i) out|=outside(i,q[i],.03);
            if(out) { begin_recovery(q); mode="recovering"; }
            else for(size_t i=0;i<7;++i) command[i]=target[i]=o.mock_q[i];
            for(size_t i=0;i<7;++i) q[i]=command[i]-o.mock_load[i];
        }
        if(mode!="recovering") motion.reset(command);
        bool lead_limited=false;
        unsigned long status_seq=0, config_ack=0, config_revision=0;
        std::string config_error;
        auto heartbeat=Clock::now(), phase=heartbeat, next=heartbeat, last_emit=heartbeat;
        auto record_fault=[&](const std::string& why) {
            if(first_fault_diagnostics=="null" && !o.mock) {
                first_fault_diagnostics=feedbackDiagnostics(feedback,lower,upper);
                std::cerr<<"[R1 feedback fault] "<<why<<" "<<first_fault_diagnostics<<std::endl;
            }
        };
        // Torque release (mode 0). Used for operator STOP, exit and faults where
        // the drive itself cannot be trusted: lost/invalid feedback, left FOC.
        auto release=[&](const std::string& why, bool fault) {
            if(fault) record_fault(why);
            mode=fault?"fault":"stopped"; reason=why;
            if(gravity_diag.armed()) gravity_diag.status=fault?"failed":"stopped";
            gravity_ramp.reset();
            integral.reset();
            lower=o.lower; upper=o.upper;
            target=command; motion.reset(command);
            if (!o.mock) bus.stop();
        };
        auto active=[&]() { return mode=="hold"||mode=="moving"||mode=="position"||mode=="home"||mode=="ready"||mode=="recovering"; };
        // Soft fault: latched compliant hold at the measured pose. The arm is not
        // dropped; only fault-kp holds it, so it yields instead of pushing. It
        // needs live, error-free FOC feedback; otherwise it is a torque release.
        auto damp=[&](const std::string& why) {
            if(mode=="damping"||mode=="fault"||mode=="stopped") return;
            if(!(o.fault_kd>0) || !damp_feedback_ok || !(mode=="enabling"||active())) { release(why,true); return; }
            record_fault(why);
            mode="damping"; reason=why;
            if(gravity_diag.armed()) gravity_diag.status="failed";
            damp_q=q;
            command=target=q;
            motion.reset(command);
            integral.reset();
            lower=o.lower; upper=o.upper;
        };
        auto capture_feedback=[&]() {
            for(size_t i=0;i<7;++i) command[i]=target[i]=std::clamp(q[i],o.lower[i],o.upper[i]);
            motion.reset(command);
            integral.reset();
            lower=o.lower; upper=o.upper;
            mode="hold";
        };
        auto hold=[&]() {
            // Preserve the PD position error supporting the load. Replacing this
            // with measured q on every key event ratchets the arm down under gravity.
            target=command;
            motion.reset(command);
            mode="hold";
            gravity_diag.hold();
            // o.gravity_scale already holds the last requested ramp level.
        };
        auto send=[&](bool probing) {
            if(o.mock) return;
            const bool damping=mode=="damping";
            const Q setpoint=damping?damp_q:integral.apply(command);
            std::array<ArmJointCmd,7> cmds{};
            for(size_t i=0;i<7;++i) {
                cmds[i].mode=1; cmds[i].timed_out=false;
                cmds[i].q=static_cast<float>(setpoint[i]);
                cmds[i].kp=probing?0:static_cast<float>(damping?o.fault_kp:gains(o,o.kp,o.joint_kp)[i]);
                cmds[i].kd=probing?0:static_cast<float>(damping?o.fault_kd:gains(o,o.kd,o.joint_kd)[i]);
                // SDK expects URDF/output-side Nm and applies calibration signs
                // and motor gearing itself. Do not multiply by the gear ratio.
                cmds[i].tau=probing?0:static_cast<float>(gravity_ramp.applied[i]);
            }
            bus.step(cmds);
        };
        auto emit=[&]() {
            auto flush_output=[&]() {
                while(output_offset<output_pending.size()) {
                    auto n=write(STDOUT_FILENO,output_pending.data()+output_offset,output_pending.size()-output_offset);
                    if(n>0) output_offset+=static_cast<size_t>(n);
                    else if(n<0 && errno==EINTR) continue;
                    else { if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK) running=0; return false; }
                }
                output_pending.clear(); output_offset=0; return true;
            };
            // Detailed records may exceed PIPE_BUF. Never truncate a JSON line
            // or block the control loop on a slow diagnostic reader.
            if(!flush_output()) return;
            std::ostringstream s;
            double tracking_error=0;
            for(size_t i=0;i<7;++i) tracking_error=std::max(tracking_error,std::abs(command[i]-q[i]));
            const Q setpoint=mode=="damping"?damp_q:integral.apply(command);
            s<<std::setprecision(10)<<"{\"protocol\":3,\"seq\":"<<status_seq++
             <<",\"capabilities\":[\"move\",\"config\",\"gravity\",\"gravity_diagnostic\",\"gravity_z_validation\""
             <<",\"gravity_diagnostic_target\",\"integral\",\"damping\",\"recover\",\"startup_recovery\"]"
             <<",\"config_ack\":"<<config_ack<<",\"config_revision\":"<<config_revision
             <<",\"config_error\":\""<<config_error<<"\",\"config\":{"
             <<"\"joint_speed\":"<<o.speed<<",\"joint_accel\":"<<o.acceleration
             <<",\"joint_jerk\":"<<o.jerk<<",\"kp\":"<<jsonGains(o.kp,o.joint_kp,o.joint_gains)
             <<",\"kd\":"<<jsonGains(o.kd,o.joint_kd,o.joint_gains)
             <<",\"kp_base\":"<<o.kp<<",\"kd_base\":"<<o.kd
             <<",\"lead_limit\":"<<o.lead_limit<<",\"tracking_limit\":"<<o.tracking_limit
             <<",\"move_timeout\":"<<o.move_timeout<<",\"ki\":"<<o.ki<<",\"integral_limit\":"<<jsonGains(o.integral_limit,o.joint_integral_limit,o.joint_limits)
             <<",\"fault_kp\":"<<o.fault_kp<<",\"fault_kd\":"<<o.fault_kd
             <<",\"recover_margin\":"<<o.recover_margin<<"}"
             <<",\"mock\":"<<(o.mock?"true":"false")
             <<",\"mode\":\""<<mode<<"\",\"reason\":\""<<reason<<"\",\"q\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<(std::isfinite(q[i])?q[i]:0);
            s<<"],\"command\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<command[i];
            s<<"],\"target\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<target[i];
            s<<"],\"position_setpoint\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<setpoint[i];
            s<<"],\"integral\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<integral.value[i];
            s<<"],\"command_velocity\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<motion.input.current_velocity[i];
            s<<"],\"command_acceleration\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<motion.input.current_acceleration[i];
            s<<"],\"gravity_model\":"<<(o.have_gravity?"true":"false")
             <<",\"gravity_tool_mass_kg\":"<<o.gravity_model.tool_mass
             <<",\"gravity_tool_com_m\":["<<o.gravity_model.tool_com[0]<<","<<o.gravity_model.tool_com[1]<<","<<o.gravity_model.tool_com[2]<<"]"
             <<",\"gravity_scale\":"<<o.gravity_scale<<",\"gravity_gain\":"<<gravity_ramp.gain
             <<",\"gravity_saturated\":"<<(gravity_ramp.saturated?"true":"false")
             <<",\"gravity_diag_status\":\""<<gravity_diag.status<<"\""
             <<",\"gravity_diag_elapsed_s\":"<<gravity_diag.elapsed
             <<",\"gravity_diag_start_scale\":"<<gravity_diag.start_scale
             <<",\"gravity_diag_target_scale\":"<<gravity_diag.target_scale
             <<",\"gravity_arm_nm\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<gravity_prediction.arm[i];
            s<<"],\"gravity_tool_nm\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<gravity_prediction.tool[i];
            s<<"],\"tau_ff\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<gravity_ramp.applied[i];
            s<<"],\"dq\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<measured_dq[i];
            s<<"],\"tau_est\":[";
            for(size_t i=0;i<7;++i) s<<(i?",":"")<<measured_tau[i];
            s<<"],\"feedback_valid\":"<<(feedback_valid?"true":"false")
             <<",\"feedback_diagnostics\":"<<(o.mock?"[]":feedbackDiagnostics(feedback,lower,upper))
             <<",\"first_fault_feedback\":"<<first_fault_diagnostics
             <<",\"max_tracking_error\":"<<tracking_error
             <<",\"lead_limited\":"<<(lead_limited?"true":"false")<<"}\n";
            output_pending=s.str();
            flush_output();
        };
        while(running) {
            const auto now=Clock::now();
            bool fresh=true, sound=true, within=true, foc=true, recoverable=o.recover_margin>0;
            if(!o.mock) {
                feedback=bus.feedbackSnapshot();
                const auto& states=feedback.urdf; const auto& ages=feedback.age_s;
                for(size_t i=0;i<7;++i) {
                    q[i]=states[i].q;
                    measured_dq[i]=std::isfinite(states[i].dq)?states[i].dq:0;
                    measured_tau[i]=std::isfinite(states[i].tau_est)?states[i].tau_est:0;
                    fresh &= ages[i]<.2;
                    sound &= std::isfinite(q[i])&&std::isfinite(states[i].dq)&&states[i].error_code==0;
                    within &= q[i]>=lower[i]-.03&&q[i]<=upper[i]+.03;
                    recoverable &= std::isfinite(q[i])&&!outside(i,q[i],o.recover_margin);
                    foc &= states[i].mode==1;
                }
                feedback_valid=fresh&&sound&&within;
                damp_feedback_ok=fresh&&sound&&foc;
                if(mode=="probing") {
                    if(feedback_valid) { capture_feedback(); mode="enabling"; phase=now; }
                    else if(fresh&&sound&&recoverable) {
                        begin_recovery(q); recovery_pending=true; mode="enabling"; phase=now;
                    }
                    else if(age(phase)>2) release("initial_feedback_unavailable_or_invalid",true);
                } else if(mode=="enabling"||active()) {
                    if(!fresh||!sound) release("feedback_stale_or_faulted",true);
                    else if(mode=="enabling") {
                        if(!within) release("joint_outside_limits",true);
                        else if(foc) { mode=recovery_pending?"recovering":"hold"; phase=now; recovery_pending=false; }
                        else if(age(phase)>1) release("FOC_enable_timeout",true);
                    } else if(!foc) release("left_FOC",true);
                    else if(!within) damp("joint_outside_limits");
                } else if(mode=="damping") {
                    if(!fresh||!sound||!foc) release("damping_feedback_lost",true);
                }
            }
            // Only the UI event loop refreshes this lease. A hung UI cannot keep
            // moving: probing releases, an enabled arm is damped (or released).
            if(age(heartbeat)>.5) {
                if(mode=="probing") release("UI_heartbeat_timeout",true);
                else if(mode=="enabling"||active()) damp("UI_heartbeat_timeout");
            }
            char buffer[2048];
            ssize_t n=read(STDIN_FILENO,buffer,sizeof(buffer));
            if(n==0) { release("parent_disconnected",false); break; }
            if(n>0) pending.append(buffer,static_cast<size_t>(n));
            if(n<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR) { release("input_error",true); break; }
            if(pending.size()>8192) { damp("protocol_overflow"); pending.clear(); }
            size_t end;
            while((end=pending.find('\n'))!=std::string::npos) {
                std::string line=pending.substr(0,end); pending.erase(0,end+1);
                std::istringstream in(line); std::string op,extra; in>>op;
                if(op=="PING") { heartbeat=now; continue; }
                if(op=="STOP"||op=="QUIT") {
                    release("operator_stop",false); if(op=="QUIT") running=0; continue;
                }
                if(op=="DAMP") { damp("service_damp"); continue; }
                if(op=="RECOVER") {
                    // Only from damping, with healthy feedback inside the limits.
                    // Holds the measured pose; never resumes the old reference.
                    if(mode=="damping" && damp_feedback_ok && (o.mock||within)) { capture_feedback(); reason="recovered"; }
                    continue;
                }
                if(!active()) continue; // latched fault/stop/damping; never re-enable via input
                if(mode=="recovering") continue; // finish the startup recovery first
                if(op=="GRAVITY_TICK") {
                    if(gravity_diag.armed() && !(in>>extra)) gravity_observation=Clock::now();
                    continue;
                }
                if(op=="GRAVITY_DIAG") {
                    try {
                        double target_scale=.5;
                        if(in>>extra) {
                            target_scale=std::stod(extra);
                            require(!(in>>extra), "gravity_diagnostic_invalid_command");
                        }
                        require(!integral.enabled(), "gravity_diagnostic_requires_integral_off");
                        require(mode=="hold" && o.have_gravity && !gravity_ramp.saturated && feedback_valid,
                                "gravity_diagnostic_requires_healthy_hold");
                        for(size_t i=0;i<7;++i)
                            require(std::abs(command[i]-target[i])<1e-9 &&
                                    std::abs(motion.input.current_velocity[i])<1e-9,
                                    "gravity_diagnostic_requires_stationary_reference");
                        gravity_diag.start(command,q,o.gravity_scale,gravity_ramp.gain,target_scale,
                                           uniform(gains(o,o.kp,o.joint_kp)),uniform(gains(o,o.kd,o.joint_kd)));
                        gravity_observation=Clock::now();
                    } catch(const std::exception& e) { damp(e.what()); }
                    continue;
                }
                if(op=="GRAVITY_Z_PREPARE") {
                    try {
                        require(!(in>>extra) && mode=="hold" && feedback_valid && !gravity_ramp.saturated,
                                "gravity_z_requires_healthy_hold");
                        gravity_diag.guard(command,target,q,uniform(gains(o,o.kp,o.joint_kp)),uniform(gains(o,o.kd,o.joint_kd)));
                        gravity_diag.prepareZ(o.gravity_scale,gravity_ramp.gain);
                    } catch(const std::exception& e) { damp(e.what()); }
                    continue;
                }
                if(gravity_diag.armed() && op!="HOLD" && !(gravity_diag.status=="z_validation" && op=="TARGET")) {
                    damp("gravity_diagnostic_only_session"); continue;
                }
                if(op=="CONFIG") {
                    // Atomic update at a held COMMAND reference, never measured feedback.
                    hold();
                    Options candidate=o;
                    unsigned long revision=0;
                    bool parsed=bool(in>>revision>>candidate.speed>>candidate.acceleration>>candidate.jerk
                                     >>candidate.kp>>candidate.kd>>candidate.lead_limit
                                     >>candidate.tracking_limit>>candidate.move_timeout);
                    if(in>>extra) parsed=false;
                    // CONFIG carries one kp/kd: per-joint gains survive an unchanged pair
                    // (GUI "apply" of other settings) and give way to a new one.
                    if(candidate.kp!=o.kp||candidate.kd!=o.kd) candidate.joint_gains=false;
                    config_error.clear();
                    try {
                        require(parsed && revision>config_ack, "invalid_config_record");
                        validateSettings(candidate);
                        o=candidate;
                        motion.input.max_velocity.fill(o.speed);
                        motion.input.max_acceleration.fill(o.acceleration);
                        motion.input.max_jerk.fill(o.jerk);
                        motion.reset(command);
                        config_revision=revision;
                    } catch(const std::exception& e) {
                        config_error=e.what();
                    }
                    if(revision>config_ack) config_ack=revision;
                }
                else if(op=="HOLD") hold();
                else if(op=="HOME") { target.fill(0); mode="home"; phase=now; }
                else if(op=="READY") { target=o.ready; mode="ready"; phase=now; }
                else if(op=="TARGET"||op=="MOVE") {
                    Q candidate{}; bool valid=true;
                    for(size_t i=0;i<7;++i) {
                        if(!(in>>candidate[i])) valid=false;
                        valid &= std::isfinite(candidate[i])&&candidate[i]>=o.lower[i]&&candidate[i]<=o.upper[i]
                            &&(op=="MOVE"||std::abs(candidate[i]-q[i])<=.12);
                    }
                    if(in>>extra) valid=false;
                    if(!valid) damp("invalid_joint_target");
                    else { target=candidate; mode=op=="MOVE"?"position":"moving"; phase=now; }
                } else damp("invalid_command");
            }
            if(!running) break;
            if(active()) {
                bool reached=false, advance=true;
                Q proposed=command;
                try {
                    reached=motion.propose(target,lower,upper);
                    proposed=motion.output.new_position;
                } catch(const std::exception& e) {
                    damp(e.what());
                }
                for(size_t i=0;i<7&&active();++i) {
                    if(std::abs(command[i]-q[i])>o.tracking_limit) { damp("tracking_error"); break; }
                    // Gate forward progress, never drag a held reference along
                    // with sagging feedback. Motion reducing a large error is allowed.
                    if(std::abs(proposed[i]-q[i])>o.lead_limit&&
                       std::abs(proposed[i]-q[i])>std::abs(command[i]-q[i])+1e-9) advance=false;
                }
                lead_limited=!advance;
                if(active()) {
                    if(advance) {
                        command=proposed;
                        motion.accept();
                    } else {
                        motion.reset(command);
                        reached=false;
                    }
                    if(o.mock) for(size_t i=0;i<7;++i) q[i]=command[i]+integral.value[i]-o.mock_load[i];
                    // Integrate only at a (nearly) stationary reference with valid
                    // feedback; never during recovery or a gravity experiment.
                    if(mode!="recovering" && !gravity_diag.armed() && feedback_valid)
                        integral.update(command,q,motion.input.current_velocity,.01);
                    // HOLD means the reference trajectory has finished, not zero
                    // measured tracking error: a PD arm can retain a load offset.
                    // A reference timeout keeps holding where the reference is.
                    if(reached) {
                        // Drop the finished recovery trajectory: Ruckig would keep
                        // reporting its out-of-limit start as a position extremum.
                        if(mode=="recovering") { lower=o.lower; upper=o.upper; motion.reset(command); reason="startup_recovered"; }
                        mode="hold";
                    }
                    else if(mode=="moving"&&age(phase)>5) { reason="jog_tracking_timeout"; hold(); }
                    else if(mode=="home"&&age(phase)>o.move_timeout) { reason="home_timeout"; hold(); }
                    else if(mode=="ready"&&age(phase)>o.move_timeout) { reason="ready_timeout"; hold(); }
                    else if(mode=="position"&&age(phase)>o.move_timeout) { reason="position_timeout"; hold(); }
                    else if(mode=="recovering"&&age(phase)>kRecoveryTimeout) damp("startup_recovery_timeout");
                }
            }
            // Never evaluate/apply gravity during probing, enabling or a latched
            // stop/fault. Damping keeps it. The model uses measured q, not the setpoint.
            if((active()||mode=="damping")&&o.have_gravity) {
                try {
                    if(gravity_diag.armed()) {
                        require(age(gravity_observation)<=.25, "gravity_diagnostic_observer_timeout");
                        gravity_diag.guard(command,target,q,uniform(gains(o,o.kp,o.joint_kp)),uniform(gains(o,o.kd,o.joint_kd)));
                        if(gravity_diag.status=="z_validation")
                            require(std::abs(o.gravity_scale-gravity_diag.target_scale)<1e-9 &&
                                    std::abs(gravity_ramp.gain-gravity_diag.target_scale)<1e-9,
                                    "gravity_z_gain_changed");
                        o.gravity_scale=gravity_diag.advance(o.gravity_scale,.01);
                    }
                    gravity_prediction=o.gravity_model.evaluate(q);
                    gravity_ramp.update(gravity_prediction.total,o.gravity_scale,.01);
                    if(gravity_diag.armed())
                        require(!gravity_ramp.saturated, "gravity_diagnostic_gravity_saturated");
                } catch(const std::exception& e) {
                    // Damping keeps the last applied feedforward rather than dropping it.
                    if(mode!="damping") damp(e.what());
                }
            } else gravity_ramp.reset();
            if(mode=="probing") send(true);
            else if(mode=="enabling"||active()||mode=="damping") send(false);
            if(age(last_emit)>.05) { emit(); last_emit=now; }
            next+=std::chrono::milliseconds(10);
            if(next<Clock::now()) next=Clock::now();
            std::this_thread::sleep_until(next);
        }
        if(mode!="fault"&&mode!="stopped") release("process_exit",false);
        emit();
        if(!o.mock) { for(int i=0;i<10;++i) {bus.stop(); std::this_thread::sleep_for(std::chrono::milliseconds(10));} }
        bus.close();
        return mode=="fault"?2:0;
    } catch(const std::exception& e) {
        std::cerr<<"Keyboard worker: "<<e.what()<<"\n";
        bus.stop(); bus.close(); return 2;
    }
}
