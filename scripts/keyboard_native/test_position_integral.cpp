#include "position_integral.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    using Q = PositionIntegral::Q;
    PositionIntegral off;
    Q reference{}, measured{}, still{}, moving{};
    measured[3]=-.04;
    off.update(reference,measured,still,.01);
    assert(off.value[3]==0 && off.apply(reference)[3]==0);       // ki=0: original behaviour

    PositionIntegral integral;
    integral.configure(1.5,.03);
    // Plant: static error = (load - kp*integral)/kp, i.e. q = reference + integral - sag.
    const double sag=.04;
    for(int i=0;i<2000;++i) {
        measured=integral.apply(reference); measured[3]-=sag;
        integral.update(reference,measured,still,.01);
    }
    assert(std::abs(integral.value[3]-.03)<1e-12);                // clamped at the limit
    const double sag2=.02;
    integral.reset();
    for(int i=0;i<2000;++i) {
        measured=integral.apply(reference); measured[3]-=sag2;
        integral.update(reference,measured,still,.01);
    }
    measured=integral.apply(reference); measured[3]-=sag2;
    assert(std::abs(reference[3]-measured[3])<=integral.deadband+1e-12);  // error removed
    const double held=integral.value[3];
    moving[0]=.1;                                                 // frozen during motion
    measured[3]-=.01;
    integral.update(reference,measured,moving,.01);
    assert(integral.value[3]==held);
    measured=integral.apply(reference); measured[3]-=sag2-.0001; // inside deadband: no hunting
    integral.update(reference,measured,still,.01);
    assert(integral.value[3]==held);
    integral.update(reference,measured,still,10);                 // missed cycle cannot jump

    // Stick-slip plant (field 2026-11-03, J2): the joint stays put until the PD
    // spring exceeds static friction (0.01 rad x kp), then slides to the setpoint
    // minus a kinetic-friction stop (0.0005 rad). Without the sign-change hold
    // this cycles -0.004 <-> +0.0055 rad indefinitely.
    PositionIntegral stick;
    stick.configure(1.5,.03);
    double q=.004;
    int slips=0;
    auto run=[&](double seconds, const Q& velocity) {
        for(int i=0;i<static_cast<int>(seconds/.01);++i) {
            const double setpoint=stick.apply(reference)[1];
            if(std::abs(setpoint-q)>.01) { q=setpoint-(setpoint>q?.0005:-.0005); ++slips; }
            measured=reference; measured[1]=q;
            stick.update(reference,measured,velocity,.01);
        }
    };
    run(30,still);
    assert(slips==1 && stick.crossed[1]);                         // one slip, then held
    assert(std::abs(reference[1]-q)<.006);
    const double after_slip=stick.value[1];
    run(5,still);
    assert(stick.value[1]==after_slip && slips==1);
    Q step{}; step[1]=.1;                                         // the next move re-arms it
    run(.01,step);
    assert(!stick.crossed[1] && stick.side[1]==0);
    run(.5,still);
    assert(stick.value[1]!=after_slip);                           // integrating again
    // End of a move (field 2026-11-03 21:50, J2): the joint slips across the
    // reference as the reference comes to rest. Within settle_s that only
    // re-arms the side and the integral keeps removing the offset; a sign
    // change after settle_s still holds the integral.
    PositionIntegral settle;
    settle.configure(1.5,.03);
    measured=reference; measured[1]=reference[1]-.0014;          // error +0.0014
    for(int i=0;i<15;++i) settle.update(reference,measured,still,.01);
    measured[1]=reference[1]+.0035;                               // slipped: error -0.0035
    const double before_slip=settle.value[1];
    for(int i=0;i<100;++i) settle.update(reference,measured,still,.01);
    assert(!settle.crossed[1] && settle.side[1]==-1 && settle.value[1]<before_slip);
    measured[1]=reference[1]-.002;
    settle.update(reference,measured,still,.01);
    assert(settle.crossed[1]);

    // Per-joint limits: equal torque authority, 0.03 rad x kp 100 = 0.015 rad x kp 200.
    PositionIntegral joints;
    joints.configure(1.5,Q{.03,.015,.03,.015,.03,.03,.03});
    for(int i=0;i<4000;++i) {
        measured=joints.apply(reference); measured[1]-=sag; measured[3]-=sag;
        joints.update(reference,measured,still,.01);
    }
    assert(std::abs(joints.value[1]-.015)<1e-12 && std::abs(joints.value[3]-.015)<1e-12);
    bool rejected=false;
    try { integral.configure(6,.03); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
    rejected=false;
    try { joints.configure(1.5,Q{.03,.06,.03,.03,.03,.03,.03}); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
    std::cout<<"position integral passed\n";
}
