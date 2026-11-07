// Guards, ramp and cancellation of the one-shot gravity diagnostic.
#include "gravity_diagnostic.h"
#include <cassert>
#include <iostream>

template<class F> void rejected(F fn) {
    bool failed=false;
    try { fn(); } catch(const std::runtime_error&) { failed=true; }
    assert(failed);
}
int main() {
    GravityDiagnostic::Q q{};
    GravityDiagnostic d;
    rejected([&]{d.start(q,q,.3,.25,.5,100,1);});   // start scale not settled
    rejected([&]{d.start(q,q,.25,.25,.5,60,1);});
    rejected([&]{d.start(q,q,.5,.5,.5,100,1);});    // target must exceed start
    rejected([&]{d.start(q,q,.5,.5,1.1,100,1);});
    d.start(q,q,.25,.25,.5,100,1);
    rejected([&]{d.prepareZ(.25,.25);});
    double scale=.25;
    for(int i=0;i<1000;++i) {
        double previous=scale;
        scale=d.advance(scale,.01);
        assert(scale>=previous && scale<=.5 && scale-previous<=.00025000001);
        d.guard(q,q,q,100,1);
    }
    assert(scale==.5 && d.status=="holding");
    rejected([&]{d.start(q,q,.25,.25,.5,100,1);});
    auto moved=q; moved[3]=.01001;
    rejected([&]{d.guard(q,q,moved,100,1);});
    moved=q; moved[5]=.0001;
    rejected([&]{d.guard(moved,q,q,100,1);});
    rejected([&]{d.guard(q,moved,q,100,1);});
    rejected([&]{d.guard(q,q,q,100,.5);});
    rejected([&]{d.prepareZ(.5,.49);});
    d.prepareZ(.5,.5);
    assert(d.armed());
    d.guard(moved,moved,q,100,1);
    d.hold();
    assert(d.status=="z_validation");
    rejected([&]{d.prepareZ(.5,.5);});
    GravityDiagnostic cancelled;
    cancelled.start(q,q,.25,.25,.5,100,1);
    scale=cancelled.advance(.25,.01);
    cancelled.hold();
    rejected([&]{cancelled.prepareZ(.5,.5);});
    assert(cancelled.status=="cancelled");
    assert(cancelled.advance(scale,100)==scale);
    GravityDiagnostic slow;
    slow.start(q,q,.25,.25,.5,100,1);
    assert(slow.advance(.25,10)<=.2505); // missed loop cannot catch up abruptly

    // Field case 2026-11-02: the reference is above the sagging arm. Raising
    // feedforward lifts the arm toward the reference; that is NOT a fault.
    GravityDiagnostic lift;
    GravityDiagnostic::Q reference{}, sagged{};
    reference[3]=.35; sagged[3]=.31;                 // J4 sags 0.04 rad below the command
    lift.start(reference,sagged,.5,.5,1.,100,1);
    auto toward=sagged; toward[3]=.334;              // moved 0.024 rad back toward the command
    lift.guard(reference,reference,toward,100,1);
    toward[3]=.355;                                  // up to 0.01 rad overshoot is tolerated
    lift.guard(reference,reference,toward,100,1);
    toward[3]=.361;                                  // overshoot beyond the margin is rejected
    rejected([&]{lift.guard(reference,reference,toward,100,1);});
    auto away=sagged; away[3]=.299;                  // sagging further away is rejected
    rejected([&]{lift.guard(reference,reference,away,100,1);});
    for(int i=0;i<1000;++i) scale=lift.advance(i?scale:.5,.01);
    assert(scale==1. && lift.status=="holding");
    lift.prepareZ(1.,1.);
    std::cout<<"gravity diagnostic guards/ramp/cancellation passed\n";
}
