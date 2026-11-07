// Every rejection reason is reported for a snapshot with one bad joint of each kind.
#include "feedback_diagnostics.h"
#include <iostream>
#include <limits>

int main() {
    ArmFeedbackSnapshot f;
    std::array<double,7> lo{},hi{}; hi.fill(1);
    for(int i=0;i<7;++i) {
        f.ready[i]=true; f.age_s[i]=.01; f.motor_ids[i]=i;
        f.urdf[i].q=.5; f.motor[i].q=.5;
    }
    f.ready[0]=false; f.age_s[0]=std::numeric_limits<double>::infinity();
    f.age_s[1]=.25;
    f.urdf[2].q=std::numeric_limits<float>::quiet_NaN();
    f.urdf[3].error_code=17;
    f.urdf[4].q=-.031;
    f.urdf[5].q=-.029; f.urdf[5].mode=0; f.urdf[5].timeout=1;
    const auto json=feedbackDiagnostics(f,lo,hi);
    if(json.find("no_packet")==std::string::npos || json.find("stale")==std::string::npos ||
       json.find("nonfinite")==std::string::npos || json.find("motor_error")==std::string::npos ||
       json.find("outside_limits")==std::string::npos) return 1;
    std::cout<<json<<'\n';
    return 0;
}
