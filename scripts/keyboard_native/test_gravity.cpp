#include "gravity_model.h"
#include <iostream>
#include <iomanip>

int main(int argc,char** argv) {
    try {
        if(argc==2) {
            auto model=gravity::Model::parse(argv[1]);
            gravity::Q q{};
            while(std::cin>>q[0]) {
                for(int i=1;i<7;++i) gravity::check(bool(std::cin>>q[i]));
                auto result=model.evaluate(q);
                for(auto v:result.total) std::cout<<std::setprecision(17)<<v<<' ';
                std::cout<<'\n';
            }
            return 0;
        }
        gravity::Model model;
        model.g={0,0,-9.81};
        for(auto& link:model.links) { link.rotation=gravity::identity(); link.axis={0,1,0}; }
        model.links[0].mass=1; model.links[0].com={1,0,0};
        auto result=model.evaluate({});
        gravity::check(std::abs(result.total[0]+9.81)<1e-12);
        for(int i=1;i<7;++i) gravity::check(result.total[i]==0);
        gravity::Ramp ramp;
        gravity::Q high{100,-100,100,-100,100,-100,100};
        for(int i=0;i<1000;++i) {
            auto previous=ramp.applied;
            ramp.update(high,1,.01);
            for(int j=0;j<7;++j) {
                gravity::check(std::abs(ramp.applied[j]-previous[j])<=.020000001);
                gravity::check(std::abs(ramp.applied[j])<=ramp.caps[j]+1e-12);
            }
        }
        gravity::check(ramp.saturated&&ramp.gain==1);
        ramp.reset();
        gravity::check(ramp.gain==0&&!ramp.saturated);
        for(auto v:ramp.applied) gravity::check(v==0);
        ramp.update(high,1,10); // missed cycles remain slew-limited
        for(auto v:ramp.applied) gravity::check(std::abs(v)<=.04);
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
