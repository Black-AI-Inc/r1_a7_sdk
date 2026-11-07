#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Static gravity torques for the 7-joint chain plus a tool, evaluated by the
// worker at the measured q. Same model as GravityModel in
// scripts/gravity_model.py, which serializes it with native_record().
namespace gravity {
using V = std::array<double,3>;
using M = std::array<double,9>;   // 3x3, row-major
using Q = std::array<double,7>;
inline V add(V a, V b) { for(int i=0;i<3;++i) a[i]+=b[i]; return a; }
inline V sub(V a, V b) { for(int i=0;i<3;++i) a[i]-=b[i]; return a; }
inline V scale(V a, double s) { for(auto& v:a) v*=s; return a; }
inline double dot(V a,V b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline V cross(V a,V b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline V mv(M a,V b) { V r{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) r[i]+=a[i*3+j]*b[j]; return r; }
inline M mm(M a,M b) { M r{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) r[i*3+j]+=a[i*3+k]*b[k*3+j]; return r; }
inline M identity() { return {1,0,0,0,1,0,0,0,1}; }
// Rotation by angle q about the unit axis a (Rodrigues).
inline M rotation(V a,double q) {
    const double c=std::cos(q), s=std::sin(q);
    M r{}; M skew{0,-a[2],a[1],a[2],0,-a[0],-a[1],a[0],0};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        r[i*3+j]=(i==j?c:0)+(1-c)*a[i]*a[j]+s*skew[i*3+j];
    return r;
}
inline void check(bool ok) { if(!ok) throw std::runtime_error("invalid_gravity_model"); }

struct Link { M rotation{}; V origin{}, axis{}, com{}; double mass=0; };
struct Result { Q arm{}, tool{}, total{}; };
struct Model {
    std::array<Link,7> links{};
    V g{}, tool_com{};
    double tool_mass=0;
    // Whitespace-separated record: version 1, gravity vector, then per link the
    // joint origin rotation (row-major) and translation, joint axis, mass and
    // COM in the link frame, then tool mass and COM in the Link7 frame.
    static Model parse(const std::string& record) {
        std::istringstream in(record);
        auto read=[&]() { double v=0; check(bool(in>>v)&&std::isfinite(v)); return v; };
        auto vec=[&]() { return V{read(),read(),read()}; };
        check(read()==1);
        Model model; model.g=vec();
        // This controller's profile supports an upright base only.
        check(std::abs(model.g[0])<1e-12&&std::abs(model.g[1])<1e-12&&std::abs(model.g[2]+9.81)<1e-9);
        for(auto& l:model.links) {
            for(auto& v:l.rotation) v=read();
            V rows[3]{{l.rotation[0],l.rotation[1],l.rotation[2]},
                      {l.rotation[3],l.rotation[4],l.rotation[5]},
                      {l.rotation[6],l.rotation[7],l.rotation[8]}};
            for(int i=0;i<3;++i) for(int j=0;j<3;++j)
                check(std::abs(dot(rows[i],rows[j])-(i==j?1:0))<1e-8);
            check(std::abs(dot(rows[0],cross(rows[1],rows[2]))-1)<1e-8);
            l.origin=vec(); l.axis=vec(); l.mass=read(); l.com=vec();
            check(dot(l.origin,l.origin)<=1&&dot(l.com,l.com)<=1&&
                  std::abs(dot(l.axis,l.axis)-1)<1e-8&&l.mass>=0&&l.mass<=10);
        }
        model.tool_mass=read(); model.tool_com=vec();
        check(model.tool_mass>=0&&model.tool_mass<=2.5&&dot(model.tool_com,model.tool_com)<=1);
        std::string extra; check(!(in>>extra));
        return model;
    }
    Result evaluate(Q q) const {
        M r=identity(); V p{};
        std::array<V,7> origins{}, axes{};
        Result result;
        for(int i=0;i<7;++i) {
            check(std::isfinite(q[i]));
            const auto& l=links[i];
            p=add(p,mv(r,l.origin)); r=mm(r,l.rotation);
            origins[i]=p; axes[i]=mv(r,l.axis);
            r=mm(r,rotation(l.axis,q[i]));
            V com=add(p,mv(r,l.com));
            // Opposite of the physical gravity torque: generalized dU/dq.
            for(int j=0;j<=i;++j)
                result.arm[j]-=dot(axes[j],cross(sub(com,origins[j]),scale(g,l.mass)));
        }
        V com=add(p,mv(r,tool_com));
        for(int j=0;j<7;++j) {
            result.tool[j]=-dot(axes[j],cross(sub(com,origins[j]),scale(g,tool_mass)));
            result.total[j]=result.arm[j]+result.tool[j];
            check(std::isfinite(result.total[j]));
        }
        return result;
    }
};

// These are commissioning feedforward caps, NOT motor continuous torque ratings.
// They limit only the added gravity term; the motor's PD term remains separate.
// The gain rises to requested_scale over 2 s and the applied torque slews at
// most 2 N*m/s per joint, so enabling or a model jump never steps the torque.
struct Ramp {
    Q applied{};
    const Q caps{8,8,6,6,4,1.5,1.5};   // N*m, J1..J7
    double gain=0;
    bool saturated=false;               // some joint wanted more than its cap
    void reset() { applied.fill(0); gain=0; saturated=false; }
    void update(Q total,double requested_scale,double dt) {
        check(std::isfinite(requested_scale)&&requested_scale>=0&&requested_scale<=1&&std::isfinite(dt)&&dt>=0);
        dt=std::min(dt,.02); // A missed cycle must not cause a catch-up torque step.
        gain=std::min(requested_scale,gain+requested_scale*dt/2.);
        saturated=false;
        for(int j=0;j<7;++j) {
            check(std::isfinite(total[j]));
            double desired=total[j]*gain;
            saturated|=std::abs(desired)>caps[j];
            desired=std::clamp(desired,-caps[j],caps[j]);
            applied[j]+=std::clamp(desired-applied[j],-2*dt,2*dt);
        }
    }
};
} // namespace gravity
