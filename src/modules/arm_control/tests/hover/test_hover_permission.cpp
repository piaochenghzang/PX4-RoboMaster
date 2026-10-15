#include "../../grasp/HoverPermit.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace so101_hover;
Input good() {
    Input v; v.now=v.received=v.sample=1000000; v.task=1; v.sequence=1; v.epoch=1;
    v.valid=v.flight_safe=v.allocation_ok=true; v.motor_max=.8f;
    v.speed=.01f; v.angular_speed=.02f; v.tilt=.02f; v.position_error=.01f;
    v.alignment_error=.1f; v.relative_speed=.02f; v.orientation_error=.5f;
    return v;
}
void step(Gate &g,Input &v) { v.now+=50000; v.received=v.sample=v.now; ++v.sequence; g.update(v); }
void ready(Gate &g,Input &v) { g.update(v); for(int i=0;i<40;++i) step(g,v); assert(g.start_ready); }
int main() {
    unsigned n=0;
    { Gate g; Input v; g.update(v); assert(!g.start_ready && (g.denial&Missing)); ++n; }
    { Gate g; auto v=good(); ready(g,v); assert(!g.close_geometry_ready);
      v.alignment_error=.002f; v.relative_speed=.001f; v.orientation_error=.01f; step(g,v);
      assert(g.close_geometry_ready); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.position_error=.035f; step(g,v);
      assert(!g.start_ready && (g.denial&Position)); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.now+=100001; g.update(v);
      assert(!g.start_ready && (g.denial&Stale)); ++n; }
    { Gate g; auto v=good(); v.sample=v.now+20001; g.update(v);
      assert(!g.start_ready && (g.denial&Stale)); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.flight_safe=false; step(g,v);
      assert(!g.start_ready && (g.denial&Flight)); ++n; }
    { Gate g; auto v=good(); v.motor_max=.95f; g.update(v); assert(g.denial&Allocation); ++n; }
    { Gate g; auto v=good(); v.allocation_ok=false; g.update(v); assert(g.denial&Allocation); ++n; }
    { Gate g; auto v=good(); v.speed=.06f; g.update(v); assert(g.denial&Speed);
      v.speed=.01f; v.angular_speed=.11f; g.update(v); assert(g.denial&Angular); ++n; }
    { Gate g; auto v=good(); v.tilt=.1f; g.update(v); assert(g.denial&Tilt); ++n; }
    { Gate g; auto v=good(); v.speed=std::numeric_limits<float>::quiet_NaN(); g.update(v);
      assert(g.denial&Invalid); ++n; }
    { Gate g; auto v=good(); ready(g,v); ++v.reset_key; step(g,v);
      assert(g.blocked && (g.denial&Reset)); step(g,v); assert(!g.start_ready); ++n; }
    { Gate g; auto v=good(); ready(g,v); ++v.epoch; step(g,v);
      assert(g.blocked); v.task=2; ready(g,v); assert(g.start_ready); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.now-=1000000; v.received=v.sample=v.now; g.update(v);
      assert(g.blocked && !g.start_ready); ++n; }
    { Gate g; auto v=good(); ready(g,v); --v.sequence; v.now+=1000; v.received=v.sample=v.now; g.update(v);
      assert(g.blocked && !g.start_ready); ++n; }
    { Gate g; auto v=good(); g.update(v); for(int i=0;i<39;++i)step(g,v); assert(!g.start_ready);
      step(g,v); assert(g.start_ready); ++n; }
    { Gate g; auto v=good(); v.valid=false; g.update(v); assert(g.denial&Invalid); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.now+=1000; v.received=v.now; v.sample=v.now;
      g.update(v); assert(g.blocked && !g.start_ready); ++n; }
    { Gate g; auto v=good(); ready(g,v); v.now+=100001; v.received=v.now; g.update(v);
      assert(!g.start_ready && (g.denial&Stale)); ++n; }
    { Preparation p; p.request(1,1,1,true,true); assert(p.prepared_task==1 && p.result==1);
      p.update(1,false); assert(!p.prepared_task && p.result==3);
      p.update(1,true); assert(!p.prepared_task); ++n; }
    { Preparation p; p.request(1,2,1,true,true); assert(!p.prepared_task && p.result==2); ++n; }
    { Preparation p; p.request(1,1,1,true,false); assert(!p.prepared_task && p.result==2); ++n; }
    { Preparation p; p.request(1,1,1,false,true); assert(!p.prepared_task && p.result==2); ++n; }
    { Preparation p; p.request(1,1,1,true,true); p.update(2,true);
      assert(!p.prepared_task && p.result==3); ++n; }
    { Preparation p; p.request(2,1,1,true,true); p.update(1,false);
      p.request(2,1,1,true,true); assert(!p.prepared_task && p.result==3);
      p.request(1,1,1,true,true); assert(!p.prepared_task && p.result==3); ++n; }
    { Preparation p; p.request(0,1,1,true,true); assert(!p.prepared_task && !p.last_request); ++n; }
    return n==26 ? 0 : 1;
}
