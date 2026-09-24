// Standalone Gazebo Transport probe, not linked into flight firmware.
#ifdef SO101_CLOCK_PLUGIN

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {

class GraspClock : public WorldPlugin
{
private:
    transport::NodePtr node_;
    transport::PublisherPtr pub_;
    event::ConnectionPtr update_connection_;

    double last_publish_sim_time_{-1.0};

public:
    void Load(physics::WorldPtr world, sdf::ElementPtr) override
    {
        if (!world) {
            gzerr << "[GraspClock] Invalid world\n";
            return;
        }

        node_.reset(new transport::Node());
        node_->Init(world->Name());

        // Clock only drives a 10 Hz experiment scheduler.
        // Publishing at 100 Hz avoids flooding Gazebo Transport with
        // one message for every 1 ms physics step.
        pub_ = node_->Advertise<msgs::Time>("~/grasp/clock", 1);

        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                [this](const common::UpdateInfo &info)
                {
                    const double now = info.simTime.Double();

                    // 100 Hz clock.
                    if (last_publish_sim_time_ >= 0.0 &&
                        now - last_publish_sim_time_ < 0.01)
                    {
                        return;
                    }

                    last_publish_sim_time_ = now;

                    msgs::Time msg;
                    msg.set_sec(info.simTime.sec);
                    msg.set_nsec(info.simTime.nsec);

                    pub_->Publish(msg);
                });

        gzmsg << "[GraspClock] Publishing ~/grasp/clock at <=100 Hz\n";
    }
};

GZ_REGISTER_WORLD_PLUGIN(GraspClock)

} // namespace gazebo

#else
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <ignition/math/Quaternion.hh>

std::mutex mutex;
double angle=0, speed=0, torque=0, sim_time=0;
double cube_x=0, cube_y=0, cube_z=0;
double base_z=1;
unsigned samples=0, contact_frames=0, bilateral=0;
bool state_finite=true, have_cube=false;
bool support_clear=false;
ignition::math::Quaterniond cube_rotation;
double fn[2]{}, ft[2]{}, fz[2]{}, tx[2]{}, cy[2]{}, cz[2]{};
unsigned force_frames=0;
bool have_stats=false,have_clock=false,clock_reversed=false;
unsigned reordered_messages=0;
double last_contact=-1,last_status_sim=-1;
auto last_clock_wall=std::chrono::steady_clock::now();
void status(ConstVector2dPtr &m) {
    std::lock_guard<std::mutex> lock(mutex);
    angle=m->x(); speed=m->y(); ++samples;
    last_status_sim=sim_time;
    state_finite=state_finite && std::isfinite(angle) && std::isfinite(speed);
}
void effort(ConstVector3dPtr &m) {
    std::lock_guard<std::mutex> lock(mutex); torque=m->x();
}
void stats(ConstWorldStatisticsPtr &m) {
    std::lock_guard<std::mutex> lock(mutex);
    // Statistics are coarse and can arrive after newer contact timestamps.
    if(!have_clock) sim_time=m->sim_time().sec()+m->sim_time().nsec()*1e-9;
    have_stats=true;
}
void clockUpdate(ConstTimePtr &m) {
    std::lock_guard<std::mutex> lock(mutex);
    const double stamp=m->sec()+m->nsec()*1e-9;
    if(have_clock && stamp<sim_time-1e-8) {
        ++reordered_messages;
        if(sim_time-stamp>.1) clock_reversed=true;
        return;
    }
    if(!have_clock || stamp>sim_time) {
        sim_time=stamp;have_clock=true;last_clock_wall=std::chrono::steady_clock::now();
    }
}
void poses(ConstPosesStampedPtr &m) {
    std::lock_guard<std::mutex> lock(mutex);
    for (int i=0;i<m->pose_size();++i) if(m->pose(i).name()=="gripper_fixture") base_z=m->pose(i).position().z();
    for (int i=0;i<m->pose_size();++i) if(m->pose(i).name()=="cube_support") support_clear=m->pose(i).position().x()>5;
    for (int i=0;i<m->pose_size();++i) if(m->pose(i).name()=="test_cube") {
        cube_x=m->pose(i).position().x(); cube_y=m->pose(i).position().y();
        cube_z=m->pose(i).position().z(); have_cube=true;
        const auto &q=m->pose(i).orientation();
        cube_rotation.Set(q.w(),q.x(),q.y(),q.z());
    }
}
void modelInfo(ConstModelPtr &m) {
    if(m->name()=="cube_support" && m->has_pose()) {
        std::lock_guard<std::mutex> lock(mutex);
        support_clear=m->pose().position().x()>5;
    }
}
void contacts(ConstContactsPtr &m) {
    std::lock_guard<std::mutex> lock(mutex);
    const double stamp=m->time().sec()+m->time().nsec()*1e-9;
    if(last_contact>=0 && stamp<last_contact-1e-8) {
        // Gazebo Transport may deliver adjacent physics samples out of order.
        // Never move the control clock backwards; large jumps invalidate the run.
        ++reordered_messages;
        return;
    }
    if(stamp<=last_contact) return; // Duplicate messages are not new physics steps.
    last_contact=stamp;
    bool fixed=false, moving=false;
    for(int i=0;i<m->contact_size();++i) {
        const auto &c=m->contact(i);
        std::string pair=c.collision1()+" "+c.collision2();
        if(pair.find("test_cube::")==std::string::npos) continue;
        fixed |= pair.find("gripper_fixed_finger_collision")!=std::string::npos;
        moving |= pair.find("moving_jaw_collision")!=std::string::npos;
        int side=pair.find("gripper_fixed_finger_collision")!=std::string::npos ? 0 :
                 pair.find("moving_jaw_collision")!=std::string::npos ? 1 : -1;
        if(side<0 || !have_cube) continue;
        bool first=c.collision1().find("test_cube::")!=std::string::npos;
        for(int j=0;j<c.wrench_size() && j<c.normal_size() && j<c.position_size();++j) {
            const auto &w=first ? c.wrench(j).body_1_wrench() : c.wrench(j).body_2_wrench();
            // Gazebo ODE stores wrench in each link frame, normals/points in world.
            auto f=cube_rotation.RotateVector({w.force().x(),w.force().y(),w.force().z()});
            auto t=cube_rotation.RotateVector({w.torque().x(),w.torque().y(),w.torque().z()});
            ignition::math::Vector3d n(c.normal(j).x(),c.normal(j).y(),c.normal(j).z());
            double normal=std::abs(f.Dot(n));
            fn[side]+=normal; ft[side]+=(f-n*f.Dot(n)).Length();
            fz[side]+=f.Z(); tx[side]+=t.X();
            cy[side]+=normal*c.position(j).y(); cz[side]+=normal*c.position(j).z();
        }
    }
    ++contact_frames; if(fixed && moving) ++bilateral;
    if(have_cube) ++force_frames;
}
int main(int argc,char **argv) {
    const int duration=std::getenv("SO101_TEST_SECONDS") ? std::atoi(std::getenv("SO101_TEST_SECONDS")) : 45;
    const int attitude=std::getenv("SO101_ATTITUDE_CASE") ? std::atoi(std::getenv("SO101_ATTITUDE_CASE")) : 0;
    const double stall_limit=std::getenv("SO101_STALL_SECONDS") ? std::atof(std::getenv("SO101_STALL_SECONDS")) : 15;
    const bool disturb=std::getenv("SO101_DISTURB")!=nullptr, carry=std::getenv("SO101_CARRY")!=nullptr;
    if(duration<45 || duration>600 || attitude<0 || attitude>6 || !std::isfinite(stall_limit) || stall_limit<2) return 4;
    gazebo::client::setup(argc,argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node()); node->Init("default");
    auto a=node->Subscribe("~/so101/gripper/status",status);
    auto b=node->Subscribe("~/so101/gripper/status/effort",effort);
    auto c=node->Subscribe("~/physics/contacts",contacts);
    auto d=node->Subscribe("~/pose/info",poses);
    auto e=node->Subscribe("~/world_stats",stats);
    auto f=node->Subscribe("~/model/info",modelInfo);
    auto clock_sub=node->Subscribe("~/grasp/clock",clockUpdate);
    auto pub=node->Advertise<gazebo::msgs::Vector2d>("~/so101/gripper/command");
    auto physics=node->Advertise<gazebo::msgs::Physics>("~/physics");
    auto load=node->Advertise<gazebo::msgs::Vector3d>("~/grasp/disturbance");
    auto lift=node->Advertise<gazebo::msgs::Vector3d>("~/grasp/lift");
    auto modify=node->Advertise<gazebo::msgs::Model>("~/model/modify");
    auto control=node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    const auto wall_start=std::chrono::steady_clock::now();
    auto wall_now=[&] { return std::chrono::duration<double>(std::chrono::steady_clock::now()-wall_start).count(); };
    auto fail=[&](int code,const char *reason) {
        std::cerr<<"ABORT code="<<code<<" reason="<<reason<<std::endl;
        gazebo::client::shutdown(); return code;
    };
    // Runner starts gzserver paused. Nothing is advanced by a fixed startup sleep.
    while(true) {
        bool ready; {std::lock_guard<std::mutex> lock(mutex);ready=have_stats;}
        if(ready && pub->HasConnections() && control->HasConnections() && modify->HasConnections()
           && (!attitude || physics->HasConnections()) && (!disturb || load->HasConnections())
           && (!carry || lift->HasConnections())) break;
        if(wall_now()>15) return fail(5,"startup_not_ready");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    double origin; {std::lock_guard<std::mutex> lock(mutex);origin=sim_time;last_clock_wall=std::chrono::steady_clock::now();}
    auto command=[&](double target) {gazebo::msgs::Vector2d msg;msg.set_x(target);msg.set_y(0);pub->Publish(msg);};
    command(.6);
    gazebo::msgs::WorldControl start;start.set_pause(false);control->Publish(start);
    std::cerr<<std::setprecision(12)<<"EVENT start sim="<<origin<<" elapsed=0"<<std::endl;
    std::cout<<std::setprecision(12);
    std::cout<<"wall_s,sim_s,elapsed_sim_s,target,q,qd,torque,cube_x,cube_y,cube_z,pose_seen,samples,contact_frames,bilateral_frames,finite,roll,pitch,yaw,force_frames,fn_fixed,ft_fixed,fz_fixed,tx_fixed,cy_fixed,cz_fixed,fn_moving,ft_moving,fz_moving,tx_moving,cy_moving,cz_moving,base_z,support_clear\n";
    double next_command=0,next_log=0,next_support=20,previous_command=-1;
    bool release_requested=false,release_confirmed=false;
    double target=.6;
    while(true) {
        double now,clock_age,status_age; bool reset,valid;
        {
            std::lock_guard<std::mutex> lock(mutex);
            now=sim_time;reset=clock_reversed;valid=state_finite;
            clock_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-last_clock_wall).count();
            status_age=last_status_sim<0 ? now-origin : now-last_status_sim;
        }
        const double t=now-origin;
        if(reset || t< -1e-8) return fail(7,"sim_time_reversed");
        if(clock_age>stall_limit) return fail(6,"sim_clock_stalled_or_missing");
        if(!valid) return fail(8,"nonfinite_joint_state");
        if(t>2 && status_age>.5) return fail(9,"joint_feedback_stale");
        if(t+1e-8>=next_command) {
            if(previous_command>=0 && t-previous_command>.25) return fail(10,"sim_command_deadline_missed");
            previous_command=t;
            target=std::max(-.15,.6-std::max(0.,t-5.)*.1);
            command(target);
            if(disturb) {
                ignition::math::Vector3d force(0,0,0);
                if(t>=40 && t<41) force.Y()=.015;
                if(t>=60 && t<61) force.Y()=-.015;
                if(t>=80 && t<81) force.X()=.015;
                if(t>=100 && t<101) force.X()=-.015;
                gazebo::msgs::Vector3d msg;gazebo::msgs::Set(&msg,force);load->Publish(msg);
            }
            if(carry) {
                double h=0;
                if(t>=40 && t<50) {double u=(t-40)/10;h=.1*u*u*(3-2*u);}
                else if(t>=50 && t<65) h=.1;
                else if(t>=65 && t<75) {double u=(t-65)/10;h=.1*(1-u*u*(3-2*u));}
                gazebo::msgs::Vector3d msg;msg.set_x(h);msg.set_y(0);msg.set_z(0);lift->Publish(msg);
            }
            if(attitude && t>=30) {
                double u=std::min(1.,(t-30)/5), smooth=u*u*(3-2*u);
                const double pi=3.141592653589793;
                const double rolls[]={0,pi/2,-pi/2,pi,0,0,pi/4};
                const double pitches[]={0,0,0,0,pi/2,-pi/2,pi/4};
                auto g=ignition::math::Quaterniond(rolls[attitude]*smooth,pitches[attitude]*smooth,0).RotateVector({0,0,-9.8066});
                gazebo::msgs::Physics msg;msg.set_type(gazebo::msgs::Physics::ODE);gazebo::msgs::Set(msg.mutable_gravity(),g);physics->Publish(msg);
            }
            // Do not replay a burst of old commands after scheduler delay.
            next_command=(std::floor((t+1e-8)*10)+1)/10.;
        }
        if(!release_requested && t>=20) {
            bool safe;
            {std::lock_guard<std::mutex> lock(mutex);safe=have_cube && samples>0 && contact_frames>0 && bilateral>.9*contact_frames;}
            if(!safe) return fail(2,"no_bilateral_contact_before_release");
            release_requested=true;
            std::cerr<<"EVENT release_requested sim="<<now<<" elapsed="<<t<<std::endl;
        }
        if(release_requested && !release_confirmed) {
            bool clear;{std::lock_guard<std::mutex> lock(mutex);clear=support_clear;}
            if(clear) {
                release_confirmed=true;
                std::cerr<<"EVENT release_confirmed sim="<<now<<" elapsed="<<t<<std::endl;
            } else if(t>=23) return fail(3,"support_removal_unconfirmed");
            else if(t>=next_support) {
                gazebo::msgs::Model pedestal;pedestal.set_name("cube_support");
                gazebo::msgs::Set(pedestal.mutable_pose(),ignition::math::Pose3d(10,0,0,0,0,0));modify->Publish(pedestal);
                next_support=t+.5;
            }
        }
        if(t+1e-8>=next_log || t>=duration) {
            std::lock_guard<std::mutex> lock(mutex);
            std::cout<<wall_now()<<','<<now<<','<<t<<','<<target<<','<<angle<<','<<speed<<','<<torque<<','
                <<cube_x<<','<<cube_y<<','<<cube_z<<','<<have_cube<<','<<samples<<','<<contact_frames<<','<<bilateral<<','<<state_finite
                <<','<<cube_rotation.Roll()<<','<<cube_rotation.Pitch()<<','<<cube_rotation.Yaw()<<','<<force_frames;
            for(int s=0;s<2;++s) {
                double count=force_frames ? force_frames : 1;
                std::cout<<','<<fn[s]/count<<','<<ft[s]/count<<','<<fz[s]/count<<','<<tx[s]/count
                         <<','<<(fn[s]>0 ? cy[s]/fn[s] : NAN)<<','<<(fn[s]>0 ? cz[s]/fn[s] : NAN);
                fn[s]=ft[s]=fz[s]=tx[s]=cy[s]=cz[s]=0;
            }
            std::cout<<','<<base_z<<','<<support_clear<<std::endl;
            force_frames=samples=contact_frames=bilateral=0;
            next_log=std::floor(t+1e-8)+1;
        }
        if(t>=duration) {
            std::cerr<<"EVENT completed sim="<<now<<" elapsed="<<t<<std::endl;
            gazebo::client::shutdown();return 0;
        }
        // Wall time only yields CPU and detects infrastructure stalls; never drives actions.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif
