// Read-only Gazebo observer: all joint commands must go through PX4.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>

std::mutex lock;
double sim=-1, pose_stamp=-1, contact_stamp=-1;
ignition::math::Pose3d vehicle, gripper_relative, gripper, cube;
bool have_gripper=false, have_cube=false;
unsigned frames=0, fixed_frames=0, moving_frames=0, bilateral=0;
unsigned other_frames=0, support_frames=0, support_arm_frames=0;
std::set<std::string> seen_pairs;
double jaw_angle=NAN, jaw_speed=NAN, jaw_torque=NAN, jaw_receive_sim=-1;
bool jaw_seen=false;
double normal_force[2]{}, tangent_force[2]{}, vertical_force[2]{};
double contact_torque[3]{}, contact_points[2]{};
void jawStatus(ConstVector2dPtr &m) {
    std::lock_guard<std::mutex> guard(lock);
    jaw_angle=m->x();jaw_speed=m->y();jaw_receive_sim=sim;jaw_seen=true;
}
void jawEffort(ConstVector3dPtr &m) {
    std::lock_guard<std::mutex> guard(lock);jaw_torque=m->x();
}
void clockUpdate(ConstTimePtr &m) {
    std::lock_guard<std::mutex> guard(lock);
    sim=std::max(sim,m->sec()+m->nsec()*1e-9);
}
void poses(ConstPosesStampedPtr &m) {
    std::lock_guard<std::mutex> guard(lock);
    double stamp=m->time().sec()+m->time().nsec()*1e-9;
    if(stamp<=pose_stamp) return;
    pose_stamp=stamp;
    // Classic link poses here are model-relative, model poses are world-relative.
    for(int i=0;i<m->pose_size();++i) {
        if(m->pose(i).name()=="yhang550") vehicle=gazebo::msgs::ConvertIgn(m->pose(i));
    }
    for(int i=0;i<m->pose_size();++i) {
        const auto &p=m->pose(i);
        if(p.name()=="yhang550::so101::gripper_link") {
            gripper_relative=gazebo::msgs::ConvertIgn(p);have_gripper=true;
        } else if(p.name()=="test_cube") {
            cube=gazebo::msgs::ConvertIgn(p);have_cube=true;
        }
    }
    gripper=vehicle*gripper_relative;
}
void contacts(ConstContactsPtr &m) {
    std::lock_guard<std::mutex> guard(lock);
    double stamp=m->time().sec()+m->time().nsec()*1e-9;
    if(stamp<=contact_stamp) return;
    contact_stamp=stamp;
    bool fixed=false,moving=false,other=false,support=false,support_arm=false;
    for(int i=0;i<m->contact_size();++i) {
        const auto &c=m->contact(i);
        const std::string pair=c.collision1()+" "+c.collision2();
        if((pair.find("test_cube::")!=std::string::npos || pair.find("cube_support::")!=std::string::npos)
           && seen_pairs.insert(pair).second) {
            std::cerr<<"CONTACT sim="<<stamp<<" pair="<<pair<<std::endl;
        }
        support_arm |= pair.find("cube_support::")!=std::string::npos && pair.find("yhang550::")!=std::string::npos;
        if(pair.find("test_cube::")==std::string::npos) continue;
        fixed |= pair.find("gripper_fixed_finger_collision")!=std::string::npos;
        moving |= pair.find("moving_jaw_collision")!=std::string::npos;
        support |= pair.find("cube_support::")!=std::string::npos;
        other |= pair.find("yhang550::")!=std::string::npos && pair.find("gripper_fixed_finger_collision")==std::string::npos && pair.find("moving_jaw_collision")==std::string::npos;
        int side=pair.find("gripper_fixed_finger_collision")!=std::string::npos ? 0 :
                 pair.find("moving_jaw_collision")!=std::string::npos ? 1 : -1;
        if(side<0 || !have_cube) continue;
        const bool first=c.collision1().find("test_cube::")!=std::string::npos;
        for(int j=0;j<c.wrench_size() && j<c.normal_size();++j) {
            const auto &w=first ? c.wrench(j).body_1_wrench() : c.wrench(j).body_2_wrench();
            // Diagnostic approximation: link-local ODE wrench, world normal.
            // The nearest observed cube pose is not exactly contact-synchronized.
            auto f=cube.Rot().RotateVector({w.force().x(),w.force().y(),w.force().z()});
            ignition::math::Vector3d n(c.normal(j).x(),c.normal(j).y(),c.normal(j).z());
            normal_force[side]+=std::abs(f.Dot(n));
            tangent_force[side]+=(f-n*f.Dot(n)).Length();
            vertical_force[side]+=f.Z();
            auto torque=cube.Rot().RotateVector({w.torque().x(),w.torque().y(),w.torque().z()});
            for(int axis=0;axis<3;++axis) contact_torque[axis]+=torque[axis];
            contact_points[side]+=1;
        }
    }
    ++frames;fixed_frames+=fixed;moving_frames+=moving;bilateral+=fixed&&moving;
    other_frames+=other;support_frames+=support;support_arm_frames+=support_arm;
}
void printPose(const ignition::math::Pose3d &p) {
    std::cout<<','<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()
             <<','<<p.Rot().W()<<','<<p.Rot().X()<<','<<p.Rot().Y()<<','<<p.Rot().Z();
}
int main(int argc,char **argv) {
    gazebo::client::setup(argc,argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());node->Init("default");
    auto a=node->Subscribe("~/grasp/clock",clockUpdate);
    auto b=node->Subscribe("~/pose/info",poses);
    auto c=node->Subscribe("~/physics/contacts",contacts);
    auto d=node->Subscribe("~/so101/gripper/status",jawStatus);
    auto e=node->Subscribe("~/so101/gripper/status/effort",jawEffort);
    std::cout<<"sim_s,pose_s,gripper_seen,cube_seen,gx,gy,gz,gw,gqx,gqy,gqz,cx,cy,cz,cw,cqx,cqy,cqz,contact_frames,fixed_fraction,moving_fraction,bilateral_fraction,cube_other_fraction,cube_support_fraction,support_arm_fraction,jaw_seen,jaw_angle,jaw_speed,jaw_torque_Nm,jaw_receive_age_s,fixed_normal_N,moving_normal_N,fixed_tangent_N,moving_tangent_N,finger_vertical_N,contact_torque_x_Nm,contact_torque_y_Nm,contact_torque_z_Nm,fixed_contact_points,moving_contact_points\n"<<std::setprecision(10);
    double last=-1;
    auto wall=std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now()-wall<std::chrono::seconds(40)) {
        {
            std::lock_guard<std::mutex> guard(lock);
            if(sim>=0 && sim-last>=.099) {
                last=sim;wall=std::chrono::steady_clock::now();
                const double n=std::max(1u,frames);
                std::cout<<sim<<','<<pose_stamp<<','<<have_gripper<<','<<have_cube;
                printPose(gripper);printPose(cube);
                std::cout<<','<<frames<<','<<fixed_frames/n<<','<<moving_frames/n<<','<<bilateral/n<<','<<other_frames/n<<','<<support_frames/n<<','<<support_arm_frames/n
                         <<','<<jaw_seen<<','<<jaw_angle<<','<<jaw_speed<<','<<jaw_torque<<','<<sim-jaw_receive_sim
                         <<','<<normal_force[0]/n<<','<<normal_force[1]/n<<','<<tangent_force[0]/n<<','<<tangent_force[1]/n
                         <<','<<(vertical_force[0]+vertical_force[1])/n
                         <<','<<contact_torque[0]/n<<','<<contact_torque[1]/n<<','<<contact_torque[2]/n
                         <<','<<contact_points[0]/n<<','<<contact_points[1]/n<<std::endl;
                frames=fixed_frames=moving_frames=bilateral=0;
                other_frames=support_frames=support_arm_frames=0;
                for(int i=0;i<2;++i) normal_force[i]=tangent_force[i]=vertical_force[i]=0;
                for(int i=0;i<3;++i) contact_torque[i]=0;
                contact_points[0]=contact_points[1]=0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cerr<<"simulation clock stalled\n";
    gazebo::client::shutdown();return 2;
}
