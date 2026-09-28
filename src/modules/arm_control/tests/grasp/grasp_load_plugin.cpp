// Isolated-gripper test actuation. Never attaches the payload to the gripper.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <mutex>
#include <algorithm>

namespace gazebo {
class GraspLoadPlugin : public WorldPlugin {
    physics::WorldPtr world;
    transport::NodePtr node;
    transport::SubscriberPtr force_sub,lift_sub;
    event::ConnectionPtr update;
    std::mutex mutex;
    ignition::math::Vector3d force;
    double lift=0;
    common::Time force_time;
    void Force(ConstVector3dPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex);
        force.Set(msg->x(),msg->y(),msg->z());
        if(!force.IsFinite() || force.Length()>0.05) force.Set(0,0,0);
        force_time=world->SimTime();
    }
    void Lift(ConstVector3dPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if(std::isfinite(msg->x())) lift=std::max(0.,std::min(.1,msg->x()));
    }
    void Update() {
        std::lock_guard<std::mutex> lock(mutex);
        auto cube=world->ModelByName("test_cube");
        if(cube && (world->SimTime()-force_time).Double()<0.3)
            cube->GetLink("link")->AddForce(force);
        auto fixture=world->ModelByName("gripper_fixture");
        if(!fixture) return;
        auto joint=fixture->GetJoint("fixture_anchor");
        if(joint && joint->HasType(physics::Base::SLIDER_JOINT)) {
            // Gravity compensation for 87 g base + 12 g jaw, not the payload.
            const double command=.099*9.8066+500*(lift-joint->Position(0))-15*joint->GetVelocity(0);
            joint->SetForce(0,std::max(-5.,std::min(5.,command)));
        }
    }
public:
    void Load(physics::WorldPtr w,sdf::ElementPtr) override {
        world=w; node.reset(new transport::Node()); node->Init(w->Name());
        force_sub=node->Subscribe("~/grasp/disturbance",&GraspLoadPlugin::Force,this);
        lift_sub=node->Subscribe("~/grasp/lift",&GraspLoadPlugin::Lift,this);
        update=event::Events::ConnectWorldUpdateBegin(std::bind(&GraspLoadPlugin::Update,this));
    }
};
GZ_REGISTER_WORLD_PLUGIN(GraspLoadPlugin)
}
