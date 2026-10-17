// Gazebo sim-time clock used by the full-arm regression runner.
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
