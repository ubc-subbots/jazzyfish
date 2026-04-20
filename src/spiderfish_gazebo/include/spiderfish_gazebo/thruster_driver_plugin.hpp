#ifndef SPIDERFISH_GAZEBO__THRUSTER_DRIVER_PLUGIN
#define SPIDERFISH_GAZEBO__THRUSTER_DRIVER_PLUGIN

#include <vector>
#include <string>
#include <thread>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <sdf/sdf.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace spiderfish_gazebo
{

    using std::placeholders::_1;

    class ThrusterDriver : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
    {

    public:

        ThrusterDriver(void);

        ~ThrusterDriver(void) override;

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo &_info, 
                       gz::sim::EntityComponentManager &_ecm) override;

    private:

        void GetRosNamespace(std::shared_ptr<const sdf::Element> ros_sdf);

        void GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd);

        void SpinNode(void);

        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_cmd;

        std::vector<gz::sim::Link> thruster;
        std::vector<double> thrust_values;
        std::thread spinThread;
        std::string topic_name;

        unsigned int thruster_count;
        
    };

}
#endif // SPIDERFISH_GAZEBO__THRUSTER_DRIVER_PLUGIN