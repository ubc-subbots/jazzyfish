#ifndef SPIDERFISH_GAZEBO__GROUND_TRUTH_SENSOR
#define SPIDERFISH_GAZEBO__GROUND_TRUTH_SENSOR

#include <vector>
#include <thread>
#include <string>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <sdf/sdf.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace spiderfish_gazebo
{

    using std::placeholders::_1;

    class GroundTruthSensor : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPostUpdate
    {

    public:

        // Constructor
        GroundTruthSensor(void);

        // Destructor
        ~GroundTruthSensor(void) override;

        /** Collects all neccessary parameters and initializes the ROS 2 node.
         * * @param _entity The entity this plugin is attached to.
         * @param _sdf    A pointer to the plugin's SDF element.
         * @param _ecm    The Entity-Component Manager.
         * @param _eventMgr The Event Manager.
         */
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        /** Publishes the ground truth pose of the AUV
         * */
        void PostUpdate(const gz::sim::UpdateInfo &_info, 
                        const gz::sim::EntityComponentManager &_ecm) override;

    private:

        /** Spins ROS2 node on a dedicated thread to remain non-blocking
         * */
        void SpinNode(void);

        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr state_publisher;
        
        gz::sim::Entity model_entity{gz::sim::kNullEntity};
        
        std::string state_topic;
        int count;

        std::thread spinThread;
        std::string topic_name;
        int update_rate;
        rclcpp::Time prev_time;
    };

}
#endif // SPIDERFISH_GAZEBO__GROUND_TRUTH_SENSOR