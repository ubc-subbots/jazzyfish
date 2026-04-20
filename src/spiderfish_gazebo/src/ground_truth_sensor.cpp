#include "spiderfish_gazebo/ground_truth_sensor.hpp"
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/common/Console.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>
#include <thread>
#include <functional>

namespace spiderfish_gazebo
{

    GroundTruthSensor::GroundTruthSensor() 
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        this->node = rclcpp::Node::make_shared("ground_truth_sensor");
    }

    GroundTruthSensor::~GroundTruthSensor() 
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (this->spinThread.joinable()) {
            this->spinThread.join();
        }
    }

    void GroundTruthSensor::Configure(const gz::sim::Entity &_entity,
                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                      gz::sim::EntityComponentManager &_ecm,
                                      gz::sim::EventManager &/*_eventMgr*/)
    {
        (void)_ecm;
        this->model_entity = _entity;

        if (_sdf->HasElement("state_topic"))
        {
            this->state_topic = _sdf->Get<std::string>("state_topic");
        }
        else
        {
            gzerr << "state_topic value not specified, exiting.\n";
            exit(1);
        }

        if (_sdf->HasElement("update_rate"))
        {
            this->update_rate = _sdf->Get<int>("update_rate");
        }
        else
        {
            gzmsg << "update_rate value not specified, using default: 1Hz.\n";
            this->update_rate = 1;
        }

        this->state_publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(this->state_topic, 10);

        this->spinThread = std::thread(std::bind(&GroundTruthSensor::SpinNode, this));

        this->prev_time = node->now();

        gzmsg << "Ground Truth sensor successfully started!\n";
    }

    void GroundTruthSensor::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        rclcpp::Time now = node->now();
        if ((now - this->prev_time).seconds() < (1.0 / this->update_rate))
        {
            return;
        }
        this->prev_time = now;

        gz::math::Pose3d pose = gz::sim::worldPose(this->model_entity, _ecm);

        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.pose.pose.position.x = pose.Pos().X();
        msg.pose.pose.position.y = pose.Pos().Y();
        msg.pose.pose.position.z = pose.Pos().Z();
        msg.pose.pose.orientation.x = pose.Rot().X();
        msg.pose.pose.orientation.y = pose.Rot().Y();
        msg.pose.pose.orientation.z = pose.Rot().Z();
        msg.pose.pose.orientation.w = pose.Rot().W();

        msg.pose.covariance[0] = 0.001;
        msg.pose.covariance[7] = 0.001;
        msg.pose.covariance[14] = 0.001;
        msg.pose.covariance[21] = 0.001;
        msg.pose.covariance[28] = 0.001;
        msg.pose.covariance[35] = 0.001;
        
        msg.header.stamp = now;
        msg.header.frame_id = "map";
        
        this->state_publisher->publish(msg);
    }

    void GroundTruthSensor::SpinNode()
    {
        rclcpp::spin(node);
    }

}

GZ_ADD_PLUGIN(
    spiderfish_gazebo::GroundTruthSensor,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(spiderfish_gazebo::GroundTruthSensor, "spiderfish_gazebo::GroundTruthSensor")