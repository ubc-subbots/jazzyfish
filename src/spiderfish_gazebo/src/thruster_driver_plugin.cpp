#include "spiderfish_gazebo/thruster_driver_plugin.hpp"
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>
#include <thread>
#include <functional>

using namespace std::placeholders;

namespace spiderfish_gazebo
{

    ThrusterDriver::ThrusterDriver() 
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        this->node = rclcpp::Node::make_shared("thruster_driver");
    }

    ThrusterDriver::~ThrusterDriver() 
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (this->spinThread.joinable()) {
            this->spinThread.join();
        }
    }

    void ThrusterDriver::Configure(const gz::sim::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::EventManager &/*_eventMgr*/)
    {
        gz::sim::Model model(_entity);
        if (!model.Valid(_ecm)) {
            gzerr << "ThrusterDriver plugin must be attached to a model entity.\n";
            return;
        }

        if (_sdf->HasElement("thruster_count"))
        {
            this->thruster_count = _sdf->Get<unsigned int>("thruster_count");
        }
        else
        {
            gzerr << "thruster_count value not specified, exiting.\n";
            exit(1);
        }

        std::shared_ptr<const sdf::Element> ros_namespace;
        if (_sdf->HasElement("ros")) {
            ros_namespace = _sdf->Clone()->GetElement("ros");
        }
        this->GetRosNamespace(ros_namespace);
        
        this->thrust_values = std::vector<double>(this->thruster_count, 0);
        this->force_cmd = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                            this->topic_name, 
                            10, 
                            std::bind(&ThrusterDriver::GetForceCmd, this, _1));

        RCLCPP_INFO(node->get_logger(), "Listening on %s \n", this->topic_name.c_str());

        for (unsigned int i = 1; i <= thruster_count; i++)
        {
            std::string nested_model_name = "thruster" + std::to_string(i);
            gz::sim::Entity nested_model_entity = model.ModelByName(_ecm, nested_model_name);

            if (nested_model_entity != gz::sim::kNullEntity) {
                gz::sim::Model nested_model(nested_model_entity);
                gz::sim::Entity linkEntity = nested_model.LinkByName(_ecm, "thruster");
                
                if (linkEntity != gz::sim::kNullEntity) {
                    this->thruster.push_back(gz::sim::Link(linkEntity));
                    this->thruster.back().EnableVelocityChecks(_ecm, true); 
                } else {
                    gzerr << "Thruster link not found in " << nested_model_name << "\n";
                }
            } else {
                gzerr << "Nested model not found: " << nested_model_name << "\n";
            }
        }
    
        this->spinThread = std::thread(std::bind(&ThrusterDriver::SpinNode, this));
    }

    void ThrusterDriver::GetRosNamespace(std::shared_ptr<const sdf::Element> ros_sdf)
    {
        std::string _namespace;
        std::string topic;
    
        if (ros_sdf && ros_sdf->HasElement("namespace"))
        {
            _namespace = ros_sdf->Get<std::string>("namespace");
        }
        else
        {
            _namespace = "spiderfish/spiderfish_gazebo";
        }

        if (ros_sdf && ros_sdf->HasElement("remapping"))
        {
            topic = ros_sdf->Get<std::string>("remapping");
        }
        else
        {
            topic = "thruster_values";
        }

        this->topic_name = _namespace + "/" + topic;
    }

    void ThrusterDriver::GetForceCmd(const std_msgs::msg::Float64MultiArray::SharedPtr joint_cmd)
    {
        if (joint_cmd->data.size() != this->thruster_count)
        {
            RCLCPP_WARN(node->get_logger(), "message size does not match thruster count, ignoring command.\n");
            return;
        }

        for (unsigned int i = 0; i < this->thruster_count; i++)
        {
            this->thrust_values[i] = joint_cmd->data[i];
        }
    }

    void ThrusterDriver::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused) return;

        for (unsigned int i = 0; i < this->thruster.size(); i++)
        {
            gz::math::Vector3d local_force(0, 0, this->thrust_values[i]);
            
            std::optional<gz::math::Pose3d> pose = this->thruster[i].WorldPose(_ecm);
            if (pose.has_value()) {
                gz::math::Vector3d world_force = pose.value().Rot() * local_force;
                this->thruster[i].AddWorldForce(_ecm, world_force);
            }
        }
    }

    void ThrusterDriver::SpinNode()
    {
        rclcpp::spin(node);
    }

}

GZ_ADD_PLUGIN(
    spiderfish_gazebo::ThrusterDriver,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(spiderfish_gazebo::ThrusterDriver, "spiderfish_gazebo::ThrusterDriver")