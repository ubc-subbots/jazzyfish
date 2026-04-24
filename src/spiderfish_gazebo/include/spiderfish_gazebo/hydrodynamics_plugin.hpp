#ifndef SPIDERFISH_GAZEBO__HYDRODYNAMICS_PLUGIN
#define SPIDERFISH_GAZEBO__HYDRODYNAMICS_PLUGIN

#include <functional>
#include <vector>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

#include "include/gazebo_utils.hpp"
#include "include/math_utils.hpp"

namespace spiderfish_gazebo 
{

    class HydrodynamicsPlugin : 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {

    public:
        
        HydrodynamicsPlugin();

        ~HydrodynamicsPlugin() override = default;

    protected:

        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) override;

    private:

        bool GetWorldParameters(const std::shared_ptr<const sdf::Element>& world_sdf);

        bool GetModelParameters(const std::shared_ptr<const sdf::Element>& model_sdf);

        Eigen::Vector6d GetVelocityVector(gz::sim::EntityComponentManager &_ecm);

        Eigen::Vector6d GetAccelerationVector(gz::sim::EntityComponentManager &_ecm);

        void SetWrenchVector(const Eigen::Vector6d& wrench, gz::sim::EntityComponentManager &_ecm);

        void ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel, const Eigen::Matrix6d& _Ma, Eigen::Matrix6d &_Ca) const;

        void ComputeDampingMatrix(const Eigen::Vector6d& _vel, Eigen::Matrix6d &_D) const;

        Eigen::Matrix6d GetAddedMass() const;

        void ApplyBuoyancyForce(gz::sim::EntityComponentManager &_ecm);

        gz::sim::Link link;
        gz::sim::Model model;

        Eigen::Matrix6d added_mass;
        Eigen::Matrix6d coriolis_matrix;
        Eigen::Matrix6d damping_matrix;

        Eigen::Matrix6d DLinForwardSpeed;

        Eigen::Matrix6d linear_damping;
        Eigen::Matrix6d non_linear_damping;
        Eigen::Vector6d quadratic_damping;

        gz::math::Vector3d rel_CoB;
        gz::math::Vector3d gravity;
        gz::math::Vector3d com_offset;

        double fluid_density;

        double scalingAddedMass; 
        double offsetAddedMass; 
        double scalingDamping;
        double offsetLinearDamping;
        double offsetLinForwardSpeedDamping;
        double offsetNonLinDamping;
        double scalingBuoyancy;

        double mass;
        double volume;
        bool is_neutrally_buoyant;

    };

}

GZ_ADD_PLUGIN(spiderfish_gazebo::HydrodynamicsPlugin,
              gz::sim::System,
              spiderfish_gazebo::HydrodynamicsPlugin::ISystemConfigure,
              spiderfish_gazebo::HydrodynamicsPlugin::ISystemPreUpdate)

#endif