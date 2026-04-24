#include "spiderfish_gazebo/hydrodynamics_plugin.hpp"

#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <iostream>
#include <cmath>

namespace spiderfish_gazebo
{

    HydrodynamicsPlugin::HydrodynamicsPlugin() {}

    void HydrodynamicsPlugin::Configure(const gz::sim::Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        gz::sim::EntityComponentManager &_ecm,
                                        gz::sim::EventManager &/*_eventMgr*/)
    {
        this->model = gz::sim::Model(_entity);
        if (!this->model.Valid(_ecm))
        {
            gzerr << "Hydrodynamics plugin must be attached to a model entity.\n";
            return;
        }

        GetWorldParameters(_sdf);
        GetModelParameters(_sdf);

        auto worldEntity = gz::sim::worldEntity(_ecm);
        auto gravityComp = _ecm.Component<gz::sim::components::Gravity>(worldEntity);
        if (gravityComp)
        {
            this->gravity = gravityComp->Data();
        }
        else
        {
            this->gravity = gz::math::Vector3d(0.0, 0.0, -9.81);
        }

        bool status = true;
        std::string base_link_name = GetSdfElement<std::string>(&status, _sdf, "base_link", "");
        
        gz::sim::Entity link_entity = gz::sim::kNullEntity;
        
        auto all = _ecm.EntitiesByComponents(gz::sim::components::Link(), gz::sim::components::Name(base_link_name));
        
        for (const auto &ent : all)
        {
             gz::sim::Entity parent = _ecm.ParentEntity(ent);
             while (parent != gz::sim::kNullEntity)
             {
                 if (parent == this->model.Entity())
                 {
                     link_entity = ent;
                     break;
                 }
                 parent = _ecm.ParentEntity(parent);
             }
             if (link_entity != gz::sim::kNullEntity) break;
        }

        this->link = gz::sim::Link(link_entity);

        if (!this->link.Valid(_ecm))
        {
            gzerr << "Base link '" << base_link_name << "' not found in model! Check that the name matches your <include> name or <link> name exactly.\n";
            return;
        }

        auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(this->link.Entity());
        if (inertialComp)
        {
            this->com_offset = inertialComp->Data().Pose().Pos();
            this->rel_CoB = this->rel_CoB - this->com_offset;
        }
        else
        {
            this->com_offset = gz::math::Vector3d::Zero;
        }

        this->mass = 0;
        auto all_model_links = _ecm.EntitiesByComponents(gz::sim::components::Link());
        for (const auto &ent : all_model_links)
        {
            gz::sim::Entity parent = _ecm.ParentEntity(ent);
            bool belongs_to_model = false;
            while (parent != gz::sim::kNullEntity)
            {
                if (parent == this->model.Entity())
                {
                    belongs_to_model = true;
                    break;
                }
                parent = _ecm.ParentEntity(parent);
            }
            
            if (belongs_to_model)
            {
                auto l_inertial = _ecm.Component<gz::sim::components::Inertial>(ent);
                if (l_inertial)
                {
                    this->mass += l_inertial->Data().MassMatrix().Mass();
                }
            }
        }

        this->DLinForwardSpeed.setZero();

        this->link.EnableVelocityChecks(_ecm, true);
        this->link.EnableAccelerationChecks(_ecm, true);
    }

    bool HydrodynamicsPlugin::GetWorldParameters(const std::shared_ptr<const sdf::Element>& world_sdf)
    {
        bool status = true;
        this->fluid_density = GetSdfElement<double>(&status, world_sdf, "fluid_density", 1028.00);
        return status;
    }

    bool HydrodynamicsPlugin::GetModelParameters(const std::shared_ptr<const sdf::Element>& model_sdf)
    {
        bool status = true;
        std::shared_ptr<const sdf::Element> hydro_model;
        if (model_sdf->HasElement("hydrodynamic_model"))
        {
            hydro_model = std::const_pointer_cast<sdf::Element>(model_sdf)->GetElement("hydrodynamic_model");
        }
        else
        {
            gzerr << "hydrodynamic_model not specified.\n";
            return false;
        }

        this->scalingAddedMass = GetSdfElement<double>(&status, hydro_model, "scalingAddedMass", 1.00);
        this->offsetAddedMass = GetSdfElement<double>(&status, hydro_model, "offsetAddedMass", 0.00);

        this->added_mass = GetSdfMatrix(&status, hydro_model, "added_mass");

        this->scalingDamping = GetSdfElement<double>(&status, hydro_model, "scalingDamping", 1.00);
        this->offsetLinearDamping = GetSdfElement<double>(&status, hydro_model, "offetLineaDamping", 0.00);
        this->offsetLinForwardSpeedDamping = GetSdfElement<double>(&status, hydro_model, "offsetLinForwardSpeedDamping", 0.00);

        this->offsetNonLinDamping = GetSdfElement<double>(&status, hydro_model, "offsetNonLinDamping", 0.00);

        this->volume = GetSdfElement<double>(&status, model_sdf, "volume", 0.0);

        Eigen::Vector6d CoB_eig = GetSdfVector(&status, model_sdf, "center_of_buoyancy");
        this->rel_CoB = gz::math::Vector3d(CoB_eig[0], CoB_eig[1], CoB_eig[2]);

        Eigen::Vector6d linear_damping_vec = GetSdfVector(&status, hydro_model, "linear_damping");
        this->linear_damping = ToDiagonalMatrix(linear_damping_vec);

        this->quadratic_damping = GetSdfVector(&status, hydro_model, "quadratic_damping");
        this->non_linear_damping = ToDiagonalMatrix(this->quadratic_damping);

        this->scalingBuoyancy = GetSdfElement<double>(&status, hydro_model, "scalingBuoyancy", 1.00);
        this->is_neutrally_buoyant = GetSdfElement<bool>(&status, model_sdf, "neutrally_buoyant", false);

        return status;
    }

    void HydrodynamicsPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                        gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused || !this->link.Valid(_ecm)) return;

        Eigen::Vector6d velocity = this->GetVelocityVector(_ecm);
        Eigen::Vector6d acceleration = this->GetAccelerationVector(_ecm);
        
        if (std::isnan(velocity.norm()) || std::isnan(acceleration.norm()))
        {
            return;
        }

        Eigen::Vector6d velRel = ToNED(velocity);

        this->ComputeAddedCoriolisMatrix(velRel, this->added_mass, this->coriolis_matrix);
        this->ComputeDampingMatrix(velRel, this->damping_matrix);

        // Eigen::Matrix6d Ma = this->GetAddedMass();

        Eigen::Vector6d damping = -this->damping_matrix * velRel;

        Eigen::Vector6d added = Eigen::Vector6d::Zero(); 
        
        Eigen::Vector6d cor = -this->coriolis_matrix * velRel;

        Eigen::Vector6d tau = damping + added + cor;

        if (!std::isnan(tau.norm()))
        {
            this->SetWrenchVector(FromNED(tau), _ecm);
        }

        this->ApplyBuoyancyForce(_ecm);
    }

    Eigen::Vector6d HydrodynamicsPlugin::GetVelocityVector(gz::sim::EntityComponentManager &_ecm)
    {
        auto lin_vel_comp = _ecm.Component<gz::sim::components::LinearVelocity>(this->link.Entity());
        auto ang_vel_comp = _ecm.Component<gz::sim::components::AngularVelocity>(this->link.Entity());

        gz::math::Pose3d world_pose = this->link.WorldPose(_ecm).value_or(gz::math::Pose3d());

        gz::math::Vector3d linear_vel = world_pose.Rot().RotateVectorReverse(
            lin_vel_comp ? lin_vel_comp->Data() : gz::math::Vector3d::Zero);
        gz::math::Vector3d angular_vel = world_pose.Rot().RotateVectorReverse(
            ang_vel_comp ? ang_vel_comp->Data() : gz::math::Vector3d::Zero);

        Eigen::Vector6d velocity;
        velocity << linear_vel.X(), linear_vel.Y(), linear_vel.Z(),
                    angular_vel.X(), angular_vel.Y(), angular_vel.Z();

        return velocity;
    }

    Eigen::Vector6d HydrodynamicsPlugin::GetAccelerationVector(gz::sim::EntityComponentManager &_ecm)
    {
        auto lin_acc_comp = _ecm.Component<gz::sim::components::LinearAcceleration>(this->link.Entity());
        auto ang_acc_comp = _ecm.Component<gz::sim::components::AngularAcceleration>(this->link.Entity());

        gz::math::Pose3d world_pose = this->link.WorldPose(_ecm).value_or(gz::math::Pose3d());

        gz::math::Vector3d linear_acc = world_pose.Rot().RotateVectorReverse(
            lin_acc_comp ? lin_acc_comp->Data() : gz::math::Vector3d::Zero);
        gz::math::Vector3d angular_acc = world_pose.Rot().RotateVectorReverse(
            ang_acc_comp ? ang_acc_comp->Data() : gz::math::Vector3d::Zero);

        Eigen::Vector6d acceleration;
        acceleration << linear_acc.X(), linear_acc.Y(), linear_acc.Z(),
                        angular_acc.X(), angular_acc.Y(), angular_acc.Z();

        return acceleration;
    }

    void HydrodynamicsPlugin::SetWrenchVector(const Eigen::Vector6d& wrench, gz::sim::EntityComponentManager &_ecm)
    {
        gz::math::Vector3d force(wrench(0), wrench(1), wrench(2));
        gz::math::Vector3d torque(wrench(3), wrench(4), wrench(5));

        gz::math::Pose3d world_pose = this->link.WorldPose(_ecm).value_or(gz::math::Pose3d());

        gz::math::Vector3d world_force = world_pose.Rot().RotateVector(force);
        gz::math::Vector3d world_torque = world_pose.Rot().RotateVector(torque);

        gz::math::Vector3d com_world_offset = world_pose.Rot().RotateVector(this->com_offset);
        world_torque += com_world_offset.Cross(world_force);

        this->link.AddWorldWrench(_ecm, world_force, world_torque);
    }

    void HydrodynamicsPlugin::ComputeAddedCoriolisMatrix(const Eigen::Vector6d& _vel, const Eigen::Matrix6d& /*_Ma*/, Eigen::Matrix6d &_Ca) const
    {
        Eigen::Vector6d ab = this->GetAddedMass() * _vel;
        Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
        _Ca << Eigen::Matrix3d::Zero(), Sa, 
               Sa, -1 * CrossProductOperator(ab.tail<3>());
    }

    void HydrodynamicsPlugin::ComputeDampingMatrix(const Eigen::Vector6d& _vel, Eigen::Matrix6d &_D) const
    {
        _D.setZero();

        _D = -1 * (this->linear_damping + this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
                    _vel[0] * (this->DLinForwardSpeed +
                    this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

        for (int i = 0; i < 6; i++)
        {
            _D(i, i) += -1 * (this->non_linear_damping(i, i) + this->offsetNonLinDamping) * std::fabs(_vel[i]);
        }
        _D *= this->scalingDamping;
    }

    Eigen::Matrix6d HydrodynamicsPlugin::GetAddedMass() const
    {
        Eigen::Matrix6d M_d = this->added_mass + this->offsetAddedMass * Eigen::Matrix6d::Identity();

        return this->scalingAddedMass * M_d;
    }

    void HydrodynamicsPlugin::ApplyBuoyancyForce(gz::sim::EntityComponentManager &_ecm)
    {
        double vol = this->volume;
        double rho = this->fluid_density;
        gz::math::Vector3d buoyancy_force;

        if (!this->is_neutrally_buoyant)
        {
            buoyancy_force = -rho * vol * this->gravity * this->scalingBuoyancy;
        }
        else
        {
            buoyancy_force = -this->mass * this->gravity;
        }

        gz::math::Pose3d world_pose = this->link.WorldPose(_ecm).value_or(gz::math::Pose3d());

        gz::math::Vector3d cob_offset_from_origin = this->com_offset + this->rel_CoB;
        gz::math::Vector3d offset_world = world_pose.Rot().RotateVector(cob_offset_from_origin);
        gz::math::Vector3d torque = offset_world.Cross(buoyancy_force);

        this->link.AddWorldWrench(_ecm, buoyancy_force, torque);
    }

}

GZ_ADD_PLUGIN(spiderfish_gazebo::HydrodynamicsPlugin,
              gz::sim::System,
              spiderfish_gazebo::HydrodynamicsPlugin::ISystemConfigure,
              spiderfish_gazebo::HydrodynamicsPlugin::ISystemPreUpdate)