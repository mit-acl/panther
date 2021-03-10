/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <exprtk.hpp>
// #include <gazebo/math/gzmath.hh>

namespace gazebo
{
class ModelPush : public ModelPlugin
{
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model_
    model_ = _parent;

    // std::string world_Name = _parent->GetWorldName();
    world_ = _parent->GetWorld();

    if (_sdf->HasElement("traj_x") == false ||  //////
        _sdf->HasElement("traj_y") == false ||  //////
        _sdf->HasElement("traj_z") == false)
    {
      std::cout << "Missing fields (traj_x, traj_y or traj_z), ABORTING!" << std::endl;
      abort();
    }

    symbol_table_t symbol_table;
    symbol_table.add_variable("t", t_);
    symbol_table.add_constants();
    expression_t expression;
    expression.register_symbol_table(symbol_table);

    parser_t parser;

    std::string string_x = _sdf->Get<std::string>("traj_x");
    std::string string_y = _sdf->Get<std::string>("traj_y");
    std::string string_z = _sdf->Get<std::string>("traj_z");

    parser.compile(string_x, expression);
    traj_compiled_.push_back(expression);
    parser.compile(string_y, expression);
    traj_compiled_.push_back(expression);
    parser.compile(string_z, expression);
    traj_compiled_.push_back(expression);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
  }

public:
  // Called by the world_ update start event
  void OnUpdate()
  {
// http://gazebosim.org/tutorials/?tut=ros_wrapper_versions
// See example at
// http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__camera_8cpp_source.html#navrow2:~:text=00079%20%23%20if%20GAZEBO_MAJOR_VERSION%20%3E%3D%207
#if GAZEBO_MAJOR_VERSION >= 8
    t_ = world_->SimTime().Double();
#else
    t_ = world_->GetSimTime().Double();
#endif
    // https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Pose3.html
    ignition::math::Pose3d pose(traj_compiled_[0].value(), traj_compiled_[1].value(), traj_compiled_[2].value(), 0.0,
                                0.0,
                                0.0);  // = orig_pose;

    model_->SetWorldPose(pose);

    // Apply a small linear velocity to the model_.
    //  model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 2.0 * time));
  }

  // Pointer to the model_
private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  double t_;
  std::vector<expression_t> traj_compiled_;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}  // namespace gazebo
