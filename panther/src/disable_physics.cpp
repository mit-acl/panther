#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

public:
  physics::WorldPtr _world;
  gazebo::event::ConnectionPtr _update_connection;
  void Load(physics::WorldPtr ptr, sdf::ElementPtr _sdf)
  {
    _world = ptr;
    _update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&WorldPluginTutorial::OnUpdate, this));
  }

  void OnUpdate()
  {
// http://gazebosim.org/tutorials/?tut=ros_wrapper_versions
// See example at
// http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__camera_8cpp_source.html#navrow2:~:text=00079%20%23%20if%20GAZEBO_MAJOR_VERSION%20%3E%3D%207
#if GAZEBO_MAJOR_VERSION >= 8
    if (_world->PhysicsEnabled())
    {
      printf("**DISABLING PHYSICS**\n");
      _world->SetPhysicsEnabled(false);
    }
#else
    if (_world->GetEnablePhysicsEngine())
    {
      printf("**DISABLING PHYSICS**\n");
      _world->EnablePhysicsEngine(false);
    }
#endif
  }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}  // namespace gazebo
