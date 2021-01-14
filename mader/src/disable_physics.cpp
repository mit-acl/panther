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
    if (_world->PhysicsEnabled())
    {
      printf("**DISABLING PHYSICS**\n");
      _world->SetPhysicsEnabled(false);
    }
  }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}  // namespace gazebo