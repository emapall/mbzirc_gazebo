#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include "std_msgs/String.h"

using namespace ros;

namespace gazebo
{
  class RosModelPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RosModelPlugin::OnUpdate, this));

      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      ROS_INFO("Hello World!");
      chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      std_msgs::String msg;

      msg.data = "henlos";
      chatter_pub.publish(msg);
      ros::spinOnce();

      ROS_INFO("fine ciclo");


    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RosModelPlugin)
  
}