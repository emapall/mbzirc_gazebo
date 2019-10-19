#include "ros/ros.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include<ignition/math.hh>
#include<iostream>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Point.h"


ignition::math::Pose3d traj(float t, geometry_msgs::Point& tgtPosRos){
    float x=1;
    float y=0.3*t;
    float z=3;

    tgtPosRos.x=x;
    tgtPosRos.y=y;
    tgtPosRos.z=z;
    return ignition::math::Pose3d(x,y,z,0,0,0);
}

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world = this->model->GetWorld();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      baloonPos_pub=n.advertise<geometry_msgs::Point>("target_pos",1);
      ROS_INFO("HELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        if(!initFlag){
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        initFlag=true;
        }
        //std::cout<<world->SimTime().Float()<<std::endl;
        simTime=world->SimTime().Float();
        this->model->SetWorldPose(traj(simTime, tgtPosRos));

        if(ros::ok()){
          baloonPos_pub.publish(tgtPosRos);
        }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //pointer to the world
    private: physics::WorldPtr world;
    private: common::Time currentTime;
    
    private: bool initFlag=false;//not quite shure where beginning's stuff should be 
    private: float simTime;

     ros::Publisher baloonPos_pub;
     ros::NodeHandle n;

    private: geometry_msgs::Point tgtPosRos; 
    
    //= n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}