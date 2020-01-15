#include "ros/ros.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include<ignition/math.hh>
#include <math.h>
#include<iostream>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PointStamped.h"


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

        if(n.getParam("balloon_trajectory", trajectory))
            ROS_INFO("Found parameter: setting balloon trajectory to %s", trajectory.c_str());
        else    {
            ROS_WARN("Can't find trajectory type");
        }

        x=0;
        y=0;
        z=0;
      // Store the pointer to the model
      this->model = _parent;
      this->world = this->model->GetWorld();
      theta = (float) rand() / RAND_MAX *2* M_PI;

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

      baloonPos_pub=n.advertise<geometry_msgs::PointStamped>("target_pos",1);
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
        this->model->SetWorldPose(traj(simTime, tgtPosRos, theta, trajectory));

        if(ros::ok()){
          tgtPosRos.header.stamp = ros::Time::now();
          baloonPos_pub.publish(tgtPosRos);
        }
    }



    public: ignition::math::Pose3d traj(float t, geometry_msgs::PointStamped& tgtPosRos, double theta, std::string trajectory)
    {


         if( !strcmp(trajectory.c_str(),"STATIC" ) ) {

             x=13*sin(theta);
             y=13*cos(theta);
             z=3;
        }

         if( !strcmp(trajectory.c_str(),"OTTO" ) ) {

             x=30*(cos(0.1*t)/(1+pow((sin(0.1*t)), 2)));
             y=30*((cos(0.1*t)*sin(0.1*t))/(1+pow(sin(0.1*t),2)));
             z=5;
         }

        tgtPosRos.point.x=x;
        tgtPosRos.point.y=y;
        tgtPosRos.point.z=z;
        return ignition::math::Pose3d(x,y,z,0,0,0);
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
    private: double theta;
    private: std::string trajectory;
        float x;
        float y;
        float z;

     ros::Publisher baloonPos_pub;
     ros::NodeHandle n;

    private: geometry_msgs::PointStamped tgtPosRos;

    //= n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
