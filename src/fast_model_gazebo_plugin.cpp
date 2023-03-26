#include "fast_model/fast_model_gazebo_plugin.h"
#include <chrono>
#include <thread>
#include <string>

using namespace std;
using namespace gazebo;

FastModelGazebo::FastModelGazebo() {

}

FastModelGazebo::~FastModelGazebo() {

}

void FastModelGazebo::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) {
        ROS_INFO("FastModelPlugin missing <robotNamespace>, "
            "defaults to \"%s\"", robot_namespace_.c_str());
    } else {
        robot_namespace_ = 
            sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) {
        ROS_WARN("FastModelPlugin (ns = %s) missing <odometryTopic>, "
            "defaults to \"%s\"", 
            robot_namespace_.c_str(), odometry_topic_.c_str());
    } else {
        odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    // odometry_rate_ = 20.0;
    // if (!sdf->HasElement("odometryRate")) {
    //     ROS_WARN("FastModelPlugin (ns = %s) missing <odometryRate>, "
    //         "defaults to %f",
    //         robot_namespace_.c_str(), odometry_rate_);
    // } else {
    //     odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    // } 

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("FastModelPlugin (ns = " << robot_namespace_
            << "). A ROS node for Gazebo has not been initialized, "
            << "unable to load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));
    
    odom_sub_ = rosnode_->subscribe(odometry_topic_, 10, &FastModelGazebo::odomCallback, this);

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&FastModelGazebo::UpdateChild, this));

}

void FastModelGazebo::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    boost::mutex::scoped_lock scoped_lock(lock);
    odom_msg_ = *odom_msg;
}

void FastModelGazebo::UpdateChild() {
    boost::mutex::scoped_lock scoped_lock(lock);
    parent_->SetLinearVel(ignition::math::Vector3(odom_msg_.twist.twist.linear.x,
                                        odom_msg_.twist.twist.linear.y,
                                        odom_msg_.twist.twist.linear.z ));

    parent_->SetAngularVel(ignition::math::Vector3(odom_msg_.twist.twist.angular.x,
                                        odom_msg_.twist.twist.angular.y,
                                        odom_msg_.twist.twist.angular.z ));

    ignition::math::Pose3d pose;
    pose.SetX( odom_msg_.pose.pose.position.x);
    pose.SetY( odom_msg_.pose.pose.position.y);
    pose.SetZ( odom_msg_.pose.pose.position.z);
    pose.Rot().W( odom_msg_.pose.pose.orientation.w);
    pose.Rot().X( odom_msg_.pose.pose.orientation.x);
    pose.Rot().Y( odom_msg_.pose.pose.orientation.y);
    pose.Rot().Z( odom_msg_.pose.pose.orientation.z);
    pose.Rot().Normalize();
    parent_->SetWorldPose(pose);
}

void FastModelGazebo::FiniChild() {

}

GZ_REGISTER_MODEL_PLUGIN(FastModelGazebo)