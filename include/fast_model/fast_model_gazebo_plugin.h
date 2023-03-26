#pragma once

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>
#include <cmath>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

class FastModelGazebo : public ModelPlugin {
public:
    FastModelGazebo();
    ~FastModelGazebo();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

protected: 
    virtual void UpdateChild();
    virtual void FiniChild();
private:
    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    ros::Subscriber odom_sub_;

    boost::mutex lock;

    nav_msgs::Odometry odom_msg_;

    std::string robot_namespace_;
    std::string odometry_topic_;
    // std::string odometry_frame_;
    // std::string robot_base_frame_;
    double odometry_rate_;
};

}