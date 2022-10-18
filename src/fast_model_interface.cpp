#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/VFR_HUD.h>
#include <std_msgs/Float64.h>
#include <tracker_msgs/TrackerCmd.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
// #include <sensor_msgs/BatteryState.h>
// #include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <fast_model/fast_model.h>

enum px4_state{
    WAITE_FOR_ARM,
    WAITE_FOR_OFFBOARD,
    OK
};

class FastModelInterface : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void all_in_one_timer_CB(const ros::TimerEvent&);
        void cmd_CB(const tracker_msgs::TrackerCmd::ConstPtr &msg);

        ros::Publisher mav_pos_pub_;
        ros::Publisher mav_vel_pub_;
        ros::Publisher state_pub_;
        ros::Publisher vfr_hub_pub_;
        ros::Publisher fake_ctrl_cmd_pub_;

        ros::Subscriber ctrl_cmd_sub_;

        fast_model::FastModel model_;

        ros::Timer AllinOne_timer_;
};

void FastModelInterface::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    mav_pos_pub_ = priv_nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10);
    mav_vel_pub_ = priv_nh.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10);
    state_pub_ = priv_nh.advertise<mavros_msgs::State>("mavros/state", 10);
    vfr_hub_pub_ = priv_nh.advertise<mavros_msgs::VFR_HUD>("mavros/vfr_hud", 10);
    fake_ctrl_cmd_pub_ = priv_nh.advertise<geometry_msgs::Twist>("ctrl_cmd", 10);

    ctrl_cmd_sub_ = priv_nh.subscribe("plan_cmd", 10, &FastModelInterface::cmd_CB, this);

    AllinOne_timer_ = priv_nh.createTimer(ros::Duration(0.01), &FastModelInterface::all_in_one_timer_CB, this);

    double init_px, init_py, init_pz, init_yaw, init_V;
    priv_nh.param<double>("init_position/x", init_px, 0.0);
    priv_nh.param<double>("init_position/y", init_py, 0.0);
    priv_nh.param<double>("init_position/z", init_pz, 500.0);
    priv_nh.param<double>("init_position/yaw", init_yaw, 0.0);
    priv_nh.param<double>("init_flight_speed", init_V, 20.0);

    fast_model::FastModel::State init_state;
    init_state.pos = {init_px, init_py, init_pz};
    init_state.gs_dir = {std::cos(init_yaw), std::sin(init_yaw)};
    init_state.roll = 0.0;
    init_state.V = init_V;
    init_state.vz = 0.0;
    fast_model::FastModel::Control init_u;
    init_u.V = init_V;
    init_u.alt = init_pz;
    init_u.roll = 0.0;
    model_.setInitState(init_state);
    model_.setCtrl(init_u);
}

void FastModelInterface::cmd_CB(const tracker_msgs::TrackerCmd::ConstPtr &msg) {
    fast_model::FastModel::Control u;
    u.alt = msg->altitude;
    u.roll = -msg->lateral_acc/CONSTANTS_ONE_G;
    u.V = msg->ground_speed;
    geometry_msgs::Twist fc_msg;
    fc_msg.linear.x = u.roll;
    fc_msg.linear.y = 0.0;
    fc_msg.linear.z = 0.5;
    fake_ctrl_cmd_pub_.publish(fc_msg);
    model_.setCtrl(u);
}

void FastModelInterface::all_in_one_timer_CB(const ros::TimerEvent&) {
    static ros::Time last_time = ros::Time::now();
    double dt = (ros::Time::now() - last_time).toSec();
    dt = std::max(std::min(dt, 0.1), 0.001);
    last_time = ros::Time::now();
    ROS_INFO_THROTTLE(10.0, "dt: %.4f", dt);
    model_.propagate(dt);
    nav_msgs::Odometry odom = model_.getOdomfromState();

    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header = odom.header;
    pos_msg.pose = odom.pose.pose;
    mav_pos_pub_.publish(pos_msg);

    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = odom.header;
    vel_msg.twist = odom.twist.twist;
    mav_vel_pub_.publish(vel_msg);

    mavros_msgs::State state_msg;
    state_msg.header = odom.header;
    state_msg.connected = true;
    state_msg.armed = true;
    state_msg.mode = mavros_msgs::State::MODE_PX4_OFFBOARD;
    state_pub_.publish(state_msg);

    mavros_msgs::VFR_HUD vfr_hub_msg;
    vfr_hub_msg.airspeed = Eigen::Vector3d(odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z).norm();
    vfr_hub_pub_.publish(vfr_hub_msg);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FastModelInterface, nodelet::Nodelet);