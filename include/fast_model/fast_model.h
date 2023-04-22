#ifndef FAST_MODEL_H_
#define FAST_MODEL_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <fast_model/AlphaFilter.hpp>
#include <tf/transform_datatypes.h>
#include <geo/geo.h>

namespace fast_model
{
    class FastModel {
        public:
            typedef struct {
                Eigen::Vector3d pos;
                Eigen::Vector2d gs_dir;// ground speed direction, always in normalized
                double vz;
                double roll;
                double V;
                // precise
                double pitch;
            } State;
            
            typedef struct {
                double roll;
                double V;
                double alt;
            } Control;

            typedef struct {
                double roll; // bank
                double pitch; // attack
                double thrust;
            } PreciseControl;

            FastModel() {
                V_filter_.setParameters(0.01, 9.0);
                vz_filter_.setParameters(0.01, 3.0);
                roll_filter_.setParameters(0.01, 0.08);
                scale_acc_thrust_ = 3.0;
            };
            ~FastModel();

            void setInitState(State s) {
                if (!state_set_) {
                    V_filter_.reset(s.V);
                    vz_filter_.reset(s.vz);
                    roll_filter_.reset(s.roll);
                    state_set_ = true;
                }
                s_ = s;
            };

            void setCtrl(Control u) {
                if (!ctrl_set_) {
                    ctrl_set_ = true;
                }
                u_.alt = u.alt;
                u_.roll = constraint(u.roll, -55.0/180.0*M_PI, 55.0/180.0*M_PI);
                u_.V = u.V;
            }

            void setPreciseCtrl(PreciseControl u) {
                if (!ctrl_set_) {
                    ctrl_set_ = true;
                }
                pu_.pitch = constraint(u.pitch, -10.0/180.0*M_PI, 20.0/180.0*M_PI);
                pu_.roll = constraint(u.roll, -25.0/180.0*M_PI, 25.0/180.0*M_PI);
                pu_.thrust = constraint(u.thrust, 0.0, 1.0);
            }

            void propagate(double dt) {
                s_.V = V_filter_.update(u_.V);
                s_.vz = vz_filter_.update(constraint(alt_gain_*(u_.alt - s_.pos(2)), -max_vz_, max_vz_));
                s_.pos(2) = s_.pos(2) + s_.vz * dt;
                double Vxy = 0.0;
                if (std::abs(s_.vz) < s_.V) {
                    Vxy = sqrt(s_.V*s_.V - s_.vz*s_.vz);
                }
                s_.pos.head<2>() = s_.pos.head<2>() + Vxy*s_.gs_dir*dt;
                double roll = roll_filter_.update(u_.roll);
                s_.roll = roll;
                double lat_acc = -roll*CONSTANTS_ONE_G;
                double omega;
                if (Vxy > 0.1)
                    omega = lat_acc / Vxy;
                else
                    omega = 0.0;
                double theta = atan2(s_.gs_dir(1), s_.gs_dir(0));
                theta = normalize_theta(theta + omega*dt);
                s_.gs_dir = Eigen::Vector2d(std::cos(theta), std::sin(theta));
            };

            // void precise_propagate(double dt) {
            //     acc_filter_.update(pu_.thrust*scale_acc_thrust_);
            // }

            nav_msgs::Odometry getOdomfromState() {
                nav_msgs::Odometry res;
                res.header.stamp = ros::Time::now();
                res.header.frame_id = "world";
                res.pose.pose.position.x = s_.pos(0);
                res.pose.pose.position.y = s_.pos(1);
                res.pose.pose.position.z = s_.pos(2);
                double Vxy = 0.0;
                if (std::abs(s_.vz) < s_.V) {
                    Vxy = sqrt(s_.V*s_.V - s_.vz*s_.vz);
                }
                // TODO attitude
                double pitch = atan2(s_.vz, Vxy);
                double yaw = atan2(s_.gs_dir(1), s_.gs_dir(0));
                auto q = tf::createQuaternionFromRPY(s_.roll, -pitch, yaw);
                res.pose.pose.orientation.w = q.w();
                res.pose.pose.orientation.x = q.x();
                res.pose.pose.orientation.y = q.y();
                res.pose.pose.orientation.z = q.z();
                Eigen::Vector3d vel;
                vel.head<2>() = Vxy * s_.gs_dir.normalized();
                vel(2) = s_.vz;
                res.twist.twist.linear.x = vel(0);
                res.twist.twist.linear.y = vel(1);
                res.twist.twist.linear.z = vel(2);
                res.twist.twist.angular.x = 0.0;
                res.twist.twist.angular.y = 0.0;
                res.twist.twist.angular.z = 0.0;
                return res;
            };

        private:
            inline double normalize_theta(double theta) {
                if (theta >= -M_PI && theta < M_PI)
                    return theta;
                
                double multiplier = std::floor(theta / (2*M_PI));
                theta = theta - multiplier*2*M_PI;
                if (theta >= M_PI)
                    theta -= 2*M_PI;
                if (theta < -M_PI)
                    theta += 2*M_PI;

                return theta;
            }
            inline double constraint(double x, double a, double b) {
                return std::max(std::min(x, b), a);
            }
            State s_;
            Control u_;
            PreciseControl pu_;
            bool ctrl_set_{false};
            bool state_set_{false};
            double alt_gain_{2.0};
            double max_vz_{5.0};
            AlphaFilter<double> V_filter_;
            AlphaFilter<double> vz_filter_;
            AlphaFilter<double> roll_filter_;

            // precise state
            AlphaFilter<double> acc_filter_;
            AlphaFilter<double> pitch_filter_;
            double scale_acc_thrust_;
    };
} // namespace fast_model
#endif