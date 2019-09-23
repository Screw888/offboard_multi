//
// Created by zhouhua on 19-9-15.
//

#ifndef OFFBOARD_MULTI_OFFBOARD_HPP
#define OFFBOARD_MULTI_OFFBOARD_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include "math.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <Eigen/Core>
#include <vector>       // std::vector
#include <algorithm>    // std::reverse

using namespace std;
using namespace Eigen;
class MultiOffboard {
public:
    enum  {
        TAKEOFF,
        WAYPOINT,
        LAND
    };

    ~MultiOffboard() {};
    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);

    void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav3_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav4_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos);
    void Oninit();
    void targte_local_pos();
    void add_way_points();

    mavros_msgs::State uav1_current_state;
    mavros_msgs::State uav2_current_state;
    mavros_msgs::State uav3_current_state;
    mavros_msgs::State uav4_current_state;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    geometry_msgs::PoseStamped uav1_current_local_pos;
    geometry_msgs::PoseStamped uav2_current_local_pos;
    geometry_msgs::PoseStamped uav3_current_local_pos;
    geometry_msgs::PoseStamped uav4_current_local_pos;

    geometry_msgs::PoseStamped uav1_target_pose;
    geometry_msgs::PoseStamped uav2_target_pose;
    geometry_msgs::PoseStamped uav3_target_pose;
    geometry_msgs::PoseStamped uav4_target_pose;

    ros::NodeHandle nh;
    ros::Subscriber uav1_local_position_sub;
    ros::ServiceClient uav1_set_mode_client;
    ros::ServiceClient uav1_arming_client;
    ros::Publisher uav1_local_pos_pub;
    ros::Subscriber uav1_state_sub;
    ros::Subscriber uav2_state_sub;
    ros::Subscriber uav3_state_sub;
    ros::Subscriber uav4_state_sub;
    ros::Subscriber uav1_vfr_hud_sub;
    ros::Publisher uav1_gps_global_pos_pub;
    ros::Publisher uav1_global_pos_pub;
    ros::Publisher uav1_g_speed_control_pub;

    ros::Subscriber uav2_local_position_sub;
    ros::ServiceClient uav2_set_mode_client;
    ros::ServiceClient uav2_arming_client;
    ros::Publisher uav2_local_pos_pub;

    ros::Subscriber uav3_local_position_sub;
    ros::ServiceClient uav3_set_mode_client;
    ros::ServiceClient uav3_arming_client;
    ros::Publisher uav3_local_pos_pub;

    ros::Subscriber uav4_local_position_sub;
    ros::ServiceClient uav4_set_mode_client;
    ros::ServiceClient uav4_arming_client;
    ros::Publisher uav4_local_pos_pub;

    bool is_offboard = false;
    ros::Time last_request_;
private:

    float curr_altitude;
    int state_ = TAKEOFF;
    geometry_msgs::PoseStamped target_pos_;
    vector<geometry_msgs::PoseStamped> way_points;
};

#endif //OFFBOARD_MULTI_OFFBOARD_HPP
