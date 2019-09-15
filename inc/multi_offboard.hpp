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

using namespace std;
class MultiOffboard {
public:
    ~MultiOffboard() {};

    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos);
    void Oninit();
    void targte_local_pos();

    mavros_msgs::State my_current_state;
    mavros_msgs:: VFR_HUD current_vfr_hud;
    geometry_msgs::PoseStamped current_local_pos;
    float curr_altitude;

    ros::NodeHandle nh;
    ros::ServiceClient uav1_set_mode_client;
    ros::ServiceClient uav1_arming_client;
    ros::Publisher uav1_local_pos_pub;
    ros::Subscriber uav1_state_sub;
    ros::Subscriber uav1_vfr_hud_sub;
    ros::Publisher uav1_gps_global_pos_pub;
    ros::Publisher uav1_global_pos_pub;
    ros::Publisher uav1_g_speed_control_pub;
    ros::Subscriber uav1_local_position_sub;

};

#endif //OFFBOARD_MULTI_OFFBOARD_HPP
