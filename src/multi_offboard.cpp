/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


//#include <mavros_msgs/>
#include "multi_offboard.hpp"

void MultiOffboard::state_cb(const mavros_msgs::State::ConstPtr& msg){
    my_current_state = *msg;
    ROS_INFO("state: %d",my_current_state.connected);
}


void MultiOffboard::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
//    curr_altitude = current_vfr_hud.altitude;
//    ROS_INFO("altitude = %d",  current_vfr_hud.altitude);
}

void MultiOffboard::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;
    curr_altitude = current_local_pos.pose.position.z;
//    ROS_INFO("pose.z = %.2f", current_local_pos.pose.position.z);
}

bool MultiOffboard::pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) > 1.0f;
}



void MultiOffboard::Oninit() {
    uav1_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, &MultiOffboard::state_cb, this);
    uav1_vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("uav1/mavros/vfr_hud", 10, &MultiOffboard::vrf_hud_cb, this);
    uav1_gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/raw/fix", 1000);
    uav1_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/global", 1000);
    uav1_g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/setpoint_velocity/cmd_vel", 100);
    uav1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, &MultiOffboard::local_pos_cb, this);


    uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    MultiOffboard m_multi;

    m_multi.Oninit();

    ros::Rate rate(10.0);
    ROS_INFO("my_current_state.connected = %d", m_multi.my_current_state.connected);

    while(ros::ok() && !m_multi.my_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 5;
    pose1.pose.position.y = 5;
    pose1.pose.position.z = 5;

    geometry_msgs::PoseStamped pose2;
    pose1.pose.position.x = -5;
    pose1.pose.position.y = 5;
    pose1.pose.position.z = 5;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int count = 0;


    while(ros::ok()){
        if( m_multi.my_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( m_multi.uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( ! m_multi.my_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( m_multi.uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        m_multi.uav1_local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
