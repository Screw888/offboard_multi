/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


//#include <mavros_msgs/>
#include "multi_offboard.hpp"

void MultiOffboard::uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}

void MultiOffboard::uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

void MultiOffboard::uav3_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav3_current_state = *msg;
}

void MultiOffboard::uav4_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav4_current_state = *msg;
}


void MultiOffboard::vrf_hud_cb(const mavros_msgs::VFR_HUD::ConstPtr& msg){
    current_vfr_hud = *msg;
//    curr_altitude = current_vfr_hud.altitude;
//    ROS_INFO("altitude = %d",  current_vfr_hud.altitude);
}

void MultiOffboard::uav1_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_current_local_pos = *msg;
}

void MultiOffboard::uav2_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_current_local_pos = *msg;
}

void MultiOffboard::uav3_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav3_current_local_pos = *msg;
}

void MultiOffboard::uav4_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav4_current_local_pos = *msg;
}

bool MultiOffboard::pos_reached(geometry_msgs::PoseStamped current_pos, geometry_msgs::PoseStamped target_pos){
    float err_px = current_pos.pose.position.x - target_pos.pose.position.x;
    float err_py = current_pos.pose.position.y - target_pos.pose.position.y;
    float err_pz = current_pos.pose.position.z - target_pos.pose.position.z;

    return sqrt(err_px * err_px + err_py * err_py + err_pz * err_pz) < 0.8f;
}

void MultiOffboard::add_way_points() {
    geometry_msgs::PoseStamped way_point;

    way_point.pose.position.x = 0;
    way_point.pose.position.y = 0;
    way_point.pose.position.z = 15;
    way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = -30;
    way_point.pose.position.z = 15;
    way_points.push_back(way_point);

    way_point.pose.position.x = 30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    way_points.push_back(way_point);

    way_point.pose.position.x = -30;
    way_point.pose.position.y = 40;
    way_point.pose.position.z = 15;
    way_points.push_back(way_point);

    std::reverse(way_points.begin(), way_points.end());
}

void MultiOffboard::targte_local_pos() {
    if (uav2_current_state.mode == "OFFBOARD") {
        switch (state_) {
            // takeoff
            case TAKEOFF:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 3;

                if (pos_reached(uav2_current_local_pos, target_pos_)){
                    state_ = WAYPOINT;
                    add_way_points();
                    ROS_INFO("Finish takeoff");
                }
                break;

            case WAYPOINT:
                if (!way_points.empty()) {
                    target_pos_ = way_points.back();
                    if (pos_reached(uav2_current_local_pos,target_pos_)) {
                        way_points.pop_back();
                        ROS_INFO("Finished one way point = (%.2f, %.2f, %.2f)",
                                target_pos_.pose.position.x, target_pos_.pose.position.y, target_pos_.pose.position.z);
                    }
                } else {
                    state_ = LAND;
                }

                break;

            case LAND:
                target_pos_.pose.position.x = 0;
                target_pos_.pose.position.y = 0;
                target_pos_.pose.position.z = 0;
                is_offboard = true;

                if (pos_reached(uav2_current_local_pos,target_pos_)) {
                    ROS_INFO("reached the land");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";

                    if ((uav1_current_state.mode != "AUTO.LAND" || uav3_current_state.mode != "AUTO.LAND" ||
                         uav4_current_state.mode != "AUTO.LAND") ) {
                        if( uav1_set_mode_client.call(land_set_mode) &&
                            land_set_mode.response.mode_sent){
                            uav2_set_mode_client.call(land_set_mode);
                            uav3_set_mode_client.call(land_set_mode);
                            uav4_set_mode_client.call(land_set_mode);
                            ROS_INFO("Land enabled");
                        }
                    }
                }

                break;
        }

        // uav2 is the head drone.
        uav2_target_pose = target_pos_;
    }

/*    ROS_INFO("target pos = %.2f, %.2f, %.2f", target_pos_.pose.position.x,
             target_pos_.pose.position.y, target_pos_.pose.position.z);*/
    uav1_target_pose.pose.position = uav2_target_pose.pose.position;
    uav3_target_pose.pose.position = uav2_target_pose.pose.position;
    uav4_target_pose.pose.position = uav2_target_pose.pose.position;

    uav1_local_pos_pub.publish(uav1_target_pose);
    uav2_local_pos_pub.publish(uav2_target_pose);
    uav3_local_pos_pub.publish(uav3_target_pose);
    uav4_local_pos_pub.publish(uav4_target_pose);
}


void MultiOffboard::Oninit() {
    uav1_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, &MultiOffboard::uav1_state_cb, this);
    uav2_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, &MultiOffboard::uav2_state_cb, this);
    uav3_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, &MultiOffboard::uav3_state_cb, this);
    uav4_state_sub = nh.subscribe<mavros_msgs::State>
            ("uav4/mavros/state", 10, &MultiOffboard::uav4_state_cb, this);

    uav1_vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>
            ("uav1/mavros/vfr_hud", 10, &MultiOffboard::vrf_hud_cb, this);
    uav1_gps_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/raw/fix", 1000);
    uav1_global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("uav1/mavros/global_position/global", 1000);
    uav1_g_speed_control_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav1/mavros/setpoint_velocity/cmd_vel", 100);


    uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 5);
    uav1_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, &MultiOffboard::uav1_local_pos_cb, this);

    uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");
    uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 5);
    uav2_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, &MultiOffboard::uav2_local_pos_cb, this);

    uav3_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");
    uav3_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    uav3_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 5);
    uav3_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, &MultiOffboard::uav3_local_pos_cb, this);

    uav4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav4/mavros/set_mode");
    uav4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav4/mavros/cmd/arming");
    uav4_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav4/mavros/setpoint_position/local", 5);
    uav4_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav4/mavros/local_position/pose", 10, &MultiOffboard::uav4_local_pos_cb, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    MultiOffboard m_multi;

    m_multi.Oninit();

    ros::Rate rate(10.0);
    ROS_INFO("is uav1 connected = %d", m_multi.uav1_current_state.connected);

    while(ros::ok() && !m_multi.uav1_current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int count = 0;

    while(ros::ok()){
        if( (m_multi.uav1_current_state.mode != "OFFBOARD" || m_multi.uav3_current_state.mode != "OFFBOARD" ||
                m_multi.uav4_current_state.mode != "OFFBOARD") &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) && !m_multi.is_offboard){
            if( m_multi.uav1_set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                m_multi.uav2_set_mode_client.call(offb_set_mode);
                m_multi.uav3_set_mode_client.call(offb_set_mode);
                m_multi.uav4_set_mode_client.call(offb_set_mode);
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( ! m_multi.uav1_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) && !m_multi.is_offboard){
                if( m_multi.uav1_arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    m_multi.uav2_arming_client.call(arm_cmd);
                    m_multi.uav3_arming_client.call(arm_cmd);
                    m_multi.uav4_arming_client.call(arm_cmd);
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        m_multi.targte_local_pos();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
