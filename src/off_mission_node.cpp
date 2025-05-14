/**
 * @file off_mission_node.cpp
 * @brief This is an offboard control example for Pursuit autopilot
 * More information can be referred in: https://cloudkernel.cn/pursuit
 *
 * @author Cloudkernel Technologies (Shenzhen) Co.,Ltd, main page: https://cloudkernel.cn
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <pursuit_msgs/VcuBaseStatus.h>

#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <mavconn/mavlink_dialect.h>

//variables
static std::vector<geometry_msgs::PoseStamped> waypoints;
static int current_wpindex = 0;//waypoint index starts from zero
static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped current_local_pos;
static pursuit_msgs::VcuBaseStatus vcu_base_status;

//All in SI units
static double nav_acc_rad_xy = 0.0f;
static double current_yaw = 0.0f;

static bool _flag_last_wp_reached = false; //flag to indicate last wp is reached in position phase
static bool _flag_mission_completed = false; //flag to indicate that the mission is completed

static ros::Time _phase_entry_timestamp; //entry timestamp of a mission phase

//mission phases
enum class ROVER_MISSION_PHASE {POS_PHASE, TWIST_CONTROL_PHASE};

static ROVER_MISSION_PHASE _mission_phase = ROVER_MISSION_PHASE::POS_PHASE;


// callbacks for subscriptions
// vehicle state
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// vehicle local position
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;

    current_yaw = tf::getYaw(current_local_pos.pose.orientation);
}

//vcu base status
void vcu_base_status_cb(const pursuit_msgs::VcuBaseStatus::ConstPtr& msg){
    vcu_base_status = *msg;
}

// init wp list from yaml file
void initTagetVector(XmlRpc::XmlRpcValue &wp_list);

// update current waypoint index
void updateWaypointIndex();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "off_mission_node");
    ros::NodeHandle nh;

    /*subscriptions, publications and services*/
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 5, state_cb);

    //subscription for local position
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 5, local_pose_cb);

    ros::Subscriber vcu_base_status_sub = nh.subscribe<pursuit_msgs::VcuBaseStatus>
            ("mavros/vcu_base_status/output", 5, vcu_base_status_cb);


    //publication for local position setpoint
    ros::Publisher local_pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 5);

    ros::Publisher nav_vel_cmd_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/vcu_command_velocity/ctrl", 5);

    //service for arm/disarm
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    //service for main mode setting
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    ROS_INFO("Starting offboard control for Pursuit autopilot demonstration...");
    ros::Duration(1.0).sleep();

    /* param loading*/
    ros::NodeHandle private_nh("~");

    // load yaml files
    XmlRpc::XmlRpcValue wp_list;
    private_nh.getParam("waypoints",wp_list);
    initTagetVector(wp_list);
    ROS_INFO("waypoint yaml loaded!");

    // update parameters
    int simulation_flag = 0;

    private_nh.getParam("simulation_flag", simulation_flag);
    private_nh.getParam("nav_acc_rad_xy", nav_acc_rad_xy);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    /*service commands*/
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode stabilized_set_mode;
    stabilized_set_mode.request.custom_mode = "STABILIZED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;


    ros::Time last_request = ros::Time::now();


    while(ros::ok()){

        if (current_wpindex == 0)
        {
            if (simulation_flag == 1)
            {
                //set offboard mode, then arm the vehicle
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(1.0))){

                    if( set_mode_client.call(offboard_set_mode) &&
                        offboard_set_mode.response.mode_sent){
                        ROS_INFO("Offboard mode enabled");
                    }

                    last_request = ros::Time::now();

                } else {
                    if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(1.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
            }

        }


        if (!_flag_mission_completed){

            switch (_mission_phase){

            /* publish local position setpoint for rover maneuvering*/
                case ROVER_MISSION_PHASE::POS_PHASE:{

                    ROS_INFO_ONCE("Starting position guidance phase");

                    geometry_msgs::PoseStamped pose; //pose to be passed to fcu

                    pose = waypoints.at(current_wpindex);

                    ROS_INFO_THROTTLE(2.0f, "Rover: go to position %5.3f, %5.3f, %5.3f \n",(double)pose.pose.position.x, (double)pose.pose.position.y,
                                        (double)pose.pose.position.z);

                    local_pos_sp_pub.publish(pose);

                    updateWaypointIndex();

                    break;
                }


            /* publish twist command for rover maneuvering*/
                case ROVER_MISSION_PHASE::TWIST_CONTROL_PHASE:{

                    ROS_INFO_ONCE("Starting twist control phase");

                    geometry_msgs::Twist twist_cmd;

                    twist_cmd.linear.x = 1.0;
                    twist_cmd.linear.y = 0.0;
                    twist_cmd.linear.z = 0.0;
                    twist_cmd.angular.x = 0.0;
                    twist_cmd.angular.y = 0.0;
                    twist_cmd.angular.z = 0.5; //about 30deg/s

                    nav_vel_cmd_pub.publish(twist_cmd);

                    //phase transition after a certain time
                    if ( ros::Time::now() - _phase_entry_timestamp > ros::Duration(6.0)){
                        _flag_mission_completed = true;
                        ROS_INFO("Mission completed");
                    }

                    break;
                }


        default:

                ROS_INFO("Unsupported mission phases");
                break;


            }

        }

        /*switch to stabilized mode and disarm when the mission is completed*/
        if (_flag_mission_completed){

            if( current_state.mode != "STABILIZED" &&
                    (ros::Time::now() - last_request > ros::Duration(1.0))){

                    if( set_mode_client.call(stabilized_set_mode) &&
                        stabilized_set_mode.response.mode_sent){
                        ROS_INFO("Stabilized mode enabled");
                    }
                    
                    last_request = ros::Time::now();

            }  else {

                if( current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){

                    if( arming_client.call(disarm_cmd) && arm_cmd.response.success){

                        ROS_INFO("Vehicle disarmed");
                    }
                    last_request = ros::Time::now();

                }

            }



        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



void initTagetVector(XmlRpc::XmlRpcValue &wp_list)
{
    waypoints.clear();

    geometry_msgs::PoseStamped tempPose;
    for (size_t i = 0; i < wp_list.size(); ++i)
    {
        tempPose.header.seq = i;
        XmlRpc::XmlRpcValue data_list(wp_list[i]);

        // get position
        tempPose.pose.position.x = data_list[0];
        tempPose.pose.position.y = data_list[1];
        tempPose.pose.position.z = data_list[2];

        // get orientation
        tf::Quaternion q = tf::createQuaternionFromYaw(data_list[3]);

        tf::quaternionTFToMsg(q, tempPose.pose.orientation);

        waypoints.push_back(tempPose);

    }

    ROS_INFO("Loaded waypoint size is %d ", (int)waypoints.size());
}


void updateWaypointIndex()
{
    float dx = current_local_pos.pose.position.x - waypoints.at(current_wpindex).pose.position.x;
    float dy = current_local_pos.pose.position.y - waypoints.at(current_wpindex).pose.position.y;
    //float yaw_sp = tf::getYaw(waypoints.at(current_wpindex).pose.orientation);

    float dist_xy_sq = dx*dx+dy*dy;

    bool is_position_reached_flag = false;
    bool is_yaw_reached_flag = true; //we don't check yaw reach for rover (nonholonomic vehicle)

    //check position reach condition
    if (dist_xy_sq<nav_acc_rad_xy*nav_acc_rad_xy){
        is_position_reached_flag = true;
    }

    if (current_wpindex == waypoints.size()-1 && current_state.armed)
        ROS_INFO_ONCE("Heading for the last waypoint");

    if (is_position_reached_flag && is_yaw_reached_flag){

        if (current_wpindex < waypoints.size()-1)
        {
            ROS_INFO_ONCE("waypoint reached, advance to next!");
            current_wpindex++;
        }
        else{
            _flag_last_wp_reached = true;

            _mission_phase = ROVER_MISSION_PHASE::TWIST_CONTROL_PHASE;

            _phase_entry_timestamp = ros::Time::now();
            ROS_INFO("The waypoint mission in POS_PHASE is finished, switch to VEL_PHASE");
        }

    }

}
