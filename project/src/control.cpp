#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/Range.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <carrot_planner/carrot_planner.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "Pioneer3DX.h"

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <time.h>

const std::string world_frame = "/map";
enum STATE {
    FORWARD,
    ROTATE
};

const float env_x_lim = 3.6;
const float env_y_lim = 6.4;
const float env_x_bound[] = {-1.8, 1.8};
const float env_y_bound[] = {-3.2, 3.2};

bool robot_forward = true;
//count the msg whose range is over thredhold for restarting the robot
//sensor_msgs::Range dataframe[8];

#define DEBUG 1

void commandcallback(const sensor_msgs::Range::ConstPtr & msg){
    static int count = 0;
#if DEBUG
    ROS_INFO("scan data from sensor frame_id: %s", msg->header.frame_id.c_str());
#endif
    const float sonar_threshold = 0.5;

    if(msg->range < sonar_threshold){
        robot_forward = false;
        count = 0;
#if DEBUG
        ROS_INFO("scan data from sensor range: %f", msg->range);
        ROS_INFO("scan data from sensor range: %d", count);
        ROS_INFO("scan data from sensor range: %s", robot_forward==0?"FALSE":"TRUE");
#endif
    }else if((msg->range >= sonar_threshold) && count < 8){
        ++count;
#if DEBUG
        ROS_INFO("scan data from sensor range: %f", msg->range);
        ROS_INFO("scan data from sensor range: %d", count);
#endif
    }else if((msg->range >= sonar_threshold) && count >= 8){
        robot_forward = true;
        ++count;
#if DEBUG
        ROS_INFO("scan data from sensor range: %f", msg->range);
        ROS_INFO("scan data from sensor range: %d", count);
        ROS_INFO("scan data from sensor range: %s", robot_forward==0?"FALSE":"TRUE");
#endif
    }
}

/**
 * Check if the robot will go out of bounds if the current Twist command is executed for the duration specified
 */
bool check_bounds(const tf::TransformListener& tf, const geometry_msgs::Twist& cmd, const ros::Duration& duration){
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped nextPose, poseOut;
    nextPose.pose.position.x = cmd.linear.x*duration.toSec();
    nextPose.pose.position.y = cmd.linear.y*duration.toSec();

    nextPose.pose.orientation = tf::createQuaternionMsgFromYaw(cmd.angular.z);
    nextPose.header.frame_id =  "/base_footprint";
    nextPose.header.stamp = ros::Time();


    try {
        tf.transformPose("/odom", nextPose, poseOut);
        geometry_msgs::Point& pos = poseOut.pose.position;
#ifdef DEBUG
        tf.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
        tf::Vector3 origin = transform.getOrigin();
        ROS_INFO("Origin: (%f %f %f)", origin.x(), origin.y(), origin.z());
        ROS_INFO("nextPose Global: (%f %f %f)", pos.x, pos.y, pos.z);
#endif
        if(pos.x > env_x_bound[0] && pos.x < env_x_bound[1] &&
           pos.y > env_y_bound[0] && pos.y < env_y_bound[1])
            return true;
    }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
#ifdef DEBUG
    ROS_INFO("Robot will be out of bounds!!!");
#endif
    return false;
}

void get_robot_pose(const tf::TransformListener& tf,
        geometry_msgs::PoseStamped& pose, std::string which_frame="/odom") {
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped basePose;

    basePose.pose.orientation.w = 1.0;
    basePose.header.frame_id = "/base_link";
    basePose.header.stamp = ros::Time();

    try {
        tf.transformPose(which_frame, basePose, pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}
void base_to_odom(const tf::TransformListener& tf, geometry_msgs::PoseStamped& pose){
    try {
        tf.transformPose("/odom", pose, pose);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

float cal_distance(geometry_msgs::PoseStamped &pos1,
        geometry_msgs::PoseStamped& pos2) {
    float distance = sqrt(
            pow(pos1.pose.position.x - pos2.pose.position.x, 2)
                    + pow(pos1.pose.position.y - pos2.pose.position.y, 2));
    return distance;
}

size_t get_near_goal_sub(std::vector<geometry_msgs::PoseStamped>& goals, geometry_msgs::PoseStamped& curpos){
    float short_distance = 0.0; //record the shortest distance
    size_t short_sub = 0;
    for (size_t i = 0; i < goals.size(); i++) {
        if ( i == 0 || cal_distance(curpos, goals.at(i)) < short_distance) {
            short_distance = cal_distance(curpos, goals.at(i));
            short_sub = i;
        }
    }
    return short_sub;
}

//take a vector of points in, then generate a reasonable path for robot to walk through each point in most less time and
//back to the start point, by default, the first element in vector is robot's start point
void path_generator(std::vector<geometry_msgs::PoseStamped>& path) {
    ROS_INFO("unsorted path");

    std::vector<geometry_msgs::PoseStamped> copy(path);
    for(size_t i = 0; i< copy.size(); i++){
        ROS_INFO("points from copy (%f, %f)", 
                float(copy.at(i).pose.position.x),
                float(copy.at(i).pose.position.y));
    }

    path.clear();
    path.resize(copy.size());
    geometry_msgs::PoseStamped startpos, curpos, temp;
    startpos = copy.at(0);
    copy.erase(copy.begin());
    size_t size = copy.size();
    size_t short_sub; //record the index of the shortest distance for swap
    for (size_t i = 0; i < size ; i++) {
        if( i == 0)
            curpos = startpos;
        else
            curpos = temp;
        //get the subscript of nearest point
        short_sub = get_near_goal_sub(copy,curpos);

        temp = copy.at(short_sub);
        copy.erase(copy.begin() + short_sub);
        path.at(i) = temp;
    }

    path.at(path.size()-1) = startpos;
    ROS_INFO("Generated path");
    for(size_t i=0; i< path.size();i++){
        ROS_INFO("points (%f, %f)",path.at(i).pose.position.x, path.at(i).pose.position.y);
    }

}


/*
 * Need to subscribe to move_base/feedback and publish to move_base/goal.
 * The feedback tells us when a goal is completed or aborted (when there is an obstacle at the goal)
 *
 * Ref: http://www.ros.org/wiki/navigation/Tutorials/SendingSimpleGoals
 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

tf::Point way_points[] = {
    tf::Point(17.91, 28.39,0),
    tf::Point(26.90, 8.87, 0),
    tf::Point(2.8, 21.69, 0),
    tf::Point(12.00, 1.58, 0),
    tf::Point(-31.81, 4.25, 0),
    tf::Point(-21.55, -15.81, 0),
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "p3dx_controller");
    ros::NodeHandle n("p3dx_control");

    // Used to determine current location from /odom frame
    tf::TransformListener tf(ros::Duration(10));

    // Wait until all the transforms are available. 
    try {
        tf.waitForTransform("/odom", "/base_link", ros::Time(), ros::Duration(5.0));
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    // Wait for the map server indirectly.
    try {
        tf.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(5.0));
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    Pioneer3DX p3dx("p3dx_control");


    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("current_goal", 10);
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped startpos, pos1, pos2, pos3, pos4;
    // Get the current pose of the robot from the /odom frame. The orientation of the current pose will be used to
    // populate the goal orientation. This will make the robot move to the goal in a straight line, more or less.
    get_robot_pose(tf, startpos, world_frame);

    path.push_back(startpos);
    for(size_t i=0; i < sizeof(way_points)/sizeof(tf::Point); i++){
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = world_frame;
        ps.pose.position.x = float(way_points[i].x());
        ps.pose.position.y = float(way_points[i].y());
        path.push_back(ps);
    }

    path_generator(path);

    MoveBaseClient client("move_base", true);
    //wait for the action server to come up
    while (!client.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while (ros::ok()) {

        for (size_t i = 0; i < path.size(); i++) {

            move_base_msgs::MoveBaseGoal goal, inplace_rot_goal;
            geometry_msgs::PoseStamped curpos;
            get_robot_pose(tf, curpos, world_frame);

            goal.target_pose.header.frame_id = world_frame;
            inplace_rot_goal.target_pose.header.frame_id = world_frame;


            goal.target_pose.pose = path.at(i).pose;
            float angle = atan2(goal.target_pose.pose.position.y - curpos.pose.position.y,
                                goal.target_pose.pose.position.x - curpos.pose.position.x);
            goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(angle);

            ROS_INFO("New Goal (%f, %f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, tf::getYaw(curpos.pose.orientation));

            // Rotate in place to face the direction of the goal
            inplace_rot_goal.target_pose.pose = curpos.pose;
            inplace_rot_goal.target_pose.pose.orientation =  goal.target_pose.pose.orientation;
            goal_pub.publish(inplace_rot_goal.target_pose);
            client.sendGoal(inplace_rot_goal);
            //client.waitForResult();
            while(!client.getState().isDone())
                ros::spinOnce();


            // Move towards the goal
            goal_pub.publish(goal.target_pose);
            client.sendGoal(goal);
            //client.waitForResult();
            while(!client.getState().isDone())
                ros::spinOnce();

            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the robot got to the goal");
            else
                ROS_INFO("The base failed to move forward for some reason");

        }

    }
    ros::spin();
    return 0;
}
