#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Pioneer3DX.h"
#include <sstream>

const std::string world_frame = "/map";

Pioneer3DX::Pioneer3DX(std::string node_name) :
    n(node_name), tf(ros::Duration(10))
    //global_costmap("global_costmap", tf), local_costmap("local_costmap", tf) 
{
        ROS_INFO("Waiting for transform");

        try {
            tf.waitForTransform(world_frame, "/base_link", ros::Time(),
                    ros::Duration(10.0));
        } catch (tf::TransformException ex) {
            ROS_ERROR("TF Error: %s", ex.what());
        }

        get_robot_pose(initial_pose);

        //vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        //goal_pub = n.advertise<geometry_msgs::PoseStamped>("current_goal", 10);
        //pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
        pcl_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud", 10);
        //range_pub = n.advertise<sensor_msgs::Range>("/ranger_scan", 16); //16 sonars

        //glp.initialize("global_planner", &global_costmap);
        //tp.initialize("local_planner", &tf, &local_costmap);
        ROS_INFO("Init subscriber");
        //goal_sub = n.subscribe("goal", 1, &Pioneer3DX::goal_callback, this);
        sonar_sub = n.subscribe("/ranger_scan", 16, &Pioneer3DX::sonar_compiler, this);
        sonar_sub_p2os = n.subscribe("/sonar", 16, &Pioneer3DX::sonar_p2os_callback, this);
        ROS_INFO("Done subscribing");

    }
void Pioneer3DX::print_pose(const geometry_msgs::PoseStamped& pose) {
    ROS_INFO(
            "Frame: %s, Pose (%f %f, A: %f)", pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation));
}
void Pioneer3DX::move_robot(std::vector<geometry_msgs::PoseStamped>& plan_obj) {
    ros::Duration time_out(30);
    ros::Time init_time = ros::Time::now();
    //ROS_INFO("Got new plan");
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped base_pose;
    geometry_msgs::PoseStamped goal_pose = plan_obj.at(plan_obj.size()-1);
    ros::Rate local_plan_rate(10.0);
    tp.setPlan(plan_obj);
    while (!tp.isGoalReached() && ros::ok()) {
        get_robot_pose(base_pose);
        pose_pub.publish(base_pose);
        //plan_obj[0] = base_pose;
        //plan_obj[1].header.stamp = ros::Time::now();
        if (tp.setPlan(plan_obj)) {
            // Local plan
            tp.computeVelocityCommands(cmd_vel);
            vel_pub.publish(cmd_vel);
            //ROS_INFO("Goal: %d", tp.isGoalReached());
            //if ((fabs(cmd_vel.linear.x) < 0.001)
            //        && (fabs(cmd_vel.angular.z) < 0.001)) {

            //    ROS_INFO(
            //            "Stuck fabs(lin x): %f fabs(ang z): %f. Setting new goal", cmd_vel.linear.x, cmd_vel.angular.z);
            //    break;
            //}
            ros::spinOnce();
            local_plan_rate.sleep();
        } else {
            ROS_ERROR("Unable to set plan");
        }
        if (ros::Time::now() - init_time > time_out) {
            ROS_INFO("Timeout while navigating to goal. Setting new goal");
            break;
        }
        if(!glp.makePlan(base_pose, goal_pose, plan_obj)){
            ROS_ERROR("Could not genrate new plan.");
            break;
        }

    }
    if (tp.isGoalReached()) {
        ROS_INFO("Goal reached");
    }
}
void Pioneer3DX::rotate_robot(const geometry_msgs::PoseStamped& goal_pose) {
    float angle_diff = M_PI;
    ros::Rate local_plan_rate(20.0);
    geometry_msgs::Twist cmd_vel;

    const float angular_speed = 1.0;

    do {
        get_robot_pose(base_pose);
        pose_pub.publish(base_pose);
        float cur_angle = tf::getYaw(base_pose.pose.orientation);
        float goal_angle = tf::getYaw(goal_pose.pose.orientation);
        angle_diff = goal_angle - cur_angle;
        if (angle_diff > M_PI)
            angle_diff = -(2 * M_PI - angle_diff);

        if (angle_diff > 0) {
            cmd_vel.angular.z = angular_speed;
        } else {
            cmd_vel.angular.z = -angular_speed;
        }
        ROS_INFO("Angles %f, %f diff %f", goal_angle, cur_angle, angle_diff);
        vel_pub.publish(cmd_vel);
        ros::spinOnce();
        local_plan_rate.sleep();
    } while (fabs(angle_diff) > 0.2 && ros::ok());
}
void Pioneer3DX::set_random_goal() {
}
void Pioneer3DX::get_robot_pose(geometry_msgs::PoseStamped& pose) {
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped base_pose;

    base_pose.pose.orientation.w = 1.0;
    base_pose.header.frame_id = "/base_link";
    base_pose.header.stamp = ros::Time();

    try {
        tf.transformPose(world_frame, base_pose, pose);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
}
//bool Pioneer3DX::check_bounds(){
//}

/**
 * Compute the angle between the current position and the goal. Note: this is not computing the difference in
 * orientation. Instead, it's using the x,y position of the goal from the base_link frame and find the angle
 */
void Pioneer3DX::rotate_to_pose(const geometry_msgs::PoseStamped& goal_pose,
        geometry_msgs::PoseStamped& rot_pose) {
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped tmp_pose;
    geometry_msgs::PoseStamped tmp_rot_pose;
    tmp_rot_pose.pose.orientation.w = 1;
    tmp_rot_pose.header.frame_id = "/base_link";
    tmp_rot_pose.header.stamp = ros::Time();

    while (ros::ok()) {
        try {
            tf.transformPose("/base_link", goal_pose, tmp_pose);
            print_pose(tmp_pose);
            float angle = atan2(tmp_pose.pose.position.y,
                    tmp_pose.pose.position.x);
            // Test
            //float angle2 = tf::getYaw(tmp_pose.pose.orientation);

            tmp_rot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                    angle);
            rot_pose.header.stamp = ros::Time();
            tf.transformPose(world_frame, tmp_rot_pose, rot_pose);
            break;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        ros::spinOnce();
    }

    //ROS_INFO("Angle1 %f, Angle 2 %f", angle, angle2);
}
void Pioneer3DX::goal_callback(
        const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Got call back");
    get_robot_pose(base_pose);

    // clear costmap
    //costmap.resetMapOutsideWindow(0, 0);

    // First rotate robot
    //ROS_INFO("Rotating robot");
    //geometry_msgs::PoseStamped rot_pose;
    //rotate_to_pose(*msg, rot_pose);
    //print_pose(rot_pose);
    goal_pub.publish(*msg);
    //rotate_robot(rot_pose);

    // Now move the robot

    std::vector<geometry_msgs::PoseStamped> plan;
    //get_robot_pose(base_pose);
    //plan.push_back(base_pose);
    ////msg->header.stamp = ros::Time();
    //plan.push_back(*msg);
    if (glp.makePlan(base_pose, *msg, plan)) {
        ROS_INFO("Moving robot");
        move_robot(plan);
    } else {
        ROS_ERROR("Could not genrate global plan.");
    }
}

void Pioneer3DX::sonar_p2os_callback(const p2os_driver::SonarArray::ConstPtr& msg) {
    sensor_msgs::Range discrete_sonar;
    std::vector<sensor_msgs::Range> sonar_data;
    for (size_t i = 0; i < msg->ranges.size()-8; i++) {
        discrete_sonar.header = msg->header;
        std::stringstream ss;
        ss << "base_ranger_link_" << i;
        discrete_sonar.header.frame_id = ss.str();
        //ROS_INFO("Frame ID %s", ss.str().c_str());
        discrete_sonar.range = msg->ranges.at(i);
        discrete_sonar.field_of_view = 0.5;
        sonar_data.push_back(discrete_sonar);
    }
    point_cloud_generator(sonar_data);
}

void Pioneer3DX::sonar_compiler(const sensor_msgs::Range::ConstPtr& msg) {
    //collecting sonar data
    if(sonar_data.empty())
        sonar_data.push_back(*msg);
    else {
        if (msg->header.stamp == sonar_data[0].header.stamp) {
            sonar_data.push_back(*msg);
        }
        else{
            point_cloud_generator(sonar_data);
            sonar_data.clear();
        }
    }

    // Start collecting data
    if (sonar_data.size() > 16)
        ROS_ERROR("sonar_data overflow %d", int(sonar_data.size()));
}

void Pioneer3DX::point_cloud_generator(std::vector<sensor_msgs::Range>& sonar_data) {
    float NUM_OF_POINTS = 40;
    sensor_msgs::PointCloud cloud_out;
    cloud_out.header.frame_id = world_frame;
    cloud_out.header.stamp = ros::Time::now();

    try {
        // For each ranger data, calculate a point cloud and append it to cloud_out, which is ultimately published.
        for (size_t j = 0; j < sonar_data.size(); j++) {
            if(sonar_data[j].header.frame_id != "base_ranger_link_2" && sonar_data[j].header.frame_id != "base_ranger_link_3"
            && sonar_data[j].header.frame_id != "base_ranger_link_4" && sonar_data[j].header.frame_id != "base_ranger_link_5")
                continue;
            //ROS_INFO("S %s=%f", sonar_data[j].header.frame_id.c_str(), sonar_data[j].range);
            sensor_msgs::PointCloud cloud_tmp;
            sensor_msgs::PointCloud cloud_tmp_out;
            cloud_tmp.header.frame_id = sonar_data[j].header.frame_id;

            //float length = sonar_data[j].field_of_view * sonar_data[j].range/2.0;
            float length = 0.1;
            float step = length / NUM_OF_POINTS;
            geometry_msgs::Point32 p;
            p.x = sonar_data[j].range;
            //cloud_tmp.points.push_back(p);

            // Calculate the set of points orthogonal to the line between the robot and the object. i.e, cover the field
            // of view with points at the ranges' distance.
            for (int i = 0; i < NUM_OF_POINTS / 2; i++) {
                p.y = step * (float) i;
                cloud_tmp.points.push_back(p);

            }
            for (int i = NUM_OF_POINTS / 2; i < NUM_OF_POINTS; i++) {
                p.y = -(step * (float) (i - NUM_OF_POINTS / 2));
                cloud_tmp.points.push_back(p);
            }

            tf.transformPointCloud(world_frame, cloud_tmp, cloud_tmp_out);
            cloud_out.points.insert(cloud_out.points.end(),
                    cloud_tmp_out.points.begin(), cloud_tmp_out.points.end());
        }
        pcl_pub.publish(cloud_out);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("tf error: %s", ex.what());
    }

}

Pioneer3DX::~Pioneer3DX() {
}
