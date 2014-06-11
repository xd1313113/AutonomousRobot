#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_listener.h>
#include "sensor_msgs/Range.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <carrot_planner/carrot_planner.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include "p2os_driver/SonarArray.h"

class Pioneer3DX
{
public:
    Pioneer3DX (std::string node_name);
    void move_robot(std::vector<geometry_msgs::PoseStamped>& plan_obj);
    void rotate_robot(const geometry_msgs::PoseStamped& goal_pose);
    void set_random_goal();
    void get_robot_pose(geometry_msgs::PoseStamped& pose);
    bool check_bounds();
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    //void sonar_callback(const sensor_msgs::Range::ConstPtr& msg);
    void sonar_compiler(const sensor_msgs::Range::ConstPtr& msg);
    void point_cloud_generator(std::vector<sensor_msgs::Range>& sonar_data);
    void sonar_p2os_callback(const p2os_driver::SonarArray::ConstPtr& msg);
    void rotate_to_pose(const geometry_msgs::PoseStamped& goal_pose, geometry_msgs::PoseStamped& rot_pose);
    void print_pose(const geometry_msgs::PoseStamped& pose);
    virtual ~Pioneer3DX ();

private:
    /* data */
    ros::NodeHandle n;
    tf::TransformListener tf;

    //costmap_2d::Costmap2DROS global_costmap;
    //costmap_2d::Costmap2DROS local_costmap;
    navfn::NavfnROS glp;
    base_local_planner::TrajectoryPlannerROS tp;

    // Subscribers
    ros::Subscriber goal_sub;
    ros::Subscriber sonar_sub;
    ros::Subscriber sonar_sub_p2os;

    // Advertisers
    ros::Publisher vel_pub;
    ros::Publisher goal_pub;
    ros::Publisher pose_pub;
    ros::Publisher pcl_pub; //pointcloud
    ros::Publisher range_pub;//publisher for discrete sonar range

    geometry_msgs::PoseStamped initial_pose;
    geometry_msgs::PoseStamped base_pose;

    std::vector<sensor_msgs::Range> sonar_data;
    //int count;
};
