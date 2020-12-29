#ifndef LOCAL_PLANNING_ALG
#define LOCAL_PLANNING_ALG

#include <Eigen/Eigen>
#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

namespace local_planner{

class local_planning_alg{
public:
    local_planning_alg(){}
    ~local_planning_alg(){}
    virtual void set_odom(nav_msgs::Odometry cur_odom)=0;
    virtual void set_local_map(sensor_msgs::PointCloud2ConstPtr &local_map_ptr)=0;
    virtual void set_local_map_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_ptr) = 0;
    virtual int compute_force(Eigen::Matrix<double, 3, 1> &goal, Eigen::Matrix<double, 3, 1> current_odom, Eigen::Vector3d &desired_vel)=0;
    virtual void init(ros::NodeHandle& nh)=0; // 纯虚函数

    typedef shared_ptr<local_planning_alg> Ptr;
};

}

#endif 