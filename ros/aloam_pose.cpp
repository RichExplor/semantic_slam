#include <iostream>
#include <chrono>
#include <iomanip>  // for setfill
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>


// net stuff
#include <selector.hpp>
namespace cl = rangenet::segmentation;

// standalone lib h
#include "poseloammsg.hpp"

ros::Publisher pubLaserCloud;
std::unique_ptr<cl::Net> net;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZRGB>());


// 移除掉点云某个范围内的点
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        // 在半径为 thres 的球形范围内的点被移除掉
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }

    // 重新设置点云大小
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

// 对点云进行将采用，减小对系统资源的占用，加快程序运行：
void voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZRGB>& source_cloud, pcl::PointCloud<pcl::PointXYZRGB>& filtered_cloud, const double& voxel_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  *tempCloud = source_cloud;
  voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.setInputCloud(tempCloud);
  voxel_filter.filter(filtered_cloud);
  return;
}

PoseLoamMsg::PoseLoamMsg(ros::NodeHandle& nh): nh_(nh)
{

    if(!nh_.getParam("pose_topic", pose_topic_))
        ROS_ERROR("failed to read pose topic.");
    if(!nh_.getParam("lidar_topic", lidar_topic_))
        ROS_ERROR("failed to read lidar topic.");

    std::cout<<"激光雷达话题："<<lidar_topic_<<std::endl;

    lidar_sub_.subscribe(nh_, lidar_topic_, 1);
    pose_sub_.subscribe(nh_, pose_topic_, 1);

    ROS_INFO("%s", lidar_topic_.c_str());
    sync.reset(new Sync(lidar_odom_fuse_policy(10), lidar_sub_, pose_sub_));
    sync->registerCallback(boost::bind(&PoseLoamMsg::callback, this, _1, _2));

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud", 1000);
}



void PoseLoamMsg::callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg, const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;

    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    std::cout<<"点云数量："<<laserCloudIn.size()<<std::endl;

    // 剔除掉无效的点云
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    // removeClosedPointCloud(laserCloudIn, laserCloudIn, 3.0);  // 移除周围1m球行范围的点

    uint32_t num_points = laserCloudIn.size();

    std::cout<<"点云数量："<<num_points<<std::endl;

    
    std::vector<float> values;

    for(size_t i=0; i<num_points; i++)
    {
        values.push_back(laserCloudIn.points[i].x);
        values.push_back(laserCloudIn.points[i].y);
        values.push_back(laserCloudIn.points[i].z); 
        values.push_back(laserCloudIn.points[i].intensity);
    }

      // predict
    std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);

    // get point cloud
    std::vector<cv::Vec4f> points = net->getPoints(values, num_points);

    // get color mask
    std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, num_points);

    pcl::PointCloud<pcl::PointXYZRGB> semanticCloud;
    pcl::PointCloud<pcl::PointXYZRGB> transformCloud;


    for(size_t i=0; i<points.size(); i++)
    {
        pcl::PointXYZRGB p;

        // // 剔除动态物体(假剔除，因为将静态的目标也剔除掉了) 目前仅剔除 car， 颜色顺序bgr
        // if(color_mask[i][0] == 245 && color_mask[i][1] == 150 && color_mask[i][2] == 100)
        //     continue;

        // if(color_mask[i][0] == 255 && color_mask[i][1] == 0 && color_mask[i][2] == 0)
        //     continue;

        // if(color_mask[i][0] == 200 && color_mask[i][1] == 40 && color_mask[i][2] == 255)
        //     continue;

        // if(color_mask[i][0] == 30 && color_mask[i][1] == 30 && color_mask[i][2] == 255)
        //     continue;

        // if(color_mask[i][0] == 90 && color_mask[i][1] == 30 && color_mask[i][2] == 150)
        //     continue;

        // if(color_mask[i][0] == 250 && color_mask[i][1] == 80 && color_mask[i][2] == 100)
        //     continue;

        // if(color_mask[i][0] == 180 && color_mask[i][1] == 30 && color_mask[i][2] == 80)
        //     continue;

        // if(color_mask[i][0] == 0 && color_mask[i][1] == 0 && color_mask[i][2] == 0)
        //     continue;

        // if(color_mask[i][0] == 0 && color_mask[i][1] == 0 && color_mask[i][2] == 255)
        //     continue;

        // 在半径为 thres 的球形范围内的点被移除掉
        if (points[i][0] * points[i][0] + points[i][1] * points[i][1] + points[i][2] * points[i][2] > 20*20)
            continue;

        if (points[i][0] * points[i][0] + points[i][1] * points[i][1] + points[i][2] * points[i][2] < 5*5)
            continue;


        p.x =  points[i][0];
        p.y =  points[i][1];
        p.z =  points[i][2];
        p.b =  color_mask[i][0];
        p.g =  color_mask[i][1];
        p.r =  color_mask[i][2];
        semanticCloud.points.push_back(p);   
    }

    /* pose odomtry */
    Eigen::Matrix4d tform;

    Eigen::Quaterniond quat;
	Eigen::Vector3d trans;
	quat.x() = laserOdometry->pose.pose.orientation.x;
	quat.y() = laserOdometry->pose.pose.orientation.y;
	quat.z() = laserOdometry->pose.pose.orientation.z;
	quat.w() = laserOdometry->pose.pose.orientation.w;
	trans.x() = laserOdometry->pose.pose.position.x;
	trans.y() = laserOdometry->pose.pose.position.y;
	trans.z() = laserOdometry->pose.pose.position.z;

    Eigen::Matrix3d tf_mat = quat.toRotationMatrix();

    tform.block(0,0,3,3) = tf_mat;
    tform(0, 3) = trans[0];
    tform(1, 3) = trans[1];
    tform(2, 3) = trans[2];
    tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
  
    pcl::transformPointCloud(semanticCloud, transformCloud, tform);  // 转换拼接点云

    voxel_grid_filter(transformCloud, transformCloud, 0.1);
    // voxel_grid_filter(transformCloud, transformCloud, 0.05);
    

    // 发布处理后的点云消息
    sensor_msgs::PointCloud2 laserCloudOutMsg;

    pcl::toROSMsg(transformCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "map";
    pubLaserCloud.publish(laserCloudOutMsg);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "semantic");
    ros::NodeHandle nh("~");

    std::string path;
    bool verbose = false;
    nh.getParam("model_path", path);
    nh.getParam("verbose", verbose);
    std::cout<<"path: " << path << std::endl;

    std::string backend = "tensorrt";
    // create a network
    net = cl::make_net(path, backend);
    net->verbosity(verbose);


    // start do it!!!
    PoseLoamMsg posemsg(nh);
    ros::spin();

    return 0;
}
