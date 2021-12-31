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
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>


// net stuff
#include <selector.hpp>
namespace cl = rangenet::segmentation;

// standalone lib h
#include "poseloammsg.hpp"
#include "segment.hpp"

#define RADIUS_Threshold 0.3

typedef pcl::PointXYZI PointType;


ros::Publisher pubLaserCloud;
std::unique_ptr<cl::Net> net;

pcl::PointCloud<PointType>::Ptr mapCloud(new pcl::PointCloud<PointType>());


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
void voxel_grid_filter(const pcl::PointCloud<PointType>& source_cloud, pcl::PointCloud<PointType>& filtered_cloud, const double& voxel_leaf_size)
{
  pcl::VoxelGrid<PointType> voxel_filter;
  pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());

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
    pcl::PointCloud<PointType> laserCloudIn;

    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    // 剔除掉无效的点云
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    uint32_t num_points = laserCloudIn.size();

    
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

    
    pcl::PointCloud<PointType>::Ptr fullCloud(new pcl::PointCloud<PointType>());

    int dnum = points.size();  // 当前点云数量
    int* pLabel =  (int*)calloc(dnum, sizeof(int)); // 用于区分静态和动态点云

    // 1. 使用语义分割粗略划分静态和动态点云
    PointType point_ori;
    std::vector<int> seeds;
    int count_mask = 0;
    for(size_t i=0; i<dnum; i++)
    {
        // 在半径为 thres 的球形范围内的点被移除掉
        if (points[i][0] * points[i][0] + points[i][1] * points[i][1] + points[i][2] * points[i][2] > 20*20)
            continue;

        if (points[i][0] * points[i][0] + points[i][1] * points[i][1] + points[i][2] * points[i][2] < 5*5)
            continue;

        point_ori.x =  points[i][0];
        point_ori.y =  points[i][1];
        point_ori.z =  points[i][2];
        point_ori.intensity = points[i][3];
        fullCloud->push_back(point_ori);

        // 剔除动态物体(假剔除，因为将静态的目标也剔除掉了) 目前仅剔除 car， 颜色顺序bgr
        if( (color_mask[i][0] == 245 && color_mask[i][1] == 150 && color_mask[i][2] == 100) ||
            (color_mask[i][0] == 255 && color_mask[i][1] == 0   && color_mask[i][2] == 0  ) ||
            (color_mask[i][0] == 200 && color_mask[i][1] == 40  && color_mask[i][2] == 255) ||
            (color_mask[i][0] == 30  && color_mask[i][1] == 30  && color_mask[i][2] == 255) ||
            (color_mask[i][0] == 90  && color_mask[i][1] == 30  && color_mask[i][2] == 150) ||
            (color_mask[i][0] == 250 && color_mask[i][1] == 80  && color_mask[i][2] == 100) ||
            (color_mask[i][0] == 180 && color_mask[i][1] == 30  && color_mask[i][2] == 80 ) ||
            (color_mask[i][0] == 0   && color_mask[i][1] == 0   && color_mask[i][2] == 0  ) ||
            (color_mask[i][0] == 0   && color_mask[i][1] == 0   && color_mask[i][2] == 255)
          )
        {
            pLabel[count_mask] = 1;  // 动态点云
            seeds.push_back(count_mask);
        }
        else
        {
            pLabel[count_mask] = 0;  // 静态点云
        }
        count_mask++;
    }
    
    // 2. 构建kd-tree，开始区域生长
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(fullCloud);

    while(seeds.size()>0)
    {
        int sid = seeds[seeds.size()-1];
        seeds.pop_back();

        std::vector<float> k_dis;
        std::vector<int> k_inds;

        if(fullCloud->points[sid].x < 20)
            kdtree.radiusSearch(sid, RADIUS_Threshold, k_inds, k_dis);
        else
            kdtree.radiusSearch(sid, 1.5*RADIUS_Threshold, k_inds, k_dis);

        for(int ii=0; ii<k_inds.size();++ii)
        {
            if(pLabel[k_inds[ii]]==0)  // 将原本错划分为静态点重新聚类为动态点
            {
                pLabel[k_inds[ii]] = 1;
                seeds.push_back(k_inds[ii]);
            }
        }
    }

    pcl::PointCloud<PointType> dynaCloud;
    pcl::PointCloud<PointType> statCloud;

    for(int i=0; i<fullCloud->points.size(); ++i)
    {
        point_ori = fullCloud->points[i];

        if(pLabel[i] == 1)
        {
            dynaCloud.push_back(point_ori);
        }
        else
        {
            // // 在半径为 thres 的球形范围内的点被移除掉
            // if (point_ori.x * point_ori.x + point_ori.y * point_ori.y + point_ori.z * point_ori.z > 50*50)  //20
            //     continue;

            // if (point_ori.x * point_ori.x + point_ori.y * point_ori.y + point_ori.z * point_ori.z < 2*2) // 5
            //     continue;

            statCloud.push_back(point_ori);
        }
    }

    if (pLabel != NULL)
        free(pLabel);

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

    // 发布静态点云
    pcl::transformPointCloud(statCloud, statCloud, tform);  // 转换拼接点云 statCloud
    sensor_msgs::PointCloud2 laserCloudOutMsg_stat;

    pcl::toROSMsg(statCloud, laserCloudOutMsg_stat);
    laserCloudOutMsg_stat.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg_stat.header.frame_id = "map";
    pubLaserCloud.publish(laserCloudOutMsg_stat);

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
