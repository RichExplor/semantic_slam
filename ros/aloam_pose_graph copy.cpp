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
#define Dynamatic_Threshold 0.05
#define NEAREST_Threshold 0.02

bool initSystem = false; 

typedef pcl::PointXYZI PointType;


ros::Publisher pubLaserCloud_move, pubLaserCloud_dynatic, pubLaserCloud_static, pubLaserCloud_move_full;
std::unique_ptr<cl::Net> net;

pcl::PointCloud<PointType>::Ptr mapCloud(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr lastCloud(new pcl::PointCloud<PointType>());


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

    pubLaserCloud_move = nh.advertise<sensor_msgs::PointCloud2>("/move_cloud", 1000);
    pubLaserCloud_move_full = nh.advertise<sensor_msgs::PointCloud2>("/move_cloud_full", 1000);

    pubLaserCloud_dynatic = nh.advertise<sensor_msgs::PointCloud2>("/dynamtic_cloud", 1000);
    pubLaserCloud_static = nh.advertise<sensor_msgs::PointCloud2>("/static_cloud", 1000);
}



void PoseLoamMsg::callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg, const nav_msgs::Odometry::ConstPtr &laserOdometry)
{


    std::chrono::system_clock::time_point starttime = std::chrono::high_resolution_clock::now();

    // 0. 获取当前点云世界坐标系下位姿
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


    // 1. ROS消息转换为pcl点云，默认使用XYZI类型
    pcl::PointCloud<PointType> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    // pcl::transformPointCloud(laserCloudIn, laserCloudIn, tform);  // 转换到世界坐标系

    // 2. 读取点，转换为vector存储  
    uint32_t num_points = laserCloudIn.size();
    std::vector<float> values;
    for(size_t i=0; i<num_points; i++)
    {
        values.push_back(laserCloudIn.points[i].x);
        values.push_back(laserCloudIn.points[i].y);
        values.push_back(laserCloudIn.points[i].z); 
        values.push_back(laserCloudIn.points[i].intensity);
    }

    // 3. 语义分割网络预测  前向传播->取点->标签
    std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);
    std::vector<cv::Vec4f> points = net->getPoints(values, num_points);
    std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, num_points);

    // 4. 语义分割网络粗略划分静态和动态点云
    pcl::PointCloud<PointType>::Ptr fullCloud(new pcl::PointCloud<PointType>());

    int dnum = points.size();  
    int* pLabel =  (int*)calloc(dnum, sizeof(int)); // 点云标签

    PointType point_ori;
    std::vector<int> seeds;
    for(size_t i=0; i<dnum; i++)
    {
        point_ori.x =  points[i][0];
        point_ori.y =  points[i][1];
        point_ori.z =  points[i][2];
        point_ori.intensity = points[i][3];
        fullCloud->push_back(point_ori);

        // 剔除拟运动物体(粗剔除，因为将静态的目标也剔除掉了)  颜色顺序bgr
        if( (color_mask[i][0] == 245 && color_mask[i][1] == 150 && color_mask[i][2] == 100) ||
            (color_mask[i][0] == 255 && color_mask[i][1] == 0   && color_mask[i][2] == 0  ) ||
            (color_mask[i][0] == 200 && color_mask[i][1] == 40  && color_mask[i][2] == 255) ||
            (color_mask[i][0] == 30  && color_mask[i][1] == 30  && color_mask[i][2] == 255) ||
            (color_mask[i][0] == 90  && color_mask[i][1] == 30  && color_mask[i][2] == 150) ||
            (color_mask[i][0] == 250 && color_mask[i][1] == 80  && color_mask[i][2] == 100) ||
            (color_mask[i][0] == 180 && color_mask[i][1] == 30  && color_mask[i][2] == 80 ) ||
            (color_mask[i][0] == 0   && color_mask[i][1] == 0   && color_mask[i][2] == 0  )
          )
        {
            pLabel[i] = 1;  // 动态点云
            seeds.push_back(i);
        }
        else
        {
            pLabel[i] = 0;  // 静态点云
        }
    }


    std::chrono::system_clock::time_point endtime_1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds_1 = endtime_1 - starttime;
    std::cout << "Time process all tack _1: "<< elapsed_seconds_1.count() * 1000 << "ms" << std::endl;

    
    // 5. 构建kd-tree，区域生长拟动态点云（保证点云完整性）
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(fullCloud);

    while(seeds.size()>0)
    {
        int sid = seeds[seeds.size()-1];
        seeds.pop_back();

        std::vector<float> k_dis;
        std::vector<int> k_inds;

        if(fullCloud->points[sid].x < 40)  // 根据点距离适当增加搜索区域半径
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

    // 6. 精细分割动态与静态点云
    pcl::PointCloud<PointType>::Ptr dynaCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr statCloud(new pcl::PointCloud<PointType>());

    for(int i=0; i<dnum; ++i)
    {
        point_ori = fullCloud->points[i];
        if(pLabel[i] == 1)
        {
            dynaCloud->push_back(point_ori);
        }
        else
        {
            statCloud->push_back(point_ori);
        }
    }

    // 7. 利用位姿变换的距离误差筛选真实动态点云
    pcl::transformPointCloud(*dynaCloud, *dynaCloud, tform);  // 转换到世界坐标系
    
    if(lastCloud->empty()) *lastCloud = *fullCloud;  // 仅发生在第一帧

    pcl::KdTreeFLANN<PointType> kdtreeNear;
    kdtreeNear.setInputCloud(lastCloud);

    int dynaNum = dynaCloud->points.size();
    int *pLabel1 = (int*)calloc(dynaNum, sizeof(int));

    seeds.clear();
    pcl::PointCloud<PointType> dynaCloud_Dynatic;
    for(int i=0; i<dynaNum; ++i)
    {
        vector<float> k_dis;
        vector<int> k_inds;

        point_ori = dynaCloud->points[i];
        kdtreeNear.nearestKSearch(point_ori, 1, k_inds, k_dis);

        if(k_dis[0] > NEAREST_Threshold)
        {
            seeds.push_back(i);
            pLabel1[i] = 1; // 表示距离不满足
            dynaCloud_Dynatic.push_back(point_ori);
        }
        else
        {
            pLabel1[i] = 0; // 距离满足要求，为静态点
        }
    }

    // 8. 真实动态点云区域生长，确保完整性
    pcl::KdTreeFLANN<PointType> kdtreeRadius;
    kdtreeRadius.setInputCloud(dynaCloud);

    while(seeds.size() > 0)
    {
        int sid = seeds[seeds.size()-1];
        seeds.pop_back();

        std::vector<float> k_dis;
        std::vector<int> k_inds;

        kdtreeRadius.radiusSearch(sid, Dynamatic_Threshold, k_inds, k_dis);

        for(int ii=0; ii<k_inds.size();++ii)
        {
            if(pLabel1[k_inds[ii]]==0) 
            {
                pLabel1[k_inds[ii]] = 1;
                seeds.push_back(k_inds[ii]);
            }
        }
    }

    // 9. 根据标签，从原始动态点云中筛选得到静态点云
    pcl::PointCloud<PointType> dynaCloud_Static;
    pcl::PointCloud<PointType> dynaCloud_Dynatic_full;
    for(int i=0; i<dynaNum; ++i)
    {
        point_ori = dynaCloud->points[i];

        if(pLabel1[i] == 1)
        {
            dynaCloud_Dynatic_full.push_back(point_ori);
        }
        else
        {
            dynaCloud_Static.push_back(point_ori);
        }  
    }

    std::chrono::system_clock::time_point endtime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = endtime - starttime;
    std::cout << "Time process all tack: "<< elapsed_seconds.count() * 1000 << "ms" << std::endl;


    // 10. 清空上一帧点云，变换点云位姿
    pcl::transformPointCloud(*fullCloud, *fullCloud, tform);  // 转换到世界坐标系

    lastCloud->clear();
    *lastCloud = *fullCloud;


    // 发布点云消息
    sensor_msgs::PointCloud2 laserCloudOutMsg_dynatic; // 拟动态点云
    pcl::toROSMsg(*dynaCloud, laserCloudOutMsg_dynatic);
    laserCloudOutMsg_dynatic.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg_dynatic.header.frame_id = "world";
    pubLaserCloud_dynatic.publish(laserCloudOutMsg_dynatic);

    
    sensor_msgs::PointCloud2 laserCloudOutMsg_move; // 动态点云
    pcl::toROSMsg(dynaCloud_Dynatic, laserCloudOutMsg_move);
    laserCloudOutMsg_move.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg_move.header.frame_id = "world";
    pubLaserCloud_move.publish(laserCloudOutMsg_move);


    // sensor_msgs::PointCloud2 laserCloudOutMsg_move_full; // 动态点云_full
    // pcl::toROSMsg(dynaCloud_Dynatic_full, laserCloudOutMsg_move_full);
    // laserCloudOutMsg_move_full.header.stamp = laserCloudMsg->header.stamp;
    // laserCloudOutMsg_move_full.header.frame_id = "world";
    // pubLaserCloud_move_full.publish(laserCloudOutMsg_move_full);

    sensor_msgs::PointCloud2 laserCloudOutMsg_move_full; // 动态点云_full
    pcl::toROSMsg(*fullCloud, laserCloudOutMsg_move_full);
    laserCloudOutMsg_move_full.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg_move_full.header.frame_id = "world";
    pubLaserCloud_move_full.publish(laserCloudOutMsg_move_full);


    pcl::transformPointCloud(*statCloud, *statCloud, tform);  // 将原始静态点云转换到世界坐标系
    sensor_msgs::PointCloud2 laserCloudOutMsg_static; // 静态点云
    *statCloud += dynaCloud_Static;
    pcl::toROSMsg(*statCloud, laserCloudOutMsg_static);
    laserCloudOutMsg_static.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg_static.header.frame_id = "world";
    pubLaserCloud_static.publish(laserCloudOutMsg_static);


    // 释放内存
    if (pLabel != NULL)
        free(pLabel);
    if (pLabel1 != NULL)
        free(pLabel1);

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
