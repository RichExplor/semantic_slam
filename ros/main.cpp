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
#include "infer.hpp"

static int index_x;
ros::Publisher pubLaserCloud;
std::unique_ptr<cl::Net> net;

std::vector<Eigen::Matrix4d> tforms;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZRGB>());


// 对点云进行将采用，减小对系统资源的占用，加快程序运行：
void voxel_grid_filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud, const double& voxel_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.setInputCloud(source_cloud);
  voxel_filter.filter(*filtered_cloud);
  return;
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



void laserCloudHander(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;


    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    // 剔除掉无效的点云
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    uint32_t num_points = laserCloudIn.size();

    std::cout<<"点云数量："<<num_points<<std::endl;
    

    std::vector<float> values;

    for(size_t i=0; i<num_points; i++)
    {
        values.push_back(laserCloudIn.points[i].x);
        values.push_back(laserCloudIn.points[i].y);
        values.push_back(laserCloudIn.points[i].z); 
        values.push_back(laserCloudIn.points[i].intensity);
        // std::cout<<"z："<<laserCloudIn.points[i].x<<" ";
    }


      // predict
    std::vector<std::vector<float>> semantic_scan = net->infer(values, num_points);

    // get point cloud
    std::vector<cv::Vec3f> points = net->getPoints(values, num_points);

    // get color mask
    std::vector<cv::Vec3b> color_mask = net->getLabels(semantic_scan, num_points);

    pcl::PointCloud<pcl::PointXYZRGB> semanticCloud;
    pcl::PointCloud<pcl::PointXYZRGB> transformCloud;


    for(size_t i=0; i<points.size(); i++)
    {
        pcl::PointXYZRGB p;

        // 剔除动态物体(假剔除，因为将静态的目标也剔除掉了) 目前仅剔除 car
        // if(color_mask[i][0] == 245 && color_mask[i][1] == 150 && color_mask[i][2] == 100)
        //     continue;

        p.x =  points[i][0];
        p.y =  points[i][1];
        p.z =  points[i][2];
        p.b =  color_mask[i][0];
        p.g =  color_mask[i][1];
        p.r =  color_mask[i][2];
        semanticCloud.points.push_back(p);   
    }

    // pcl::transformPointCloud(semanticCloud, transformCloud, tforms[index_x]);  // 转换拼接点云

    // voxel_grid_filter(transformCloud, transformCloud, 0.1);
    
    // *mapCloud += transformCloud;
    
    index_x++;

    // if(index_x % 100 == 0)
    // {
    //     voxel_grid_filter(mapCloud, mapCloud, 0.3);
    // }

    // // 发布处理后的点云消息
    // sensor_msgs::PointCloud2 laserCloudOutMsg;

    // pcl::toROSMsg(*mapCloud, laserCloudOutMsg);
    // laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    // laserCloudOutMsg.header.frame_id = "/camera_init";
    // pubLaserCloud.publish(laserCloudOutMsg);

    // 发布处理后的点云消息
    sensor_msgs::PointCloud2 laserCloudOutMsg;

    pcl::toROSMsg(semanticCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

}


void get_transforms(std::string pose_files, std::vector<Eigen::Matrix4d> &tforms)
{
    std::string line;
    std::ifstream ifs;
    ifs.open(pose_files, std::ios::in);

    if(!ifs)
    {
        std::cerr<<"can't open files: "<<pose_files<<std::endl;
        return ;
    }

    while(std::getline(ifs, line))
    {
        if(line.empty()) return ;
        std::stringstream linestream(line);
        std::string cell;
        std::vector<double> vdata;

        while(std::getline(linestream, cell, ' '))
        {
            vdata.push_back(std::stod(cell));
        }

        double roll, yaw, pitch;
        Eigen::Matrix4d tform;
        tf::Matrix3x3 tf_mat;
        tf_mat.setValue(vdata[0], vdata[1], vdata[2], vdata[4], vdata[5], vdata[6], vdata[8],vdata[9], vdata[10]);
        tf_mat.getRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        tf_mat.setRotation(tf::Quaternion(quat.z, -quat.x, -quat.y, quat.w));
        tform(0, 0) = tf_mat[0][0]; tform(0, 1) = tf_mat[0][1]; tform(0, 2) = tf_mat[0][2]; tform(0, 3) = vdata[11];
        tform(1, 0) = tf_mat[1][0]; tform(1, 1) = tf_mat[1][1]; tform(1, 2) = tf_mat[1][2]; tform(1, 3) = -vdata[3];
        tform(2, 0) = tf_mat[2][0]; tform(2, 1) = tf_mat[2][1]; tform(2, 2) = tf_mat[2][2]; tform(2, 3) = -vdata[7];
        tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
        tforms.push_back(tform);  
    }

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "semantic");
    ros::NodeHandle nh("~");

    std::string path, pose_files;
    bool verbose = false;
    nh.getParam("model_path", path);
    nh.getParam("verbose", verbose);
    nh.getParam("pose_files", pose_files);
    std::cout<<"path: " << path << std::endl;
    std::cout<<"pose files： "<< pose_files << std::endl;

    std::string backend = "tensorrt";
    // create a network
    net = cl::make_net(path, backend);
    net->verbosity(verbose);

    get_transforms(pose_files, tforms);  // 获取pose值



    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 1000, laserCloudHander);
    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, laserCloudHander);
    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, laserCloudHander);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud", 1000);
  
    ros::spin();

    return 0;
}
