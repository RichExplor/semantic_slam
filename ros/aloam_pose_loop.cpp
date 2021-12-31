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

#include <fstream>

// net stuff
#include <selector.hpp>
namespace cl = rangenet::segmentation;

// standalone lib h
#include "poseloammsg.hpp"

ros::Publisher pubLaserCloud;
std::unique_ptr<cl::Net> net;


struct poseStr
{
    double times;
    Eigen::Vector3d xyz;
    Eigen::Quaterniond q;
};

std::queue<poseStr> globalPose;


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



void callBackCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    Eigen::Quaterniond quat;
    Eigen::Vector3d trans;

    poseStr tempPose = globalPose.front();

    if(std::fabs(laserCloudMsg->header.stamp.toSec() - tempPose.times) > 0.00001)
    {
        std::cout<<"time is too quick or too slow ...."<<std::endl;
        return ;
    }
    else
    {
        trans = tempPose.xyz;
        quat = tempPose.q;

        globalPose.pop();
    }
    


    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;

    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    std::cout<<"点云数量："<<laserCloudIn.size()<<std::endl;

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
        // if( (color_mask[i][0] == 245 && color_mask[i][1] == 150 && color_mask[i][2] == 100) ||
        //     (color_mask[i][0] == 255 && color_mask[i][1] == 0   && color_mask[i][2] == 0  ) ||
        //     (color_mask[i][0] == 200 && color_mask[i][1] == 40  && color_mask[i][2] == 255) ||
        //     (color_mask[i][0] == 30  && color_mask[i][1] == 30  && color_mask[i][2] == 255) ||
        //     (color_mask[i][0] == 90  && color_mask[i][1] == 30  && color_mask[i][2] == 150) ||
        //     (color_mask[i][0] == 250 && color_mask[i][1] == 80  && color_mask[i][2] == 100) ||
        //     (color_mask[i][0] == 180 && color_mask[i][1] == 30  && color_mask[i][2] == 80 ) ||
        //     (color_mask[i][0] == 0   && color_mask[i][1] == 0   && color_mask[i][2] == 0  ) ||
        //     (color_mask[i][0] == 0   && color_mask[i][1] == 0   && color_mask[i][2] == 255)
        //   )
        //   continue;

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

    Eigen::Matrix3d tf_mat = quat.toRotationMatrix();

    tform.block(0,0,3,3) = tf_mat;
    tform(0, 3) = trans[0];
    tform(1, 3) = trans[1];
    tform(2, 3) = trans[2];
    tform(3, 0) = 0; tform(3, 1) = 0; tform(3, 2) = 0; tform(3, 3) = 1;
  
    pcl::transformPointCloud(semanticCloud, transformCloud, tform);  // 转换拼接点云

    voxel_grid_filter(transformCloud, transformCloud, 0.1);
    

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

    std::string lidarTop;
    nh.getParam("lidar_topic", lidarTop);

    // read loop files
    std::ifstream ftimes;
    ftimes.open("/home/lenovo/output/TaoZi/fs_loam_loop.txt");

    int loot_count = 0;
    while(!ftimes.eof())
    {
        poseStr poseTemp;
    
        std::string s;
        std::getline(ftimes, s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double times, x, y, z, qx, qy, qz, qw;
            ss >> times >> x >> y >> z >> qx >> qy >> qz >> qw;
            poseTemp.times = times;
            poseTemp.xyz = Eigen::Vector3d(x,y,z);
            poseTemp.q = Eigen::Quaterniond(qw, qx, qy, qz);  
            // std::cout.precision(19);
            // std::cout<<times<<", "<<x<<" "<<y<<" "<<z<<" "<<qx<<" --- "<<qw<<std::endl;  
        }
        
        // if((loot_count++) % 2 == 0)
            globalPose.push(poseTemp);
    }            

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud", 1000);

    ros::Subscriber subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTop, 1000, callBackCloud);

    // start do it!!!
    // PoseLoamMsg posemsg(nh);
    ros::spin();

    return 0;
}
