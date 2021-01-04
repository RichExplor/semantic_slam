# semantic_slam

This repository contains simple usage explanations of how the RangeNet++ inference works with the TensorRT and C++ interface.

Developed by [Xieyuanli Chen](http://www.ipb.uni-bonn.de/people/xieyuanli-chen/), [Andres Milioto](http://www.ipb.uni-bonn.de/people/andres-milioto/) and [Jens Behley](http://www.ipb.uni-bonn.de/people/jens-behley/).

For more details about RangeNet++, one could find in [LiDAR-Bonnetal](https://github.com/PRBonn/lidar-bonnetal).

For more details about rangenet_lib, one could find in [rangenet_lib](https://github.com/PRBonn/rangenet_lib).

---
## How to use

### 1. System dependencies

First you need to install the nvidia driver and CUDA.

- CUDA Installation guide: [Link](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

- Then you can do the other dependencies:

  ```sh
  $ sudo apt-get update 
  $ sudo apt-get install -yqq  build-essential python3-dev python3-pip apt-utils git cmake libboost-all-dev libyaml-cpp-dev libopencv-dev
  ```
  
### 2. Python dependencies

- Then install the Python packages needed:

  ```sh
  conda create -n tensorflow python=3.6
  source activate tensorflow
  conda install tensorflow-gpu==1.11.0

  sudo apt install python-empy
  sudo pip install catkin_tools trollius numpy
  ```
  
### 3. TensorRT

In order to infer with TensorRT during inference with the C++ libraries:

**run version : Ubuntu16.04 + CUDA10.0 + Cudnn7.5.0 + TensorRT-5.1.5.0**

- Install TensorRT: [Link](https://developer.nvidia.com/tensorrt).
- Our code and the pretrained model now only works with **TensorRT version 5** (Note that you need at least version 5.1.0).
- To make the code also works for higher versions of TensorRT, one could have a look at [here](https://github.com/PRBonn/rangenet_lib/issues/9).


### 4. Build the library

We use the catkin tool to build the library.

  ```sh
  mkdir -p ~/catkin_wind/src
  cd ~/catkin_wind/src
  catkin_init_workspace

  git clone https://github.com/GuoFeng-X/semantic_slam.git
  cd .. 
  catkin_make -j8
  ```

### 5. Build A-LOAM 

仅对A-LOAM中的两个launch文件进行改动，增加了激光雷达的话题参数。具体可参考[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
使用mapping后的位姿作为最终位姿。

  ```sh
  cd ~/catkin_wind/src
  git clone https://github.com/GuoFeng-X/A-LOAM.git
  cd .. 
  catkin_make -j8
  ```

### 6. Run the demo

#### 6.1 Run kitti database --velodyne HDL-64

一段kitti数据集运行视频可查看哔哩哔哩，[semantic_slam](https://www.bilibili.com/video/BV1Fy4y1m7iB/)

To run the demo, you need a pre-trained model, which can be downloaded here, [model](http://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz). 

  ```sh
  # go to the root path of the catkin workspace
  cd catkin_wind/src/rangenet_lib
  source activate tensorflow
  roslaunch semantic_slam aloam_pose_velodyne_64.launch
  rosbag play -r0.5 kitti_odometry_sequence_00.bag
  ```

#### 6.2 Run ourself database --velodyne VLP-32C
To run the demo, you need a pre-trained model, which can be downloaded here. 
仅仅改变了其中的 arch_cfg.yaml文件, 注意需要将两者的文件夹名字更换
链接: https://pan.baidu.com/s/1EG27yE0Q_1gi3UD_Ta_vZw 提取码: riqc 

  ```sh
  # go to the root path of the catkin workspace
  cd catkin_wind/src/rangenet_lib
  source activate tensorflow
  roslaunch semantic_slam aloam_pose_velodyne_32.launch
  rosbag play -r0.5 lidar_cam_imu55.bag
  ```

For more details about how to train and evaluate a model, please refer to [LiDAR-Bonnetal](https://github.com/PRBonn/lidar-bonnetal).


## Applications
#### Run SuMa++: Efficient LiDAR-based Semantic SLAM
Using rangenet_lib, we built a LiDAR-based Semantic SLAM system, called SuMa++.

You could find more implementation details in [SuMa++](https://github.com/PRBonn/semantic_suma/).

#### Generate probabilities over semantic classes for OverlapNet
OverlapNet is a LiDAR-based loop closure detection method, which uses multiple cues generated from LiDAR scans.

More information about our OverlapNet could be found [here](https://github.com/PRBonn/OverlapNet).

One could use our rangenet_lib to generate probabilities over semantic classes for training OverlapNet.

More detailed steps and discussion could be found [here](https://github.com/PRBonn/rangenet_lib/issues/31).

## Citations

If you use this library for any academic work, please cite the original [paper](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/milioto2019iros.pdf).

```
@inproceedings{milioto2019iros,
  author    = {A. Milioto and I. Vizzo and J. Behley and C. Stachniss},
  title     = {{RangeNet++: Fast and Accurate LiDAR Semantic Segmentation}},
  booktitle = {IEEE/RSJ Intl.~Conf.~on Intelligent Robots and Systems (IROS)},
  year      = 2019,
  codeurl   = {https://github.com/PRBonn/lidar-bonnetal},
  videourl  = {https://youtu.be/wuokg7MFZyU},
}
```

If you use SuMa++, please cite the corresponding [paper](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf):

```
@inproceedings{chen2019iros, 
  author    = {X. Chen and A. Milioto and E. Palazzolo and P. Giguère and J. Behley and C. Stachniss},
  title     = {{SuMa++: Efficient LiDAR-based Semantic SLAM}},
  booktitle = {Proceedings of the IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)},
  year      = {2019},
  codeurl   = {https://github.com/PRBonn/semantic_suma/},
  videourl  = {https://youtu.be/uo3ZuLuFAzk},
}
```

## License

Copyright 2019, Xieyuanli Chen, Andres Milioto, Jens Behley, Cyrill Stachniss, University of Bonn.

This project is free software made available under the MIT License. For details see the LICENSE file.
