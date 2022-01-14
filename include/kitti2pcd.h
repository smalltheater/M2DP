//
// Created by yunqi on 10/01/2022.
//

#ifndef M2DP_KITTI2PCD_H
#define M2DP_KITTI2PCD_H

#endif //M2DP_KITTI2PCD_H
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <time.h>



void readKittiPclBinData(std::string &in_file, std::string& out_file);
Eigen::MatrixXd PCARotationInvariant(pcl::PointCloud<pcl::PointXYZI>::Ptr points);
pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
int i;
const int numT=16;
const int numR=8;
const int numP=4;
const int numQ=16;

std::vector<float> vec_x;
std::vector<float> vec_y;
std::vector<float> vec_z;

Eigen::MatrixXf EigenDataAfterPCA;


