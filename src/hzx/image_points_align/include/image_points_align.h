/******************************************************************************
 * Copyright leapting.com. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _IMAGE_POINTS_ALIGN_H_
#define _IMAGE_POINTS_ALIGN_H_

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

namespace image_points_align {

typedef pcl::PointXYZI PointType;
// typedef pcl::PointXYZINormal PointType;

class Image_Points_Align {
  public:
    Image_Points_Align(ros::NodeHandle nh);
    ~Image_Points_Align();

  private:
    ros::Publisher pubPointCloudAlign;
    ros::Subscriber subPointCloud;
    ros::Subscriber subrawImg;
    ros::Subscriber subImgInfo;

  private:
    void rawImageCloudHandler(const sensor_msgs::ImageConstPtr &imgMsg);
    void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg);
    void imageInfoHandler(const sensor_msgs::CameraInfoConstPtr &imgMsg);
    void process();
    void init_Data();
};

}  // namespace
#endif  // 