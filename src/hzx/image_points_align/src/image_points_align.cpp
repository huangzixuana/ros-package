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
#include "image_points_align.h"
namespace image_points_align {

std::mutex mtx_buffer1, mtx_buffer2;
bool camera_init, pub_tf;

std_msgs::Header rgb_img_header, point_cloud_header;

std::deque<pcl::PointCloud<PointType>::Ptr> pointCloud_buffer;
std::deque<sensor_msgs::Image::ConstPtr> rawImage_buffer;

int lidar_cumulation;
std::vector<double> ext_T;
double camera_height, camera_width, camera_fx, camera_fy, camera_cx, camera_cy;
double camera_normal_x1, camera_normal_x2, camera_normal_y1, camera_normal_y2;
Eigen::Matrix4d lidar2camera, camera_matrix;
Eigen::Quaterniond quaternion;

Image_Points_Align::Image_Points_Align(ros::NodeHandle nh) {
  ros::NodeHandle nh_param("~");
  std::string rgb_image_topic, camera_info_topic, points_cloud_topic, output_align_topic;
  nh_param.param<int>("lidar_cumulation", lidar_cumulation, 10);
  nh_param.param<std::string>("rgb_image_topic", rgb_image_topic, "/camera/image_raw");
  nh_param.param<std::string>("points_cloud_topic", points_cloud_topic, "/livox/lidar");
  nh_param.param<std::string>("camera_info_topic", camera_info_topic, "/camera/camera_info");
  nh_param.param<std::string>("output_align_topic", output_align_topic, "/pointsCloud_align");
  nh_param.param<std::vector<double>>("lidar2camera_T", ext_T, std::vector<double>());

  init_Data();

  subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(points_cloud_topic.c_str(), 10, &Image_Points_Align::pointCloudHandler, this);
  subrawImg = nh.subscribe<sensor_msgs::Image>(rgb_image_topic.c_str(), 10, &Image_Points_Align::rawImageCloudHandler, this);
  subImgInfo = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic.c_str(), 10, &Image_Points_Align::imageInfoHandler, this);

  pubPointCloudAlign = nh.advertise<sensor_msgs::PointCloud2>(output_align_topic.c_str(), 10);

  std::thread data_process(&Image_Points_Align::process, this);
  data_process.detach();
}
Image_Points_Align::~Image_Points_Align() {}

void Image_Points_Align::imageInfoHandler(const sensor_msgs::CameraInfoConstPtr &imgMsg) {
  if (camera_init == false) {
    camera_height = imgMsg->height;
    camera_width = imgMsg->width;
    camera_fx = imgMsg->K[0];
    camera_fy = imgMsg->K[4];
    camera_cx = imgMsg->K[2];
    camera_cy = imgMsg->K[5];
    camera_init = true;
    camera_matrix << camera_fx, 0.0, camera_cx, 0.0,
                  0.0, camera_fy, camera_cy, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
    // camera_matrix_inv = camera_matrix.inverse();
    camera_normal_x1 = (0 - camera_cx) / camera_fx;
    camera_normal_x2 = (camera_width - camera_cx) / camera_fx;
    camera_normal_y1 = (0 - camera_cy) / camera_fy;
    camera_normal_y2 = (camera_height - camera_cy) / camera_fy;
  }
  // std::cout << "camera_matrix " << camera_matrix << std::endl;
  // std::cout << "camera_height " << camera_height << std::endl;
  // std::cout << "camera_width " << camera_width << std::endl;
  // std::cout << "camera_normal_x1 " << camera_normal_x1 << std::endl;
  // std::cout << "camera_normal_x2 " << camera_normal_x2 << std::endl;
  // std::cout << "camera_normal_y1 " << camera_normal_y1 << std::endl;
  // std::cout << "camera_normal_y2 " << camera_normal_y2 << std::endl;
}

//交点
PointType line2Panel(PointType pp1, PointType pp2, Eigen::Vector4d plane_m) {
  PointType pp;
  pp.x = NAN;
  pp.y = NAN;
  pp.z = NAN;
  double x1 = pp1.x;
  double y1 = pp1.y;
  double z1 = pp1.z;
  double x2 = pp2.x;
  double y2 = pp2.y;
  double z2 = pp2.z;
  double a = plane_m[0];
  double b = plane_m[1];
  double c = plane_m[2];
  double d = plane_m[3];
  double vpt = a * (x1 - x2) + b * (y1 - y2) + c * (z1 - z2);
  if (fabs(vpt) < 0.001) {
    return pp;
  } else {
    double t = (a * x1 + b * y1 + c * z1 + d) / vpt;
    pp.x = t * (x2 - x1) + x1;
    pp.y = t * (y2 - y1) + y1;
    pp.z = t * (z2 - z1) + z1;

    //check pp2 between pp1 and pout
    // if (fabs(getDistance(pp, pp1) + getDistance(pp1, pp2) - getDistance(pp, pp2)) < 0.001) {
    //   pp.x = NAN;
    //   pp.y = NAN;
    //   pp.z = NAN;
    // }
    return pp;
  }
}

void Image_Points_Align::rawImageCloudHandler(const sensor_msgs::ImageConstPtr &imgMsg) {
  mtx_buffer2.lock();
  rawImage_buffer.push_back(imgMsg);
  while (rawImage_buffer.size() > 1) {
    rawImage_buffer.pop_front();
  }
  mtx_buffer2.unlock();
  rgb_img_header = rawImage_buffer.front()->header;

  static tf2_ros::TransformBroadcaster m_pubBroadcaster;
  if (pub_tf) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header = rgb_img_header;
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = "camera_depth_optical_frame";
    transformStamped.transform.translation.x = ext_T[0];
    transformStamped.transform.translation.y = ext_T[1];
    transformStamped.transform.translation.z = ext_T[2];
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();
    m_pubBroadcaster.sendTransform(transformStamped);
  }
}


void Image_Points_Align::pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  point_cloud_header = pointCloudMsg->header;
  pcl::PointCloud<PointType>::Ptr laserCloud_msg(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*pointCloudMsg, *laserCloud_msg);
  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>());
  // pcl::transformPointCloud (*laserCloud_msg, *output_cloud, lidar2camera);
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform(rgb_img_header.frame_id, "camera_link", ros::Time(0), ros::Duration(0.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  // std::cout << "transformStamped " << transformStamped << std::endl;
  Eigen::Matrix4d lidar2camera_optical, camera2camera_optical;
  Eigen::Quaterniond quaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = quaternion.matrix();
  Eigen::Vector4d trans_tmp;
  trans_tmp << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, 1.0;
  camera2camera_optical.setIdentity(4, 4);
  camera2camera_optical.block<3, 3>(0, 0) = rotation_matrix;
  camera2camera_optical.block<4, 1>(0, 3) = trans_tmp;
  lidar2camera_optical = camera2camera_optical * lidar2camera;

  Eigen::Matrix4d test_tr;
  test_tr.setIdentity(4, 4);
  pcl::transformPointCloud (*laserCloud_msg, *output_cloud, lidar2camera_optical);
  // mtx_buffer.lock();
  // pointCloud_buffer.push_back(output_cloud);
  // while (pointCloud_buffer.size() > 5) {
  //   pointCloud_buffer.pop_front();
  // }
  // mtx_buffer.unlock();
  // if (pubPointCloudAlign.getNumSubscribers() > 0) {
  //   sensor_msgs::PointCloud2 laserCloudAlign;
  //   pcl::toROSMsg(*output_cloud, laserCloudAlign);
  //   laserCloudAlign.header = rgb_img_header;
  //   // laserCloudAlign.header.frame_id = "camera_link";
  //   pubPointCloudAlign.publish(laserCloudAlign);
  // }

  if (camera_init) {
    pcl::PointCloud<PointType>::Ptr laserCloud_chip_align(new pcl::PointCloud<PointType>(camera_width, camera_height));

    // laserCloud_chip_align->resize(camera_height * camera_width);
    // laserCloud_chip_align->height = camera_height;
    // laserCloud_chip_align->width = camera_width;
    for (int i = 0; i < camera_height; ++i) {
      for (int j = 0; j < camera_width; ++j) {
        laserCloud_chip_align->at(j, i).x = NAN;
        laserCloud_chip_align->at(j, i).y = NAN;
        laserCloud_chip_align->at(j, i).z = NAN;
      }
    }

    int cout_p = 0;
    PointType p0;
    p0.x = 0.0, p0.y = 0.0, p0.z = 0.0;
    Eigen::Vector4d plane_c;//z=1.0;
    plane_c << 0.0, 0.0, 1.0, -1.0;
    for (auto &pi : output_cloud->points) {
      PointType pout = line2Panel(p0, pi, plane_c);
      if (pcl::isFinite(pout)) {
        if (camera_normal_x1 < pout.x && pout.x < camera_normal_x2 && camera_normal_y1 < pout.y && pout.y < camera_normal_y2) {
          int j = (int)(pout.x * camera_fx + camera_cx);
          int i = (int)(pout.y * camera_fy + camera_cy);
          // std::cout << "i " << i << std::endl;
          // std::cout << "j " << j << std::endl;
          laserCloud_chip_align->at(j, i).x = pi.x;
          laserCloud_chip_align->at(j, i).y = pi.y;
          laserCloud_chip_align->at(j, i).z = pi.z;
          laserCloud_chip_align->at(j, i).intensity = pi.intensity;
          cout_p++;
        }
      }
    }

    // std::cout << "cout_p " << cout_p << std::endl;
    mtx_buffer1.lock();
    pointCloud_buffer.push_back(laserCloud_chip_align);
    while (((int)(pointCloud_buffer.size())) > lidar_cumulation) {
      pointCloud_buffer.pop_front();
    }
    mtx_buffer1.unlock();
  }

}



void Image_Points_Align::process() {

  ros::Rate r(10);

  while (ros::ok()) {
    if (camera_init == true) {
      if ((!rawImage_buffer.empty()) && (!pointCloud_buffer.empty())) {
        //--------------------------- data prepare---------------------------------
        // pcl::PointCloud<PointType>::Ptr laserCloud_all(new pcl::PointCloud<PointType>());
        // mtx_buffer.lock();
        // for (auto it = pointCloud_buffer.begin(); it != pointCloud_buffer.end(); it++) {
        // pcl::PointCloud<PointType>::Ptr laserCloud_chip(new pcl::PointCloud<PointType>());
        // pcl::fromROSMsg(**it, *laserCloud_chip);
        // *laserCloud_all += *laserCloud_chip;
        //   *laserCloud_all += **it;
        // }
        // *laserCloud_all += *pointCloud_buffer.front();
        // mtx_buffer.unlock();
        // std::cout << "laserCloud_all->points.size() " << laserCloud_all->points.size() << std::endl;

        cv::Mat raw_image_get;
        // rgb_img_header = rawImage_buffer.front()->header;
        mtx_buffer2.lock();
        raw_image_get = cv_bridge::toCvCopy(rawImage_buffer.front(), sensor_msgs::image_encodings::RGB8)->image.clone();
        // raw_image_get = cv_bridge::toCvCopy(rawImage_buffer.front(), sensor_msgs::image_encodings::RGB8)->image;
        mtx_buffer2.unlock();
        // cv::imshow("view", raw_image_get);
        // cv::waitKey(5);
        // int channels = raw_image_get.channels();
        // std::cout << "channels: " << channels << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud_align(new pcl::PointCloud<pcl::PointXYZRGB>());
        laserCloud_align->resize(camera_height * camera_width);
        laserCloud_align->height = camera_height;
        laserCloud_align->width = camera_width;
        mtx_buffer1.lock();
        for (int i = 0; i < camera_height; ++i) {
          for (int j = 0; j < camera_width; ++j) {
            laserCloud_align->at(j, i).x = NAN;
            laserCloud_align->at(j, i).y = NAN;
            laserCloud_align->at(j, i).z = NAN;
            laserCloud_align->at(j, i).r = 0;
            laserCloud_align->at(j, i).g = 0;
            laserCloud_align->at(j, i).b = 0;
            for (auto it = pointCloud_buffer.begin(); it != pointCloud_buffer.end(); it++) {
              if (pcl::isFinite((*it)->at(j, i))) {
                laserCloud_align->at(j, i).x = (*it)->at(j, i).x;
                laserCloud_align->at(j, i).y = (*it)->at(j, i).y;
                laserCloud_align->at(j, i).z = (*it)->at(j, i).z;
                // laserCloud_align->at(j, i).rgb = raw_image_get.ptr<uchar>(j)[i];

                // try {
                std::uint8_t r = raw_image_get.at<cv::Vec3b>(i, j).val[0];
                std::uint8_t g = raw_image_get.at<cv::Vec3b>(i, j).val[1];
                std::uint8_t b = raw_image_get.at<cv::Vec3b>(i, j).val[2];

                // std::uint8_t r = raw_image_get.at<cv::Vec3b>(i, j)[0];
                // std::uint8_t g = raw_image_get.at<cv::Vec3b>(i, j)[1];
                // std::uint8_t b = raw_image_get.at<cv::Vec3b>(i, j)[2];

                // std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                // laserCloud_align->at(j, i).rgb = *reinterpret_cast<float*>(&rgb);
                // std::uint16_t rgb = (0x0000ff & b) | (0x00ff00 & g) | (0xff0000 & r);
                // // std::cout << "rgb " << rgb << std::endl;
                // laserCloud_align->at(j, i).rgb = rgb;
                // std::cout << "i " << i << std::endl;
                // std::cout << "j " << j << std::endl;
                // std::cout << "r " << static_cast<int>(r) << std::endl;
                // std::cout << "g " << static_cast<int>(g) << std::endl;
                // std::cout << "b " << static_cast<int>(b) << std::endl;
                // laserCloud_align->at(j, i).r = static_cast<std::uint8_t>(r);
                // laserCloud_align->at(j, i).g = static_cast<std::uint8_t>(g);
                // laserCloud_align->at(j, i).b = static_cast<std::uint8_t>(b);
                laserCloud_align->at(j, i).r = r;
                laserCloud_align->at(j, i).g = g;
                laserCloud_align->at(j, i).b = b;
                break;
                // } catch (cv::Exception& e) {
                //   ROS_ERROR("remap exception: %s", e.what());
                //   continue;
                // }
              }
            }
          }
        }
        mtx_buffer1.unlock();


        // pcl::KdTreeFLANN<PointType> kdtree;
        // std::vector<int> pointIdxNKNSearch(1);
        // std::vector<float> pointNKNSquaredDistance(1);
        // kdtree.setInputCloud (laserCloud_all);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud_align(new pcl::PointCloud<pcl::PointXYZRGB>());
        // Eigen::Vector4d image_uv, pointL;
        // for (int i = 0; i < raw_image_get.cols; i = i + 2) {
        //   for (int j = 0; j < raw_image_get.rows; j = j + 2) {
        //     image_uv << (double)i, (double)j, 1.0, 1.0;
        //     Eigen::Vector4d pointL = lidar2camera_inv * camera_matrix_inv * image_uv;
        //     // std::cout << "pointL " << pointL << std::endl;
        //     PointType searchPoint;
        //     searchPoint.x = pointL[0] / pointL[3];
        //     searchPoint.y = pointL[1] / pointL[3];
        //     searchPoint.z = pointL[2] / pointL[3];
        //     if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        //       if (pointNKNSquaredDistance[0] < 0.1) { //0.01m
        //         pcl::PointXYZRGB rgb_point;
        //         rgb_point.x = laserCloud_all->points[pointIdxNKNSearch[0]].x;
        //         rgb_point.y = laserCloud_all->points[pointIdxNKNSearch[0]].y;
        //         rgb_point.z = laserCloud_all->points[pointIdxNKNSearch[0]].z;
        //         rgb_point.rgb = raw_image_get.ptr<uchar>(j)[i];
        //         laserCloud_align->points.push_back(rgb_point);
        //       }
        //     }
        //   }
        // }
        // std::cout << "laserCloud_align->points.size() " << laserCloud_align->points.size() << std::endl;

        if (pubPointCloudAlign.getNumSubscribers() > 0) {
          sensor_msgs::PointCloud2 laserCloudAlign;
          pcl::toROSMsg(*laserCloud_align, laserCloudAlign);
          laserCloudAlign.header = rgb_img_header;
          pubPointCloudAlign.publish(laserCloudAlign);
        }

      } else {
        ROS_WARN("rgb_image or pointcloud empty!!!");
      }
    }
    r.sleep();
  }
}


void Image_Points_Align::init_Data() {
  camera_init = false;
  pub_tf = true;
  double roll = ext_T[3];
  double pitch = ext_T[4];
  double yaw = ext_T[5];
  Eigen::Vector3d eulerAngle(roll, pitch, yaw);
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;
  Eigen::Vector4d trans_tmp;
  trans_tmp << ext_T[0], ext_T[1], ext_T[2], 1.0;
  lidar2camera.setIdentity(4, 4);
  lidar2camera.block<3, 3>(0, 0) = rotation_matrix;
  lidar2camera.block<4, 1>(0, 3) = trans_tmp;
  // lidar2camera_inv = lidar2camera.inverse();
  // std::cout << "lidar2camera " << lidar2camera << std::endl;
  quaternion = rotation_matrix;
}


}



