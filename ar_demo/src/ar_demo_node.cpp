#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <vector>
#include <mutex>

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace camodocal;

bool pose_init = false;

ros::Publisher object_pub;
image_transport::Publisher pub_ARimage;
Vector3d Cube_center;
queue<ImageConstPtr> img_buf;
camodocal::CameraPtr m_camera;

// swei: Use this info to check collision.
ImageConstPtr depth_image;
std::mutex m_depth;

void generate_corners(Vector3d *corners, const Vector3d &center, double width) {

    double x = center.x(), y = center.y(), z = center.z();
    double offset = width * 0.5;

    corners[0] = Vector3d(x - offset, y - offset, z - offset);
    corners[1] = Vector3d(x + offset, y - offset, z - offset);
    corners[2] = Vector3d(x - offset, y + offset, z - offset);
    corners[3] = Vector3d(x + offset, y + offset, z - offset);
    corners[4] = Vector3d(x - offset, y - offset, z + offset);
    corners[5] = Vector3d(x + offset, y - offset, z + offset);
    corners[6] = Vector3d(x - offset, y + offset, z + offset);
    corners[7] = Vector3d(x + offset, y + offset, z + offset);
}

float get_distance(const cv::Mat &depth_map, const cv::Point &pt) {

    const float radius = 2.f;

    float sum = 0.f;
    int num_sum = 0;

    for (float row = pt.y - radius; row <= pt.y + radius; ++row) {

        if (row <= 0 || row >= 480)
            continue;

        for (float col = pt.x - radius; col <= pt.x + radius; ++col) {

            if (col <= 0 || col >= 640)
                continue;

            auto val = depth_map.at<ushort>(cv::Point(col, row));

            if (val != 0) {
                sum += (float)val;
                num_sum++;
            }
        }
    }

    float avg_dist = num_sum > 0 ? sum / (float)num_sum : 0.f;
    return avg_dist * 1e-3f; // Don't forget to change to meters.
}

bool is_blocked(const cv::Mat &depth_map, const cv::Point &pt, const double z) {

    if (depth_map.empty()) {
        return false;
    }

    if (z < 0) {
        return false;
    }

    float block_dist = get_distance(depth_map, pt);

    if (block_dist <= 0.f) {
        return false;
    }

    if (block_dist < (float)z) {
        return true;
    }

    return false;
}

void draw_distorted_cube(cv::Mat &image, const Vector3d *corners) {
    
    const int image_width = m_camera->imageWidth();
    const int image_height = m_camera->imageHeight();

    cv::Mat depth_map;
    m_depth.lock();
    if (depth_image)
        depth_map = cv_bridge::toCvCopy(depth_image)->image.clone();

    m_depth.unlock();

#define INSIDE(pt) ((pt).x >= 0 && (pt).x < image_width && (pt).y >= 0 && (pt).y < image_height)

    static const int draw_x[] = { 0, 1, 5, 4, 4, 6, 2, 2, 3, 3, 6, 5};
    static const int draw_y[] = { 1, 5, 4, 0, 6, 2, 0, 3, 1, 7, 7, 7};

    // 1. Project corners.
    Vector2d corners2d[8];
    for (size_t i = 0; i < 8; ++i)
        m_camera->spaceToPlane(corners[i], corners2d[i]);

    // 2. Project distorted lines and draw.
    for (size_t order = 0; order < sizeof(draw_x)/sizeof(draw_x[0]); ++ order) {

        const Vector3d &a = corners[draw_x[order]];
        const Vector3d &b = corners[draw_y[order]];
        const Vector2d &a2d = corners2d[draw_x[order]];
        const Vector2d &b2d = corners2d[draw_y[order]];

        double dist = sqrt(
            (a2d.x() - b2d.x()) * (a2d.x() - b2d.x()) + (a2d.y() - b2d.y()) * (a2d.y() - b2d.y()));

        // Dist may goes wrong, so we make sure it has a limitation.
        int num_pixels = (int)std::min(800l, lrint(dist));

        double step_x = num_pixels > 0 ? (b.x() - a.x()) / num_pixels : 0.0;
        double step_y = num_pixels > 0 ? (b.y() - a.y()) / num_pixels : 0.0;
        double step_z = num_pixels > 0 ? (b.z() - a.z()) / num_pixels : 0.0;

        cv::Point last_point = cv::Point(a2d.x(), a2d.y());

        for (int j = 0; j < num_pixels; ++j) {

            Vector3d mid = Vector3d(a.x() + step_x * j, a.y() + step_y * j, a.z() + step_z * j);
            Vector2d mid_projected;
            m_camera->spaceToPlane(mid, mid_projected);
            cv::Point this_point = cv::Point(mid_projected.x(), mid_projected.y());

            if (INSIDE(last_point) && INSIDE(this_point) && (!is_blocked(depth_map, last_point, mid.z()))) {
                cv::line(image, last_point, this_point, cv::Scalar(0, 255, 0), 2, 8, 0);
            }

            last_point = this_point;
        }

        cv::Point finnal_point = cv::Point(b2d.x(), b2d.y());
        if (INSIDE(last_point) && INSIDE(finnal_point) && (!is_blocked(depth_map, last_point, b.z())))
            cv::line(image, last_point, finnal_point, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
}

void project_ar_object(const ImageConstPtr &img_msg,
                       const nav_msgs::Odometry::ConstPtr pose_msg) {
    // Process control.
    static int skipped_img_counter = 0;
    if (skipped_img_counter < 2) {
        skipped_img_counter++;
        return;
    }

    Vector3d camera_p(pose_msg->pose.pose.position.x,
                      pose_msg->pose.pose.position.y,
                      pose_msg->pose.pose.position.z);
    Quaterniond camera_q(
        pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);

    cv::Mat AR_image;

    if (img_msg->encoding == "8UC1") {
        cv_bridge::CvImageConstPtr ptr;
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);        
        AR_image = ptr->image.clone();
        cv::cvtColor(AR_image, AR_image, cv::COLOR_GRAY2RGB);
    } else {
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        AR_image = ptr->image.clone();
    }

    const double cube_width = 0.8;
    static int continuous_unvisible = 0;

    // First, a very rough guess.
    Vector3d cube_center_in_camera = camera_q.inverse() * (Cube_center - camera_p);
    bool unvisible = cube_center_in_camera.z() <= cube_width / 2;

    // First, corners in camera coord.
    Vector3d corners[8];

    if (!unvisible) {

        generate_corners(corners, Cube_center, cube_width);

        // Trans to camera coord.
        for (int j = 0; j < 8; j++) {
            corners[j] = camera_q.inverse() * (corners[j] - camera_p);
            if (corners[j].z() < 0) {
                unvisible = true;
                break;
            }
        }
    }

    // Draw distorted cube.
    if (!unvisible) {
        draw_distorted_cube(AR_image, corners);
    }

    if (unvisible) {
        ROS_WARN("Cube un-visible, will be re-located soon.");
        continuous_unvisible++;
    }
    else
        continuous_unvisible = 0;

    if (continuous_unvisible > 20) {
        continuous_unvisible = 0;
        Vector3d new_cube_in_camera = Vector3d(0, 0, 1 + cube_width / 2.0);
        Vector3d new_cube_in_world = camera_q * new_cube_in_camera + camera_p;
        Cube_center = new_cube_in_world;
        ROS_WARN("Cube moved to %.4f, %.4f, %.4f.",
            new_cube_in_world.x(), new_cube_in_world.y(), new_cube_in_world.z());
    }

    cv::imshow("AR_Image", AR_image);
    cv::waitKey(2);
}

void img_callback(const ImageConstPtr &img_msg) {
    if (!pose_init)
        return;

    img_buf.push(img_msg);
}

void depth_callback(const ImageConstPtr &img_msg) {
    m_depth.lock();
    depth_image = img_msg;
    m_depth.unlock();
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    if (!pose_init) {
        pose_init = true;
        return;
    }

    if (img_buf.empty())
        return;

    // Syncing.
    while ((!img_buf.empty()) && img_buf.front()->header.stamp < pose_msg->header.stamp) {
        img_buf.pop();
    }

    if (!img_buf.empty()) {
        project_ar_object(img_buf.front(), pose_msg);
        img_buf.pop();
    }
}

inline bool file_exist(const std::string &name) {
    ifstream f(name.c_str());
    return f.good();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n("~");
    object_pub = n.advertise<visualization_msgs::MarkerArray>("AR_object", 10);
    ros::Subscriber sub_img = n.subscribe("image_raw", 5, img_callback);
    ros::Subscriber sub_depth = n.subscribe("/mynteye/depth/image_raw", 5, depth_callback);

    Cube_center = Vector3d(0, 0, -1.5);

    ros::Subscriber pose_img = n.subscribe("camera_pose", 100, pose_callback);

    string calib_file;
    n.getParam("calib_file", calib_file);
    ROS_WARN("AR demo will fetch camera parameters from %s", calib_file.c_str());
    if (!file_exist(calib_file)) {
        ROS_ERROR("Camera config file missing at %s.", calib_file.c_str());
        return 1;
    }
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

    ros::Rate r(100);
    ros::Duration(1).sleep();
    ros::spin();
}
