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

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace camodocal;

const int cube_num = 1;
const double box_length = 0.1;
bool pose_init = false;
int skipped_img_counter = 0;
// swei: Whether camera view is inside the cube.
int camera_inside_or_ahead = 0;

ros::Publisher object_pub;
image_transport::Publisher pub_ARimage;
Vector3d Axis[6];
Vector3d Cube_center[3];
vector<Vector3d> Cube_corner[3];
vector<Vector3d> output_Axis[6];
vector<Vector3d> output_Cube[3];
vector<double> output_corner_dis[3];
double Cube_center_depth[3];
queue<ImageConstPtr> img_buf;
camodocal::CameraPtr m_camera;
bool look_ground = 0;
std_msgs::ColorRGBA line_color_r;
std_msgs::ColorRGBA line_color_g;
std_msgs::ColorRGBA line_color_b;

void axis_generate(visualization_msgs::Marker &line_list, Vector3d &origin,
                   int id) {

    line_list.id = id;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.a = 1.0;
    line_list.lifetime = ros::Duration();

    line_list.pose.orientation.w = 1.0;
    line_list.color.b = 1.0;
    geometry_msgs::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_r);
    p.x -= 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_g);
    p.y -= 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
    p.z += 1.0;
    line_list.points.push_back(p);
    line_list.colors.push_back(line_color_b);
}

void cube_generate(visualization_msgs::Marker &marker, Vector3d &origin,
                   int id) {

    // uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    // marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    /*
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    */
    marker.scale.x = box_length;
    marker.scale.y = box_length;
    marker.scale.z = box_length;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    geometry_msgs::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    marker.points.push_back(p);
    marker.colors.push_back(line_color_r);
    Cube_corner[id].clear();
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2,
                                       origin.y() - box_length / 2,
                                       origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2,
                                       origin.y() - box_length / 2,
                                       origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2,
                                       origin.y() + box_length / 2,
                                       origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2,
                                       origin.y() + box_length / 2,
                                       origin.z() - box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2,
                                       origin.y() - box_length / 2,
                                       origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2,
                                       origin.y() - box_length / 2,
                                       origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() - box_length / 2,
                                       origin.y() + box_length / 2,
                                       origin.z() + box_length / 2));
    Cube_corner[id].push_back(Vector3d(origin.x() + box_length / 2,
                                       origin.y() + box_length / 2,
                                       origin.z() + box_length / 2));
}

void add_object() {
    visualization_msgs::MarkerArray markerArray_msg;

    visualization_msgs::Marker line_list;
    visualization_msgs::Marker cube_list;

    for (int i = 0; i < cube_num; i++) {
        cube_generate(cube_list, Cube_center[i], i);
    }
    // cube_generate(cube_list, Cube_center[2], 2);
    markerArray_msg.markers.push_back(cube_list);

    object_pub.publish(markerArray_msg);
}

void project_object(Vector3d camera_p, Quaterniond camera_q) {

    for (int i = 0; i < cube_num; i++) {
        output_Cube[i].clear();
        output_corner_dis[i].clear();
        Vector3d cube_center_in_camera = camera_q.inverse() * (Cube_center[i] - camera_p);

        Vector2d cube_center_projected;
        m_camera->spaceToPlane(cube_center_in_camera, cube_center_projected);

        // swei: Just a rough guess.
        bool visible = cube_center_in_camera.z() > (1.414213562373095 * box_length / 2 + 0.01);
        if (visible)
            camera_inside_or_ahead = 0;
        else {
            ROS_INFO("Cube in camera coord %.4f, %.4f, %.4f.",
                cube_center_in_camera.x(), cube_center_in_camera.y(), cube_center_in_camera.z());
            camera_inside_or_ahead++;
        }

        if (visible) {
            Cube_center_depth[i] = cube_center_in_camera.z();
            for (int j = 0; j < 8; j++) {
                Vector3d cube_corner_in_camera =
                    camera_q.inverse() * (Cube_corner[i][j] - camera_p);
                output_corner_dis[i].push_back(cube_corner_in_camera.norm());
                Vector2d cube_corner_projected;
                m_camera->spaceToPlane(cube_corner_in_camera, cube_corner_projected);
                cube_corner_projected.x() = std::min(std::max(-5000.0, cube_corner_projected.x()), 5000.0);
                cube_corner_projected.y() = std::min(std::max(-5000.0, cube_corner_projected.y()), 5000.0);
                output_Cube[i].push_back(Vector3d(cube_corner_projected.x(), cube_corner_projected.y(), 1));
            }
        } else {
            Cube_center_depth[i] = -1;
        }
    }
}

void draw_object(cv::Mat &AR_image) {

    // depth sort  big---->small
    int drawing_orders[cube_num];
    for (int i = 0; i < cube_num; i++)
        drawing_orders[i] = i;

    for (int i = 0; i < cube_num; i++)
        for (int j = 0; j < cube_num - i - 1; j++) {
            if (Cube_center_depth[j] < Cube_center_depth[j + 1]) {
                double tmp = Cube_center_depth[j];
                Cube_center_depth[j] = Cube_center_depth[j + 1];
                Cube_center_depth[j + 1] = tmp;
                int tmp_index = drawing_orders[j];
                drawing_orders[j] = drawing_orders[j + 1];
                drawing_orders[j + 1] = tmp_index;
            }
        }
#if 0
    for (int k = 0; k < cube_num; k++) {
        int i = drawing_orders[k];
        // cout << "draw " << i << "depth " << Cube_center_depth[i] << endl;
        if (output_Cube[i].empty())
            continue;
        // draw color
        cv::Point *p = new cv::Point[8];
        p[0] = cv::Point(output_Cube[i][0].x(), output_Cube[i][0].y());
        p[1] = cv::Point(output_Cube[i][1].x(), output_Cube[i][1].y());
        p[2] = cv::Point(output_Cube[i][2].x(), output_Cube[i][2].y());
        p[3] = cv::Point(output_Cube[i][3].x(), output_Cube[i][3].y());
        p[4] = cv::Point(output_Cube[i][4].x(), output_Cube[i][4].y());
        p[5] = cv::Point(output_Cube[i][5].x(), output_Cube[i][5].y());
        p[6] = cv::Point(output_Cube[i][6].x(), output_Cube[i][6].y());
        p[7] = cv::Point(output_Cube[i][7].x(), output_Cube[i][7].y());

        int npts[1] = {4};
        float min_depth = 100000;
        int min_index = 5;
        for (int j = 0; j < (int)output_corner_dis[i].size(); j++) {
            if (output_corner_dis[i][j] < min_depth) {
                min_depth = output_corner_dis[i][j];
                min_index = j;
            }
        }

        cv::Point plain[1][4];
        const cv::Point *ppt[1] = {plain[0]};
        // first draw large depth plane
        int point_group[8][12] = {{0, 1, 5, 4, 0, 4, 6, 2, 0, 1, 3, 2},
                                  {0, 1, 5, 4, 1, 5, 7, 3, 0, 1, 3, 2},
                                  {2, 3, 7, 6, 0, 4, 6, 2, 0, 1, 3, 2},
                                  {2, 3, 7, 6, 1, 5, 7, 3, 0, 1, 3, 2},
                                  {0, 1, 5, 4, 0, 4, 6, 2, 4, 5, 7, 6},
                                  {0, 1, 5, 4, 1, 5, 7, 3, 4, 5, 7, 6},
                                  {2, 3, 7, 6, 0, 4, 6, 2, 4, 5, 7, 6},
                                  {2, 3, 7, 6, 1, 5, 7, 3, 4, 5, 7, 6}};

        plain[0][0] = p[point_group[min_index][4]];
        plain[0][1] = p[point_group[min_index][5]];
        plain[0][2] = p[point_group[min_index][6]];
        plain[0][3] = p[point_group[min_index][7]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));

        plain[0][0] = p[point_group[min_index][0]];
        plain[0][1] = p[point_group[min_index][1]];
        plain[0][2] = p[point_group[min_index][2]];
        plain[0][3] = p[point_group[min_index][3]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(200, 0, 0));

        if (output_corner_dis[i][point_group[min_index][2]] +
                output_corner_dis[i][point_group[min_index][3]] >
            output_corner_dis[i][point_group[min_index][5]] +
                output_corner_dis[i][point_group[min_index][6]]) {
            plain[0][0] = p[point_group[min_index][4]];
            plain[0][1] = p[point_group[min_index][5]];
            plain[0][2] = p[point_group[min_index][6]];
            plain[0][3] = p[point_group[min_index][7]];
            cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 200, 0));
        }
        plain[0][0] = p[point_group[min_index][8]];
        plain[0][1] = p[point_group[min_index][9]];
        plain[0][2] = p[point_group[min_index][10]];
        plain[0][3] = p[point_group[min_index][11]];
        cv::fillPoly(AR_image, ppt, npts, 1, cv::Scalar(0, 0, 200));
        delete p;
    }

#else
    // swei: For simplicity, just draw one rectangle.
    int i = drawing_orders[0];

    if (camera_inside_or_ahead > 0) {
        ROS_WARN("Cube unvisible.");
    }
    else {
        static const int draw_x[] = { 0, 1, 5, 4, 4, 6, 2, 2, 3, 3, 6, 5};
        static const int draw_y[] = { 1, 5, 4, 0, 6, 2, 0, 3, 1, 7, 7, 7};
        cv::Point p[8];
        p[0] = cv::Point(output_Cube[i][0].x(), output_Cube[i][0].y());
        p[1] = cv::Point(output_Cube[i][1].x(), output_Cube[i][1].y());
        p[2] = cv::Point(output_Cube[i][2].x(), output_Cube[i][2].y());
        p[3] = cv::Point(output_Cube[i][3].x(), output_Cube[i][3].y());
        p[4] = cv::Point(output_Cube[i][4].x(), output_Cube[i][4].y());
        p[5] = cv::Point(output_Cube[i][5].x(), output_Cube[i][5].y());
        p[6] = cv::Point(output_Cube[i][6].x(), output_Cube[i][6].y());
        p[7] = cv::Point(output_Cube[i][7].x(), output_Cube[i][7].y());

        for (size_t order = 0; order < sizeof(draw_x)/sizeof(draw_x[0]); ++ order)
            cv::line(AR_image, p[draw_x[order]], p[draw_y[order]], cv::Scalar(0, 255, 0), 2, 8, 0);
    }
#endif
}

void project_ar_object(const ImageConstPtr &img_msg,
                       const nav_msgs::Odometry::ConstPtr pose_msg) {
    // Just skip some frames.
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

#if 0 // swei: Since we disabled point callback, we won't care about looking down.
    // test plane
    Vector3d cam_z(0, 0, -1);
    Vector3d w_cam_z = camera_q * cam_z;
    // cout << "angle " << acos(w_cam_z.dot(Vector3d(0, 0, 1))) * 180.0 / M_PI
    // << endl;
    if (acos(w_cam_z.dot(Vector3d(0, 0, 1))) * 180.0 / M_PI < 90) {
        // ROS_WARN(" look down");
        look_ground = 1;
    } else
        look_ground = 0;
#endif

    project_object(camera_p, camera_q);
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

    draw_object(AR_image);

    if (camera_inside_or_ahead > 20) {
        camera_inside_or_ahead = 0;
        Vector3d new_cube_in_camera = Vector3d(0, 0, 1 + box_length / 2.0);
        Vector3d new_cube_in_world = camera_q * new_cube_in_camera + camera_p;
        Cube_center[0] = new_cube_in_world;
        ROS_INFO("Cube_center change to %.4f, %.4f, %.4f.",
            new_cube_in_world.x(), new_cube_in_world.y(), new_cube_in_world.z());
    }

// swei: We should abondon rviz, cause it consumes too much.
#if 0 // Go wtih rviz.
    sensor_msgs::ImagePtr AR_msg =
        cv_bridge::CvImage(img_msg->header, "bgr8", AR_image).toImageMsg();
    ROS_INFO("Publishing AR image.");
    pub_ARimage.publish(AR_msg);
#else // No rviz, just a window to show.
    cv::imshow("AR_Image", AR_image);
    cv::waitKey(2);
#endif
}

#if 0 // swei: To make the demo simpler.
void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg) {
    if (!look_ground)
        return;
    int height_range[30];
    double height_sum[30];
    for (int i = 0; i < 30; i++) {
        height_range[i] = 0;
        height_sum[i] = 0;
    }
    for (unsigned int i = 0; i < point_msg->points.size(); i++) {
        // double x = point_msg->points[i].x;
        // double y = point_msg->points[i].y;
        double z = point_msg->points[i].z;
        int index = (z + 2.0) / 0.1;
        if (0 <= index && index < 30) {
            height_range[index]++;
            height_sum[index] += z;
        }
        // cout << "point " << " z " << z << endl;
    }
    int max_num = 0;
    int max_index = -1;
    for (int i = 1; i < 29; i++) {
        if (max_num < height_range[i]) {
            max_num = height_range[i];
            max_index = i;
        }
    }
    if (max_index == -1)
        return;
    int tmp_num = height_range[max_index - 1] + height_range[max_index] +
                  height_range[max_index + 1];
    double new_height = (height_sum[max_index - 1] + height_sum[max_index] +
                         height_sum[max_index + 1]) /
                        tmp_num;
    // ROS_WARN("detect ground plain, height %f", new_height);
    if (tmp_num < (int)point_msg->points.size() / 2) {
        // ROS_INFO("points not enough");
        return;
    }
    // update height
    for (int i = 0; i < cube_num; i++) {
        Cube_center[i].z() = new_height + box_length / 2.0;
    }
    add_object();
}
#endif

void img_callback(const ImageConstPtr &img_msg) {
    if (!pose_init)
        return;

    img_buf.push(img_msg);
    add_object();
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
    ros::Subscriber sub_img;

    sub_img = n.subscribe("image_raw", 5, img_callback);

    Cube_center[0] = Vector3d(0, 0, -1 + box_length / 2.0);

    ros::Subscriber pose_img = n.subscribe("camera_pose", 100, pose_callback);

    // swei: Not sure whether this will cause any trouble, turn it off for now.
    // ros::Subscriber sub_point = n.subscribe("pointcloud", 2000, point_callback);
    
    // swei: Will not need to publish this one, since no rviz will be used.
    // image_transport::ImageTransport it(n);
    // pub_ARimage = it.advertise("AR_image", 1000);

    string calib_file;
    n.getParam("calib_file", calib_file);
    ROS_WARN("AR demo will fetch camera parameters from %s", calib_file.c_str());
    if (!file_exist(calib_file)) {
        ROS_ERROR("Camera config file missing at %s.", calib_file.c_str());
        return 1;
    }
    m_camera =
        CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

    ros::Rate r(100);
    ros::Duration(1).sleep();
    add_object();
    add_object();
    ros::spin();
}
