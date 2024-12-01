#include <ros/ros.h>
#include <iostream>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>

// Declare global variables
ros::Publisher goal_pub, initial_pose_pub;
cv::Mat map, color_map;
double map_resolution, map_width, map_height;
double map_origin_x, map_origin_y;
double robot_x = 0.0, robot_y = 0.0;
geometry_msgs::Quaternion robot_orientation;

// Flags for modes
bool setting_initial_pose = false;
bool setting_goal = false;

// Map callback
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_resolution = msg->info.resolution;
    map_width = msg->info.width * map_resolution;
    map_height = msg->info.height * map_resolution;
    map = cv::Mat(msg->info.height, msg->info.width, CV_8UC1);

    for (int i = 0; i < msg->info.height; i++) {
        for (int j = 0; j < msg->info.width; j++) {
            int data = msg->data[i * msg->info.width + j];
            map.at<uchar>(i, j) = (data == 0) ? 255 :   // Free space
                      (data == 100) ? 0 :               // Occupied
                      (data == -1) ? 127 :              // Unknown
                      75;                               // Any other value
        }
    }

    // Add instructions to the map
    cv::rectangle(map, cv::Point(10, 10), cv::Point(1650, 70), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(map, "Press 'I' and then click on a desired point on the map to set initial pose, 'G' and click to set goal, obtain instant robot position by pressing 'P'.", 
                cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
}

// Mouse callback
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    double map_x = x * map_resolution;
    double map_y = y * map_resolution;

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (setting_goal) {
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = "map";
            goal.pose.position.x = map_x;
            goal.pose.position.y = map_y;
            goal.pose.position.z = 0.0;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;

            ROS_INFO("Goal set at: (%f, %f)", map_x, map_y);
            goal_pub.publish(goal);
            setting_goal = false; // Disable goal mode after setting
        }

        if (setting_initial_pose) {
            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.stamp = ros::Time::now();
            initial_pose.header.frame_id = "map";
            initial_pose.pose.pose.position.x = map_x;
            initial_pose.pose.pose.position.y = map_y;
            initial_pose.pose.pose.position.z = 0.0;
            initial_pose.pose.pose.orientation.x = 0.0;
            initial_pose.pose.pose.orientation.y = 0.0;
            initial_pose.pose.pose.orientation.z = 1.0;
            initial_pose.pose.pose.orientation.w = 0.0;

            ROS_INFO("Initial pose set at: (%f, %f)", map_x, map_y);
            initial_pose_pub.publish(initial_pose);
            setting_initial_pose = false; // Disable initial pose mode after setting
        }
    }
}

// Position callback
void positionCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    if (map.empty()) {
        ROS_WARN("Map is not yet received!");
        return;
    }

    for (int i = 0; i < msg->transforms.size(); i++) {
        if (msg->transforms[i].header.frame_id == "odom") {
            robot_x = msg->transforms[i].transform.translation.x;
            robot_y = msg->transforms[i].transform.translation.y;
            robot_orientation = msg->transforms[i].transform.rotation;
        }
    }
}

// Laser callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (map.empty()) {
        ROS_WARN("Map is not yet received!");
        return;
    }

    cv::cvtColor(map, color_map, cv::COLOR_GRAY2BGR);
    double robot_yaw = tf::getYaw(robot_orientation);

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < msg->range_max) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double laser_x = robot_x + msg->ranges[i] * cos(angle + robot_yaw);
            double laser_y = robot_y + msg->ranges[i] * sin(angle + robot_yaw);
            int pixel_x = static_cast<int>((map_width / 2 + laser_x) / map_resolution);
            int pixel_y = static_cast<int>((map_height / 2 + laser_y) / map_resolution);
            cv::circle(color_map, cv::Point(pixel_x, pixel_y), 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    int robot_pixel_x = static_cast<int>((map_width / 2 + robot_x) / map_resolution);
    int robot_pixel_y = static_cast<int>((map_height / 2 + robot_y) / map_resolution);
    cv::circle(color_map, cv::Point(robot_pixel_x, robot_pixel_y), 15, cv::Scalar(255, 0, 0), -1);

    cv::imshow("map", color_map);
    char key = cv::waitKey(1);

    // Handle key presses
    if (key == 'i' || key == 'I') {
        setting_initial_pose = true;
        setting_goal = false;
        ROS_INFO("Setting initial pose: Click on the map.");
    } else if (key == 'g' || key == 'G') {
        setting_goal = true;
        setting_initial_pose = false;
        ROS_INFO("Setting goal: Click on the map.");
    } else if (key == 'p' || key == 'P') {
        double robot_yaw_deg = tf::getYaw(robot_orientation) * 180.0 / M_PI; // Convert yaw to degrees
        ROS_INFO("Robot position: x = %f, y = %f, yaw = %f degrees", robot_x, robot_y, robot_yaw_deg);
        
    }
}

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_rviz");
    ros::NodeHandle n;

    ros::Subscriber map_sub = n.subscribe("map", 10, mapCallback);
    ros::Subscriber tf_sub = n.subscribe("tf", 1000, positionCallback);
    ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, laserCallback);
    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    cv::namedWindow("map", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("map", mouseCallback);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}



