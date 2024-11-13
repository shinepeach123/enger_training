#include "tools.h"
#include "ros/console.h"
#include <cmath>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include "state_machine_.h"


std::vector<set_target_poses> set_tar_poses;



nav_msgs::Odometry trans_global2car(const nav_msgs::Odometry& target_pose, const nav_msgs::Odometry& cur_pose, double target_yaw) {
    // 提取 cur_pose 中的旋转信息并生成 tf2 Transform
    tf2::Transform cur_transform;
    tf2::Quaternion cur_orientation;
    tf2::fromMsg(cur_pose.pose.pose.orientation, cur_orientation);
    cur_orientation.normalize();
    cur_transform.setRotation(cur_orientation);
    cur_transform.setOrigin(tf2::Vector3(cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y, cur_pose.pose.pose.position.z));

    // 提取 target_pose 的位置并生成 tf2 Vector3
    tf2::Vector3 target_position(target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_pose.pose.pose.position.z);

    // 计算相对于 cur_pose 的坐标
    tf2::Vector3 transformed_position = cur_transform.inverse() * target_position;

    // 构建返回的 nav_msgs::Odometry 对象，只设置位置部分
    nav_msgs::Odometry transformed_odom;
    transformed_odom.pose.pose.position.x = transformed_position.x();
    transformed_odom.pose.pose.position.y = transformed_position.y();
    transformed_odom.pose.pose.position.z = transformed_position.z();
    ROS_INFO("Transformed target position: (%f, %f, %f)", transformed_odom.pose.pose.position.x, transformed_odom.pose.pose.position.y, transformed_odom.pose.pose.position.z);

    // 将 target_yaw 转换为局部坐标系下的四元数
    tf2::Quaternion target_yaw_quaternion;
    target_yaw_quaternion.setRPY(0, 0, target_yaw);
    tf2::Quaternion transformed_orientation = cur_orientation.inverse() * target_yaw_quaternion * cur_orientation;

    // 将变换后的姿态四元数赋值到输出的对象中
    transformed_odom.pose.pose.orientation = tf2::toMsg(transformed_orientation);

    return transformed_odom;
}


void add_tar_pose(float x, float y, float yaw) {
    set_target_poses add_tar_pose_pose;
    add_tar_pose_pose.x = x;
    add_tar_pose_pose.y = y;
    add_tar_pose_pose.yaw = yaw;

    
    set_tar_poses.push_back(add_tar_pose_pose);
    ROS_INFO("Added waypoint: (%.2f, %.2f, %.2f)", x, y, yaw);
}