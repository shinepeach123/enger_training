#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include "ros/init.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include <robo_arm.h>
#include <unistd.h>
#include <vector>
#include <nav_msgs/Odometry.h>

arm_pose current_arm_pose;
arm_pose red_circle;
arm_pose blue_circle;
arm_pose green_circle;
std::vector<arm_pose> arm_control;


double low_pass_fliter_x(double x){ //简易滤波器，取过去50次的平均值
    static double prev=0;
    double y = 0.9*prev+0.1*x;
    prev = y;
    return y;
}

double low_pass_fliter_y(double x){ //简易滤波器，取过去50次的平均值
    static double prev=0;
    double y = 0.9*prev+0.1*x;
    prev = y;
    return y;
}
void RobotArm::test() {
    ROS_INFO("arm begin testen");
    int temp_index = 

    add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
    add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
    
    
    add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    
    
    //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    
    
    
    
    //add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
    //add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
    // add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
    // add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_OPEN);
    // add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
    // add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_CLOSE);
    //  add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);

    // add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_GREEN, PAW_OPEN);
    // add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_GREEN, PAW_OPEN);
    // add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_OPEN);
    // add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
    // add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_GREEN, PAW_CLOSE);
    // add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -150, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    // add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -150, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
    //add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);

    //add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_BLUE, PAW_OPEN);
    // add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_GREEN, PAW_OPEN);
    // add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_OPEN);
    // add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
    // add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_GREEN, PAW_CLOSE);
    // add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);

    //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -150, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);



    //add_arm_pose(RED_X, RED_Y, -3, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
    //add_arm_pose(RED_X, RED_Y, -1, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);

    //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -1, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -150, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
    //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -1, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);

    do {
        //if (arm_arrived(arm_control[temp_index])){
        arm_pose_pub (temp_index);
        temp_index ++;
        ROS_INFO("choose index %d",temp_index);
        //}
        ros::Duration(arm_control[temp_index-1].temp_time).sleep();
    }
    while (temp_index < arm_control.size() && ros::ok());

    ROS_INFO("arm has testen");
}

void RobotArm::handle_move_blue() {
    //ROS_INFO("arm begin testen");
    int temp_index = 
   add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
 add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    do {
        //if (arm_arrived(arm_control[temp_index])){
        arm_pose_pub (temp_index);
        temp_index ++;
        ROS_INFO("choose index %d",temp_index);
        //}
        ros::Duration(arm_control[temp_index-1].temp_time).sleep();
    }
    while (temp_index < arm_control.size() && ros::ok());

    ROS_INFO("moved to blue");
    
}

void RobotArm::handle_move_green() {
    //ROS_INFO("arm begin testen");
    int temp_index = 
  add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
 add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
  
    do {
        //if (arm_arrived(arm_control[temp_index])){
        arm_pose_pub (temp_index);
        temp_index ++;
        ROS_INFO("choose index %d",temp_index);
        //}
        ros::Duration(arm_control[temp_index-1].temp_time).sleep();
    }
    while (temp_index < arm_control.size() && ros::ok());

    ROS_INFO("moved to green");
}

double  clamp(double value, double min, double max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}
int RobotArm::test_vision(char color) {
    static double red_x = CIRCULAR_RED_X;
    static double red_y = CIRCULAR_RED_Y;
    static double blue_x = CIRCULAR_BLUE_X;
    static double blue_y = CIRCULAR_BLUE_Y;
    static double green_x = CIRCULAR_GREEN_X;
    static double green_y = CIRCULAR_GREEN_Y;

    int return_value = 0;

    int judge = 0;


    if(color == 'r' && vision_data_.color == 1){ //（规定红色为1，蓝色为2，绿色为3）
        red_y -= -(vision_data_.x - 160) * 0.5;
        red_x -= -(vision_data_.y - 120) * 0.5;
        red_y = clamp(red_y, CIRCULAR_RED_Y - 80, CIRCULAR_RED_Y + 80);
        red_x = clamp(red_x, CIRCULAR_RED_X - 80, CIRCULAR_RED_X + 80); 
        ROS_INFO("tar x = %.4f ,tar_y = %.4f",vision_data_.x - 160,vision_data_.y - 120);
        int temp_index = add_arm_pose_vision(red_x, red_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        //d_arm_pose_vision(red_x, red_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        do {
            //if (arm_arrived(arm_control[temp_index])){
            arm_pose_pub (temp_index);
            temp_index ++;
            //ROS_INFO("choose index %d",temp_index);
            //}
            ros::Duration(arm_control[temp_index-1].temp_time).sleep();
        }
        while (temp_index < arm_control.size() && ros::ok());
        judge = 1;
    }

    if(color == 'b'){ //（规定红色为1，蓝色为2，绿色为3）

        if(vision_data_.color == 2){
            blue_y -= -(vision_data_.x - 160) * 0.15;
            blue_x -= -(vision_data_.y - 120) * 0.15;
            //blue_y = clamp(blue_x, CIRCULAR_BLUE_Y - 20, CIRCULAR_BLUE_Y + 20);
            //blue_x = clamp(blue_y, CIRCULAR_BLUE_X - 20, CIRCULAR_BLUE_X + 80);
            judge = 1; 
        }
        ROS_INFO("looking for blue");
        ROS_INFO("tar x = %.4f ,tar_y = %.4f",vision_data_.x - 160,vision_data_.y - 120);
        int temp_index = add_arm_pose_vision(blue_x, blue_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose_vision(blue_x, blue_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        
        do {
            //if (arm_arrived(arm_control[temp_index])){
            arm_pose_pub (temp_index);
            temp_index ++;
            //ROS_INFO("choose index %d",temp_index);
            //}
            ros::Duration(arm_control[temp_index-1].temp_time).sleep();
        }
        while (temp_index < arm_control.size() && ros::ok());
    }

    if(color == 'g'){ //（规定红色为1，蓝色为2，绿色为3）
        double x1 = red_circle.x;
        double y1 = red_circle.y;
        double x2 = blue_circle.x;
        double y2 = blue_circle.y;
        double destence = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        double k = (y2 - y1) / (x2 - x1);
        double b = (y1 - k * x1);

    }

    //     if(vision_data_.color == 3){
    //         double errorx = vision_data_.x - 160;
    //         double errory = vision_data_.y - 120;
    //         //errory = clamp(errory, -20, 20);
    //         //errorx = clamp(errorx, -20, 20);
    //         green_y -= -(errory) * 0.05;
    //         green_x -= (errorx) * 0.05; 
    //     green_y = clamp(green_y, CIRCULAR_GREEN_Y - 30, CIRCULAR_GREEN_Y + 30);
    //     green_x = clamp(green_x, CIRCULAR_GREEN_X - 30, CIRCULAR_GREEN_X + 30);
    //         judge = 1; 
    //     }
    //     ROS_INFO("tar x = %.4f ,tar_y = %.4f",vision_data_.x - 160,vision_data_.y - 120);
    //     int temp_index = add_arm_pose_vision(green_x, green_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    //     add_arm_pose_vision(green_x, green_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    //     //add_arm_pose_vision(red_x, red_y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
    //     do {
    //         //if (arm_arrived(arm_control[temp_index])){
    //         arm_pose_pub (temp_index);
    //         temp_index ++;
    //         //ROS_INFO("choose index %d",temp_index);
    //         //}
    //         ros::Duration(arm_control[temp_index-1].temp_time).sleep();
    //     }
    //     while (temp_index < arm_control.size() && ros::ok());
    //}

    if (fabs(vision_data_.x - 160) < LIMIT_VISION_X && fabs(vision_data_.y - 120) < LIMIT_VISION_Y
    && judge == 1) {
        switch (color) {
            case 'r':
                red_circle.x = red_x;
                red_circle.y = red_y;
                ROS_INFO("red_circle successfully founded");
                return_value =  1;
                break;
            case 'g':
                green_circle.x = green_x;
                green_circle.y = green_y;
                ROS_INFO("green_circle successfully founded");
                return_value = 3;
                break;
            case 'b':
                blue_circle.x = blue_x;
                blue_circle.y = blue_y;
                ROS_INFO("blue_circle successfully founded");
                return_value = 2;
                break;
        }
    }
    return return_value;
    
}


void RobotArm::choose (char color) {
    int temp_index=0;
    //temp_index = add_arm_pose(INIT_POSE_X, INIT_POSE_Y, CAR_HIGHT, CAM_ANGLE, PAW_OPEN);
    if (color == 'r') {
        ROS_INFO("CHOOSE RED");
        add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -1, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -100, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, -1, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        // add_arm_pose(RED_X, RED_Y, CAR_HIGHT, CAM_ANGLE_RED, PAW_OPEN);
        // add_arm_pose(RED_X, RED_Y, CAR_HIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        // add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
    } else if (color == 'g') {
        ROS_INFO("CHOOSE GREEN");
        add_arm_pose(GREEN_X, GREEN_Y, CAR_HIGHT, CAM_ANGLE_GREEN, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, CAR_HIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
        add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
    } else if (color == 'b') {
        ROS_INFO("CHOOSE BLUE");
        add_arm_pose(BULE_X, BULE_Y, CAR_HIGHT, CAM_ANGLE_BLUE, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, CAR_HIGHT, CAM_ANGLE_BLUE, PAW_CLOSE);
        add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_BLUE, PAW_CLOSE);
    }
    
    
    do {
        //if (arm_arrived(arm_control[temp_index])){
        arm_pose_pub (temp_index);
        temp_index ++;
        ROS_INFO("choose index %d",temp_index);
        //}
        ros::Duration(arm_control[temp_index-1].temp_time).sleep();
    }
    while (temp_index < arm_control.size() && ros::ok());
    ROS_INFO("has chooosen");
}

int RobotArm::vision_correction(char color){//初版视觉矫正代码。使用阻塞式编写
    int temp_index=0;
    temp_index = add_arm_pose(INIT_POSE_X, INIT_POSE_Y, CAR_HIGHT, CAM_ANGLE, PAW_OPEN);
    switch (color) {
        case 'r':
            add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, TEKEUP_HEIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
            break;
        case 'g':
            add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, TEKEUP_HEIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
            break;
        case 'b':
            add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, TEKEUP_HEIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
            break;
    }
    do {
        arm_pose_pub (temp_index);
        if (arm_arrived(arm_control[temp_index])) {//todo 下位节点写一个返回位置的callback
            temp_index ++;
            ROS_INFO("moved to %c,ready to do correction",color);
        }
    }
    while (temp_index < arm_control.size() && ros::ok());
    int result = 0;
    while(!result && ros::ok()){//阻塞式接收,调整机械臂位置
        result = vision(color);
    }
    return result;
}

int RobotArm::vision(char color) {
    int result = 0;
    if (fabs(vision_data_.x) > 160) {
        movelittle(vision_data_.x > 160 ? '<' : '>');
    }
    // 如果 x 在可接受范围内，处理 y 方向
    else if (fabs(vision_data_.y) > 120) {
        movelittle(vision_data_.y > 120 ? '^' : 'v');
    }

    // else if (fabs(vision_data_.x) <= LIMIT_VISION_X && fabs(vision_data_.y) <= LIMIT_VISION_Y) {
    //     switch (color) {
    //         case 'r':
    //             red_circle = current_arm_pose;
    //             result = 1;
    //             ROS_INFO("red_circle successfully founded");
    //             break;
    //         case 'g':
    //             green_circle = current_arm_pose;
    //             result = 2;
    //             ROS_INFO("green_circle successfully founded");
    //             break;
    //         case 'b':
    //             blue_circle = current_arm_pose;
    //             result = 3;
    //             ROS_INFO("blue_circle successfully founded");
    //             break;
    //     }
    // }
    // return result;
}

void RobotArm::movelittle(char direction) {
    arm_pose last_pose = arm_control.back();

    if (direction == '<') {
        last_pose.x += MOVE_LITTLE_DISTANCE;
    } else if (direction == '>') {
        last_pose.x -= MOVE_LITTLE_DISTANCE;
    } else if (direction == '^') {
        last_pose.y += MOVE_LITTLE_DISTANCE;
    } else if (direction == 'v') {
        last_pose.y -= MOVE_LITTLE_DISTANCE;
    } 

    int temp_index = add_arm_pose_from_define(last_pose);
    //while (!arm_arrived(last_pose)) {
    arm_pose_pub(temp_index);
}

void RobotArm::put_down(char color){
    int temp_index=0;
    ROS_INFO("begin put down");
    if (color == 'r') {
        ROS_INFO("PUT DOWN RED");
        temp_index =     add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(RED_X, RED_Y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(red_circle.x, red_circle.y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(red_circle.x, red_circle.y, -160, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(red_circle.x, red_circle.y, -160, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(red_circle.x, red_circle.y, -1, CAM_ANGLE_RED, PAW_OPEN);
        
    } else if (color == 'g') {
        ROS_INFO("PUT DOWN GREEN");
        temp_index =     add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(GREEN_X, GREEN_Y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(green_circle.x, green_circle.y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(green_circle.x, green_circle.y, -160, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(green_circle.x, green_circle.y, -160, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(green_circle.x, green_circle.y, -1, CAM_ANGLE_RED, PAW_OPEN);
        
    } else if (color == 'b') {
        ROS_INFO("PUT DOWN BLUE");
        temp_index =     add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(BULE_X, BULE_Y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(blue_circle.x, blue_circle.y, -1, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(blue_circle.x, blue_circle.y, -160, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(blue_circle.x, blue_circle.y, -160, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(blue_circle.x, blue_circle.y, -1, CAM_ANGLE_RED, PAW_OPEN);
    }

    do {
        arm_pose_pub (temp_index);
        if (arm_arrived(arm_control[temp_index])) {
            temp_index ++;
            ROS_INFO("put down index %d",temp_index);
        }
    }
    while (temp_index < arm_control.size() && ros::ok());
    ROS_INFO("has put down");
}


int RobotArm::add_arm_pose(float x, float y, float z, float cam_angle, float paw_angle)
{
    arm_pose temp_pose;
    temp_pose.x = x;
    temp_pose.y = y;
    temp_pose.z = z;
    temp_pose.cam_angle = cam_angle;
    temp_pose.paw_angle = paw_angle;

    //this temp this temp this temp this temp this temp this temp this temp this temp
    double last_z = arm_control.back().z;
    if (fabs(last_z - z) != 0) {
        temp_pose.temp_time = 0.07 * fabs(last_z - z);

    } else {
        temp_pose.temp_time = 2;
    }
    ROS_INFO("temp_time %f",temp_pose.temp_time);
    //this temp this temp this temp this temp this temp this temp this temp this temp

    arm_control.push_back(temp_pose);
    return arm_control.size() - 1; // 返回新加入元素的索引值
}

int RobotArm::add_arm_pose_vision(float x, float y, float z, float cam_angle, float paw_angle)
{
    arm_pose temp_pose;
    temp_pose.x = x;
    temp_pose.y = y;
    temp_pose.z = z;
    temp_pose.cam_angle = cam_angle;
    temp_pose.paw_angle = paw_angle;

    //this temp this temp this temp this temp this temp this temp this temp this temp
    double last_z = arm_control.back().z;
    if (fabs(last_z - z) != 0) {
        temp_pose.temp_time = 0.09 * fabs(last_z - z);

    } else {
        temp_pose.temp_time = 0.2;
    }
    //ROS_INFO("temp_time %f",temp_pose.temp_time);
    //this temp this temp this temp this temp this temp this temp this temp this temp

    arm_control.push_back(temp_pose);
    return arm_control.size() - 1; // 返回新加入元素的索引值
}

int RobotArm::add_arm_pose_from_define(arm_pose pose)
{
    arm_control.push_back(pose);
    return arm_control.size() - 1; // 返回新加入元素的索引值
}


void RobotArm::arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
nav_msgs::Odometry target_arm_pose_sub;
target_arm_pose_sub = *msg;
current_arm_pose.cam_angle = target_arm_pose_sub.pose.pose.position.x;
current_arm_pose.x = target_arm_pose_sub.pose.pose.orientation.x;
current_arm_pose.y = target_arm_pose_sub.pose.pose.orientation.y;
current_arm_pose.z = target_arm_pose_sub.pose.pose.orientation.z;
current_arm_pose.paw_angle = target_arm_pose_sub.pose.pose.orientation.w;
//ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
}

void RobotArm::vision_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
std_msgs::ColorRGBA temp_vision_msg;
temp_vision_msg = *msg;
vision_data_.x = temp_vision_msg.r;
vision_data_.y = temp_vision_msg.g;
vision_data_.color = temp_vision_msg.b;//
//ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
}
void RobotArm::arm_pose_pub(int index){
        nav_msgs::Odometry target_arm_pose_pub;
        target_arm_pose_pub.pose.pose.position.x = arm_control[index].cam_angle; // ע������Ӧ���� num.x ������ arm_control[num].cam_angle
        target_arm_pose_pub.pose.pose.orientation.x = arm_control[index].x;
        target_arm_pose_pub.pose.pose.orientation.y = arm_control[index].y;
        target_arm_pose_pub.pose.pose.orientation.z = arm_control[index].z;
        target_arm_pose_pub.pose.pose.orientation.w = arm_control[index].paw_angle;
        arm_pub.publish(target_arm_pose_pub); // ����Ϊ target_arm_pose_pub
        //ROS_INFO("arm_moving_to_target");
        ros::Duration(0.1).sleep();
}
bool RobotArm::arm_arrived(arm_pose target_arm_pose){
    if(fabs(current_arm_pose.x - target_arm_pose.x)<0.3 &&
        fabs(current_arm_pose.y - target_arm_pose.y)<0.3 &&
        fabs(current_arm_pose.z - target_arm_pose.z)<0.3 &&
        fabs(current_arm_pose.cam_angle - target_arm_pose.cam_angle)<0.3 &&
        fabs(current_arm_pose.paw_angle - target_arm_pose.paw_angle)<0.3
    ){
        return true;
    }else{
        return false;
    }
}

    



