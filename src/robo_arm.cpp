#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include "std_msgs/ColorRGBA.h"
#include <robo_arm.h>
#include <vector>
#include <nav_msgs/Odometry.h>

#define ARM1_LENGTH 220.0  // ��۳���
#define ARM2_LENGTH 180.0  // С�۳���
#define GRIPPER_LENGTH 120.0  // ��צ����
#define MAX_ITER 2000       // ����������
#define LEARNING_RATE 0.05  // ѧϰ�ʣ������ݶ���
#define TOLERANCE 0.1      // ��������
#define PI 3.14159265359

#define DEG_TO_RAD(angleInDegrees) ((angleInDegrees) * PI / 180.0)
#define RAD_TO_DEG(angleInRadians) ((angleInRadians) * 180.0 / PI)

class RobotArm {
public:
    RobotArm(){
        arm_pub = nh.advertise<nav_msgs::Odometry>("/arm_ctrl", 10);
        arm_sub = nh.subscribe<nav_msgs::Odometry>("/arm_read",10, &RobotArm::arm_pose_cb,this);
        arm_pose init_pose = { //TODO ���޸ģ���kxdԼ����ʼλ��
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
        .cam_angle = 0.0,
        .paw_angle = 0.0
        };
        arm_control.push_back(init_pose);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher arm_pub;
    ros::Subscriber arm_sub;

    struct arm_pose{
        double x;
        double y;
        double z;
        double cam_angle;
        double paw_angle;
    };

    arm_pose current_arm_pose;

    std::vector<arm_pose> arm_control;

    void arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
    nav_msgs::Odometry target_arm_pose_sub;
    target_arm_pose_sub = *msg;
    current_arm_pose.cam_angle = target_arm_pose_sub.pose.pose.position.x;
    current_arm_pose.x = target_arm_pose_sub.pose.pose.orientation.x;
    current_arm_pose.y = target_arm_pose_sub.pose.pose.orientation.y;
    current_arm_pose.z = target_arm_pose_sub.pose.pose.orientation.z;
    current_arm_pose.paw_angle = target_arm_pose_sub.pose.pose.orientation.w;
    //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
    };

    void arm_pose_pub(){
        for (size_t i = 0; i < arm_control.size(); ++i) {
            nav_msgs::Odometry target_arm_pose_pub;
            while(arm_arrived(arm_control[i]) == false){
            target_arm_pose_pub.pose.pose.position.x = arm_control[i].cam_angle; // ע������Ӧ���� num.x ������ arm_control[num].cam_angle
            target_arm_pose_pub.pose.pose.orientation.x = arm_control[i].x;
            target_arm_pose_pub.pose.pose.orientation.y = arm_control[i].y;
            target_arm_pose_pub.pose.pose.orientation.z = arm_control[i].z;
            target_arm_pose_pub.pose.pose.orientation.w = arm_control[i].paw_angle;
            arm_pub.publish(target_arm_pose_pub); // ����Ϊ target_arm_pose_pub
            ROS_INFO("arm_moving_to_target");
            ros::Duration(0.1).sleep();
            }
        }
    };

    bool arm_arrived(arm_pose target_arm_pose){
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


};

class RobotArm_2{//���ݷ���
public:
    RobotArm_2(){
        arm_pub = nh.advertise<nav_msgs::Odometry>("/arm_ctrl", 10);
        arm_sub = nh.subscribe<nav_msgs::Odometry>("/arm_read",10, &RobotArm_2::arm_pose_cb,this);
        arm_pose init_pose = { //TODO ���޸ģ���kxdԼ����ʼλ��
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
        .cam_angle = 0.0,
        .paw_angle = 0.0
        };
        arm_control.push_back(init_pose);
    }

    // Ŀ��λ��
    typedef struct {
        double x;
        double y;
    } Position;

    // �ؽڽǶ�
    typedef struct {
        double theta1;
        double theta2;
        double theta3;
    } Angles;

    // �����е��ĩ�˵ĵ�ǰλ��
    Position forward_kinematics(Angles angles) {
        Position pos;
        double theta1_rad = DEG_TO_RAD(angles.theta1);
        double theta2_rad = DEG_TO_RAD(angles.theta2);
        double theta3_rad = DEG_TO_RAD(angles.theta3);

        // ����λ�� A, B, C
        double Ax = ARM1_LENGTH * cos(theta1_rad);
        double Ay = ARM1_LENGTH * sin(theta1_rad);

        double Bx = Ax + ARM2_LENGTH * cos(theta1_rad + theta2_rad);
        double By = Ay + ARM2_LENGTH * sin(theta1_rad + theta2_rad);

        double Cx = Bx + GRIPPER_LENGTH * cos(theta1_rad + theta2_rad + theta3_rad);
        double Cy = By + GRIPPER_LENGTH * sin(theta1_rad + theta2_rad + theta3_rad);

        pos.x = Cx;
        pos.y = Cy;
        return pos;
    }

    // ����Ŀ��λ�ú͵�ǰĩ��λ��֮������
    double compute_error(Position target, Position current) {
        return sqrt((target.x - current.x) * (target.x - current.x) + (target.y - current.y) * (target.y - current.y));
    }

    // ʹ���ݶ��½������ؽڽǶ�
    void inverse_kinematics(Position target, Angles *angles) {
        for (int i = 0; i < MAX_ITER; i++) {
            // ���㵱ǰ��λ��
            Position current = forward_kinematics(*angles);

            // �������
            double error = compute_error(target, current);
            if (error < TOLERANCE) {
                printf("ѭ��������%d\n", i);
                printf("������Ŀ��λ�ã����Ϊ: %.3f\n", error);
                return;
            }

            // �����ݶȣ�ʹ����ֵ΢�֣�
            double delta = 1e-6;
            Angles gradients;
            
            // ���� theta1 ���ݶ�
            angles->theta1 += delta;
            Position pos1 = forward_kinematics(*angles);
            gradients.theta1 = (compute_error(target, pos1) - error) / delta;
            angles->theta1 -= delta;

            // ���� theta2 ���ݶ�
            angles->theta2 += delta;
            Position pos2 = forward_kinematics(*angles);
            gradients.theta2 = (compute_error(target, pos2) - error) / delta;
            angles->theta2 -= delta;

            // ���� theta3 ���ݶ�
            angles->theta3 += delta;
            Position pos3 = forward_kinematics(*angles);
            gradients.theta3 = (compute_error(target, pos3) - error) / delta;
            angles->theta3 -= delta;

            // ʹ���ݶ�������ÿ���Ƕ�
            angles->theta1 -= LEARNING_RATE * gradients.theta1;
            angles->theta2 -= LEARNING_RATE * gradients.theta2;
            angles->theta3 -= LEARNING_RATE * gradients.theta3;
        }
        printf("�ﵽ���������������Ϊ: %.3f\n", compute_error(target, forward_kinematics(*angles)));
    }

private:
    ros::NodeHandle nh;
    ros::Publisher arm_pub;
    ros::Subscriber arm_sub;

    struct arm_pose{
        double x;
        double y;
        double z;
        double cam_angle;
        double paw_angle;
    };

    arm_pose current_arm_pose;

    std::vector<arm_pose> arm_control;

    void arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
    nav_msgs::Odometry target_arm_pose_sub;
    target_arm_pose_sub = *msg;
    current_arm_pose.cam_angle = target_arm_pose_sub.pose.pose.position.x;
    current_arm_pose.x = target_arm_pose_sub.pose.pose.orientation.x;
    current_arm_pose.y = target_arm_pose_sub.pose.pose.orientation.y;
    current_arm_pose.z = target_arm_pose_sub.pose.pose.orientation.z;
    current_arm_pose.paw_angle = target_arm_pose_sub.pose.pose.orientation.w;
    //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
    };

    void arm_pose_pub(){
        for (size_t i = 0; i < arm_control.size(); ++i) {
            nav_msgs::Odometry target_arm_pose_pub;
            while(arm_arrived(arm_control[i]) == false){
            target_arm_pose_pub.pose.pose.position.x = arm_control[i].cam_angle; // ע������Ӧ���� num.x ������ arm_control[num].cam_angle
            target_arm_pose_pub.pose.pose.orientation.x = arm_control[i].x;
            target_arm_pose_pub.pose.pose.orientation.y = arm_control[i].y;
            target_arm_pose_pub.pose.pose.orientation.z = arm_control[i].z;
            target_arm_pose_pub.pose.pose.orientation.w = arm_control[i].paw_angle;
            arm_pub.publish(target_arm_pose_pub); // ����Ϊ target_arm_pose_pub
            ROS_INFO("arm_moving_to_target");
            ros::Duration(0.1).sleep();
            }
        }
    };

    bool arm_arrived(arm_pose target_arm_pose){
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


};

