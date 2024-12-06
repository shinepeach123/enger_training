#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "state_machine_.h"
#include <ros/ros.h>
#include "tools.h"


//nav_msgs::Odometry cur_pose;
// nav_msgs::Odometry target_pose;

//校赛逻辑：event=BATCH_STORED(处理放置)
//state = INIT->DELIVER_TO_PROCESSING->STORE_FIRST_BATCH->COMPLETE
void cur_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose = *msg;
    //ROS_INFO(":cur_pose.pose.pose.position.x = %.5f",cur_pose.pose.pose.position.x);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    RobotFSM fsm;    
    ros::Subscriber curr_pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("/aft_mapped_to_init", 10, cur_pose_cb);
    ros::Publisher tar_pose_pub = nh.advertise<nav_msgs::Odometry>("/target_pose", 10);
    set_target_pose.pose.pose.position.x=0;
    fsm.set_state(State::INIT);
    //fsm.set_state(State::TEST);
    while(ros::ok()){
        fsm.processEvent(Event::BATCH_DELIVERED);
        pub_target_pose.pose.pose.position.z =  can_move;
        tar_pose_pub.publish(pub_target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
