#include "ros/console.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include <cmath>
#include <iostream>
#include <state_machine_.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tools.h"
#include <vector>
#include "robo_arm.h"

nav_msgs::Odometry cur_pose;
nav_msgs::Odometry pub_target_pose;
nav_msgs::Odometry set_target_pose;
int set_tar_pose_index = 0;
float set_tar_yaw = 0;
float eg = 0;
bool add_flag = 0;
int move_index;
int can_move = 0;
nav_msgs::Odometry lock_pose;
int lock_flag = 0;
int cir_index;

int found_color = 0;
int arm_ctrl_num = 0;

int state_judge = 0;

int istest = 0;

void move_stop () {
    can_move = 0;
}

void move_start () {
    can_move = 1;
}

void move_lock() {
    can_move = 2;
}

void move(int index){
    
    
        set_target_pose.pose.pose.position.x = set_tar_poses[index].x;
        set_target_pose.pose.pose.position.y = set_tar_poses[index].y;
        //ROS_INFO("set_target_pose.pose.pose.position: (%.2f, %.2f)", set_target_pose.pose.pose.position.x, set_target_pose.pose.pose.position.y);

        set_tar_yaw = set_tar_poses[index].yaw;
        //ROS_INFO("currx is %.5f",cur_pose.pose.pose.position.x);
        pub_target_pose = trans_global2car( set_target_pose, cur_pose, set_tar_yaw);
        
        //ROS_INFO("pubtarx: (%.2f)", pub_target_pose.pose.pose.position.x);
        
        //target_pose.pose.pose.position.x-=0.5;
        
        //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
        // double x_error = set_target_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
        // double y_error = set_target_pose.pose.pose.position.y - cur_pose.pose.pose.position.y;
        // //ROS_INFO("error_y:%d",cur_pose_is_ok);
        // ROS_INFO("index:%d",index); 
        // if(cur_pose_is_ok == 1){
        //     if(fabs(x_error)<0.05 && fabs(y_error)<0.05 && yaw_is_ok == 1){
        //         yaw_is_ok = 0;
        //         ROS_INFO("arrived!");
        //         if(index < set_tar_poses.size() - 1) {
        //             index ++;
        //         }
        //     //currentState = State::COMPLETE;
        //     }
        // ROS_INFO("x=%.2f,y=%.2f,z=%.2f",
        // target_pose.pose.pose.position.x,target_pose.pose.pose.position.y,target_pose.pose.pose.position.z);
        //target_pose.pose.pose.orientation = cur_pose.pose.pose.orientation;
        // }
   
}
bool car_arrive (int index) {
    nav_msgs::Odometry temp_pose;
    temp_pose.pose.pose.position.x = set_tar_poses[index].x;
    temp_pose.pose.pose.position.y = set_tar_poses[index].y;
    double x_error = temp_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
    double y_error = temp_pose.pose.pose.position.y - cur_pose.pose.pose.position.y;
    if(fabs(x_error)<0.08 && fabs(y_error)<0.08 &&   yaw_is_ok_for_circular == 1){
                yaw_is_ok_for_circular = 0;
                ROS_INFO("arrived  !");
                return 1;
            //currentState = State::COMPLETE;
    } else return 0;
}


bool car_arrive_circular (int index) {
    nav_msgs::Odometry temp_pose;
    temp_pose.pose.pose.position.x = set_tar_poses[index].x;
    temp_pose.pose.pose.position.y = set_tar_poses[index].y;
    double x_error = temp_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
    double y_error = temp_pose.pose.pose.position.y - cur_pose.pose.pose.position.y;
    if(fabs(x_error)<0.01 && fabs(y_error)<0.01 && yaw_is_ok == 1){
                yaw_is_ok = 0;
                ROS_INFO("arrived circaulr!");
                return 1;
            //currentState = State::COMPLETE;
    } else return 0;
}

void RobotFSM::processEvent(Event event) {
    nav_msgs::Odometry return_value;
    switch (currentState) {
        case State::INIT://1、用于检查机器状态

            robot_arm_.test();
            //robot_arm_.test();
            
            //currentState = State::DELIVER_TO_PROCESSING;

            //currentState = State::DELIVER_TO_STORAGE;
            if(istest){
                //currentState = State::TEST_MOVEMENT;

                //currentState = State::READY_ARM; //静态全流程测试

                move_lock();

            }else{
                currentState = State::DELIVER_TO_PROCESSING;
            }
            
            break;

        case State::DELIVER_TO_PROCESSING://2、开到粗加工区s
            move_start();
           if (add_flag != 1) {
            move_index = add_tar_pose(0.25, -0.06, 3.1415/2);
            //ROS_INFO("move_index111: %d", move_index);
            //add_tar_pose(0.29, -0.2, 0);
            //add_tar_pose(0.8, -0.2, 0);
            add_tar_pose(1.92, -0.11, 3.1415/2);
            //ROS_INFO("move_index2222: %d", move_index);
            add_tar_pose(2.04, -0.2, 3.1415/3);
            add_tar_pose(2, -0.2, 3.1415/4);
            add_tar_pose(1.99, -0.2, 3.1415/5);
            add_tar_pose(1.98, -0.2, 0);
            add_tar_pose(1.98, -0.4, 0);
            add_tar_pose(1.9, -0.6, 0);
            add_tar_pose(1.8, -0.8, 0);
            cir_index = add_tar_pose(1.86,-0.93,-0.01); //succ
            //ROS_INFO("circleindex = %d",cir_index);
            add_flag = 1;
            }
            move(move_index);
            //ROS_INFO("move_index: %d", move_index);
            if (car_arrive(move_index)) {
                if(move_index == cir_index) {
                    if (car_arrive_circular(cir_index)) {
                        move_index ++;
                    }
                }else {
                    move_index ++;
                }
                
                //ROS_INFO("move_index: %d", move_index);
            } 
            if(move_index == set_tar_poses.size()) {

                add_flag = 0;

                currentState = State::READY_ARM;

                //currentState = State::DELIVER_TO_STORAGE;

                move_lock();
                ROS_INFO("DELIVER_TO_PROCESSING finish!");
            }
            break;
        case State::READY_ARM:
            robot_arm_.test2();
            currentState = State::LOOKING_FOR_CIRCLE;//2.5 展开机械臂 
            break;
        case State::LOOKING_FOR_CIRCLE://3、找圆 
            handleTest_vision(event);
            break;
        case State::MOVE_GREEN:
            handle_move_green();
            break;
        case State::MOVE_BLUE:
            handle_move_blue();
            break;
        case State::STORE_FIRST_BATCH: //4、放置 
                found_color =0;
                arm_ctrl_num = 0;
            robot_arm_.put_down('r');
            robot_arm_.put_down('g');
            robot_arm_.put_down('b');
            currentState = State::FETCH_FIRST_BATCH;
            ROS_INFO("Fetching second batch of materials.");
            if(state_judge == 1){
                if(istest){
                    currentState = State::COMPLETE;
                }else{
                    currentState = State::COMPLETE;
                }
                ROS_INFO("returning to start");
            }
            break;
        case State::FETCH_FIRST_BATCH://5、再拿起来 
            robot_arm_.pickup('b');
            robot_arm_.pickup('g');
            robot_arm_.pickup('r');
            if(state_judge == 0){
                if(!istest){//isnottest
                    currentState = State::DELIVER_TO_STORAGE;
                    state_judge ++;
                }else{//istest
                    currentState = State::READY_ARM;
                    ROS_INFO("into ready arm2");
                    state_judge ++;
                }
            }
            break;
        case State::DELIVER_TO_STORAGE://6、开到暂存区
            move_start();
           if (add_flag != 1) {
            move_index = 
            add_tar_pose(1.85,-1.83,0);
            add_tar_pose(1.86,-1.85,-3.1415/7);

            add_tar_pose(1.85,-1.88,-3.1415/5);

            add_tar_pose(1.7,-1.89,-3.1415/4);
            add_tar_pose(1.7,-1.9,-3.1415/3);

            add_tar_pose(1.7,-1.92,-3.1415/2);
            //add_tar_pose(1.4,-1.89,-3.1415/2);


            add_tar_pose(1.2,-1.8,-3.1415/2);
            add_tar_pose(0.98,-1.86,-3.1415/2);
            cir_index = add_tar_pose(1,-1.85,-3.1415/2);
            add_flag = 1;
            }

            move(move_index);
            //ROS_INFO("move_index: %d", move_index);
            if (car_arrive(move_index)) {
                if(move_index == cir_index) {
                    if (car_arrive_circular(cir_index)) {
                        move_index ++;
                    }
                }else {
                    move_index ++;
                }
                
                //ROS_INFO("move_index: %d", move_index);
            } 
            if(move_index == set_tar_poses.size()) {
                add_flag = 0;
                currentState = State::READY_ARM;
                move_stop();
                ROS_INFO("DELIVER_TO_STORAGE finish!");
            }
            break;
        case State::RETURN_TO_START:
            move_start();
           if (add_flag != 1) {
            move_index = 
                add_tar_pose(0.81,-1.82,-3.1415/2);
                add_tar_pose(0.88,-1.63,-3.1415);
                add_tar_pose(0.93,-0.04,-3.1415);
                add_tar_pose(0.2,-0.2,-3.141);
                add_tar_pose(0.2,-0.2,-3.141);
                cir_index = add_tar_pose(0,-0,-3.141);
                add_flag = 1;
            }
            move(move_index);
            //ROS_INFO("move_index: %d", move_index);
            if (car_arrive(move_index)) {
                if(move_index == cir_index) {
                    if (car_arrive_circular(cir_index)) {
                        move_index ++;
                    }
                }else {
                    move_index ++;
                }
                ROS_INFO("move_index: %d", move_index);
            } 
            if(move_index == set_tar_poses.size()) {

                add_flag = 0;

                currentState = State::COMPLETE;
                move_lock();
                ROS_INFO("DELIVER_TO_PROCESSING finish!");
            }
            break;
        case State::COMPLETE:
            handleComplete(event);
            break;
        }
}

// void RobotFSM::processEvent(Event event) {
//     handleInit(event);
//     ROS_INFO("process_init");
// }

void RobotFSM::handleTest(Event event){
    static int move_index;
    static int add_flag = 0;

    robot_arm_.test();


    // if (add_flag != 1) {
    // move_index = add_tar_pose(1.85, -0.29, 0);
    // add_tar_pose(1.88,-0.92,0);

    // // add_tar_pose(1.85,-1.88,0);
    // // add_tar_pose(1.64,-1.81,-3.1415/2);
    // // add_tar_pose(0.98,-1.84,-3.1415/2);
    // // add_tar_pose(0.98,-1.84,-3.1415/2);
    // // add_tar_pose(0.84,-1.78,-3.1415/2);
    // // add_tar_pose(0.84,-1.6,-3.1415/2);
    // //  add_tar_pose(0.84,-1,-3.1415/2);
    // // add_tar_pose(0.88,-0.16,-3.1415/2);
    // // add_tar_pose(0.2,-0.2,-3.1415/2);
    // // add_tar_pose(0.2,-0.2,0);
    // // add_tar_pose(0,-0,0);

    // // add_flag = 1;
    // }
    // //add_tar_pose(0.2,0.2, 3.1415);//TODO 输入正确的坐标
    // move(move_index);
    // ROS_INFO("move_index: %d", move_index);
    // if (car_arrive(move_index)) {
    //     move_index ++;
    //     ROS_INFO("move_index: %d", move_index);
    // } 
    // if(move_index == set_tar_poses.size()) {
    //     add_flag = 0;
    //     //currentState = State::STORE_FIRST_BATCH;
    //     ROS_INFO("test finish!");
    // }
}
void RobotFSM::handle_move_green(){
    static int move_index;
    static int add_flag = 0;
    robot_arm_.handle_move_green();
    currentState = State::LOOKING_FOR_CIRCLE;
}

void RobotFSM::handle_move_blue(){
    static int move_index;
    static int add_flag = 0;
    robot_arm_.handle_move_blue();
    currentState = State::LOOKING_FOR_CIRCLE;
}


void RobotFSM::handleTest_vision(Event event){
    if(found_color == 1){
        arm_ctrl_num = 1;
    }
    if(found_color == 2){
        arm_ctrl_num = 2;
    }
    if(found_color == 3){
        arm_ctrl_num = 3;
    }
    //ROS_INFO("color_found = %d",arm_ctrl);
    switch (arm_ctrl_num) {
        case 0:
            found_color =robot_arm_.test_vision('b');
            if(found_color == 1){
                currentState = State::MOVE_GREEN;
            }
        break;
        case 1:
            found_color =robot_arm_.test_vision('g');
            if(found_color == 2){
                currentState = State::STORE_FIRST_BATCH;
            }
        break;
        case 2:
            // arm_ctrl =robot_arm_.test_vision('r');
            // if(found_color == 3){
            //     //currentState = State::COMPLETE;
            // }
        break;
        case 3:
        break;
    
    }
    
}

void RobotFSM::handleInit(Event event){
    // ROS_INFO("MISSION_START!");
    // //while(cur_pose.pose.pose.position.x == 0);
    // //{
    //     ROS_INFO("Waiting for SLAM to initialize...");
    // //}
    // ROS_INFO("Slam init finished!");//雷达完成初始化
    // ROS_INFO("Init finished!");

    robot_arm_.test();

    //TODO检查各个模块，还缺少检查地盘和检查机械臂的代码，需要标志位的检查
    currentState = State::DELIVER_TO_PROCESSING;
    //currentState = State::STORE_FIRST_BATCH;


    //currentState = State::LOOKING_FOR_CIRCLE;
    
    //currentState = State::COMPLETE;
}

void RobotFSM::handleReadQRCode(Event event) {
    //fechting_order = get_QRCODE_order();
    nh_.setParam("fechting_order",fechting_order);
    //加入判断，如果参数服务器中的fechting_order为0，则重新执行扫码程序，超过五秒还没有扫描到，则发出警报，随后退出
    //如果参数服务器中的fechting_order不为0，则继续执行以下程序
    if (event == Event::QR_CODE_READ) {//在当前状态
        currentState = State::FETCH_FIRST_BATCH;
        ROS_INFO("Fetching first batch of materials.");
    }
}

// void RobotFSM::handleDeliverToProcessing(Event event) { //步骤2，移动到粗加工区
//     static int move_index;
//     static int add_flag = 0;
//     move_start();

//     // robot_arm_.test();


//     if (add_flag != 1) {
//     move_index = add_tar_pose(0.29, -0.29, 0);
//     ROS_INFO("move_index111: %d", move_index);
     
//     add_tar_pose(0.29, -0.2, 0);
     
//     add_tar_pose(0.8, -0.2, 0);
//     add_tar_pose(1.3, -0.2, 0);

//     //add_tar_pose(0, 0, 0);
//     ROS_INFO("move_index2222: %d", move_index);
    
    
//     add_tar_pose(1.85, -0.29, 0);

//     add_tar_pose(1.88,-0.92,-0.01); //succ

//     // add_tar_pose(1.85,-1.88,0);
//     // add_tar_pose(1.64,-1.81,-3.1415/2);
//     // add_tar_pose(0.98,-1.84,-3.1415/2);
//     // add_tar_pose(0.98,-1.84,-3.1415/2);
//     // add_tar_pose(0.84,-1.78,-3.1415/2);
//     // add_tar_pose(0.84,-1.6,-3.1415/2);
//     //  add_tar_pose(0.84,-1,-3.1415/2);
//     // add_tar_pose(0.88,-0.16,-3.1415/2);
//     // add_tar_pose(0.2,-0.2,-3.1415/2);
//     // add_tar_pose(0.2,-0.2,0);
//     // add_tar_pose(0,-0,0);

//     add_flag = 1;
//     }
//     //add_tar_pose(0.2,0.2, 3.1415);//TODO 输入正确的坐标
//     move(move_index);
//     ROS_INFO("move_index: %d", move_index);
//     if (car_arrive(move_index)) {
//         move_index ++;
//         ROS_INFO("move_index: %d", move_index);
//     } 
//     if(move_index == set_tar_poses.size()) {
//         add_flag = 0;
//         currentState = State::READY_ARM;
//         move_stop();
//         ROS_INFO("test finish!");
//     }
//}

// void RobotFSM::handleStoreFirstBatch(Event event) {
//     // if (event == Event::BATCH_STORED) {
    
        
//         //robot_arm_.choose('r');
//         robot_arm_.put_down('r');
//         robot_arm_.put_down('g');
//          robot_arm_.put_down('b');
//          //robot_arm_.put_down('g');
//         // robot_arm_.put_down('r');
//         // robot_arm_.choose('g');
//         // robot_arm_.put_down('g');
//         // robot_arm_.choose('b');
//         // robot_arm_.put_down('b');



//         //int temp_index = 0;
//         //temp_index = add_tar_pose(2, 2, 3.1415);//红原前面的坐标
//         //move(temp_index);
//         //robot_arm_.choose('b');
//         //robot_arm_.put_down('g''r');
//         //temp_index = add_tar_pose(2, 2, 3.1415);//蓝原前面的坐标
//         //move(temp_index);
//         //robot_arm_.choose('g');
//         //robot_arm_.put_down('b''r');
//         //temp_index = add_tar_pose(2, 2, 3.1415);//绿原前面的坐标
//         //move(temp_index);

        
//         currentState = State::COMPLETE;

//         //currentState = State::FETCH_SECOND_BATCH;
//         ROS_INFO("Fetching second batch of materials.");
//     // }
//     // currentState = State::COMPLETE;
// }

// void RobotFSM::handleFetchSecondBatch(Event event) {
//     if (event == Event::BATCH_FETCHED) {
//         currentState = State::DELIVER_TO_PROCESSING;
//         ROS_INFO("Delivering second batch to processing area.");
//     }
// }

// void RobotFSM::handleStoreSecondBatch(Event event) {
//     if (event == Event::BATCH_STORED) {
//         currentState = State::RETURN_TO_START;
//         ROS_INFO("Returning to start.");
//     }
// }

// void RobotFSM::handleReturnToStart(Event event) {
//     if (event == Event::RETURNED_TO_START) {
//         currentState = State::COMPLETE;
//         ROS_INFO("Task complete.");
//     }
// }

void RobotFSM::handleComplete(Event event) {
    // Task is complete, no further action needed
    // target_pose = cur_pose;
    ROS_INFO("Task completed.");
}


