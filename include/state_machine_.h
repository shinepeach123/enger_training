#ifndef STATE_MACHINE__H
#define STATE_MACHINE__H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <robo_arm.h>

extern nav_msgs::Odometry cur_pose;
extern nav_msgs::Odometry pub_target_pose;
extern nav_msgs::Odometry set_target_pose;
extern int can_move;
extern nav_msgs::Odometry lock_pose;
extern int lock_flag;


enum class State {
    INIT,
    READ_QR_CODE,
    FETCH_FIRST_BATCH,
    DELIVER_TO_PROCESSING,
    STORE_FIRST_BATCH,
    FETCH_SECOND_BATCH,
    STORE_SECOND_BATCH,
    RETURN_TO_START,
    COMPLETE,
    TEST,
    TEST_VISION,
    LOOKING_FOR_CIRCLE,
    MOVE_GREEN,
    MOVE_BLUE,
    READY_ARM,
    DELIVER_TO_STORAGE,
    TEST_MOVEMENT
};

enum class Event {
    QR_CODE_READ,
    BATCH_FETCHED,
    BATCH_DELIVERED,
    BATCH_STORED,
    RETURNED_TO_START,
    TASK_COMPLETE,
    TEST
};

class RobotFSM {
public:
    RobotFSM() : currentState(State::INIT), robot_arm_() {}

    void processEvent(Event event);


    void set_state(State state) {
        currentState = state;
    }
    
private:  
    int fechting_order;

    ros::NodeHandle nh_;

    State currentState;

    RobotArm robot_arm_;

    void handleTest(Event event);

    void handleTest_vision(Event event);

    void handleInit(Event event);

    void handleReadQRCode(Event event);

    void handleFetchFirstBatch(Event event);

    void handleDeliverToProcessing(Event event);

    void handleStoreFirstBatch(Event event);

    void handleFetchSecondBatch(Event event);

    void handleStoreSecondBatch(Event event);

    void handleReturnToStart(Event event);

    void handleComplete(Event event);

    void handle_move_blue();

    void handle_move_green();
};

#endif
