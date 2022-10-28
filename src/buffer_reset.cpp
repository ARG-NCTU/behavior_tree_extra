#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behavior_tree/behavior_tree.h>

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}

class Reset{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_reset;
        int state_memo = 0;
    public:
        bt::Action action;
        Reset();
        void publishClear(bool state);
        void init();
        void actionSet(int state);
        void actionSetLastState();
};

Reset :: Reset() : action(ros::this_node::getName()){
    pub_reset = n.advertise<std_msgs::Bool>("state_manager/reset", 10);
}

void Reset :: publishClear(bool state){
    ros::Rate rate(15);

    std_msgs :: Bool pub_msg_reset;
    pub_msg_reset.data = true;
    state_memo = 1;
    pub_reset.publish(pub_msg_reset);

    rate.sleep();
    return;
}

void Reset :: init(){
    state_memo = 0;
    return;
}

void Reset :: actionSetLastState(){
    ros::Rate rate(15);
    actionSet(1);
    rate.sleep();
    return;
}

void Reset :: actionSet(int state){
    switch(state){
        case 1:
            action.set_success();
            break;
        case 0:
            action.set_running();
            break;
        case -1:
            action.set_failure();
            break;
    }
    action.publish();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_reset");

    Reset action_reset;

    while(ros::ok()){
        if(action_reset.action.is_active() && action_reset.action.active_has_changed()){
            // ROS_INFO("Action: Start");
            action_reset.init();
            // action_reset.actionSet(0);
            action_reset.publishClear(true);
            action_reset.actionSet(1);
        }   
        else if(action_reset.action.active_has_changed() && !(action_reset.action.is_active())){
            // ROS_INFO("Action: Done");
            // action_reset.actionSet(1);
        }
        else if(action_reset.action.is_active()){
            // ROS_INFO("Action: Running");
        }else{
            action_reset.actionSetLastState();
        }
        ros::spinOnce();
    }
    return 0;
}

