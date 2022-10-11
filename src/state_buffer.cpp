#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_waypoint_state;
        ros::Subscriber sub_reset_signal;

        bool waypoint_running = false;
        bool waypoint_memo = false;
        bool waypoint_state = false;
    public:
        bt::Condition condition;
        State();
        void conditionSet(bool state);
        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
        void resetCallback(const std_msgs::Bool::ConstPtr& msg);
        void stateEval();
};

State :: State() : condition(ros::this_node::getName()){
    sub_waypoint_state = n.subscribe<std_msgs::Bool>("waypoint/state", 1,  &State::stateCallback, this);
    sub_reset_signal = n.subscribe<std_msgs::Bool>("state_manager/reset", 1,  &State::resetCallback, this);
}

void State :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void State :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    waypoint_running = msg->data;
    return;
}

void State :: resetCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){ 
        waypoint_state = false; 
        waypoint_memo = false;
    }
    return;
}

void State :: stateEval(){
    ros::Rate rate(30);
    if(waypoint_running && !waypoint_state){ waypoint_memo = true; }
    if(waypoint_memo && !waypoint_running){ waypoint_state = true; }

    // cout << "running: " << waypoint_running;
    // cout << " memo: " << waypoint_memo;
    // cout << " finish: " << waypoint_state << endl;

    conditionSet(waypoint_state);
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_buffer");
    State single_waypoint;
    while(ros::ok()){
        single_waypoint.stateEval();
        ros::spinOnce();
    }
    return 0;
}