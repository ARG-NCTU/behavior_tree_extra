#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_signal_state;
        ros::Subscriber sub_reset_signal;

        bool signal_running = false;
        bool signal_memo = false;
        bool buffer_state = false;
    public:
        bt::Condition condition;
        State();
        void conditionSet(bool state);
        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
        void resetCallback(const std_msgs::Bool::ConstPtr& msg);
        void stateEval();
};

State :: State() : condition(ros::this_node::getName()){
    sub_signal_state = n.subscribe<std_msgs::Bool>("waypoint/state", 1,  &State::stateCallback, this);
    sub_reset_signal = n.subscribe<std_msgs::Bool>("state_manager/reset", 1,  &State::resetCallback, this);
}

void State :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void State :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    signal_running = msg->data;
    return;
}

void State :: resetCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){ 
        buffer_state = false; 
    }
    return;
}

void State :: stateEval(){
    ros::Rate rate(30);
    if(signal_running){ buffer_state = true; }

    // cout << "running: " << signal_running;
    // cout << " memo: " << signal_memo;
    // cout << " finish: " << buffer_state << endl;

    conditionSet(buffer_state);
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_buffer");
    State step_buffer;
    while(ros::ok()){
        step_buffer.stateEval();
        ros::spinOnce();
    }
    return 0;
}