#include <iostream>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

class Node2{
    protected:
        ros::NodeHandle m_nh;
        ros::Publisher m_answerPub_ = m_nh.advertise<std_msgs::Int64>("answer", 10);
        ros::Subscriber m_correctSub_ = m_nh.subscribe("updown", 10, &Node2::CorrectCallback, this);

        std_msgs::Int64 m_answer;
        int m_correct = 0;

    public:
        Node2();
        ~Node2();
        void CorrectCallback(const std_msgs::Int64ConstPtr &msg);
        void InsertNumber();
        void PrintUpDown();
        bool _finish = false;
};

Node2::Node2(){};

Node2::~Node2(){};

void Node2::CorrectCallback(const std_msgs::Int64ConstPtr &msg){

    m_correct = msg->data;
    Node2::PrintUpDown();
    if(_finish == false){
        Node2::InsertNumber();
    }
}

void Node2::InsertNumber(){

    std::cout << "Insert Number : ";
    std::cin >> m_answer.data;

    if(m_answer.data > 100){
        _finish = true;
    }
    
    m_answerPub_.publish(m_answer);

}

void Node2::PrintUpDown(){
    if(m_correct == 0){
        ROS_INFO(" Correct! ");
        _finish = true;
    }
    else if(m_correct == -1){
        ROS_INFO("UP!");
    }
    else if(m_correct == 1){
        ROS_INFO("Down!");
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "week0_node2");

    ros::NodeHandle nh;

    Node2 node2;

    ros::Rate loop_rate(10);

    node2.InsertNumber();

    while(ros::ok()){
        
        ros::spinOnce();

        if(node2._finish == true){
            ros::shutdown();
        }

        loop_rate.sleep();
    }

    return 0;

}
