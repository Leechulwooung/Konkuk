#include <iostream>
#include <ros/ros.h>
#include <random>

#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

class Node1{
    protected:
        ros::NodeHandle m_nh;
        ros::Subscriber m_answerSub_ = m_nh.subscribe("answer", 10, &Node1::AnswerCallback, this);
        ros::Publisher m_correctPub_ = m_nh.advertise<std_msgs::Int64>("updown", 10);

        int m_number = 0;   // Created Random Number
        int m_answer = 0; // Sub number
        std_msgs::Int64 m_correct;
        bool m_answer_toggle = false;

    public:
        Node1();
        ~Node1();
        void AnswerCallback(const std_msgs::Int64ConstPtr& msg);
        void CreateNumber();
        void CreateUpdown();
        bool _finish = false;
};

Node1::Node1(){}

Node1::~Node1(){}

void Node1::AnswerCallback(const std_msgs::Int64ConstPtr& msg){
    m_answer = msg->data;
    m_answer_toggle = true;
}

void Node1::CreateNumber(){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(1,100);

    m_number = dis(gen);

    ROS_INFO("Number : %d", m_number);
}

void Node1::CreateUpdown(){
    
    if(m_answer_toggle == true){
        if(m_answer < m_number){
            m_correct.data = -1;
            ROS_INFO("UP!");
        }
        else if(m_answer > m_number){
            m_correct.data = 1;
            ROS_INFO("Down!");
        }
        else if(m_answer == m_number){
            m_correct.data = 0;
            ROS_INFO("Correct!");
            _finish = true;
        }

        m_correctPub_.publish(m_correct);
        m_answer_toggle = false;
        
    }

}

int main(int argc, char** argv){

    ros::init(argc, argv, "week0_node1");

    Node1 node1;

    node1.CreateNumber();

    ros::Rate loop_rate(10);

    while(ros::ok()){
        node1.CreateUpdown();
        ros::spinOnce();

        if(node1._finish == true){
            ros::shutdown();
        }

        loop_rate.sleep();
    }

    return 0;

}