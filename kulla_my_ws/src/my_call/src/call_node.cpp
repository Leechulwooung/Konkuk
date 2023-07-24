//pub node

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <std_msgs/Bool.h>
//#include <time.h>

class Call{
    protected:
        ros::NodeHandle g_nh;
        ros::Publisher goal_choice_pub = g_nh.advertise<std_msgs::Int32>("call_topic", 100);

        std_msgs::Int32 g_answer;

    public:
        Call();
        ~Call();
        void InsertNumber();
        bool _finish = false;
};

Call::Call(){}; // 생성자 : 클래스로 객체가 선언되면 그냥 자동으로 수행되는 함수 -> 1. 초기화 

Call::~Call(){};  // 소멸자 : 클래스로 객체가 종료되면 그냥 자동으로 수행되는 함수 -> 동적할당 : new / delete -> 생성자에서 int *ptr = new int : delete를 선언해주는게 소멸자

void Call::InsertNumber(){

    std::cout << "Enter call goal 1~4 : ";
    std::cin >> g_answer.data;                 // 지금은 cin으로 했지만 이 것을 웹 사이트 노드에서 받아 오는 값으로 바꿔주면 될듯?!?!?!

    ///////////////입력값 확인용///////////////
    if(g_answer.data == 1){
        ROS_INFO("Go to 1");
    }
    else if(g_answer.data == 2){
        ROS_INFO("Go to 2");
    }
    else if(g_answer.data == 3){
        ROS_INFO("Go to 3");
    }
    else if(g_answer.data == 4){
        ROS_INFO("Go to 4");
    }
    //////////////////////////////////////////
    
    if(g_answer.data > 4){
        _finish = true;
    }
    

    goal_choice_pub.publish(g_answer);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "call_node");

    ros::NodeHandle nh;

    Call goal_choice;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        goal_choice.InsertNumber();
        
        if(goal_choice._finish == true){
            ros::shutdown();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
}