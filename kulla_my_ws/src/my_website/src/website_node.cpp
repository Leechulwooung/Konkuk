//pub node

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <std_msgs/Bool.h>
//#include <time.h>

class Wedsite{
    protected:
        ros::NodeHandle g_nh;
        ros::Publisher goal_choice_first_pub = g_nh.advertise<std_msgs::Int32>("website_first_topic", 100);
        ros::Publisher goal_choice_second_pub = g_nh.advertise<std_msgs::Int32>("website_second_topic", 100);
        ros::Publisher goal_choice_third_pub = g_nh.advertise<std_msgs::Int32>("website_click_topic", 100);

        ros::Publisher servo1_pub = g_nh.advertise<std_msgs::Int32>("servo1", 100);
        ros::Publisher servo2_pub = g_nh.advertise<std_msgs::Int32>("servo2", 100);
        //ros::Subscirber waypoint_sub = g_nh.subscribe<std_msgs::Int32>("topic", 100, &Wedsite::WaypointCallback, this);
        //ros::Subscirber servo_sub = g_nh.subscribe<std_msgs::Int32>("topic", 100, &Wedsite::ServoCallback, this);

        int servo_answer = 0;

        std_msgs::Int32 g_answer;
        std_msgs::Int32 servo1;
        std_msgs::Int32 servo2;
 
        //int waypointNum = 0; // 웹사이트에서 거점에서 받아온 정보 데이터를 asign하는 변수

    public:
        Wedsite();
        ~Wedsite();
        void InsertNumber();
        void servo_open_close();
        //void WaypoinCallback(const std_msgs::Int32ConstPtr& msg);
        //void ServoCallback(const std_msgs::Int32ConstPtr& msg);
        bool _finish = false;
};

Wedsite::Wedsite(){}; // 생성자 : 클래스로 객체가 선언되면 그냥 자동으로 수행되는 함수 -> 1. 초기화 

Wedsite::~Wedsite(){};  // 소멸자 : 클래스로 객체가 종료되면 그냥 자동으로 수행되는 함수 -> 동적할당 : new / delete -> 생성자에서 int *ptr = new int : delete를 선언해주는게 소멸자

/*
// 웹사이트에서 받아온 거점 callback
void Wedsite::WaypointCallback(const std_msgs::Int32ConstPtr& msg){
    waypointNum = msg->data;
}

// 웹사이트에서 서보가 callback
void Wedsite::ServoCallback(const std_msgs::Int32ConstPtr& msg){
    servo_answer = msg->data;
}
*/

void Wedsite::InsertNumber(){

    std::cout << "Enter goal 1~8 : ";
    std::cin >> g_answer.data;                 // 지금은 cin으로 했지만 이 것을 웹 사이트 노드에서 받아 오는 값으로 바꿔주면 될듯?!?!?!
    // g_answer.data => waypointNum 으로 바꿔서 쓴다.

    ///////////////입력값 확인용///////////////
    if(g_answer.data == 1){
        ROS_INFO("Go to A");
    }
    else if(g_answer.data == 2){
        ROS_INFO("Go to B");
    }
    else if(g_answer.data == 3){
        ROS_INFO("Go to C");
    }
    else if(g_answer.data == 4){
        ROS_INFO("Go to D");
    }

    goal_choice_first_pub.publish(g_answer);

    if(g_answer.data == 5){
        ROS_INFO("Go to A");
    }
    else if(g_answer.data == 6){
        ROS_INFO("Go to B");
    }
    else if(g_answer.data == 7){
        ROS_INFO("Go to C");
    }
    else if(g_answer.data == 8){
        ROS_INFO("Go to D");
    }

    goal_choice_second_pub.publish(g_answer);
    //////////////////////////////////////////

    if(g_answer.data == 9){
        ROS_INFO("Go to next goal");
    }

    goal_choice_third_pub.publish(g_answer);


    if(g_answer.data > 9){
        _finish = true;
    }

}

void Wedsite::servo_open_close(){
    
    std::cout << "Enter a servo1 num : ";
    std::cin >> servo_answer;

    servo1.data = 180;
    servo2.data = 180;
    
    // servo1에 대한 조건문
    if(servo_answer == 10){
        servo1.data = 0;
        ROS_INFO("Open servo1");
    }
    else if(servo_answer == 11){
        servo1.data = 180;
        ROS_INFO("Cloase servo1");
    }

    // servo2에 대한 조건문
    if(servo_answer == 12){
        servo2.data = 0;
        ROS_INFO("Open servo2");
    }
    else if(servo_answer == 13){
        servo2.data = 180;
        ROS_INFO("Close servo2");
    }

    if(servo_answer > 8){
        _finish = true;
    }  
    
    servo1_pub.publish(servo1); // servo1에 publish
    servo2_pub.publish(servo2); // servo2에 publish
}

int main(int argc, char **argv){
    ros::init(argc, argv, "website_node");

    ros::NodeHandle nh;

    Wedsite goal_choice;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        //goal_choice.servo_open_close();

        goal_choice.InsertNumber();
        
        if(goal_choice._finish == true){
            ros::shutdown();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
}