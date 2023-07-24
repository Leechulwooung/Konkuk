// ***기본적인 sub, pub 틀을 만들어 주고 그 후에 사이사이에 넣어야하는 알고리즘을 넣어준다.***

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void callback(const std_msgs::Int32::ConstPtr& msg){
    std_msgs::String result;

    // ***callback 함수에 쓰이는 알고리즘은 함수를 따로 빼줘서 만들어줘서 사용한다.***
    if(msg->data == 1){
        //ROS_INFO("Up");
        result.data = "Up";
    }
    else if(msg->data == -1){
        //ROS_INFO("Down");
        result.data = "Down";
    }
    else{
        //ROS_INFO("Correct");
        result.data = "Correct";
        ros::shutdown();
    }

    ROS_INFO("Result : %s", result.data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "week1_node2");
    ros::NodeHandle nh;

    ros::Publisher week1_ans_pub = nh.advertise<std_msgs::Int32>("week1_ans_topic", 1000);
    // nh.subscriber 뒤에 <std_msgs::Int32>는 생략해 줘도 된다. callback 함수에서 <std_msgs::Int32>를 부르기 때문에
    // callbacm 앞에 & -> 레퍼런스 처리 해준다.
    ros::Subscriber week1_reans_sub = nh.subscribe<std_msgs::Int32>("week1_reans_topic",1000, &callback);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::Int32 msg;
        std::cout << "Enter a number : ";
        std::cin >> msg.data;
        week1_ans_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
