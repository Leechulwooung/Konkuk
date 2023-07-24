// ***기본적인 sub, pub 틀을 만들어 주고 그 후에 사이사이에 넣어야하는 알고리즘을 넣어준다.***

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "week1_node1");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    srand(time(NULL));
    int random_num = (rand()%100) + 1;

    ros::Publisher week1_reans_pub = nh.advertise<std_msgs::Int32>("week1_reans_topic", 1000);
    ros::Subscriber week1_ans_sub = nh.subscribe<std_msgs::Int32>("week1_ans_topic",1000,
                                                                 [&](const std_msgs::Int32::ConstPtr& msg){
                                                                    // ***callback 함수에 쓰이는 알고리즘은 함수를 따로 빼줘서 만들어줘서 사용한다.***
                                                                    ROS_INFO("Received %d, Random number : %d", msg->data, random_num);

                                                                        std_msgs::Int32 result;

                                                                        if(msg->data < random_num){
                                                                            ROS_INFO("Up");
                                                                            result.data = 1;
                                                                        }
                                                                        else if(msg->data > random_num){
                                                                            ROS_INFO("Down");
                                                                            result.data = -1;
                                                                        }
                                                                        else{
                                                                            ROS_INFO("Correct");
                                                                            result.data = 0;
                                                                            ros::shutdown();
                                                                        }

                                                                        week1_reans_pub.publish(result);
                                                                        
                                                                 });
    
    srand(time(0));


    // ros::spin() 은 rate 맞춰줘도 계속 돌아간다 주기 조절이 안 된다. 콜백함수를 계속 호출하고 있는 것.
    // ros::spinOnce() 는 함수가 돌 때 콜백함수를 호출 하는 거다.
    // 그래서 while문에 spinOnce() 넣고 rate.sleep 하면 주기적으로 도는게 된다.
    // pub에서 sub으로 보 낼 때 while(ros::ok()) 형식으로 무조건 쓰기!!!!!!!!
    while(ros::ok()){
        ros::spinOnce();    //
        loop_rate.sleep();
    }
    
    return 0;
}
