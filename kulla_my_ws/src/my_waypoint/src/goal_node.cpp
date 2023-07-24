//sub node

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
//#include <time.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <math.h>

class My_waypoint{
  protected:
    ros::NodeHandle g_nh;
    ros::Subscriber website_first_sub = g_nh.subscribe<std_msgs::Int32>("website_first_topic",100, &My_waypoint::website_first_callback, this);
    ros::Subscriber website_second_sub = g_nh.subscribe<std_msgs::Int32>("website_second_topic",100, &My_waypoint::website_second_callback, this);
    ros::Subscriber website_click_sub = g_nh.subscribe<std_msgs::Int32>("website_click_topic",100, &My_waypoint::website_click_callback, this);
    ros::Subscriber call_sub = g_nh.subscribe<std_msgs::Int32>("call_topic",100, &My_waypoint::call_callback, this);

    ros::Publisher goal_pub = g_nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);

    geometry_msgs::PoseStamped first_pose_stamped;
    geometry_msgs::PoseStamped second_pose_stamped;
    geometry_msgs::PoseStamped call_pose_stamped;

    int website_first_answer = 0;
    int website_second_answer = 0;
    int website_click_answer = 0;   
    int call_answer = 0;

    bool web_first_toggle = false;
    bool web_second_toggle = false;
    bool call_toggle = false;

    double a_x = -2.3726302345039585e-07, a_y = -1.9903066158294678, a_z = 0.01561849853981587, a_w = 0.9998780238125857;
    double b_x = 1.9903068542480469, b_y = 0.0074262963607907295, b_z = 0.9999999999999971, b_w = 7.549789954891874e-08; 
    double c_x = 2.390336817370553e-07, c_y = 2.005160093307495, c_z = 0.7071067544940086, c_w = 0.7071068078790854; 
    double d_x = -2.005159854888916, d_y = 0.007426772732287645, d_z = -0.7071067966408575, d_w = 0.7071067657322372; 

  public:
    My_waypoint();
    ~My_waypoint();
    void website_first_callback(const std_msgs::Int32ConstPtr& msg);
    void website_second_callback(const std_msgs::Int32ConstPtr& msg);
    void website_click_callback(const std_msgs::Int32ConstPtr& msg);
    void call_callback(const std_msgs::Int32ConstPtr& msg);
    void goal_select();
    void call_goal_select();
    void short_distance();

    // 한 번만 돌게 하기 위해서 추가한 토글 변수
    bool loop_only_once_website_toggle = false;
    bool loop_only_once_call_toggle = false;
};

My_waypoint::My_waypoint(){}

My_waypoint::~My_waypoint(){}

// 사용자1에게 받는 목적지 거점
void My_waypoint::website_first_callback(const std_msgs::Int32ConstPtr& msg){
  website_first_answer = msg->data;
  web_first_toggle = true;

  // 토픽이 들어오면 토글 true
  loop_only_once_website_toggle = true;
}

// 사용자2에게 받는 목적지 거점
void My_waypoint::website_second_callback(const std_msgs::Int32ConstPtr& msg){
  website_second_answer = msg->data;
  web_second_toggle = true;

  // 토픽이 들어오면 토글 true
  loop_only_once_website_toggle = true;
}

// 목적지 1에서 목적지 2로 가게 하는 callback함수 : 최종 목적지
void My_waypoint::website_click_callback(const std_msgs::Int32ConstPtr& msg){
  website_click_answer = msg->data;
}


// 사용자2에게 받는 경유지
void My_waypoint::call_callback(const std_msgs::Int32ConstPtr& msg){
  call_answer = msg->data;
  call_toggle = true;

  // 토픽이 들어오면 토글 true
  loop_only_once_call_toggle = true;
}

// 배송 보낼 거점 설정
void My_waypoint::goal_select(){

  // 사용자1에게 받는 목적지 거점 좌표  
  if(web_first_toggle == true){
    
    if(website_first_answer == 1){

      //first_pose_stamped.header.stamp = ros::Time::now();
      first_pose_stamped.header.frame_id = "map";
      first_pose_stamped.pose.position.x = a_x;
      first_pose_stamped.pose.position.y = a_y;
      first_pose_stamped.pose.orientation.z = a_z;
      first_pose_stamped.pose.orientation.w = a_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

    }
    else if(website_first_answer == 2){

      //first_pose_stamped.header.stamp = ros::Time::now();
      first_pose_stamped.header.frame_id = "map";
      first_pose_stamped.pose.position.x = b_x;
      first_pose_stamped.pose.position.y = b_y;
      first_pose_stamped.pose.orientation.z = b_z;
      first_pose_stamped.pose.orientation.w = b_w;
    
      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

    }
    else if(website_first_answer == 3){

      //first_pose_stamped.header.stamp = ros::Time::now();
      first_pose_stamped.header.frame_id = "map";
      first_pose_stamped.pose.position.x = c_x;
      first_pose_stamped.pose.position.y = c_y;
      first_pose_stamped.pose.orientation.z = c_z;
      first_pose_stamped.pose.orientation.w = c_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

    }
    else if(website_first_answer == 4){

      //first_pose_stamped.header.stamp = ros::Time::now();
      first_pose_stamped.header.frame_id = "map";
      first_pose_stamped.pose.position.x = d_x;
      first_pose_stamped.pose.position.y = d_y;
      first_pose_stamped.pose.orientation.z = d_z;
      first_pose_stamped.pose.orientation.w = d_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

    }
    
    goal_pub.publish(first_pose_stamped);
    web_first_toggle = false;

  }

  // 사용자 2에게 받는 목적지 거점 좌표
  if(web_second_toggle == true){
    
    // 사용자1에게 받는 목적지 거점 좌표
    //if(website_answer == 1 || website_answer ==5){
    if(website_second_answer == 5){

      //second_pose_stamped.header.stamp = ros::Time::now();
      second_pose_stamped.header.frame_id = "map";
      second_pose_stamped.pose.position.x = a_x;
      second_pose_stamped.pose.position.y = a_y;
      second_pose_stamped.pose.orientation.z = a_z;
      second_pose_stamped.pose.orientation.w = a_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", second_pose_stamped.pose.position.x, second_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", second_pose_stamped.pose.orientation.z, second_pose_stamped.pose.orientation.w);

    }
    else if(website_second_answer == 6){

      //second_pose_stamped.header.stamp = ros::Time::now();
      second_pose_stamped.header.frame_id = "map";
      second_pose_stamped.pose.position.x = b_x;
      second_pose_stamped.pose.position.y = b_y;
      second_pose_stamped.pose.orientation.z = b_z;
      second_pose_stamped.pose.orientation.w = b_w;
    
      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", second_pose_stamped.pose.position.x, second_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", second_pose_stamped.pose.orientation.z, second_pose_stamped.pose.orientation.w);

    }
    else if(website_second_answer == 7){

      //second_pose_stamped.header.stamp = ros::Time::now();
      second_pose_stamped.header.frame_id = "map";
      second_pose_stamped.pose.position.x = c_x;
      second_pose_stamped.pose.position.y = c_y;
      second_pose_stamped.pose.orientation.z = c_z;
      second_pose_stamped.pose.orientation.w = c_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", second_pose_stamped.pose.position.x, second_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", second_pose_stamped.pose.orientation.z, second_pose_stamped.pose.orientation.w);

    }
    else if(website_second_answer == 8){

      //second_pose_stamped.header.stamp = ros::Time::now();
      second_pose_stamped.header.frame_id = "map";
      second_pose_stamped.pose.position.x = d_x;
      second_pose_stamped.pose.position.y = d_y;
      second_pose_stamped.pose.orientation.z = d_z;
      second_pose_stamped.pose.orientation.w = d_w;

      // Print the pose information to the terminal
      ROS_INFO("Goal pose:");
      ROS_INFO("  Website_Position: x = %f, y = %f", second_pose_stamped.pose.position.x, second_pose_stamped.pose.position.y);
      ROS_INFO("  Website_Orientaion: z = %f, w = %f", second_pose_stamped.pose.orientation.z, second_pose_stamped.pose.orientation.w);

    }
    
    //goal_pub.publish(second_pose_stamped);
    //web_second_toggle = false;

  }
}

///////////////////////// 호출할 때 위치 설정/////////////////////////////////
void My_waypoint::call_goal_select(){

  if(call_toggle == true){
    if(call_answer == 1){

      //call_pose_stamped.header.stamp = ros::Time::now();
      call_pose_stamped.header.frame_id = "map";
      call_pose_stamped.pose.position.x = a_x;
      call_pose_stamped.pose.position.y = a_y;
      call_pose_stamped.pose.orientation.z = a_z;
      call_pose_stamped.pose.orientation.w = a_w;

      // Print the pose information to the terminal
      ROS_INFO("Call pose:");
      ROS_INFO("  Call_Position: x = %f, y = %f", call_pose_stamped.pose.position.x, call_pose_stamped.pose.position.y);
      ROS_INFO("  Call_Orientaion: z = %f, w = %f", call_pose_stamped.pose.orientation.z, call_pose_stamped.pose.orientation.w);

    }
    else if(call_answer == 2){

      //call_pose_stamped.header.stamp = ros::Time::now();
      call_pose_stamped.header.frame_id = "map";
      call_pose_stamped.pose.position.x = b_x;
      call_pose_stamped.pose.position.y = b_y;
      call_pose_stamped.pose.orientation.z = b_z;
      call_pose_stamped.pose.orientation.w = b_w;
    
      // Print the pose information to the terminal
      ROS_INFO("Call pose:");
      ROS_INFO("  Call_Position: x = %f, y = %f", call_pose_stamped.pose.position.x, call_pose_stamped.pose.position.y);
      ROS_INFO("  Call_Orientaion: z = %f, w = %f", call_pose_stamped.pose.orientation.z, call_pose_stamped.pose.orientation.w);

    }
    else if(call_answer == 3){

      //call_pose_stamped.header.stamp = ros::Time::now();
      call_pose_stamped.header.frame_id = "map";
      call_pose_stamped.pose.position.x = c_x;
      call_pose_stamped.pose.position.y = c_y;
      call_pose_stamped.pose.orientation.z = c_z;
      call_pose_stamped.pose.orientation.w = c_w;

      // Print the pose information to the terminal
      ROS_INFO("Call pose:");
      ROS_INFO("  Call_Position: x = %f, y = %f", call_pose_stamped.pose.position.x, call_pose_stamped.pose.position.y);
      ROS_INFO("  Call_Orientaion: z = %f, w = %f", call_pose_stamped.pose.orientation.z, call_pose_stamped.pose.orientation.w);

    }
    else if(call_answer == 4){

      //call_pose_stamped.header.stamp = ros::Time::now();
      call_pose_stamped.header.frame_id = "map";
      call_pose_stamped.pose.position.x = d_x;
      call_pose_stamped.pose.position.y = d_y;
      call_pose_stamped.pose.orientation.z = d_z;
      call_pose_stamped.pose.orientation.w = d_w;

      // Print the pose information to the terminal
      ROS_INFO("Call pose:");
      ROS_INFO("  Call_Position: x = %f, y = %f", call_pose_stamped.pose.position.x, call_pose_stamped.pose.position.y);
      ROS_INFO("  Call_Orientaion: z = %f, w = %f", call_pose_stamped.pose.orientation.z, call_pose_stamped.pose.orientation.w);

    }
    
    goal_pub.publish(call_pose_stamped);
    call_toggle = false;

  }
}
/////////////////////////////////////////////////////////////////////


///////////void 초기 등록 거점 과 경유지에서 등록 거점 비교 함수()
void My_waypoint::short_distance(){
  
  if(website_second_answer == 5 || website_second_answer == 6 || website_second_answer == 7 || website_second_answer == 8 || website_click_answer == 9){
    
    // ROS_INFO("--------------------------------------------------");

    // ROS_INFO("1_call_stamped : %f", call_pose_stamped.pose.position.x);
    // ROS_INFO("1_call_stamped : %f", call_pose_stamped.pose.position.y);

    // ROS_INFO("1_first_stamped : %f", first_pose_stamped.pose.position.x);
    // ROS_INFO("1_first_stamped : %f", first_pose_stamped.pose.position.y);
    // ROS_INFO("1_second_stamped_x : %f", second_pose_stamped.pose.position.x);
    // ROS_INFO("1_second_stamped_y : %f", second_pose_stamped.pose.position.y);

    ROS_INFO("click : %d", website_click_answer);

    // ROS_INFO(" 1 distance : %f, 2 distance : %f", sqrt(abs(pow((call_pose_stamped.pose.position.x - first_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - first_pose_stamped.pose.position.y), 2))), sqrt(abs(pow((call_pose_stamped.pose.position.x - second_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - second_pose_stamped.pose.position.y), 2))));

    // ROS_INFO("---------------------------------------------------");

    // ROS_INFO("2_call_stamped : %f", call_pose_stamped.pose.position.x);
    // ROS_INFO("2_call_stamped : %f", call_pose_stamped.pose.position.y);

    // ROS_INFO("2_first_stamped : %f", first_pose_stamped.pose.position.x);
    // ROS_INFO("2_first_stamped : %f", first_pose_stamped.pose.position.y);
    // ROS_INFO("2_second_stamped_x : %f", second_pose_stamped.pose.position.x);
    // ROS_INFO("2_second_stamped_y : %f", second_pose_stamped.pose.position.y);
    // ROS_INFO("---------------------------------------------------");

    // 호출한 위치에서 사용자 1의 목적지가 사용자 2의 목적지 보다 거리가 짧은 경우
    if(sqrt(abs(pow((call_pose_stamped.pose.position.x - first_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - first_pose_stamped.pose.position.y), 2))) < sqrt(abs(pow((call_pose_stamped.pose.position.x - second_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - second_pose_stamped.pose.position.y), 2)))){

      ROS_INFO("click_first : %d", website_click_answer);

      ROS_INFO("1_Stopover pose:");
      ROS_INFO("  1_Short_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  1_Short_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

      goal_pub.publish(first_pose_stamped);

      // 수령자 1이 물건을 수령한 후 '수령 완료' 버튼을 누르면 다음 목적지로 간다.
      if(website_click_answer == 9){
        goal_pub.publish(second_pose_stamped);
      }
    }
    // 호출한 위치에서 사용자 1의 목적지가 사용자 2의 목적지 보다 거리가 먼 경우
    else if(sqrt(abs(pow((call_pose_stamped.pose.position.x - first_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - first_pose_stamped.pose.position.y), 2))) > sqrt(abs(pow((call_pose_stamped.pose.position.x - second_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - second_pose_stamped.pose.position.y), 2)))){
     
      ROS_INFO("click_second : %d", website_click_answer);

      ROS_INFO("2_Stopover pose:");
      ROS_INFO("  2_Short_Position: x = %f, y = %f", second_pose_stamped.pose.position.x, second_pose_stamped.pose.position.y);
      ROS_INFO("  2_Short_Orientaion: z = %f, w = %f", second_pose_stamped.pose.orientation.z, second_pose_stamped.pose.orientation.w);

      goal_pub.publish(second_pose_stamped);

      // 수령자 1이 물건을 수령한 후 '수령 완료' 버튼을 누르면 다음 목적지로 간다.
      if(website_click_answer == 9){
        goal_pub.publish(first_pose_stamped);
      }
    }
    // 호출한 위치에서 사용자 1의 목적지가 사용자 2의 목적지의 거리가 같은 경우, 먼저 배송을 보낸 사용자 1의 목적지에 먼저 가라.
    else if(sqrt(abs(pow((call_pose_stamped.pose.position.x - first_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - first_pose_stamped.pose.position.y), 2))) == sqrt(abs(pow((call_pose_stamped.pose.position.x - second_pose_stamped.pose.position.x), 2) + pow((call_pose_stamped.pose.position.y - second_pose_stamped.pose.position.y), 2)))){
      
      ROS_INFO("click_third : %d", website_click_answer);

      ROS_INFO("3_Stopover pose:");
      ROS_INFO("  3_Short_Position: x = %f, y = %f", first_pose_stamped.pose.position.x, first_pose_stamped.pose.position.y);
      ROS_INFO("  3_Short_Orientaion: z = %f, w = %f", first_pose_stamped.pose.orientation.z, first_pose_stamped.pose.orientation.w);

      goal_pub.publish(first_pose_stamped);

      // 수령자 1이 물건을 수령한 후 '수령 완료' 버튼을 누르면 다음 목적지로 간다.
      if(website_click_answer == 9){
        goal_pub.publish(second_pose_stamped);
      }
    }

    ROS_INFO("---------------------------------------");

    web_first_toggle = false;
    web_second_toggle = false;
  }

}
//////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_node");

  My_waypoint my_waypoint; 

  ros::Rate loop_rate(10);

  while(ros::ok()){

    // 한 번만 돌게 하기 위해서 toggle 조건 판단

    // website에서 배송 보낼 거점 결정
    if(my_waypoint.loop_only_once_website_toggle){
      my_waypoint.goal_select();
      my_waypoint.short_distance();
      my_waypoint.loop_only_once_website_toggle = false; // goal_select() 함수를 한 번 수행했으므로 토글 false
    }

    // 호출할 거점 정보
    if(my_waypoint.loop_only_once_call_toggle){
      my_waypoint.call_goal_select();
      my_waypoint.loop_only_once_call_toggle = false; // goal_select() 함수를 한 번 수행했으므로 토글 false
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}