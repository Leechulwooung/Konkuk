#include "ros/ros.h"
#include "geometry_msgs/Twist.h"    // /turtle1/cmd_vel는 geometry_msgs 메시지 타입으로 구성. linear, angular 필드로 구성
#include "turtlesim/Pose.h"
#include <cmath>

class TurtleHexagon
{
protected:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);                // turtlesim_node 에 pub 한다. /turtle1cmd_vel은 거북이의 선속도(linear velocity)와 각속도(anglar velocity)를 조절하기 위한 메시지 토픽.
    ros::Subscriber pose_sub_ = nh_.subscribe("/turtle1/pose", 1000, &TurtleHexagon::poseCallback, this);   // turtlesim_node 에서 sub 한다. /turtle1/pose은 거북이의 위치와 방향 정보를 포함하는 토픽.

    double x_ = 0;                  // /turtle1/pose 에서 x 위치 정보를 받아아 와서 저장하는 변수
    double y_ = 0;                  // /turtle1/pose 에서 y 위치 정보를 받아아 와서 저장하는 변수
    double theta_ = 0;              // /turtle1/pose 에서 theta 방향 정보를 받아아 와서 저장하는 변수
    bool ready_to_move_ = false;    // 거북이가 초기 위치를 설정하는 데 사용. 거북이가 위치 정보를 수신하기 전까지는 이 변수의 값은 false.

public:
    TurtleHexagon();    // 생성자
    ~TurtleHexagon();   // 소멸자
    void poseCallback(const turtlesim::Pose::ConstPtr& msg);
    void move(double linear_speed, double distance);
    void rotate(double angular_speed, double angle);
    void drawHexagon();
};

TurtleHexagon::TurtleHexagon(){}
TurtleHexagon::~TurtleHexagon(){}

// turtlesim_node의 토픽인 /turtle1/pose를 sub 하고, 이 토픽으로부터 거북이의 위치와 방향 정보를 읽어와서 'x_', 'y_', 'theta_' 멤버 변수에 저장하는 poseCallback 메서드.
void TurtleHexagon::poseCallback(const turtlesim::Pose::ConstPtr& msg)  
{
    x_ = msg->x;                // x_ 변수에 turtlesim_node의 토픽인 /turtle1/pose로부터 받은 x의 위치 정보 값을 받음.
    y_ = msg->y;                // y_ 변수에 turtlesim_node의 토픽인 /turtle1/pose로부터 받은 y의 위치 정보 값을 받음.
    theta_ = msg->theta;        // theta_ 변수에 turtlesim_node의 토픽인 /turtle1/pose로부터 받은 theta의 방향 정보 값을 받음.

    if (!ready_to_move_)        // ready_to_move_ 변수가 false인 경우 (전에 거북이의 위치가 업데이트되지 않았던 경우)
    {
        ready_to_move_ = true;  // 'poseCallback'이 처음 호출될 때 이 변수의 값을 true로 설정.
        drawHexagon();          // 육각형을 그려라.
    }
}

// 거북이가 움직이는 함수
void TurtleHexagon::move(double linear_speed, double distance)  // 첫번째 인자 거북이의 선속도를 나타내는 linear_speed, 두 번째 인자는 이동해야하는 거리 distance
{
    geometry_msgs::Twist vel;       // vel 객체 선언 (거북이의 속도와 방향을 나타내는 메시지)
    vel.linear.x = linear_speed;    // move 함수에 입력 받은 선속도(linear_speed) 값을 Twist 메세지에서 linear 필드 안에 x 필드에 저장한다. : x 필드는 로봇의 전진 속도를 나타냄.
    double current_distance = 0;    // curren_distance 는 거북이의 현재 이동한 거리를 나타냄.
    double elapsed_time = 0;        // elapsed_time 은 경과한 시간을 나타냄.
    ros::Rate loop_rate(100);       // 루프 주기를 설정. "100"이라는 것은 100Hz를 말하는 것으로 0.01초 간격으로 처리가 반복됨. -> 1초에 100번이 실행됨.

    while (current_distance < distance) // 현재까지 이동한 거리 current_distance가 목표 이동 거리 distance보다 작은 동안 로봇을 움직이도록 함.
    {
        ros::Time t1 = ros::Time::now();                    // 현재 시간 t1
        current_distance += linear_speed * elapsed_time;    // 거북이가 이동한 거리를 계산. current_distance 변수는 로봇이 이동한 총 거리를 나타내며, 선속도 linear_speed와 경과 시간 elapsed_time의 곱은 거북이가 이동한 거리이다. current_distance 변수에 이 값을 더해 거북이가 이동한 총 거리를 업데이트.
        vel_pub_.publish(vel);                              // vel 객체 발행 (거북이를 제어하는 turtlesim_node가 이 메시지를 sub하여 거북이 제어)
        ros::spinOnce();                                    // 새로운 메시지 확인 및 처리
        loop_rate.sleep();                                  // 루프 실행 주기 조절
        elapsed_time = (ros::Time::now() - t1).toSec();     // 현재 시간과 t1 변수에 저장된 이전 시간의 차이를 계산하여 elapsed_time 변수에 할당.이전 시간부터 현재까지 걸린 시간을 초 단위로 계산하기 위해 toSec()함수 사용.
                                                            // elapsed_time 변수는 각 루프 반복에서 현재 속도와 걸린 시간을 곱하여 이동한 거리를 계산하는데 사용.
    }

    vel.linear.x = 0;                                       // 루프가 종료되면 vel 객체의 x축 방향 선속도를 0으로 설정하여 거북이 정지.
    vel_pub_.publish(vel);                                  // vel 객체 발행 (거북이를 제어하는 turtlesim_node가 이 메시지를 sub하여 거북이 제어)
}

void TurtleHexagon::rotate(double angular_speed, double angle)  // 첫번째 인자 거북이의 각속도를 나타내는 angular_speed, 두 번째 인자는 회전해야하는 각도 angular
{
    geometry_msgs::Twist vel;           // vel 객체 선언 (거북이의 속도와 방향을 나타내는 메시지)
    vel.angular.z = angular_speed;      // move 함수에 입력 받은 각속도(angular_speed) 값을 Twist 메세지에서 angular 필드 안에 z 필드에 저장한다. : z 필드는 로봇의 각 속도를 나타냄. vel.angular.z에 angular_speed를 할당하여 회전 속도 지정.
    double current_angle = 0;           // curren_angule 는 거북이의 현재 회전한 각도를 나타냄.
    double elapsed_time = 0;            // elapsed_time 은 경과한 시간을 나타냄.
    ros::Rate loop_rate(100);           // 루프 주기를 설정. "100"이라는 것은 100Hz를 말하는 것으로 0.01초 간격으로 처리가 반복됨. -> 1초에 100번이 실행됨.

    while (current_angle < angle)       // 현재 회전한 각도 current_angle 이 지정된 각도 angle 보다 작을 때까지 회전.
    {
        ros::Time t1 = ros::Time::now();                    // 현재 시간 t1
        current_angle += angular_speed * elapsed_time;      // 회전한 각도 current_angle 계산. 이때 elapsed_time을 이용해 시간에 따른 회전 각도 변화량 계산.
        vel_pub_.publish(vel);                              // vel 객체 Twist 메시지 발행 (거북이를 제어하는 turtlesim_node가 이 메시지를 sub하여 거북이 제어)
        ros::spinOnce();                                    // 새로운 메시지 확인 및 처리
        loop_rate.sleep();                                  // 루프 실행 주기 조절
        elapsed_time = (ros::Time::now() - t1).toSec();     // 현재 시간과 t1 변수에 저장된 이전 시간의 차이를 계산하여 elapsed_time 변수에 할당.이전 시간부터 현재까지 걸린 시간을 초 단위로 계산하기 위해 toSec()함수 사용.
                                                            // elapsed_time 변수는 각 루프 반복에서 현재 속도와 걸린 시간을 곱하여 이동한 거리를 계산하는데 사용.
    }

    vel.angular.z = 0;                     // 루프가 종료되면 vel 객체의 vel.angular.z에 0을 할당하여 회전 정지.
    vel_pub_.publish(vel);                 // vel 객체 Twist 메시지 발행 (거북이를 제어하는 turtlesim_node가 이 메시지를 sub하여 거북이 제어)
}


// 정육각형을 그려라
void TurtleHexagon::drawHexagon()           // 거북이가 육각형을 그리는 함수
{
    const double side_length = 2.0;         // side_legnth는 육각형의 한 변의 길이를 나타낸 변수 (단위 m)

    for (int i = 0; i < 6; ++i)             // for 문으로 6번 반복
    {
        move(1.0, side_length);             // 거북이를 직진 시키는 move 함수. (선속도, 이동 거리)
        rotate(M_PI / 3.0, M_PI / 3.0);     // 거북이를 회전 시키는 roate 함수. (M_PI / 3.0 은 60도를 라디안 값으로 표시 (1.04719755...), 육각형의 내각은 120도 이므로 회전할 각도는 M_PI / 3.0 = 60도)
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_hexagon");    // week2_node의 node 이름 : turtle_hexagon, 노드 초기화 진행
    TurtleHexagon turtle_hexagon;               // turtleHexagon 클래스를 인스턴스화하여 turtle_hexagon 변수에 할당
    ros::spin();                                // 노드가 종료될 때까지 루프를 유지

    return 0;
}


//===============ros::Time을 써서 하는 경우==================
/*
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

class TurtleHexagon
{
protected:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Subscriber pose_sub_ = nh_.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, &TurtleHexagon::poseCallback, this);

    double x_ = 0;
    double y_ = 0;
    double theta_ = 0;
    bool ready_to_move_ = false;

public:
    TurtleHexagon();    // 생성자
    ~TurtleHexagon();   // 소멸자
    void poseCallback(const turtlesim::Pose::ConstPtr& msg);
    void move(double linear_speed, double distance);
    void rotate(double angular_speed, double angle);
    void drawHexagon();
};

TurtleHexagon::TurtleHexagon(){}
TurtleHexagon::~TurtleHexagon(){}

void TurtleHexagon::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    x_ = msg->x;
    y_ = msg->y;
    theta_ = msg->theta;

    if (!ready_to_move_)
    {
        ready_to_move_ = true;
        drawHexagon();
    }
}

// 거북이가 움직이는 함수
void TurtleHexagon::move(double linear_speed, double distance)
{
    geometry_msgs::Twist vel;
    vel.linear.x = linear_speed;
    double t0 = ros::Time::now().toSec();
    double current_distance = 0;
    ros::Rate loop_rate(100);

    while (current_distance < distance)
    {
        vel_pub_.publish(vel);
        double t1 = ros::Time::now().toSec();
        current_distance = linear_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    vel.linear.x = 0;
    vel_pub_.publish(vel);
}

// 거북이 대가리 회전 함수
void TurtleHexagon::rotate(double angular_speed, double angle)
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular_speed;
    double t0 = ros::Time::now().toSec();
    double current_angle = 0;
    ros::Rate loop_rate(100);

    while (current_angle < angle)
    {
        vel_pub_.publish(vel);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    vel.angular.z = 0;
    vel_pub_.publish(vel);
}

// 정육각형을 그려라
void TurtleHexagon::drawHexagon()
{
    const double side_length = 1.0;

    for (int i = 0; i < 6; ++i)
    {
        move(1.0, side_length);
        rotate(M_PI / 3.0, M_PI / 3.0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_hexagon");
    TurtleHexagon turtle_hexagon;
    ros::spin();

    return 0;
}
*/