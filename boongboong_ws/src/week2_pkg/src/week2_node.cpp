#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "cmath"
#include "visualization_msgs/Marker.h"

class TurtleHexagon{

    protected:
    ros::NodeHandle _nh;
    ros::Publisher _vel_pub = _nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Subscriber _vel_sub = _nh.subscribe("/turtle1/pose",1000, &TurtleHexagon::poseCallback, this);
    ros::Publisher _visualization = _nh.advertise<visualization_msgs::Marker>("/visualization", 1000);

    double _x = 0;
    double _y = 0;
    double _theta = 0;
    double _ready_to_move = false;

    public:
    TurtleHexagon();
    ~TurtleHexagon();
    void poseCallback(const turtlesim::Pose::ConstPtr& msg);
    void move(double linear_speed, double distance);
    void rotate(double angular_speed, double angle);
    void drawHexagon();
    void rviz_marker();
};

TurtleHexagon::TurtleHexagon(){
    ros::Time::init();
}
TurtleHexagon::~TurtleHexagon(){}

void TurtleHexagon::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    _x = msg->x;
    _y = msg->y;
    _theta = msg->theta;

    if(!_ready_to_move)
    {
        _ready_to_move = true;
        drawHexagon();
        rviz_marker();
    }
    
}

void TurtleHexagon::move(double linear_speed, double distance)
{
    geometry_msgs::Twist vel;
    vel.linear.x = linear_speed;
    double current_distance = 0;
    double elapsed_time = 0;
    ros::Rate loop_rate(100);

    while(current_distance < distance)
    {
        ros::Time t1 = ros::Time::now();
        current_distance += linear_speed * elapsed_time;
        _vel_pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
        elapsed_time = (ros::Time::now() - t1).toSec();
    }

    vel.linear.x = 0;
    _vel_pub.publish(vel);
}

void TurtleHexagon::rotate(double angular_speed, double angle)
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular_speed;
    double current_angle = 0;
    double elapsed_time = 0;
    ros::Rate loop_rate(100);

    while(current_angle < angle)
    {
        ros::Time t1 = ros::Time::now();
        current_angle += elapsed_time * angular_speed;
        _vel_pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
        elapsed_time = (ros::Time::now() - t1).toSec();
    }

    vel.angular.z = 0;
    _vel_pub.publish(vel);
}

void TurtleHexagon::drawHexagon()
{
    const double side_length = 2.0;

    // 0-5번만 실행
    // for(int i =0; i < 6; i++){
    //     move(2.0, side_length);
    //     rotate((M_PI / 3.0), (M_PI / 3.0));
    // }

    // 무한하게 실행
    while(1){
       
        move(2.0, side_length);
        rotate((M_PI / 3.0), (M_PI / 3.0));
        rviz_marker();
    }
}

 void TurtleHexagon::rviz_marker(){

    //visualization_msgs::Marker marker;
    // marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time();
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::SPHERE;
    // marker.pose.position.x = _x;
    // marker.pose.position.y = _y;
    // marker.scale.x = 1;
    // marker.scale.y = 1;
    // marker.scale.z = 1;
    // marker.color.a = 1.0;
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;

    // _visualization.publish(marker);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //////////////turtlesim에서 움직이는데로 marker가 따라 움직일 수 있게 위에서 x, y 좌표를 받아와서 넣어준다.//////////////////////
    marker.pose.position.x = _x;
    marker.pose.position.y = _y;
    marker.pose.position.z = 0;
    ///////////////////////////////////
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    /////////////marker의 모양 결정//////////////////
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    ////////////////////////////////
    marker.color.a = 1.0; // Don't forget to set the alpha! 투명도!!!!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    _visualization.publish( marker );

 }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_hexagon");
    TurtleHexagon turtlehexagon;
    ros::spin();

/////////////여기서 무한 반복문이 안 되는 이유가 뭘까??////////////////
    //ros::Rate loop_rate(10);

    // while(ros::ok()){

    //     TurtleHexagon turtlehexagon;

    //     ros::spinOnce();
    //     //loop_rate.sleep();
    // }
    
    return 0;
}