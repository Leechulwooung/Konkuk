#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;

  //////실질적으로 tf만들어서 쏴주는 코드//////
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );  //Origin을 정해준다. 우리가 변환하고자 하는 target frame의 원점 좌표로 만드는 것.
  tf::Quaternion q;                                          //Quaternion 좌표계를 만들어준다
  q.setRPY(0, 0, msg->theta);                                // roll(앞뒤로흔들림), pitch(옆으로 회전), yaw(차량의 헤딩 방향: 좌우로 흔들림)
  ////////////////////////////////////////////

  transform.setRotation(q);

  //************이걸 하지 않으면 데이터를 쏘지 않는다*******/////////
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));    // "world" : base(world) frame, turtle_name : target frame -> 0,0 으로 만들어준다.
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};