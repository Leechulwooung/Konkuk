/*
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// Include the necessary ROS and visualization_msgs headers
#include <ros/ros.h>
//#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class WaypointVisualizer {
    public:
        WaypointVisualizer();
        ~WaypointVisualizer();

        void readWaypointsFromFile(const std::string& filename);
        void visualizeWaypoints();

    protected:
        std::vector<std::pair<double, double>> waypoints_;
        ros::Publisher marker_pub_;
};
*/

#include "week5_node.hpp"

WaypointVisualizer::WaypointVisualizer() {
    // Initialize ROS node and publisher
    ros::NodeHandle nh;
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

WaypointVisualizer::~WaypointVisualizer(){}

//readWaypointsFromFile 함수는 파일에서 웨이포인트를 읽고 waypoints_ 벡터에 저장합니다. 
//파일 이름을 입력으로 받고 ifstream을 사용하여 파일을 열고 파일의 각 행을 반복합니다. 
//각 줄은 std::getline 및 std::istringstream을 사용하여 구문 분석되고 추출된 값은 적절한 데이터 
//유형으로 변환됩니다. x 및 y 좌표 쌍으로 표시되는 웨이포인트는 'waypoints_' 벡터에 저장됩니다.

void WaypointVisualizer::readWaypointsFromFile(const std::string& filename) { // 함수에서 waypoint를 읽고 waypoints_ 벡터에 저장하는 역할을 한다. const std::string&유형의 filename 매개변수를 사용. 경유지를 읽는 파일의 이름을 나타냄.

    std::ifstream file(filename);                               // std::ifstream (input file) 유형의 객체 file을 생성하고 제공된 filensmae으로 지정된 파일을 연다.

    if (file.is_open()) {                                       // file 객체에서 is_open() 함수를 호출하여 파일이 성공적으로 열렸는지 확인. file.is_open() 반환 값이 bool이다. -> true, false

        std::string line;           // string 단위로 읽어 온다.

        while (std::getline(file, line)) {                      // 파일이 성공적으로 열리면 함수는 경유지 읽기를 진행. std::getline(file, line) line에 file에 있는 데이터를 저장한다. 파일의 각 줄을 반복하는 루프에 들어감. 
                                                                // getline은 파일에 있는 줄바꿈 표시 전까지의 데이터를 읽는다.

            std::istringstream iss(line);                       // 루프 내에서 std::istringstream 유형의 객체 iss를 생성하고 현재 행을 전달. 문자열에서 값을 구문 분석하고 추출하는 데 사용되는 특수 스트림 클래스입니다.
            std::string index_str, x_str, y_str;                // std::string 유형의 세 가지 변수 index_str, x_str, y_str을 선언. 이 변수는 line에서 추출된 값을 저장.

            std::getline(iss, index_str, ',');                  // 구분자 ','와 함께 std::getline 함수를 사용하여 iss애서 index에 대한 값을 추출함. 
            int index = std::stoi(index_str);                   // 추출된 문자열을 std::stoi를 사용하여 정수로 변환하고 변수 index에 저장함.

            std::getline(iss, x_str, ',');                      // 'iss'에서 'x'및 'y' 값을 추출하기 위해 유사한 단께가 수행됨. 이 함수는 std::getline을 사용하여 문자열을 추출하고
            double x = std::stod(x_str);                        // std::stod를 사용하여 문자열을 double로 변환하고 x 및 y 변수에 각각 저장함.

            std::getline(iss, y_str, ',');
            double y = std::stod(y_str);

            waypoints_.push_back(std::make_pair(x, y));         // x 및 y 값을 얻은 후 함수는 std::make_pair(x, y)를 사용하여 쌍을 만들고 이를 waypoints_ 벡터에 추가함.
        }
        file.close();                                           // 파일의 모든 행이 처리되면 이 함수는 close() 함수를 사용하여 파일을 닫는다.

    } else {
        ROS_ERROR_STREAM("Failed to open file: " << filename);  // 파일 열기에 실패하면 함수는 ROS_ERROR_STREAM을 사용하여 파일 열기에 실패했음을 나타내는 오류 메시지를 인쇄함.
    }
}

/*
    'readWaypointsFromFile' 함수는 지정된 파일을 열고, 한 줄씩 읽어 각 줄에서 인덱스, x, y 값을 
    추출하여 적절한 데이터 타입으로 변환한 다음 waypoints_벡터에 쌍으로 저장한다.
*/

void WaypointVisualizer::visualizeWaypoints() {
    
    ros::Rate loop_rate(10);  // Set the visualization frequency

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.color.a = 1.0;               // Don't forget a -> 투명도
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // waypoints_ 벡터의 각 웨이포인트를 순회하면서 marker 객체에 포인트를 채우는 반복문
    for (const auto& waypoint : waypoints_) {   // 이 줄은 waypoints_ 벡터의 각 웨이포인트를 순회하는 반복문을 시작합니다. 반복문 변수인 waypoint는 각 반복마다 하나의 웨이포인트를 나타냅니다.

        geometry_msgs::Point point;             // 이 줄은 point라는 새로운 geometry_msgs::Point 객체를 생성합니다. 이 객체는 X, Y 및 Z 좌표로 구성된 3차원 점을 나타냅니다.

        point.x = waypoint.first;               // 이 줄은 point 객체의 X 좌표에 waypoint 쌍의 first 멤버 값을 할당합니다. 이 코드에서 first는 웨이포인트의 X 좌표에 해당합니다.
        point.y = waypoint.second;              // 이 줄은 point 객체의 Y 좌표에 waypoint 쌍의 second 멤버 값을 할당합니다. 이 코드에서 second는 웨이포인트의 Y 좌표에 해당합니다.
        point.z = 0.0;                          // 이 줄은 point 객체의 Z 좌표를 0.0으로 설정합니다. 웨이포인트를 2차원 공간에서 시각화하고 있기 때문에 Z 좌표는 일반적으로 0으로 설정됩니다.

        marker.points.push_back(point);         // 이 줄은 point 객체를 marker 객체의 points 벡터에 추가합니다. 각 point는 웨이포인트를 나타내며, 반복문의 각 반복마다 벡터에 하나씩 추가됩니다.
    }

    while (ros::ok()) {
        marker_pub_.publish(marker);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_visualizer_node");

    WaypointVisualizer visualizer;

    // Read waypoints from file
    std::string filename = "/home/cwl1223/boongboong_ws/src/week5_pkg/waypoint.txt";
    visualizer.readWaypointsFromFile(filename);

    // Visualize waypoints in Rviz
    visualizer.visualizeWaypoints();

    return 0;
}
