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
