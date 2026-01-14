//C++ library includes
#include <memory>
#include <chrono>
#include <math.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <functional>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//other macros
#define _USE_MATH_DEFINES


using std::placeholders::_1;
using namespace std::chrono_literals;


class WaypointVisualiser : public rclcpp::Node {

public:
    WaypointVisualiser() : Node("waypoint_visualiser_node")
    {
        this->declare_parameter("racelines_dir", "racelines/");
        this->declare_parameter("max_raceline", 3);
        this->declare_parameter("rviz_waypoints_topic", "/waypoints");

        std::string racelines_dir = this->get_parameter("racelines_dir").as_string();
        max_raceline = this->get_parameter("max_raceline").as_int();
        rviz_waypoints_topic = this->get_parameter("rviz_waypoints_topic").as_string();
        
        // Make racelines_dir absolute if relative
        if (racelines_dir[0] != '/') {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("pure_pursuit");
            racelines_dir = package_share_dir + "/" + racelines_dir;
        }
        
        this->racelines_dir_ = racelines_dir;
        current_raceline_id_ = 1;

        vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
        
        // Subscribe to state machine
        raceline_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/selected_raceline", 10,
            std::bind(&WaypointVisualiser::raceline_callback, this, _1));
        
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointVisualiser::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "this node has been launched");
        load_all_racelines();

    }

    private:
    struct csvFileData{
        std::vector<double> X;
        std::vector<double> Y;
    };


    //topic names
    std::string racelines_dir_;
    std::string rviz_waypoints_topic;
    int max_raceline;
    int current_raceline_id_;
    
    //struct initialisation
    std::map<int, csvFileData> all_racelines;

    //Publisher initialisation
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub; 
    
    //Subscriber
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr raceline_sub;

    //Timer initialisation
    rclcpp::TimerBase::SharedPtr timer_;

    //private functions
    
    void load_all_racelines() {
        RCLCPP_INFO(this->get_logger(), "Loading all racelines from: %s", racelines_dir_.c_str());
        
        for (int i = 1; i <= max_raceline; i++) {
            std::string raceline_path = racelines_dir_ + "raceline_" + std::to_string(i) + ".csv";
            csvFileData raceline_data;
            
            std::fstream csvFile;
            csvFile.open(raceline_path, std::ios::in);
            
            if (!csvFile.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", raceline_path.c_str());
                continue;
            }
            
            std::string line, word;
            while (std::getline(csvFile, line)) {
                if (line.empty()) continue;
                
                std::stringstream s(line);
                int j = 0;
                while (std::getline(s, word, ',')) {
                    if (!word.empty()) {
                        if (j == 0) {
                            raceline_data.X.push_back(std::stod(word));
                        } else if (j == 1) {
                            raceline_data.Y.push_back(std::stod(word));
                        }
                    }
                    j++;
                }
            }
            
            csvFile.close();
            
            if (raceline_data.X.size() > 0) {
                all_racelines[i] = raceline_data;
                RCLCPP_INFO(this->get_logger(), "✓ Loaded raceline_%d with %zu waypoints", i, raceline_data.X.size());
            }
        }
    }
    
    void raceline_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_raceline_id_ = msg->data;
    }

    void visualize_points() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Visualize all racelines with different colors
        for (const auto& [id, raceline] : all_racelines) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "raceline_" + std::to_string(id);
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            
            // If this is the selected raceline, make it thicker and brighter
            if (id == current_raceline_id_) {
                marker.scale.x = 0.15;  // Thick line
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;   // Yellow
                marker.color.a = 1.0;   // Fully visible
            } else {
                marker.scale.x = 0.05;  // Thin line
                // Different color for each raceline
                if (id == 1) {
                    marker.color.r = 0.0;
                    marker.color.g = 0.5;
                    marker.color.b = 1.0;  // Light blue
                } else if (id == 2) {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.5;  // Pink
                } else if (id == 3) {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.5;  // Light green
                }
                marker.color.a = 0.4;  // Semi-transparent
            }
            
            // Add all waypoints to the line strip
            for (size_t i = 0; i < raceline.X.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = raceline.X[i];
                p.y = raceline.Y[i];
                p.z = 0.0;
                marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(marker);
        }

        vis_path_pub->publish(marker_array);
    }

    void timer_callback() {
        visualize_points();
    }
    

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointVisualiser>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
