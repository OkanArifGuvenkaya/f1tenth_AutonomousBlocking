#include <memory>
#include <math.h>
#include <string>
#include <cstdlib> 
#include <vector> 
#include <sstream> 
#include <iostream> 
#include <fstream>
#include <algorithm> 
#include <Eigen/Eigen>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/int32.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {

public:
    PurePursuit() : Node("pure_pursuit_node") {
        // initialise parameters
        this->declare_parameter("racelines_dir", "/sim_ws/src/pure_pursuit/racelines/");
        this->declare_parameter("max_raceline", 3);
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("car_refFrame", "ego_racecar/base_link");
        this->declare_parameter("drive_topic", "/drive");
        this->declare_parameter("rviz_waypointselected_topic", "/waypoints");
        this->declare_parameter("global_refFrame", "map");
        this->declare_parameter("min_lookahead", 0.5);
        this->declare_parameter("max_lookahead", 1.0);
        this->declare_parameter("lookahead_ratio", 8.0);
        this->declare_parameter("K_p", 0.5);
        this->declare_parameter("steering_limit", 25.0);
        this->declare_parameter("velocity_percentage", 0.6);
        
        // Default Values
        racelines_dir = this->get_parameter("racelines_dir").as_string();
        
        // If racelines_dir is relative, make it absolute using package share directory
        if (racelines_dir[0] != '/') {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("pure_pursuit");
            racelines_dir = package_share_dir + "/" + racelines_dir;
        }
        
        max_raceline = this->get_parameter("max_raceline").as_int();
        current_raceline_id = 1;  // Start with raceline 1
        odom_topic = this->get_parameter("odom_topic").as_string();
        car_refFrame = this->get_parameter("car_refFrame").as_string();
        drive_topic = this->get_parameter("drive_topic").as_string();
        rviz_waypointselected_topic = this->get_parameter("rviz_waypointselected_topic").as_string();
        global_refFrame = this->get_parameter("global_refFrame").as_string();
        min_lookahead = this->get_parameter("min_lookahead").as_double();
        max_lookahead = this->get_parameter("max_lookahead").as_double();
        lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
        K_p = this->get_parameter("K_p").as_double();
        steering_limit =  this->get_parameter("steering_limit").as_double();
        velocity_percentage =  this->get_parameter("velocity_percentage").as_double();
        
        //initialise subscriber sharedptr obj
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
        subscription_raceline = this->create_subscription<std_msgs::msg::Int32>("/selected_raceline", 10, std::bind(&PurePursuit::raceline_callback, this, _1));
        timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));
        raceline_print_timer_ = this->create_wall_timer(200ms, std::bind(&PurePursuit::print_current_raceline, this));

        //initialise publisher sharedptr obj
        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
        //vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
        vis_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_waypointselected_topic, 10);

        //initialise tf2 shared pointers
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO (this->get_logger(), "this node has been launched");

        load_all_racelines();
        switch_raceline(current_raceline_id);

    }

    ~PurePursuit() {}

private:
    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    geometry_msgs::msg::Quaternion odom_quat;
    struct csvFileData{
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;
        //double x_worldRef, y_worldRef, x_carRef, y_carRef;
        int index;
        int velocity_index;

        Eigen::Vector3d p1_world; // Coordinate of point from world reference frame
        Eigen::Vector3d p1_car; // Coordinate of point from car reference frame (to be calculated)
    };

    Eigen::Matrix3d rotation_m;
    
    double x_car_world;
    double y_car_world;


    std::string odom_topic;
    std::string car_refFrame;
    std::string drive_topic;
    std::string global_refFrame;
    std::string rviz_waypointselected_topic;
    std::string racelines_dir;
    int max_raceline;
    int current_raceline_id;
    double K_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double curr_velocity = 0.0;
    
    bool emergency_breaking = false;
    std::string current_lane = "left"; // left or right lane
    
    
    //file object
    std::fstream csvFile_waypoints; 

    //struct initialisation
    csvFileData waypoints;
    std::map<int, csvFileData> all_racelines;  // Store all racelines
    int num_waypoints;

    //Timer initialisation
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr raceline_print_timer_;

    //declare subscriber sharedpointer obj
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_raceline;

    //declare publisher sharedpointer obj
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive; 
    //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_point_pub; 


    //declare tf shared pointers
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //private functions
    double to_radians(double degrees) {
        double radians;
        return radians = degrees*M_PI/180.0;
    }

    double to_degrees(double radians) {
        double degrees;
        return degrees = radians*180.0/M_PI;
    }
    
    double p2pdist(double &x1, double &x2, double &y1, double &y2) {
        double dist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        return dist;
    }

    void load_all_racelines() {
        RCLCPP_INFO(this->get_logger(), "Loading all racelines from: %s", racelines_dir.c_str());
        
        for (int i = 1; i <= max_raceline; i++) {
            std::string raceline_path = racelines_dir + "raceline_" + std::to_string(i) + ".csv";
            csvFileData raceline_data;
            raceline_data.index = 0;
            raceline_data.velocity_index = 0;
            
            std::fstream csvFile;
            csvFile.open(raceline_path, std::ios::in);
            
            if (!csvFile.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", raceline_path.c_str());
                continue;
            }
            
            std::string line, word;
            int line_count = 0;
            while (std::getline(csvFile, line)) {
                if (line.empty()) continue;
                
                std::stringstream s(line);
                int j = 0;
                while (std::getline(s, word, ',')) {
                    if (!word.empty()) {
                        try {
                            if (j == 0) {
                                raceline_data.X.push_back(std::stod(word));
                            } else if (j == 1) {
                                raceline_data.Y.push_back(std::stod(word));
                            } else if (j == 2) {
                                raceline_data.V.push_back(std::stod(word));
                            }
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(this->get_logger(), "Error parsing line %d in raceline_%d: %s", line_count, i, e.what());
                        }
                    }
                    j++;
                }
                line_count++;
            }
            
            csvFile.close();
            
            if (raceline_data.X.size() > 0) {
                all_racelines[i] = raceline_data;
                RCLCPP_INFO(this->get_logger(), "✓ Loaded raceline_%d with %zu waypoints", i, raceline_data.X.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Failed to load raceline_%d - no data parsed", i);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "All racelines loaded successfully! Total: %zu racelines", all_racelines.size());
    }

    void find_nearest_forward_waypoint() {
        if (waypoints.X.empty()) return;
        
        int best_index = 0;
        double best_score = std::numeric_limits<double>::max();
        bool found = false;
        
        // Get vehicle heading
        double vehicle_heading = atan2(2.0 * (odom_quat.z * odom_quat.w + odom_quat.x * odom_quat.y), 
                                      1.0 - 2.0 * (odom_quat.y * odom_quat.y + odom_quat.z * odom_quat.z));
        
        // Target distance: prefer waypoints around min_lookahead distance
        double target_dist = min_lookahead;
        
        // Find waypoint that is:
        // 1. In front of the car
        // 2. Aligned with car's heading direction
        // 3. Around lookahead distance away (not too close, not too far)
        for (int i = 0; i < num_waypoints; i++) {
            double dx = waypoints.X[i] - x_car_world;
            double dy = waypoints.Y[i] - y_car_world;
            
            // Transform to car's local frame
            double local_x = dx * cos(-vehicle_heading) - dy * sin(-vehicle_heading);
            double local_y = dx * sin(-vehicle_heading) + dy * cos(-vehicle_heading);
            
            // Only consider waypoints in FRONT (local_x > 0) and not too far to the side
            if (local_x > 0 && abs(local_y) < 3.0) {  // Within 3m laterally
                double dist = sqrt(dx*dx + dy*dy);
                
                // Calculate waypoint direction relative to car
                double waypoint_angle = atan2(dy, dx);
                double angle_diff = abs(waypoint_angle - vehicle_heading);
                // Normalize angle difference to [0, PI]
                while (angle_diff > M_PI) angle_diff -= 2*M_PI;
                angle_diff = abs(angle_diff);
                
                // Score: combination of distance error and angle alignment
                // Prefer waypoints that are:
                // - Around target_dist away (not too close)
                // - Well aligned with car's heading
                double dist_error = abs(dist - target_dist);
                double score = dist_error + 2.0 * angle_diff;  // Angle is more important
                
                if (score < best_score && dist > 0.5) {  // Minimum 0.5m away
                    best_score = score;
                    best_index = i;
                    found = true;
                }
            }
        }
        
        // If no good waypoint found, use nearest forward waypoint
        if (!found) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No optimal waypoint found! Using nearest forward.");
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < num_waypoints; i++) {
                double dx = waypoints.X[i] - x_car_world;
                double dy = waypoints.Y[i] - y_car_world;
                double local_x = dx * cos(-vehicle_heading) - dy * sin(-vehicle_heading);
                
                if (local_x > 0) {
                    double dist = sqrt(dx*dx + dy*dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_index = i;
                        found = true;
                    }
                }
            }
        }
        
        // Last resort: use absolute nearest
        if (!found) {
            RCLCPP_WARN(this->get_logger(), "⚠️  No forward waypoint! Using absolute nearest.");
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < num_waypoints; i++) {
                double dx = waypoints.X[i] - x_car_world;
                double dy = waypoints.Y[i] - y_car_world;
                double dist = sqrt(dx*dx + dy*dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_index = i;
                }
            }
        }
        
        waypoints.index = best_index;
        double final_dx = waypoints.X[best_index] - x_car_world;
        double final_dy = waypoints.Y[best_index] - y_car_world;
        double final_dist = sqrt(final_dx*final_dx + final_dy*final_dy);
        RCLCPP_INFO(this->get_logger(), "📍 Selected waypoint index %d (dist: %.2fm, score: %.2f)", 
                    best_index, final_dist, best_score);
    }

    void switch_raceline(int raceline_id) {
        if (all_racelines.find(raceline_id) == all_racelines.end()) {
            RCLCPP_ERROR(this->get_logger(), "Raceline %d not found!", raceline_id);
            return;
        }
        
        current_raceline_id = raceline_id;
        waypoints = all_racelines[raceline_id];
        num_waypoints = waypoints.X.size();
        
        // Find nearest forward waypoint after switch
        find_nearest_forward_waypoint();
        
        RCLCPP_INFO(this->get_logger(), "🔄 SWITCHED to raceline_%d (%d waypoints, starting at index %d)", 
                    raceline_id, num_waypoints, waypoints.index);
    }

    void raceline_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int new_raceline_id = msg->data;
        
        if (new_raceline_id != current_raceline_id && new_raceline_id >= 1 && new_raceline_id <= max_raceline) {
            switch_raceline(new_raceline_id);
        }
    }

    void print_current_raceline() {
        RCLCPP_INFO(this->get_logger(), "📍 Following raceline_%d", current_raceline_id);
    }
    
    void download_waypoints () { //put all data in vectors
        // This function is deprecated - now using load_all_racelines() instead
    }

    void visualize_points(Eigen::Vector3d &point) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;

        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.id = 1;
        vis_point_pub->publish(marker);

    }

    void get_waypoint() {
    double longest_distance = 0;
    int final_i = -1;
    // Set the search range for waypoints
    int start = waypoints.index;
    int end = (start + 500) % num_waypoints;

    double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead); 

    // Extract the vehicle's orientation from the odometry message
    double vehicle_heading = atan2(2.0 * (odom_quat.z * odom_quat.w + odom_quat.x * odom_quat.y), 1.0 - 2.0 * (odom_quat.y * odom_quat.y + odom_quat.z * odom_quat.z));

    auto checkWaypoint = [&](int i) {
        double dx = waypoints.X[i] - x_car_world;
        double dy = waypoints.Y[i] - y_car_world;
        double waypoint_angle = atan2(dy, dx);
        double angle_diff = abs(waypoint_angle - vehicle_heading);

        // Check if waypoint is in front of the vehicle and within lookahead distance
        if ((angle_diff < M_PI_2 || angle_diff > 3 * M_PI_2) && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) > longest_distance) {
                longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                final_i = i;
            }
        }
    };

    // Iterate over the waypoint range to find the best waypoint
    for (int i = start; i != end; i = (i + 1) % num_waypoints) {
        checkWaypoint(i);
    }

    if (final_i != -1) {
        waypoints.index = final_i; 
    } else if (final_i == -1 && longest_distance == 0) {
        // This handles the case when no waypoint is found within the lookahead distance
        waypoints.index = (waypoints.index + 1) % num_waypoints; // Move to the next waypoint
    }

    // Find the closest point to the car, and use the velocity index for that
    double shortest_distance = p2pdist(waypoints.X[start], x_car_world, waypoints.Y[start], y_car_world);
    int velocity_i = start;
    for (int i = start; i != end; i = (i + 1) % num_waypoints) {
        if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) < shortest_distance) {
            shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            velocity_i = i;
        }
    }

    waypoints.velocity_index = velocity_i;
}




    
    void get_waypoint_obstacle_avoidance() {
        /*
        TO CONSIDER: Edge cases where there are walls. The thing with RRT 
        
        1. Fetch next coordinate for the current lane
        2. Run local occupancy grid, and check if there is anything on the current path. If not, RETURN coordinate
        3. Change lane, and then run step 1 again. 
        
        This is where 

        
        
        */
    }

    void quat_to_rot(double q0, double q1, double q2, double q3) { //w,x,y,z -> q0,q1,q2,q3
        double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
        double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
        double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));
     
        double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
        double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
        double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));
     
        double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
        double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
        double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

        rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22; //fill rotation matrix
    }

    void transformandinterp_waypoint() { //pass old waypoint here
        //initialise vectors
        waypoints.p1_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;

        //rviz publish way point
        visualize_points(waypoints.p1_world);
        //look up transformation at that instant from tf_buffer_
        geometry_msgs::msg::TransformStamped transformStamped;
        
        try {
            // Get the transform from the base_link reference to world reference frame
            transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame,tf2::TimePointZero);
        } 
        catch (tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
        }

        //transform points (rotate first and then translate)
        Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

        waypoints.p1_car = (rotation_m*waypoints.p1_world) + translation_v;
    }

    double p_controller() { // pass waypoint
        double r = waypoints.p1_car.norm(); // r = sqrt(x^2 + y^2)
        double y = waypoints.p1_car(1);
        double angle = K_p * 2 * y / pow(r, 2); // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

        return angle;
    }

    double get_velocity(double steering_angle) {
        double velocity;
        
        if (waypoints.V[waypoints.index]) {  // I find better results using .index vs. .velocity_index
            velocity = waypoints.V[waypoints.index] * velocity_percentage;
        } else { // For waypoints loaded without velocity profiles
            if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
                velocity = 6.0 * velocity_percentage; 
            } 
            else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
                velocity = 2.5 * velocity_percentage;
            } 
            else {
                velocity = 2.0 * velocity_percentage;
            }
        }

        // if (emergency_breaking) velocity = 0.0;  // Do not move if you are about to run into a wall
            
        return velocity;
    }

    void publish_message (double steering_angle) {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        if (steering_angle < 0.0) {
            drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit)); //ensure steering angle is dynamically capable
        } else {
            drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit)); //ensure steering angle is dynamically capable
        }

        curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
        drive_msgObj.drive.speed = curr_velocity;

        RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p, velocity_percentage);

        publisher_drive->publish(drive_msgObj);
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
        odom_quat = odom_submsgObj->pose.pose.orientation;
        x_car_world = odom_submsgObj->pose.pose.position.x;
        y_car_world = odom_submsgObj->pose.pose.position.y;
        // interpolate between different way-points 
        get_waypoint();
        // Lane switching implementation
        get_waypoint_obstacle_avoidance();

        //use tf2 transform the goal point 
        transformandinterp_waypoint();

        // Calculate curvature/steering angle
        double steering_angle = p_controller();

        //publish object and message: AckermannDriveStamped on drive topic 
        publish_message(steering_angle);
    }
    
    void timer_callback () {
        // Periodically check parameters and update
        K_p = this->get_parameter("K_p").as_double();
        velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
