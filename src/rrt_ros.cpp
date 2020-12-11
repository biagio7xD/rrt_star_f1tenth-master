#include "rrt/rrt.h"
#include "rrt/csv_reader.h"

#include <visualization_msgs/Marker.h>

#include <thread>
#include <chrono>
#include <nav_msgs/Odometry.h>

static const bool debug = false;
const int MIN_ITER = 1300;

/// RRT Object Constructor
/// @param nh - node handle to the ros node
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()), tf2_listener_(tf_buffer_)
{
    // ROS Topic Names
    std::string pose_topic, scan_topic, drive_topic, global_plan_path, local_path_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("local_path_topic", local_path_topic);

    ros::NodeHandle privateNodeHandler("~");
    privateNodeHandler.param<std::string>("global_trajectory", global_plan_path, "");
    cout<<global_plan_path<<"\n";

    // Load Input Map from map_server
    input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

    if(input_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    ROS_INFO("Map Load Successful.");

    // Read Global Path
    Global_Plan_Reader::TRJReader reader(global_plan_path);

    global_path_ = reader.getData();

    // get transform from laser to map
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // ROS Publishers and Subscribers

    //Pub dynamic map
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);

    //sub to odom and scan topic
    scan_sub_ = nh_.subscribe(scan_topic, 1, &RRT::scan_callback, this);
    sleep(1);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &RRT::pose_callback, this);


    //pub rrt local path
    local_path_pub = nh_.advertise<nav_msgs::Path>(local_path_topic, 1);


    //pub viz topic
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_viz_marker", 1);
    waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_viz_marker", 1000);

    // Tree Visualization
    points_.header.frame_id = line_strip_.header.frame_id = "/map";
    points_.ns = "rrt_viz";
    points_.pose.orientation.w = line_strip_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_LIST;
    points_.scale.x = 0.1;
    points_.scale.y = 0.1;
    line_strip_.scale.x = 0.05;
    points_.color.g = 1.0f;
    points_.color.a = 1.0;
    line_strip_.color.r = 1.0;
    line_strip_.color.a = 1.0;


    // Map Data
    map_cols_ = input_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    // RRT Parameters
    nh_.getParam("max_rrt_iters", max_rrt_iters_);
    nh_.getParam("max_expansion_distance", max_expansion_distance_);
    nh_.getParam("collision_checking_points", collision_checking_points_);
    nh_.getParam("goal_tolerance", goal_tolerance_);
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance_);

    // Local Map Parameters
    nh_.getParam("inflation_radius", inflation_radius_);

    // RRT* Parameters
    nh_.getParam("enable_rrtstar", enable_rrtstar_);
    nh_.getParam("search_radius", search_radius_);


    ROS_INFO("Created new RRT Object.");
}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }
    const auto translation = tf_laser_to_map_.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map_.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/6);
    const auto end = static_cast<int>(5*scan_msg->ranges.size()/6);
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*(start-1);

    for(int i=start; i<end; ++i)
    {
        theta+=angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        // laser hit x, y in base_link frame
        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > lookahead_distance_ || y_base_link > lookahead_distance_) continue;

        // laser hit x, y in base_link frame
        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        std::vector<int> index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map);

        for(const auto& index: index_of_expanded_obstacles)
        {
            if(input_map_.data[index] != 100)
            {
                input_map_.data[index] = 100;
                new_obstacles_.emplace_back(index);
            }
        }
    }

    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 10)
    {
        for(const auto index: new_obstacles_)
        {
            input_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        //ROS_INFO("Obstacles Cleared");
    }

    dynamic_map_pub_.publish(input_map_);
   // ROS_INFO("Map Updated");
}

/// The pose callback when subscribed to particle filter's inferred pose (RRT* Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    current_x_ = pose_msg->pose.pose.position.x;
    current_y_ = pose_msg->pose.pose.position.y;

    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y});

    std::vector<Node> tree;

    tree.emplace_back(Node(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, 0.0, -1));

    std::cout<<local_path_.size()<<"\n";
    if(local_path_.size() == 0) {

        int count = 0;
        while (count < max_rrt_iters_) {
            count++;

            // Sample a Node
            const auto sample_node = sample();

            // Check if it is not occupied
            if (is_collided(sample_node[0], sample_node[1])) {
                ROS_DEBUG("Sample Node Colliding");
                continue;
            }

            // Find the nearest node in the tree to the sample node
            const int nearest_node_id = nearest(tree, sample_node);

            // Steer the tree from the nearest node towards the sample node
            Node new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);

            const auto current_node_index = tree.size();

            // Check if the segment joining the two nodes is in collision
            if (is_collided(tree[nearest_node_id], new_node)) {
                ROS_DEBUG("Sample Node Edge Colliding");
                continue;
            } else if (enable_rrtstar_) {
                new_node.cost = cost(tree, new_node);
                const auto near_neighbour_indices = near(tree, new_node);

                std::vector<bool> is_near_neighbor_collided;
                int best_neighbor = new_node.parent_index;

                for (const int near_node_index: near_neighbour_indices) {
                    if (is_collided(tree[near_node_index], new_node)) {
                        is_near_neighbor_collided.push_back(true);
                        continue;
                    }
                    is_near_neighbor_collided.push_back(false);

                    double cost = tree[near_node_index].cost + line_cost(tree[near_node_index], new_node);

                    if (cost < new_node.cost) {
                        new_node.cost = cost;
                        new_node.parent_index = near_node_index;
                        best_neighbor = near_node_index;
                    }
                }

                for (int i = 0; i < near_neighbour_indices.size(); i++) {
                    if (is_near_neighbor_collided[i] || i == best_neighbor) {
                        continue;
                    }
                    if (tree[near_neighbour_indices[i]].cost > new_node.cost + line_cost(
                            new_node, tree[near_neighbour_indices[i]])) {
                        ROS_DEBUG("Rewiring Parents");
                        tree[near_neighbour_indices[i]].parent_index = current_node_index;
                    }
                }
            }
            tree.emplace_back(new_node);

            ROS_DEBUG("Sample Node Edge Found");
            if (is_goal(new_node, trackpoint[0], trackpoint[1]) && count >= MIN_ITER) {
                ROS_DEBUG("Goal reached. Backtracking ...");
                local_path_ = find_path(tree, new_node);
                ROS_INFO("Path Found");
                break;
            }
        }
    }

    if(!local_path_.empty())
    {
        publish_path(local_path_);
    }

    if(!local_path_.empty())
    {

        const auto distance_sqr = hypot(current_x_ - local_path_[0][0], current_y_ - local_path_[0][1]);
        std::cout << "DISTANZA DAL GOAL :" << distance_sqr << "\n";
        if (distance_sqr <= 3.0) { local_path_.clear(); }
    }
}




/// Publish way point with respect to the given input properties
/// @param way_point - (x, y) wrt to the frame id
/// @param frame_id - frame represented in waypoint
/// @param r - red intensity
/// @param g - green intensity
/// @param b - blue intenisty
/// @param transparency (Do not put 0)
/// @param scale_x
/// @param scale_y
/// @param scale_z
void RRT::add_way_point_visualization(const std::array<double, 2>& way_point, const std::string& frame_id, double r,
        double g, double b, double transparency = 0.5, double scale_x=0.2, double scale_y=0.2, double scale_z=0.2)
{
    visualization_msgs::Marker way_point_marker;
    way_point_marker.header.frame_id = frame_id;
    way_point_marker.header.stamp = ros::Time();
    way_point_marker.ns = "pure_pursuit";
    way_point_marker.id = unique_id_;
    way_point_marker.type = visualization_msgs::Marker::SPHERE;
    way_point_marker.action = visualization_msgs::Marker::ADD;
    way_point_marker.pose.position.x = way_point[0];
    way_point_marker.pose.position.y = way_point[1];
    way_point_marker.pose.position.z = 0;
    way_point_marker.pose.orientation.x = 0.0;
    way_point_marker.pose.orientation.y = 0.0;
    way_point_marker.pose.orientation.z = 0.0;
    way_point_marker.pose.orientation.w = 1.0;
    way_point_marker.scale.x = 0.2;
    way_point_marker.scale.y = 0.2;
    way_point_marker.scale.z = 0.2;
    way_point_marker.color.a = transparency;
    way_point_marker.color.r = r;
    way_point_marker.color.g = g;
    way_point_marker.color.b = b;
    waypoint_viz_pub_.publish(way_point_marker);
    unique_id_++;
}

/// visualize all way points in the global path
void RRT::visualize_waypoint_data()
{
    const size_t increment = global_path_.size()/100;
    for(size_t i=0; i<global_path_.size(); i=i+increment)
    {
        add_way_point_visualization(global_path_[i], "map", 1.0, 0.0, 1.0, 0.5);
    }
    //ROS_INFO("Published All Global WayPoints.");
}
