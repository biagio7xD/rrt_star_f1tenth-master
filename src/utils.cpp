//
// Created by biagio on 26/07/20.
//
#include "rrt/rrt.h"
#include "rrt/cubic_spline_planner.hpp"


void RRT::publish_path(std::vector<std::array<double, 2>> path)
{


    vecD x,y;

    for(int i=path.size()-1; i>=0; i--)
    {
        x.emplace_back(path[i][0]);
        y.emplace_back(path[i][1]);
    }
    vecD tx, ty, tyaw, tc; //punti interpolazione x ed y == tx ed ty
    Spline2D csp = calc_spline_course(x, y, tx, ty, tyaw, tc, 0.3);

    for(int i = 0; i<tx.size(); i++)
    {
        if(checkpoint_collision(tx.at(i), ty.at(i), 1))
        {
            local_path_.clear();
        }
    }

    if(local_path_.size() > 0) {

        //path__.header.frame_id = "map";
        nav_msgs::Path path_to_pub;
        path_to_pub.header.frame_id="map";

        visualization_msgs::Marker path_dots;
        path_dots.header.frame_id = "map";
        path_dots.id = 20;
        path_dots.ns = "path";
        path_dots.type = visualization_msgs::Marker::POINTS;
        path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.05;
        path_dots.action = visualization_msgs::Marker::ADD;
        path_dots.pose.orientation.w = 1.0;
        path_dots.color.g = 0.0;
        path_dots.color.r = 1.0;
        path_dots.color.a = 1.0;

        for (int i = 0; i < tx.size(); i++) {
            geometry_msgs::Point p; // for visualization
            geometry_msgs::PoseStamped coord;
            p.x = tx.at(i);
            p.y = ty.at(i);

            coord.pose.position.x = tx.at(i);
            coord.pose.position.y = ty.at(i);

            path_dots.points.push_back(p);
            path_to_pub.poses.emplace_back(coord);

        }
        local_path_pub.publish(path_to_pub);
        tree_viz_pub_.publish(path_dots);
    }
}



int RRT::get_row_major_index(const double x_map, const double y_map)
{
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    return y_index*map_cols_ + x_index;
}


std::vector<int> RRT::get_expanded_row_major_indices(const double x_map, const double y_map)
{
    std::vector<int> expanded_row_major_indices;
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    for(int i=-inflation_radius_+x_index; i<inflation_radius_+1+x_index; i++)
    {
        for(int j=-inflation_radius_+y_index; j<inflation_radius_+1+y_index; j++)
        {
            expanded_row_major_indices.emplace_back(j*map_cols_ + i);
        }
    }
    return expanded_row_major_indices;
}

//return the next goal at a pre-defined distance
std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose)
{
    try
    {
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for(int i=0; i<global_path_.size(); ++i)
    {
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = global_path_[i][0];
        goal_way_point.position.y = global_path_[i][1];
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

        if(goal_way_point.position.x < 0) continue;

        double distance = std::abs(lookahead_distance_ -
                                   sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2)));

        if(distance < best_trackpoint_distance)
        {
            const auto row_major_index = get_row_major_index(global_path_[i][0], global_path_[i][1]);
            if (input_map_.data[row_major_index] == 100) continue;
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    return global_path_[best_trackpoint_index];
}
