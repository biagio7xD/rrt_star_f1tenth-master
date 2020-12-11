#include "rrt/rrt.h"
#include "rrt/csv_reader.h"



std::array<double, 2> RRT::sample()
{
    std::uniform_real_distribution<>::param_type x_param(0, lookahead_distance_);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_distance_, lookahead_distance_);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;
    tf2::doTransform(sample_point, sample_point, tf_laser_to_map_);

    return {sample_point.position.x, sample_point.position.y};
}


int RRT::nearest(const std::vector<Node> &tree, const std::array<double, 2> &sampled_point)
{
    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();
    for(size_t i=0; i< tree.size(); i++)
    {
        const auto distance_sqr = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if(distance_sqr < nearest_node_distance)
        {
            nearest_node = i;
            nearest_node_distance = distance_sqr;
        }
    }
    return nearest_node;
}


Node RRT::steer(const Node &nearest_node, const int nearest_node_index, const std::array<double, 2> &sampled_point)
{
    const double x = sampled_point[0] - nearest_node.x;
    const double y = sampled_point[1] - nearest_node.y;
    const double distance = pow(x, 2) + pow(y, 2);

    Node new_node{};
    if(distance < max_expansion_distance_)
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        const double theta = atan2(y, x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}


bool RRT::is_collided(const Node &nearest_node, const Node &new_node)
{
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(is_collided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}


bool RRT::is_goal(const Node &latest_added_node, double goal_x, double goal_y)
{
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2)+pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance_;
}


std::vector<std::array<double, 2>> RRT::find_path(const std::vector<Node> &tree, const Node &latest_added_node)
{
    std::vector<std::array<double, 2>> found_path;
    Node current_node = latest_added_node;

    while(current_node.parent_index != -1)
    {
        std::array<double, 2> local_trackpoint{current_node.x, current_node.y};
        found_path.emplace_back(local_trackpoint);
        current_node = tree[current_node.parent_index];
    }

    if(current_node.parent_index == -1)
    {
        std::array<double, 2> local_trackpoint{current_node.x, current_node.y};
        found_path.emplace_back(local_trackpoint);
    }
    return found_path;
}


bool RRT::is_collided(const double x_map, const double y_map)
{
    const auto index = get_row_major_index(x_map, y_map);
    return input_map_.data[index] == 100;
}


double RRT::cost(const std::vector<Node> &tree, const Node &new_node)
{
    return tree[new_node.parent_index].cost + line_cost(tree[new_node.parent_index], new_node);
}


double RRT::line_cost(const Node &n1, const Node &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}


std::vector<int> RRT::near(const std::vector<Node> &tree, const Node &node)
{
    std::vector<int> near_neighbor_indices;
    for(int i=0; i<tree.size(); i++)
    {
        const double distance = sqrt(pow(node.x - tree[i].x, 2) + pow(node.y - tree[i].y, 2));
        if(distance < search_radius_)
        {
            near_neighbor_indices.push_back(i);
        }
    }
    return near_neighbor_indices;
}

bool RRT::checkpoint_collision(const double x_map, const double y_map, int inflation_obs)
{
    if(is_collided(x_map, y_map))
    {
        return true;
    }
    return false;
}


