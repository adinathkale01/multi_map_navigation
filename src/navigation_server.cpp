#include "multi_map_navigation/navigation_server.h" // Include the header for this class

NavigationServer::NavigationServer(const std::string &db_path)
    : as_(nh_, "navigate_to_goal", boost::bind(&NavigationServer::execute, this, _1), false), // Initialize action server
      wormhole_manager_(db_path) // Initialize wormhole manager with the DB path
{
    current_map_ = "room1";  // Assume initial map is "room1"
    as_.start();  // Start the action server
    ROS_INFO("Navigation Action Server started successfully. Ready to receive goals.");
}

bool NavigationServer::move_base_to(double x, double y, double yaw)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // Create action client for move_base
    if (!ac.waitForServer(ros::Duration(5.0))) // Wait max 5 seconds for server
    {
        ROS_ERROR("Failed to connect to move_base server within timeout.");
        return false;
    }

    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map"; // Reference map frame
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.pose.position.x = x;
    mb_goal.target_pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw); // Convert yaw to quaternion
    q.normalize();
    mb_goal.target_pose.pose.orientation = tf2::toMsg(q); // Convert quaternion to ROS message


    ac.sendGoal(mb_goal); // Send goal
    ac.waitForResult();   // Block until goal is reached

    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED; // Return whether the goal succeeded
}

// Action Callback: Handling Navigation Goals
void NavigationServer::execute(const multi_map_navigation::NavigateToGoalGoalConstPtr &goal)
{
    ROS_INFO("Received goal: target_map: %s, target_x: %f, target_y: %f",
             goal->target_map.c_str(), goal->target_x, goal->target_y);

    double yaw = M_PI / 2; // Default yaw (90 degrees facing up)
    multi_map_navigation::NavigateToGoalResult result;


    // If the robot is already in the target map:
    if (goal->target_map == current_map_)
    {
        ROS_INFO("Already in target map. Moving directly to goal...");
        if (!move_base_to(goal->target_x, goal->target_y, yaw))
        {
            result.success = false;
            result.message = "Failed to reach goal";
            as_.setAborted(result);
            return;
        }
        result.success = true;
        result.message = "Reached";
        ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
        as_.setSucceeded(result);
        return;
    }

    // Try to find a direct wormhole from current → target map:

    auto direct = wormhole_manager_.getWormholeToMap(current_map_, goal->target_map);
    if (direct.first != -9999 && direct.second != -9999)
    {
        ROS_INFO("Found direct wormhole from %s to %s", current_map_.c_str(), goal->target_map.c_str());

        if (!move_base_to(direct.first, direct.second, yaw))
        {
            result.success = false;
            result.message = "Failed to reach wormhole";
            as_.setAborted(result);
            return;
        }

        map_switcher_.switchToMap(goal->target_map); // Switch maps using MapSwitcher
        current_map_ = goal->target_map;

        if (!move_base_to(goal->target_x, goal->target_y, yaw))
        {
            result.success = false;
            result.message = "Failed to reach goal";
            as_.setAborted(result);
            return;
        }

        result.success = true;
        result.message = "Reached";
        ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
        as_.setSucceeded(result);
        return;
    }

    //If no direct wormhole, route via room1 (default hub):

    if (current_map_ != "room1")
    {
        auto to_intermediate = wormhole_manager_.getWormholeToMap(current_map_, "room1");
        if (to_intermediate.first == -9999 || to_intermediate.second == -9999)
        {
            ROS_ERROR("No wormhole from %s to map1", current_map_.c_str());
            result.success = false;
            result.message = "No wormhole to map1";
            as_.setAborted(result);
            return;
        }

        ROS_INFO("Moving from %s to map1 via (%f, %f)", current_map_.c_str(), to_intermediate.first, to_intermediate.second);

        if (!move_base_to(to_intermediate.first, to_intermediate.second, yaw))
        {
            result.success = false;
            result.message = "Failed to reach intermediate wormhole";
            as_.setAborted(result);
            return;
        }

        map_switcher_.switchToMap("room1");
        current_map_ = "room1";
    }
    
    //  From room1, move to final target map:



    auto to_final = wormhole_manager_.getWormholeToMap("room1", goal->target_map);
    if (to_final.first == -9999 || to_final.second == -9999)
    {
        ROS_ERROR("No wormhole from map1 to %s", goal->target_map.c_str());
        result.success = false;
        result.message = "No wormhole from map1 to target";
        as_.setAborted(result);
        return;
    }

    ROS_INFO("Moving from map1 to %s via (%f, %f)", goal->target_map.c_str(), to_final.first, to_final.second);

    if (!move_base_to(to_final.first, to_final.second, yaw))
    {
        result.success = false;
        result.message = "Failed to reach final wormhole";
        as_.setAborted(result);
        return;
    }

    map_switcher_.switchToMap(goal->target_map);
    current_map_ = goal->target_map;

    if (!move_base_to(goal->target_x, goal->target_y, yaw))
    {
        result.success = false;
        result.message = "Failed to reach goal";
        as_.setAborted(result);
        return;
    }

    result.success = true;
    result.message = "Reached";
    ROS_INFO("Robot successfully reached the goal in map: %s", current_map_.c_str());
    as_.setSucceeded(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_server_node"); // Initialize ROS node
    ros::NodeHandle nh;

    std::string db_path;
    nh.param<std::string>("wormhole_db_path", db_path,
                          ros::package::getPath("multi_map_navigation") + "/database/wormholes.db");// Get DB path param

    ROS_INFO("Starting navigation server with wormhole database at: %s", db_path.c_str());
    NavigationServer navigation_server(db_path); // Create server object


    ros::spin(); // Keep node alive and responsive

    return 0;
}
