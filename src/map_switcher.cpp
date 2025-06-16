#include "multi_map_navigation/map_switcher.h" // Include the header for the MapSwitcher class

// Constructor for the MapSwitcher class
MapSwitcher::MapSwitcher() : nh_()  // Initialize the ROS NodeHandle
{
    // Retrieve the parameter "map_folder" from the ROS pararmeter server.
    // If not set, Use the default path:"<pacakge_path>/maps"
    nh_.param<std::string>("map_folder", map_folder_,
                           ros::package::getPath("multi_map_navigation") + "/maps");

    // Log the map folder being used
    ROS_INFO("Map switcher initialized with map folder: %s", map_folder_.c_str());
    if (map_folder_.empty())
    {
        ROS_ERROR("Map folder is not set. Please set the 'map_folder' parameter.");
        return;
    }
}
// Function to switch to a new map given by its name (without .yaml extension )
void MapSwitcher::switchToMap(const std::string &map_name)
{   
    // Construct the full path to the map YAML file
    std::string map_yaml_path = map_folder_ + "/" + map_name + ".yaml";

    // Try to open the map file to check if it exists and is readable
    std::ifstream map_file(map_yaml_path);
    if (!map_file.is_open())
    {
        ROS_ERROR("Failed to open map file: %s", map_yaml_path.c_str());
        return; // Exit if the file can't be opend
    }

    // Validate that the map YAML file contains an "image:" line

    std::string line;
    bool valid = false;
    while (std::getline(map_file, line))
    {
        if (line.find("image:") != std::string::npos)
        {
            valid = true; // File is considered valid if contains an image entry
            break;
        }
    }

    if (!valid)
    {
        ROS_ERROR("Invalid map file format: %s", map_yaml_path.c_str());
        return; // Exit if the map YAML doesn't have a valid image field 
    }
    
    // close the file after checking 
    map_file.close();

    // Command to launch the map_server node with the selected map

    std::string command = "rosrun map_server map_server " + map_yaml_path;

    // Use system() to run the command in background (&) - not best practice in production
   
    int result = system((command + " &").c_str());
    ros::Duration(1.0).sleep();

    // Give some time for command executed successfully 

    if (result == 0)
    {
        ROS_INFO("Successfully switched to map: %s", map_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to switch to map: %s", map_name.c_str());
    }
}



// 1. MapSwitcher is a helper class designed to dynamically switch maps at runtime in a multi-map navigation ROS setup.
// 2. It loads a new map by launching map_server with a new .yaml file.
// It includes:
// parameter reading(map_folder)
// File existence and format checking
// System call to launch map server 