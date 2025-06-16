#include "multi_map_navigation/wormhole_manager.h" // Include the header file for this class

// Constructor: Opens a connection to the SQLite database
WormholeManager::WormholeManager(const std::string &db_path)
{   
    // Attempt to open the database file at the provided path
    if (sqlite3_open(db_path.c_str(), &db_))
    {
        // If opening fails, print the error message using ROS logging
        ROS_ERROR("Failed to open database: %s", sqlite3_errmsg(db_));
        db_ = nullptr; // Ensure db_ is null if opening failed
    }
}

// Destructor: Closes the SQLite database connection if open
WormholeManager::~WormholeManager()
{
    if (db_)
        sqlite3_close(db_); // Close the database connection
}

/**
 * Function: getWormholeToMap
 * Purpose : Retrieve the coordinates of the wormhole that leads from the current map to the target map.
 * Inputs  : current_map - name of the current map
 *           target_map  - name of the target map
 * Output  : A pair (x, y) representing the wormhole coordinates; (-9999, -9999) if not found
 */

std::pair<double, double> WormholeManager::getWormholeToMap(const std::string &current_map, const std::string &target_map)
{
    // Default coordinates if wormhole not found
    double x =  -9999 , y = -9999;


    // SQL query to select wormhole coordinates based on map names
    std::string sql = "SELECT from_x, from_y FROM wormholes WHERE from_map = ? AND to_map = ?;";
    sqlite3_stmt *stmt; // SQLite statement object


    // Log the SQL query being executed
    ROS_INFO("Executing query: %s", sql.c_str());

    // Prepare the SQL statement
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK)
    { 
        // Bind the current_map to the first placeholder (?)
        sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_TRANSIENT);
        // Bind the target_map to the second placeholder (?)
        sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_TRANSIENT);

        // Execute the query and check if a row is returned
        int step_result = sqlite3_step(stmt);
        if (step_result == SQLITE_ROW)
        {

            // Retrieve the x and y values from the result row
            x = sqlite3_column_double(stmt, 0);           
            y = sqlite3_column_double(stmt, 1);

            // Log the coordinates           
            ROS_INFO("Wormhole found at (%f, %f)", x, y); 
        }
        else
        {
            // Warn that no wormhole was found
            ROS_WARN("No wormhole found from %s to %s. Step result: %d", current_map.c_str(), target_map.c_str(), step_result);
        }
         // Finalize the prepared statement to free memory
        sqlite3_finalize(stmt); 
    }
    else
    {
        // Log error if preparing the statement failed
        ROS_ERROR("Query failed: %s", sqlite3_errmsg(db_));
    }
    // Return the result (x, y) coordinates
    return {x, y};
}


// Constructor: Opens a connection to the database.
// Destructor: Closes the connection safely.
// getWormholeToMap: Queries the database to find a wormhole coordinate (from_x, from_y) where from_map = current_map and to_map = target_map.