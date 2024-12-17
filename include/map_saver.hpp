#ifndef MAP_SAVER_HPP
#define MAP_SAVER_HPP

#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>  // For extracting yaw from quaternions

// Function declaration
void saveOccupancyGridAsPGM(const nav_msgs::OccupancyGrid& grid, const std::string& map_path, const std::string& map_name);

#endif // MAP_SAVER_HPP
