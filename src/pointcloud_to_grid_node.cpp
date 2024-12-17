#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <pointcloud_to_grid/MyParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "map_saver.hpp"

nav_msgs::OccupancyGridPtr height_grid(new nav_msgs::OccupancyGrid);
GridMap grid_map;
ros::Publisher pub_hgrid; // Publisher for the height grid
ros::Subscriber sub_pc2;  // Subscriber for the point cloud
ros::Publisher pub_modified_pc2; // Publisher for the transformed point cloud

// Persistent tf2 objects
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener = nullptr;

PointXY getIndex(double x, double y) {
  PointXY ret;
  ret.x = int(fabs(x - grid_map.topleft_x) / grid_map.cell_size);
  ret.y = int(fabs(y - grid_map.topleft_y) / grid_map.cell_size);
  return ret;
}

void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level) {
  grid_map.cell_size = config.cell_size;
  grid_map.position_x = config.position_x;
  grid_map.position_y = config.position_y;
  grid_map.cell_size = config.cell_size;
  grid_map.length_x = config.length_x;
  grid_map.length_y = config.length_y;
  grid_map.cloud_in_topic = config.cloud_in_topic;
  grid_map.height_factor = config.height_factor;
  grid_map.frame_out = config.frame_out;
  grid_map.maph_topic_name = config.maph_topic_name;
  grid_map.cloud_in_topic = config.cloud_in_topic;
  grid_map.background_color = config.background_color;
  grid_map.occupied_color = config.occupied_color;
  grid_map.save_map_path = config.save_map_path;
  grid_map.save_map_name = config.save_map_name;
  grid_map.initGrid(height_grid);
  grid_map.paramRefresh();


  // Check if the save_map parameter is set to true
  if (config.save_map) {
      ROS_INFO("Save map trigger received!");

      // Save the map
      saveOccupancyGridAsPGM(*height_grid, grid_map.save_map_path, grid_map.save_map_name);

      ROS_INFO("Map saved successfully to: %s", (grid_map.save_map_path + grid_map.save_map_name).c_str());

      // Reset the save_map parameter to false
      config.save_map = false;
  }

}

void pointcloudCallback(const pcl::PCLPointCloud2 &msg) {
  pcl::PointCloud<pcl::PointXYZ> out_cloud;
  pcl::fromPCLPointCloud2(msg, out_cloud);

  geometry_msgs::TransformStamped transformStamped;

  // Retrieve the transform from "world" to "world_orb"
  try {
    transformStamped = tfBuffer.lookupTransform("world", "world_orb", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Transform point cloud to world frame
  for (auto& point : out_cloud.points) {
    geometry_msgs::Point point_msg;
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.z = point.z;

    geometry_msgs::Point transformed_point_msg;
    tf2::doTransform(point_msg, transformed_point_msg, transformStamped);

    // Update the point coordinates after transformation
    point.x = transformed_point_msg.x;
    point.y = transformed_point_msg.y;
    point.z = transformed_point_msg.z;
  }

  // Delete points above and below the height threshold
  float max_z = 100.0;
  float min_z = 0.2;
  out_cloud.erase(std::remove_if(out_cloud.points.begin(), out_cloud.points.end(),
                                 [min_z, max_z](const pcl::PointXYZ &p) {
                                   return p.z < min_z || p.z > max_z;
                                 }),
                  out_cloud.points.end());
  
  // Delete outlier points
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(out_cloud.makeShared());
  sor.setMeanK(5);
  sor.setStddevMulThresh(1.0);
  sor.filter(out_cloud);
  

  // Publish the transformed point cloud
  sensor_msgs::PointCloud2 transformed_pc_msg;
  pcl::toROSMsg(out_cloud, transformed_pc_msg);
  transformed_pc_msg.header.stamp = ros::Time::now();
  transformed_pc_msg.header.frame_id = "world"; // Ensure the correct frame
  pub_modified_pc2.publish(transformed_pc_msg);



  // Initialize grid
  grid_map.initGrid(height_grid);
  std::vector<signed char> hpoints(grid_map.cell_num_x * grid_map.cell_num_y);

  for (auto& p : hpoints) { p = grid_map.background_color; }

  for (auto& out_point : out_cloud.points) {
    // Only process points above and below the height threshold
    if (out_point.z > min_z && out_point.z < max_z) {
      if (out_point.x > 0.01 || out_point.x < -0.01) {
        if (out_point.x > grid_map.bottomright_x && out_point.x < grid_map.topleft_x) {
          if (out_point.y > grid_map.bottomright_y && out_point.y < grid_map.topleft_y) {
            PointXY cell = getIndex(out_point.x, out_point.y);
            if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y) {
              hpoints[cell.y * grid_map.cell_num_x + cell.x] = grid_map.occupied_color; //prev: out_point.z * grid_map.height_factor;
            } else {
              ROS_WARN_STREAM("Cell out of range: " << cell.x << " - " << grid_map.cell_num_x
                                                    << " ||| " << cell.y << " - " << grid_map.cell_num_y);
            }
          }
        }
      }
    }
  }

  height_grid->header.stamp = ros::Time::now();
  height_grid->header.frame_id = "world";  // Set frame to world frame
  height_grid->info.map_load_time = ros::Time::now();
  height_grid->data = hpoints;
  pub_hgrid.publish(height_grid);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_to_grid_node"); // This should be called first!

  ros::NodeHandle nh;

  // Initialize dynamic reconfigure
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;
  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);

  // Initialize persistent tf2 objects (to read transforms)
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  // Publishers and Subscribers
  pub_hgrid = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.maph_topic_name, 1);
  pub_modified_pc2 = nh.advertise<sensor_msgs::PointCloud2>("grid_pointcloud", 1); // New publisher
  sub_pc2 = nh.subscribe(grid_map.cloud_in_topic, 1, pointcloudCallback);

  ros::spin();

  delete tfListener;  // Clean up
  return 0;
}
