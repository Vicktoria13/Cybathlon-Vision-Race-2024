cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
 
 project(passthrough)
 
 find_package(PCL 1.2 REQUIRED)
 include_directories(${PCL_INCLUDE_DIRS})



add_executable (filtring ../src/filtering.cpp)
target_link_libraries (filtring ${PCL_LIBRARIES})


add_executable (voxel_grid ../src/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})


project(pcd_read)



add_executable (pcd_read ../src/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})



project(cloud_viewer)

#version release with debug info

#on ajoute a $PCL_INCLUDE_DIRS notre dossier include



add_executable (cloud_viewer ../src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (cloud_viewer_2 ../src/cloud_viewer_2.cpp)
target_link_libraries (cloud_viewer_2 ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo ../src/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable (point_cloud_compression ../src/point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})

add_executable (octree_change_detection ../src/octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})

add_executable (correspondence_grouping ../src/correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (statistical_removal ../src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (pcd_write ../src/pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (resampling ../src/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (remove_outliers ../src/remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})

add_executable (filtreAndCrop ../src/filtreAndCrop.cpp)
target_link_libraries (filtreAndCrop ${PCL_LIBRARIES})

add_executable (median_filter ../src/median_filter.cpp)
target_link_libraries (median_filter ${PCL_LIBRARIES})

add_executable (crop ../src/crop.cpp)
target_link_libraries (crop ${PCL_LIBRARIES})

add_executable (movingLS ../src/movingLS.cpp)
target_link_libraries (movingLS ${PCL_LIBRARIES})

message("STATUS" "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")