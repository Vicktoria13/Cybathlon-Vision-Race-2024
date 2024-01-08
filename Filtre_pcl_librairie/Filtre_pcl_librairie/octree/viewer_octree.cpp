#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) {
  octomap::OcTree* octree = new octomap::OcTree("my_octree.bt");
  octovis::OcViewer viewer;
  viewer.show();

  octomap::point3d bbx_min(-1.0, -1.0, -1.0);
  octomap::point3d bbx_max(1.0, 1.0, 1.0);
  viewer.setBoundingBox(bbx_min, bbx_max);

  viewer.addOcTree(*octree);

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }

  delete octree;
  return 0;
}


//g++ -o viewer_octree viewer_octree.cpp -loctomap -loctomath -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lboost_date_time -lboost_regex -lboost_signals -lboost_program_options -lboost_chrono -lboost_atomic -lboost_serialization -lboost_locale -lboost_log -lboost_log_setup -lpthread -lboost_system -lboost_filesystem -lboost_iostreams -lboost_thread -lboost_date_time -lboost_regex -lboost_signals -lboost_program_options -lboost_chrono -lboost_atomic -lboost_serialization -lboost_locale -lboost_log -lboost_log_setup -lpthread