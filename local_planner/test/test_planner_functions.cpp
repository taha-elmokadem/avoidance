#include <gtest/gtest.h>

#include "../src/nodes/planner_functions.h"

#include "../src/nodes/common.h"

using namespace avoidance;

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z));
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  double distance = 1.0;

  std::vector<double> e_angle_filled = {-90, -30, 0, 20, 40, 90};
  std::vector<double> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;

  for (int i = 0; i < e_angle_filled.size(); i++) {
    for (int j = 0; j < z_angle_filled.size(); j++) {
      middle_of_cell.push_back(fromPolarToCartesian(e_angle_filled[i],
                                                    z_angle_filled[j], distance,
                                                    location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  std::vector<int> e_index;
  std::vector<int> z_index;
  for (int i = 0; i < e_angle_filled.size(); i++) {
    e_index.push_back(elevationAngletoIndex((int)e_angle_filled[i], ALPHA_RES));
    z_index.push_back(azimuthAngletoIndex((int)z_angle_filled[i], ALPHA_RES));
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_DOUBLE_EQ(1.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      } else {
        EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      }
    }
  }
}

TEST(PlannerFunctions, testDirectionTree) {
  // GIVEN: the node positions in a tree and some possible vehicle positions
  geometry_msgs::Point n0;
  n0.x = 0.0f;
  n0.y = 0.0f;
  n0.z = 2.5;
  geometry_msgs::Point n1;
  n1.x = 0.8f;
  n1.y = sqrtf(1 - (n1.x * n1.x));
  n1.z = 2.5;
  geometry_msgs::Point n2;
  n2.x = 1.5f;
  n2.y = n1.y + sqrtf(1 - powf(n2.x - n1.x, 2));
  n2.z = 2.5;
  geometry_msgs::Point n3;
  n3.x = 2.1f;
  n3.y = n2.y + sqrtf(1 - powf(n3.x - n2.x, 2));
  n3.z = 2.5;
  geometry_msgs::Point n4;
  n4.x = 2.3f;
  n4.y = n3.y + sqrtf(1 - powf(n4.x - n3.x, 2));
  n4.z = 2.5;
  const std::vector<geometry_msgs::Point> path_node_positions = {n4, n3, n2, n1,
                                                                 n0};

  Eigen::Vector3f p, p1, p2;
  Eigen::Vector3f postion(0.2, 0.3, 1.5);
  Eigen::Vector3f postion1(1.1, 2.3, 2.5);
  Eigen::Vector3f postion2(5.4, 2.0, 2.5);

  // WHEN: we look for the best direction to fly towards
  bool res = getDirectionFromTree(p, path_node_positions, postion);
  bool res1 = getDirectionFromTree(p1, path_node_positions, postion1);
  bool res2 = getDirectionFromTree(p2, path_node_positions, postion2);

  // THEN: we expect a direction in between node n1 and n2 for position, between
  // node n3 and n4 for position1, and not to get an available tree for the
  // position2
  ASSERT_TRUE(res);
  EXPECT_FLOAT_EQ(45.0f, p.x());
  EXPECT_FLOAT_EQ(57.0f, p.y());

  ASSERT_TRUE(res1);
  EXPECT_FLOAT_EQ(0.0f, p1.x());
  EXPECT_FLOAT_EQ(72.0f, p1.y());

  ASSERT_FALSE(res2);
}
