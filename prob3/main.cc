#include "collision.h"
#include <iostream>

int main() {

  // Resolution parameter is the step distance in the units of the test objects
  double resolution = 0.01;

  // test obstacles
  Obstacle obstacle_1(
  std::vector<Eigen::Vector2d>({{10, 10}, {10, 0}, {0, 10}}), 30);
  Obstacle obstacle_2(
  std::vector<Eigen::Vector2d>({{10, 0}, {10, 10}, {20, 10}, {20, 0}}), 20);
  Obstacle obstacle_3(
  std::vector<Eigen::Vector2d>({{0, 10}, {0, 20}, {10, 20}, {10, 10}}), 20);

  std::vector<Obstacle> obstacles = {obstacle_1, obstacle_2, obstacle_3};

  // test paths
  Path path_1({0, 0}, 15, 0, 90, 25, 15);
  Path path_2({10, 10}, 5, -45, 225, 18, 30);
  Path path_3({10, -10}, 15, -15, 15, 60, 10);

  std::vector<Path> paths = {path_1, path_2, path_3};

  // Loop over the test paths and obstacles to check for collisions
  for(int i = 0; i < paths.size(); i++) { 
    std::cout << "Path " << i+1 << std::endl;
    for(int j = 0; j < obstacles.size(); j++) { 
      std::cout << "  Object " << j+1 << std::endl;

      if(paths[i].CheckPath(resolution, obstacles[j]))
        std::cout << "    Collision!" << std::endl;
      else
        std::cout << "    No Collision" << std::endl;
    }
  }
}
