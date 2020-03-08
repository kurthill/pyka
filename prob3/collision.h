// This code determines if a certain trajectory intersects with an obstacle. 
// Trajectories are always helical arcs, and obstacles are vertical polygon 
// prisms from z=0 to some height. 
//
// To avoid the large algebraic expressions of checking if the arc intersects 
// the plane of any face, I'll try a strategy that is hopfully less error
// prone and easier to read. Simple is good.
//
// The method steps through a parameterized trajectory and checks if the 
// position is within the object at any point.
//
// Kurt Hill March 7 2020
// kurtkeyshill@gmail.com

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

struct Obstacle {
  const std::vector<Eigen::Vector2d> vertices;
  const float height;

  Obstacle(std::vector<Eigen::Vector2d> vertices, float height)
    : vertices(vertices), height(height){};
};

class Path {
  const Eigen::Vector2d _center;
  const float _radius;
  const float _start_angle;
  const float _end_angle;
  const float _start_height;
  const float _end_height;

  public:
  Path(Eigen::Vector2d center, float radius, float start_angle, 
       float end_angle, float start_height, float end_height)
    : _center(center),
      _radius(radius),
      _start_angle(start_angle),
      _end_angle(end_angle),
      _start_height(start_height),
      _end_height(end_height){};

  void Print();
  bool CheckPath(double d, Obstacle o);

  private:
  // helper functions
  bool CheckPoint(Eigen::Vector3d p, Obstacle o);
  Eigen::Vector3d GetPoint(double angle);
  
};

void Path::Print() {
  std::cout << "Path dump:" << std::endl;
  std::cout << "  Center = (" << _center[0] << ", " << _center[1] << ")" <<  std::endl;
  std::cout << "  Radius = " << _radius << std::endl;
  std::cout << "  Angles = " << _start_angle << " -> " << _end_angle << std::endl;
  std::cout << "  Height = " << _start_height << " -> " << _end_height << std::endl;
}


////////////////////////////////////////////////////////////////////////////////
// Simulate the curve as discrete points and check every point if it is inside
// the obstacle volume
// Distance scale d controls the resolution
bool Path::CheckPath(double d, Obstacle o) {

  // calculate the number of steps from the distance scale and pathlength
  const double a = (_end_angle-_start_angle) * _radius * 3.14159265359 / 180;
  const double b = _end_height - _start_height;
  const double length = sqrt(a*a + b*b);
  const unsigned int nSteps = length/d;
  for(unsigned int i = 0; i < nSteps; i++) {
    // Get the cartesian point from the angle parameter, then check if it is 
    // inside  the volume
    double angle = _start_angle + i*(_end_angle-_start_angle)/(nSteps-1);
    Eigen::Vector3d point = GetPoint(angle);
    //std::cout << " angle = " << angle << std::endl;
    if(CheckPoint(point, o))
      return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Checks to see if a given 3d point, v, is inside the obstacle volume, o.
bool Path::CheckPoint(Eigen::Vector3d v, Obstacle o) {

  // initial height check
  if( v[2] > o.height )
    return false;

  // Loop over the obstacle vertices
  // - generate inequalities and check the point against them
  // - need to loop back to the first point at the end for the final surface
  //
  // Previous point and sum of inequalities
  int sum = 0;
  Eigen::Vector2d lastp;
  for(unsigned int i = 0; i <= o.vertices.size(); i++) {

    // Get the vertex (with the wrap around to the first element)
    Eigen::Vector2d p = o.vertices[i % o.vertices.size()];

    // check inequalities
    const double x1 = lastp[0];
    const double y1 = lastp[1];
    const double x2 = p[0];
    const double y2 = p[1];
    const double m = (y2-y1)/(x2-x1);
    const double b = y1 - m*x1;
    if(i > 0) {
      if(x2 > x1)
        sum += v[1] < (v[0] * m + b);
      else if(x2 < x1)
        sum += v[1] > (v[0] * m + b);
      else {
        if(y2 > y1)
          sum += v[0] > x1;
        else
          sum += v[0] < x1;
      }
    }
    // Last step is to store the previous vertex
    lastp = p;
  }
  // If all the inequalities are satisfied...
  return sum == o.vertices.size();
}

////////////////////////////////////////////////////////////////////////////////
// Finds the cartesian point on the path given the angle parameter
Eigen::Vector3d Path::GetPoint(double angle) {
  // shift angle into standard coodinates
  double a =  90 - angle;

  // calculate cartesian coordinates
  double x = _radius * cos(a*3.14159265359/180) + _center[0];
  double y = _radius * sin(a*3.14159265359/180) + _center[1];
  // Interpolate the z coordinate
  double m = (_end_height - _start_height)/(_end_angle - _start_angle);
  double b = _start_height - m * _start_angle;
  double z = angle * m + b;

  return Eigen::Vector3d(x, y, z);
}
