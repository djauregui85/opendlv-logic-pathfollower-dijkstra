// Dijkstra
// Still not functional

#include "dijkstra.hpp"
#include <algorithm>
#include <vector>
#include <math.h>
#include "line.hpp"


double calc_position(double grid_size, int index, double minp) {
    return (double) index*grid_size + minp;
}

auto generate_graph(std::vector<Line> walls)
{

  // Generate a vector with all the walls/obstacules chopped at the size of the grid
  const double tol{0.0001}; // Tolerance for comparing floats/doubles
  const double grid_size{0.25}; //asuming square
  int xwidth{0}; // change in every iteration but if the arena is square is ok
  int ywidth{0};
  double y{0.0};
  double x{0.0};
  // The arena must be square or rectangular, this should be in a separte file where it could be updated without interfierence
  std::vector< std::pair <double, double> > obmap;
  for(Line wall : walls) {
    if (fabs(wall.x2() - wall.x1()) <= tol) {
      ywidth = (int) round((wall.y2() - wall.y1()) / grid_size);
      for (int iy = 0; iy <= ywidth; iy++) {
        y = calc_position(grid_size, iy, wall.y1());
        obmap.push_back(std::make_pair(wall.x2(), y));
      }
    } else if (fabs(wall.y2() - wall.y1()) <= tol) {
      xwidth = (int) round((wall.x2() - wall.x1()) / grid_size);
      for (int ix = 0; ix <= xwidth; ix++) {
        x = calc_position(grid_size, ix, wall.x1());
        obmap.push_back(std::make_pair(x, wall.y2()));
      }
    }    
  }
  // Check this with software guys: code use c++17 instead of 14, requires change makefile and docker alpine to 3.10 instead 3.7
  const auto [min, max] = std::minmax_element(obmap.begin(), obmap.end());
  const int grid_w = abs(xwidth);
  const int grid_h = abs(ywidth);

  if (VERBOSE) {
    std::cout << "(X,Y)" << std::endl;
    for(std::pair n : obmap) {
      std::cout << "(" << n.first << "," << n.second << ")" << std::endl;
    }
    std::cout << "xwidth = " << xwidth << ", ywidth = " << ywidth << '\n' << std::endl;
    std::cout << "grid_w = " << grid_w << ", grid_h = " << grid_h << '\n' << std::endl;  
    std::cout << "xmin = " << (*min).first << ", xmax = " << (*max).first << '\n' << std::endl;
    std::cout << "ymin = " << (*min).second << ", ymax = " << (*max).second << '\n' << std::endl;      
  }
  // Obstacle map generation *************************
  // Create and initializate the grid matrix
  double **grid;
  grid = new double*[grid_w];
  for (int i = 0; i < grid_w; ++i) {
    grid[i] = new double[grid_h];
  }
  // Initializate the grid matrix with False
  for (int i = 0; i < grid_w; ++i) {
    for (int j = 0; j < grid_h; ++j) {
      grid[i][j] = false;
    } 
  }
  // Assign true to the cells that are not obstacules
  double robot_radius{0.2};
  double d{0.0};    
  for (int ix = 0; ix < grid_w; ix++) {
    x = calc_position(grid_size, ix, (*min).first);
    for (int iy = 0; iy < grid_h; iy++) {
      y = calc_position(grid_size, iy, (*min).second);
      for(std::pair n : obmap) {
        d = std::sqrt(std::pow(n.first - x , 2.0) + std::pow(n.second - y , 2.0));
        if (d <= robot_radius) {
          grid[ix][iy] = true;
          break;
        }
      }
    }
  }





}