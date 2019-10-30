// Dijkstra

#include "dijkstra.hpp"
#include <vector>
#include "line.hpp"

auto generate_graph(std::vector<Line> walls)
{

    std::vector<double> ox;
    std::vector<double> oy;
    for(Line wall : walls) {
      ox.push_back(wall.x1());
      ox.push_back(wall.x2());
      oy.push_back(wall.y1());
      oy.push_back(wall.y2());     
    }
    std::cout << "\nX values\n" << std::endl;
    for(double n : ox) {
      std::cout << n << "," << std::endl;
    }
    std::cout << "\nY values\n" << std::endl;
    for(double m : oy) {
      std::cout << m << "," << std::endl;
    }
    
        
    self.minx = round(min(ox))
    self.miny = round(min(oy))
    self.maxx = round(max(ox))
    self.maxy = round(max(oy))
    print("minx:", self.minx)
    print("miny:", self.miny)
    print("maxx:", self.maxx)
    print("maxy:", self.maxy)

    self.xwidth = round((self.maxx - self.minx)/self.reso)
    self.ywidth = round((self.maxy - self.miny)/self.reso)
    print("xwidth:", self.xwidth)
    print("ywidth:", self.ywidth)


int a[map_width][map_height];

}