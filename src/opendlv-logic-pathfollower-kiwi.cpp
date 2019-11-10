/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "behavior.hpp"

#include <algorithm>
#include <fstream>
#include <vector>
#include <math.h>
#include <iterator> 
#include <map> 

#include "stringtoolbox.hpp"
#include "line.hpp"
#include "dijkstra.hpp"



// Create structure for node 
struct node_t {
  int x;
  int y;
  double cost;
  int pind;
};

double calc_position(double, int, double);
double calc_position(double, int, double);
int calc_xyindex(double, double, double);
int calc_index(node_t, std::pair<double, double>, int);
bool verify_node(node_t, double, std::pair<double, double>, std::pair<double, double>, bool);
void displayNode(node_t);
bool comp_node_cost(std::pair<int, node_t>, std::pair<int, node_t>); 
double get_motion_model(void);
std::vector<std::pair<double,double>> calc_final_path(node_t, std::map<int, node_t>, std::pair<double,double>, double);

// dx, dy, cost
double motion[8][3] = { 
                        { 1,  0,            1},
                        { 0,  1,            1},
                        {-1,  0,            1},
                        { 0, -1,            1},
                        {-1, -1, std::sqrt(2)},
                        {-1,  1, std::sqrt(2)},
                        { 1, -1, std::sqrt(2)},
                        { 1,  1, std::sqrt(2)}
                      };

std::vector<std::pair<double,double>> calc_final_path(node_t ngoal, std::map<int, node_t> visited, std::pair<double,double> min, double grid_size) {
  // generate final course
  std::vector< std::pair <double, double> > route;
  route.push_back(std::make_pair(calc_position(grid_size, ngoal.x, min.first),calc_position(grid_size, ngoal.y, min.second)));
  int pind = ngoal.pind;
  while (pind != -1) {
      node_t n = visited[pind];
      route.push_back(std::make_pair(calc_position(grid_size, n.x, min.first),calc_position(grid_size, n.y, min.second)));
      pind = n.pind;
  }
  return route;
} 

bool comp_node_cost(std::pair<int, node_t> a, std::pair<int, node_t> b) { 
  return (a.second.cost < b.second.cost); 
} 

// Function definition
double calc_position(double grid_size, int index, double minp) {
    return (double) index*grid_size + minp;
}

int calc_xyindex(double position, double minp, double grid_size) {
    return (int) round((position - minp)/grid_size);
}

// Index in map
int calc_index(node_t node, std::pair<double, double> min, int width) {
    return (int) round((node.y - min.second) * width + (node.x - min.first));
}

bool verify_node(node_t n, double grid_size, std::pair<double, double> min, std::pair<double, double> max, bool **grid) {
  const double px = calc_position(grid_size, n.x, min.first);
  const double py = calc_position(grid_size, n.y, min.second);

  if (px < min.first) { 
    return false;
  } else if (py < min.second) {
      return false;
  } else if (px >= max.first) {
      return false;
  } else if (py >= max.second) {
      return false;
  }    

  if (grid[n.x][n.y])
      return false;

  return true;
}

void displayNode(node_t n) {
    std::cout << "Node(" << n.x << "," << n.y << "," << n.cost << "," << n.pind << ")" << std::endl;
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("frame-id") 
      || 0 == commandlineArguments.count("freq") 
      || 0 == commandlineArguments.count("map-file") 
      || 0 == commandlineArguments.count("start-x") 
      || 0 == commandlineArguments.count("start-y")
      || 0 == commandlineArguments.count("end-x") 
      || 0 == commandlineArguments.count("end-y")
      || 0 == commandlineArguments.count("verbose")) {
    std::cerr << argv[0] << " Microservice for path planning and path following for Kiwi." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --frame-id=<The frame to use for position> --freq=<Simulation frequency> --map-file=<File where walls of the squard grid arena (without any diagonal walls as stated below) are defined as rows according to x1,y1,x2,y2;> --start-x=<X coordinate of the start position> --start-y=<Ycoordinate of the start position> --end-x=<X coordinate of the end (goal) position> --end-y=<Y coordinate of the end (goal) position> --cid=<OpenDaVINCI session> [--id=<ID if more than one sensor>] [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --verbose --cid=111 --freq=10 --frame-id=0 --map-file=/opt/simulation-map.txt --start-x=0.0 --start-y=0.0 --end-x=1.0 --end-y=1.0" << std::endl;
    retCode = 1;
  } else {
    uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    float const DT = 1/FREQ;
    double const sx = std::stod(commandlineArguments["start-x"]);
    double const sy = std::stod(commandlineArguments["start-y"]);
    double const gx = std::stod(commandlineArguments["end-x"]);
    double const gy = std::stod(commandlineArguments["end-y"]);        

    if (VERBOSE) {
      std::cout << "ID: " << ID << ", CID: " << CID << ", FREQ: " << FREQ << ", DT: " << DT << std::endl;
      std::cout << "start-x: " << sx << ", start-y: " << sy << ", end-x: " << gx << ", end-y: " << gy << std::endl;
    }
    //Behavior behavior;

    // Create walls, depends of line library, vector, stringtoolbox
    // Reads map-file
    std::vector<Line> walls;
    std::ifstream input(commandlineArguments["map-file"]);
    for (std::string str; getline(input, str);) {
      std::vector<std::string> coordinates = stringtoolbox::split(
          stringtoolbox::split(stringtoolbox::trim(str), ';')[0], ',');
      if (coordinates.size() == 4) {
        double x1{std::stof(coordinates[0])};
        double y1{std::stof(coordinates[1])};
        double x2{std::stof(coordinates[2])};
        double y2{std::stof(coordinates[3])};
        Line line{x1, y1, x2, y2};
        walls.push_back(line);
        if (VERBOSE) {
          std::cout << "Added wall from [" << x1 << "," << y1 << "] to [" << x2 << "," << y2 << "]" << std::endl;
        }
      }
    }

    // Generate a vector with all the walls/obstacules chopped at the size of the grid
    const double tol{0.0001}; // Tolerance for comparing floats/doubles
    const double grid_size{0.25}; //asuming square
    // int xwidth{0}; // change in every iteration but if the arena is square is ok
    // int ywidth{0};
    double y{0.0};
    double x{0.0};
    // The arena must be square or rectangular, this should be in a separte file where it could be updated without interfierence
    std::vector< std::pair <double, double> > obmap;
    for(Line wall : walls) {
      int xwidth{0}; // change in every iteration but if the arena is square is ok
      int ywidth{0};
      std::cout << "Current wall [" << wall.x1() << "," << wall.y1() << "] to [" << wall.x2() << "," << wall.y2() << "]" << fabs(wall.x2() - wall.x1()) << fabs(wall.y2() - wall.y1()) << std::endl;
      if (fabs(wall.x2() - wall.x1()) <= tol) {
        std::cout << "here1" << std::endl;
        ywidth = (int) round((wall.y2() - wall.y1()) / grid_size);
        std::cout << ywidth << std::endl;
        if (ywidth >= 0) {
          for (int iy = 0; iy <= ywidth; iy++) {
            y = calc_position(grid_size, iy, wall.y1());
            obmap.push_back(std::make_pair(wall.x2(), y));
            std::cout << "Current wall (" << wall.x2() << "," << y << ")" << std::endl;
          }
        } else {
          for (int iy = ywidth; iy <= 0; iy++) {
            y = calc_position(grid_size, iy, wall.y1());
            obmap.push_back(std::make_pair(wall.x2(), y));
            std::cout << "Current wall (" << wall.x2() << "," << y << ")" << std::endl;
          }
        }
      } else if (fabs(wall.y2() - wall.y1()) <= tol) {
        std::cout << "here2" << std::endl;
        xwidth = (int) round((wall.x2() - wall.x1()) / grid_size);
        std::cout << xwidth << std::endl;
        if (xwidth >= 0) {        
          for (int ix = 0; ix <= xwidth; ix++) {
            x = calc_position(grid_size, ix, wall.x1());
            obmap.push_back(std::make_pair(x, wall.y2()));
            std::cout << "Current wall (" << x << "," << wall.y2() << ")" << std::endl;
          }
        } else {
            for (int ix = xwidth; ix <= 0; ix++) {
              x = calc_position(grid_size, ix, wall.x1());
              obmap.push_back(std::make_pair(x, wall.y2()));
              std::cout << "Current wall (" << x << "," << wall.y2() << ")" << std::endl;          
            }
        }
      } else {
          std::cout << "here3" << std::endl;
      } 
    }
    // Check this with software guys: code use c++17 instead of 14, requires change makefile and docker alpine to 3.10 instead 3.7
    const auto [min, max] = std::minmax_element(obmap.begin(), obmap.end());
    const int grid_w = (int) round((max->first - min->first) / grid_size);
    const int grid_h = (int) round((max->second - min->second) / grid_size);
    // const int grid_w = abs(xwidth);
    // const int grid_h = abs(ywidth);

    if (VERBOSE) {
      std::cout << "(X,Y)" << std::endl;
      for(std::pair n : obmap) {
        std::cout << "(" << n.first << "," << n.second << ")" << std::endl;
      }
      std::cout << "grid_w = " << grid_w << ", grid_h = " << grid_h << '\n' << std::endl;  
      std::cout << "xmin = " << (*min).first << ", xmax = " << (*max).first << '\n' << std::endl;
      std::cout << "ymin = " << (*min).second << ", ymax = " << (*max).second << '\n' << std::endl;      
    }
    // Obstacle map generation *************************
    // Create and initializate the grid matrix
    bool **grid;
    grid = new bool*[grid_w];
    for (int i = 0; i < grid_w; ++i) {
      grid[i] = new bool[grid_h];
    }
    // Initializate the grid matrix with False
    for (int i = 0; i < grid_w; ++i) {
      for (int j = 0; j < grid_h; ++j) {
        grid[i][j] = false;
      } 
    }
    // Assign true to the cells that are not obstacules
    double robot_radius{0.05};
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



    // Create maps for openset and closeset
    std::map<int, node_t> unvisited, visited;

    node_t nstart, ngoal;
    nstart = {calc_xyindex(sx, min->first, grid_size), calc_xyindex(sy, min->second, grid_size), 0.0, -1};
    ngoal = {calc_xyindex(gx, min->first, grid_size), calc_xyindex(gy, min->second, grid_size), 0.0, -1};
    displayNode(nstart);
    displayNode(ngoal);

    unvisited.insert(std::pair<int, node_t> (calc_index(nstart, *min, grid_w), nstart)); // first element of the unvisited is the start node
    // Function to calculate positon in the grid,
  
    // printing map
    std::map<int, node_t>::iterator itr; 
    std::cout << "\nThe map is: \n" << std::endl; 
    std::cout << "\tKEY\tELEMENT\n" << std::endl;
    for (itr = unvisited.begin(); itr != unvisited.end(); ++itr) { 
      std::cout << "\t" << itr->first << "\t"; 
      displayNode(itr->second);  
    }

    

    while(true) {

      std::pair<int, node_t> min_cost_node = *std::min_element(unvisited.begin(), unvisited.end(), &comp_node_cost);
      int c_id = min_cost_node.first;
      std::cout << c_id << std::endl;
      node_t current = unvisited[c_id];
      displayNode(current); 

      // Exit the loop if found the goal
      if ((current.x == ngoal.x) && (current.y == ngoal.y)) {
          std::cout << "Find goal" << std::endl;
          ngoal.pind = current.pind;
          ngoal.cost = current.cost;
          break;
      }

      // Remove the item from the unvisited map
      unvisited.erase(c_id);

      // Add it to the visited map
      visited.insert(std::pair<int, node_t> (c_id, current));

      int num_mov =  sizeof(motion) / sizeof(motion[0]); 
      if (VERBOSE)
        std::cout << "Number of movements of motion model: " << num_mov << std::endl;

      // expand search grid based on motion model
      for(int i = 0; i < num_mov; ++i) {
        node_t node = { current.x + (int) motion[i][0],
                          current.y + (int) motion[i][1],
                          current.cost + motion[i][2], c_id };
        int n_id = calc_index(node, *min, grid_w);

        if (visited.count(n_id))  // use binary search node.
          continue;

        if (!verify_node(node, grid_size, *min, *max, grid))
          continue;

        if (!unvisited.count(n_id)) {
          unvisited.insert(std::pair<int, node_t> (n_id, node)); // Discover a new node in the next iteration
        } else if (unvisited[n_id].cost >= node.cost) {
            //This path is the best until now. record it!
            unvisited.erase(n_id);
            unvisited.insert(std::pair<int, node_t> (n_id, node)); 
        }
      }
    }
    std::vector<std::pair<double,double>> path = calc_final_path(ngoal, visited, *min, grid_size);
    std::cout << "(X,Y)" << std::endl;
    for(std::pair n : path) {
      std::cout << "(" << n.first << "," << n.second << ")" << std::endl;
    }    
    // }


    // // Maybe set to zero initial values because it is not get from the line
    // uint32_t const FRAME_ID{static_cast<uint32_t>(std::stoi(commandlineArguments["frame-id"]))};
    // double const X{static_cast<double>(std::stof(commandlineArguments["x"]))};
    // double const Y{static_cast<double>(std::stof(commandlineArguments["y"]))};
    // double const YAW{static_cast<double>(std::stof(commandlineArguments["yaw"]))};

    // Sensor sensor{walls, X, Y, YAW};
    // auto onFrame{[&FRAME_ID, &sensor](cluon::data::Envelope &&envelope)
    // auto onFrame{[&FRAME_ID](cluon::data::Envelope &&envelope) 
    //   {
    //     uint32_t const senderStamp = envelope.senderStamp();
    //     if (FRAME_ID == senderStamp) {
    //       auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
    //       // sensor.setFrame(frame);
    //     }
    //   }};

    // cluon::OD4Session od4{CID};
    // od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);
    // od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    // od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);

    // USE FOR THE OUTPUT TO SEND THE BEHAIVOR, OUTPUT OF "CONTROLLER PID" IN MY BLOCK DIAGRAM
    // auto atFrequency{[&VERBOSE, &behavior, &od4]() -> bool
    //   {
    //     behavior.step();
    //     auto groundSteeringAngleRequest = behavior.getGroundSteeringAngle();
    //     auto pedalPositionRequest = behavior.getPedalPositionRequest();

    //     cluon::data::TimeStamp sampleTime;
    //     od4.send(groundSteeringAngleRequest, sampleTime, 0);
    //     od4.send(pedalPositionRequest, sampleTime, 0);
    //     if (VERBOSE) {
    //       std::cout << "Ground steering angle is " << groundSteeringAngleRequest.groundSteering()
    //         << " and pedal position is " << pedalPositionRequest.position() << std::endl;
    //     }

    //     return true;
    //   }};

    // od4.timeTrigger(FREQ, atFrequency);

    // Eliminate the array to avoid memory leak
    for (int ix = 0; ix < grid_w; ++ix) {
      delete [] grid[ix];
    }
    delete [] grid;
  }
  return retCode;
}
