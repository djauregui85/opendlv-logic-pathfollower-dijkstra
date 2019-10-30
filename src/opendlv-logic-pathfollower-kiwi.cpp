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

#include "stringtoolbox.hpp"
#include "line.hpp"

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
    double const DT = 1.0 / FREQ;

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

    std::cout << "ID: " << ID << ", CID: " << CID << ", FREQ: " << FREQ << ", DT: " << DT << std::endl;
    std::cout << "aqui" << std::endl;


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

    // Check this with software guys: code use c++17 instead of 14, requires change makefile and docker alpine to 3.10 instead 3.7
    const auto [xmin, xmax] = std::minmax_element(ox.begin(), ox.end());
    std::cout << "xmin = " << *xmin << ", xmax = " << *xmax << '\n';
    const auto [ymin, ymax] = std::minmax_element(oy.begin(), oy.end());
    std::cout << "xmin = " << *ymin << ", xmax = " << *ymax << '\n';


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
  }
  return retCode;
}
