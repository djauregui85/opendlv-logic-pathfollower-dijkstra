# opendlv-logic-pathfollower-kiwi microservice

**Current version**

* Read a map-file with a rectangular shape and vertical/horizontal obstacle walls and ouput a vector with the shorest path from point A to point B using the Dijkstra algorithm 
* Variables define internally: grid size, robot radius

**Further development**

* Create a class Dijkstra and a test file
* Show a graphic using ncurses library
* Send a OD4 message with the path to a pathfollowing block of the kiwi car



**Dependencies:**

* Alpine 3.10 (lastest), `https://hub.docker.com/_/alpine`
* C++ 17 or higher



**Build command**

docker build -f Dockerfile.amd64 -t opendlv-logic-pathfollower-kiwi .


**Run command for individual testing of the microservice**

docker run -ti --rm -v "$PWD/opt":/opt --net=host opendlv-logic-pathfollower-kiwi:latest opendlv-logic-pathfollower-kiwi --verbose --cid=111 --freq=10 --frame-id=0 --map-file=/opt/simulation-map.txt --start-x=-1.25 --start-y=-1.25 --end-x=1.25 --end-y=1.25


**Microservice**

Dijkstra algorithm to find the shortest path between A and B, driver model behaivor to follow the path

**Input:**


* Map file in text format with the coordinates of the walls 
* A: start-x, start-y
* B: end-x, end-y


**Output:**

* Ground Steering signal
* Pedal signal


**Background**

*Dijkstra algorithm (Source [Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm))*


Let the node at which we are starting be called the initial node. Let the distance of node Y be the distance from the initial node to Y. Dijkstra's algorithm will assign some initial distance values and will try to improve them step by step.

1. Mark all nodes unvisited. Create a set of all the unvisited nodes called the unvisited set.

2. Assign to every node a tentative distance value: set it to zero for our initial node and to infinity for all other nodes. Set the initial node as current.

3. For the current node, consider all of its unvisited neighbours and calculate their tentative distances through the current node. Compare the newly calculated tentative distance to the current assigned value and assign the smaller one. For example, if the current node A is marked with a distance of 6, and the edge connecting it with a neighbour B has length 2, then the distance to B through A will be 6 + 2 = 8. If B was previously marked with a distance greater than 8 then change it to 8. Otherwise, the current value will be kept.

4. When we are done considering all of the unvisited neighbours of the current node, mark the current node as visited and remove it from the unvisited set. A visited node will never be checked again.

5. If the destination node has been marked visited (when planning a route between two specific nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity (when planning a complete traversal; occurs when there is no connection between the initial node and remaining unvisited nodes), then stop. The algorithm has finished.

6. Otherwise, select the unvisited node that is marked with the smallest tentative distance, set it as the new "current node", and go back to step 3.


When planning a route, it is actually not necessary to wait until the destination node is "visited" as above: the algorithm can stop once the destination node has the smallest tentative distance among all "unvisited" nodes (and thus could be selected as the next "current"). 

**Solution model** **(code explanation)**

1. Read the map and create the grid. Mark walls, obstacles and posibles colition within the radius of the robot.
2. Create the unvisited and visted index list. Use <map> data type with integer indexes.
3. The start and end node are created. The nodes are structures with postion, cost and pind (previous index). This two nodes are created with index -1 as mark. The position of the nodes is converter to grid coordinates using the function **calcxyindex**.
4. Add the start node in the the unvisited list. The key-index of this node in the list is calcualted by the function **calc_index**.
5. Start loop.
6. Search in the list for the key-index of the node with the minimum cost inside the unvisited list and set it as current node.
7. Verify if the current node is the end (goal). If it is, break the loop. If it is not, continue.
    Remove the current node from unvisited list and added to the visited list.
8. Start exploration of the neighborhood of the current node acording the motion model. 
9. For each neighbor is created as node has as pind the key-index of the current node.
10. A new key-index is created for each neighbor.
11. Verify if exist a node with that new key-index in the visted list. If it is, skip to the next neighbor.
12. Verify if the node is marked as a wall, obstacle or collition within the radius of the robot. If it is, skip to the next neighbor.
13. Check if doesn't exist a node with the new key-index in the unvisted list, then add it,
14. If it exists, then, check if the its cost is greater than equal to the cost of neighbor that is currently been explored, if it is, replace the node in this key-index with the current neighbor. This is the best path until now.
15. Repeat unitl the end node is found.
16. Etract the shortest route from the visited list tracking down all the nodes from the end node, searching for its predecesor in the list using its previous index (pind) value. The search ends when the node with pind equals -1 is found. **The algorithm have found the shortest path and it is saved a list call route.**




