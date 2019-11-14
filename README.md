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

**Solution model**

1. Read the map-file and extract the walls into a vector of lines.
2. Chop the map walls/obstacles within the internally defined grid_size.
3. Find the max/min in the coordinates X,Y of the map. Get the map width and height w.r.t. grid size.
4. Create a bool indexed grid initialized with false in all the cells (width x height).
5. Assign True to all cells that are NOT obstacles w.r.t the robot radius. It will be used later.
6. Create two key-indexed list for unvisited and visited nodes. The <map> data type is used [Step 1 algorithm].
7. Two nodes are created, nstart with the start point A and ngoal with the end point B. Both with index -1. The node structure comprise {x,y,cost,pind), which are the x,y coordinates on the grid, the cost of the node and the index of the node. The x,y coordinates are calculated with the function calcxyindex.
8. The nstart node is inserted in the unvisited list with a key-index calculated using the function calc_index. This function convert a 2D index into a 1D index. [Step 1].
9. Inside infinite loop:
    9.1 Search for the key-index of the node with the minimum cost inside the unvisited list and set it as current node.
    9.2 If the current node is equal to the goal node, break the loop
    9.3 Erase the current node from the unvisited list
    9.4 Insert the current node in the visited list
    9.5 The motion model on the grid is defined with a constant 2D array (x,y,cost). From the current node, iterate for each node in the neighborhood and check if it was already visited and verifiy if it is a obstacle, which make it invalid and must be skipped, if the node is valid, and if a new node, then it is inserted in the unvisited nodes list, otherwise if the node already was visited FALTA  




