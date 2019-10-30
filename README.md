# opendlv-logic-pathfollower-kiwi microservice

Dependencies:
Alpine 3.10 (lastest), https://hub.docker.com/_/alpine
C++ 17 or higher


Input: simulation-map.txt

The map must be in a directory /opt in the root of the microservice folder.

**Build command**

docker build -f Dockerfile.amd64 -t opendlv-logic-pathfollower-kiwi .


**Run command for individual testing of the microservice**

docker run -ti --rm -v "$PWD/opt":/opt --net=host opendlv-logic-pathfollower-kiwi:lastest opendlv-logic-pathfollower-kiwi --verbose --cid=111 --freq=10 --frame-id=0 --map-file=/opt/simulation-map.txt --start-x=0.0 --start-y=0.0 --end-x=1.0 --end-y=1.0
