# opendlv-logic-pathfollower-kiwi



**Build command**

docker build -f Dockerfile.amd64 -t opendlv-logic-pathfollower-kiwi .


**Run command**

docker run -ti --rm --net=host opendlv-logic-pathfollower-kiwi:latest opendlv-logic-pathfollower-kiwi --verbose --cid=111 --freq=10 --frame-id=0 --map-file=/opt/simulation-map.txt --start-x=0.0 --start-y=0.0 --end-x=1.0 --end-y=1.0" 