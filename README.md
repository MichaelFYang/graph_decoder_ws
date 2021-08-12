# README #

This README would normally document whatever steps are necessary to get your application up and running.

## What is this repository for?

Navigation Graph Decoder

## How to get the graph decoeder running on your workspace 

#### Copy this planner workspace
```bash
git clone https://github.com/MichaelFYang/graph_decoder_ws.git
cd graph_decoder_ws/
catkin build
source devel/setup.bash
```

#### Before launch the decoder, make sure you have the file path correct
Open the config file as the way you like
``` bash
gedit <<YOUR WORKSPACE>>/src/graph_decoder/config/default.yaml 
```

#### Launch the graph decoder
```bash
roslaunch graph_decoder decoder.launch
```

#### Instruction of how this graph decoder works
##### There are two ways to input (update) graph into the decoder
1. Publish your graph into ROS topic "/planner_nav_graph"
2. Read graph file from path listed in config file in nav_graph_decoder package by calling ROS service

Note: You need to select the graph file and change file path in "graph_decoder_ws/src/nav_graph_decoder/config/default.yaml"
```bash
rosservice call /read_graph_service "{}"
```
##### How to save graph into a graph file
Call a ROS service to save your current graph into a file
```bash
rosservice call /save_graph_service "{}"
```

#### Graph File Format
idx position.x position.y position.z \[connection idxs\]

## Who could I talk to? 

Fan Yang
(fanyang2@alumni.cmu.edu)
