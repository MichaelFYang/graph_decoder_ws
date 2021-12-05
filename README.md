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

#### Before launch the decoder, make sure you check the the configuration file
Open the config file as the way you like
``` bash
gedit <<YOUR WORKSPACE>>/src/graph_decoder/config/default.yaml 
```

#### Launch the graph decoder
```bash
roslaunch graph_decoder decoder.launch
```

## Instruction of how this graph decoder works
#### There are two ways to input (update) graph into the decoder
1. Publish your graph into ROS topic "/robot_vgraph"
2. Read graph file from path published into ROS topic "/read_file_dir"
3. Save graph file from path published into ROS topic "/save_file_dir"

#### General Graph File Format
idx position.x position.y position.z \[connection idxs\]

## Integration with FAR Planner (Multi-robots Graph Sharing and Merging)
This package could read graphs from either files or published by other robots. It decodes the graphs and re-publishes to FAR Planner. FAR Planner has its own decoder that subscribes to the graph message and merges the graph from other sources with its current graph automatically.  

## Who could I talk to? 

Fan Yang
(fanyang2@alumni.cmu.edu)
