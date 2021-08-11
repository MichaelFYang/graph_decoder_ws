/*
 * Navigation Graph Decoder
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "graph_decoder/decoder_node.h"

/***************************************************************************************/


void GraphDecoder::Init() {
    /* initialize subscriber and publisher */
    graph_sub_= nh.subscribe("/planner_nav_graph", 5, &GraphDecoder::GraphCallBack, this);
    graph_viz_pub_ = nh.advertise<MarkerArray>("/graph_decoder_viz",5);

    this->LoadParmas();
    save_graph_service_ = nh.advertiseService("/save_graph_service", &GraphDecoder::SaveGraphService, this);
    read_graph_service_ = nh.advertiseService("/read_graph_service", &GraphDecoder::ReadGraphFromFile, this);
    node_marker_.type = Marker::SPHERE_LIST;
    edge_marker_.type = Marker::LINE_LIST;
    this->ResetGraph();
}


void GraphDecoder::GraphCallBack(const nav_graph_msg::GraphConstPtr& msg) {
    shared_graph_ = *msg;
    this->ResetGraph();
    NavNodePtr temp_node_ptr = NULL;
    std::unordered_map<std::size_t, std::size_t> nodeIdx_idx_map;
    for (std::size_t i=0; i<shared_graph_.nodes.size(); i++) {
        const auto node = shared_graph_.nodes[i];
        CreateNavNode(node.position, node.id, temp_node_ptr);
        for (const auto& cid : node.connect_nodes) {
            temp_node_ptr->connect_idxs.push_back((std::size_t)cid);
        }
        if (AddNodePtrToGraph(temp_node_ptr)) {
            nodeIdx_idx_map.insert({node.id, i});
        }
    }
    for (const auto& node_ptr : graph_nodes_) { // add connections to nodes
        std::vector<std::size_t> clean_idx;
        clean_idx.clear();
        for (const auto& cid : node_ptr->connect_idxs) {
            const auto it = nodeIdx_idx_map.find(cid);
            if (it != nodeIdx_idx_map.end()) {
                const std::size_t idx = nodeIdx_idx_map.find(cid)->second;
                const NavNodePtr cnode_ptr = graph_nodes_[idx];
                node_ptr->connect_nodes.push_back(cnode_ptr);
                clean_idx.push_back(cnode_ptr->id);
            }
        }
        node_ptr->connect_idxs = clean_idx;
    }
    ROS_INFO("Graph extraction completed.");
    this->VisualizeGraph();
}

void GraphDecoder::LoadParmas() {
    const std::string prefix = "/graph_decoder/";
    nh.param<std::string>(prefix + "world_frame", gd_params_.frame_id, "/map");
    nh.param<std::string>(prefix + "file_name", gd_params_.file_name, "/home/fan-robot/graph_decoder_files/graph.txt");
    nh.param<float>(prefix + "visual_scale_ratio", gd_params_.viz_scale_ratio, 1.0);

}

void GraphDecoder::SetMarker(const VizColor& color, 
                             const std::string& ns,
                             const float scale,
                             const float alpha,  
                             Marker& scan_marker) 
{
    scan_marker.header.frame_id = gd_params_.frame_id;
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * gd_params_.viz_scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    this->SetColor(color, alpha, scan_marker);
}

void GraphDecoder::SetColor(const VizColor& color, 
                            const float alpha, 
                            Marker& scan_marker)
{
    std_msgs::ColorRGBA c;
    c.a = alpha;
    if (color == VizColor::RED) {
    c.r = 0.9f, c.g = c.b = 0.f;
    }
    else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
    }
    else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
    }
    else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
    }
    else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
    }
    else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
    }
    else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
    }
    else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
    }
    else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
    }
    else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
    }
    scan_marker.color = c;
}

void GraphDecoder::Loop() {
    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool GraphDecoder::ReadGraphFromFile(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    res.success = false;
    std::ifstream graph_file(gd_params_.file_name);
    this->ResetGraph();
    std::string str;
    std::string delimiter = " ";
    std::unordered_map<std::size_t, std::size_t> nodeIdx_idx_map;
    std::size_t ic = 0;
    while (std::getline(graph_file, str)) {
        std::vector<std::string> components;
        geometry_msgs::Point p;
        std::size_t id;
        std::size_t pos = 0;
        std::vector<std::size_t> connect_idxs;
        NavNodePtr temp_node_ptr = NULL;
        while ((pos = str.find(delimiter)) != std::string::npos) {
            std::string c = str.substr(0, pos);
            if (c.length() > 0) {
                components.push_back(str.substr(0, pos));
            }
            str.erase(0, pos + delimiter.length());
        }
        for (std::size_t i=0; i<components.size(); i++) {
            if (i == 0) id = (std::size_t)std::stoi(components[i]);
            if (i == 1) p.x = std::stof(components[i]);
            if (i == 2) p.y = std::stof(components[i]);
            if (i == 3) p.z = std::stof(components[i]);
            if (i > 3) {
                connect_idxs.push_back((std::size_t)std::stoi(components[i]));
            }
        }
        CreateNavNode(p, id, temp_node_ptr);
        temp_node_ptr->connect_idxs = connect_idxs;
        if (AddNodePtrToGraph(temp_node_ptr)) { 
            nodeIdx_idx_map.insert({id, ic});
            ic ++;
        }
    }
    for (const auto& node_ptr : graph_nodes_) { // add connections to nodes
        std::vector<std::size_t> clean_idx;
        clean_idx.clear();
        for (const auto& cid : node_ptr->connect_idxs) {
            const auto it = nodeIdx_idx_map.find(cid);
            if (it != nodeIdx_idx_map.end()) {
                const std::size_t idx = nodeIdx_idx_map.find(cid)->second;
                const NavNodePtr cnode_ptr = graph_nodes_[idx];
                node_ptr->connect_nodes.push_back(cnode_ptr);
                clean_idx.push_back(cnode_ptr->id);
            }
        }
        node_ptr->connect_idxs = clean_idx;
    }
    ROS_INFO("Graph extraction completed.");
    this->VisualizeGraph();
    res.message = "Read graph file success.";
    res.success = true;
    return true;
}

bool GraphDecoder::SaveGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    /* Output Format: "idx x y z [connect idxs]" */
    std::ofstream graph_file;
    res.success = false;
    graph_file.open(gd_params_.file_name);
    for (const auto& node_ptr : graph_nodes_) {
        graph_file << std::to_string(node_ptr->id) << " ";
        graph_file << std::to_string(node_ptr->position.x) << " ";
        graph_file << std::to_string(node_ptr->position.y) << " ";
        graph_file << std::to_string(node_ptr->position.z) << " ";
        for (const auto& cidx : node_ptr->connect_idxs) {
            graph_file << std::to_string(cidx) << " ";
        } 
        graph_file << "\n";
    }
    graph_file.close();
    ROS_INFO("Save graph file success.");
    res.success = true;
    res.message = "Save graph file success in file: " + gd_params_.file_name;
    return true;
}

void GraphDecoder::VisualizeGraph() {
    this->ResetMarkers();
    for (const auto& node_ptr : graph_nodes_) {
        node_marker_.points.push_back(node_ptr->position);
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            edge_marker_.points.push_back(node_ptr->position);
            edge_marker_.points.push_back(cnode_ptr->position);
        }
    }
    graph_marker_array_.markers.push_back(edge_marker_);
    graph_marker_array_.markers.push_back(node_marker_);
    graph_viz_pub_.publish(graph_marker_array_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "graph_decoder_node");
  GraphDecoder gd_node;
  gd_node.Init();
  gd_node.Loop();
}