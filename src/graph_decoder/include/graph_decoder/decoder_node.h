#ifndef GRAPH_DECODER_H
#define GRAPH_DECODER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <nav_graph_msg/Graph.h>
#include <nav_graph_msg/Node.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

enum VizColor {
    RED = 0,
    ORANGE = 1,
    BLACK = 2,
    YELLOW = 3,
    BLUE = 4,
    GREEN = 5,
    EMERALD = 6,
    WHITE = 7,
    MAGNA = 8,
    PURPLE = 9
};

struct NavNode {
    NavNode() = default;
    std::size_t id;
    geometry_msgs::Point position;
    std::vector<std::size_t> connect_idxs;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::vector<NavNodePtr> NodePtrStack;

struct GraphDecoderParams {
    GraphDecoderParams() = default;
    std::string frame_id;
    std::string file_name;
    float viz_scale_ratio;
};

class GraphDecoder {
public:
    GraphDecoder() = default;
    ~GraphDecoder() = default;

    void Init();
    void Loop();

private:
    ros::NodeHandle nh;
    ros::Subscriber graph_sub_;
    ros::Publisher graph_viz_pub_;
    ros::ServiceServer save_graph_service_, read_graph_service_;
    GraphDecoderParams gd_params_;
    NodePtrStack graph_nodes_;
    MarkerArray graph_marker_array_;
    Marker node_marker_, edge_marker_;

    nav_graph_msg::Graph shared_graph_;

    void LoadParmas();


    void SetMarker(const VizColor& color, 
                const std::string& ns,
                const float scale,
                const float alpha, 
                Marker& scan_marker);
    
    void SetColor(const VizColor& color, const float alpha, Marker& scan_marker);

    void GraphCallBack(const nav_graph_msg::GraphConstPtr& msg);

    bool SaveGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool ReadGraphFromFile(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void VisualizeGraph();

    inline void ResetGraph() {
        graph_nodes_.clear();
    }

    inline void ResetMarkers() {
        this->SetMarker(VizColor::GREEN, "graph_node", 0.5f, 0.5f, node_marker_);
        this->SetMarker(VizColor::YELLOW, "graph_edge", 0.1f,  0.2f, edge_marker_);
        node_marker_.points.clear();
        edge_marker_.points.clear();
        graph_marker_array_.markers.clear();
    }

    inline void CreateNavNode(const geometry_msgs::Point& point, 
                              const std::size_t& id,
                              NavNodePtr& node_ptr) 
    {
        node_ptr = std::make_shared<NavNode>();
        node_ptr->position = point;
        node_ptr->id = id;
        node_ptr->connect_idxs.clear();
        node_ptr->connect_nodes.clear();
    }

    inline bool AddNodePtrToGraph(const NavNodePtr& node_ptr) {
        if (node_ptr != NULL) {
            graph_nodes_.push_back(node_ptr);
            return true;
        }
        return false;
    }
    
};


#endif