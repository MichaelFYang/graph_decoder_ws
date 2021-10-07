#ifndef GRAPH_DECODER_H
#define GRAPH_DECODER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <visibility_graph_msg/Graph.h>
#include <visibility_graph_msg/Node.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "graph_decoder/point_struct.h"

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

typedef std::pair<Point3D, Point3D> PointPair;

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

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3
};

struct NavNode {
    NavNode() = default;
    std::size_t id;
    Point3D position;
    NodeFreeDirect free_direct;
    PointPair surf_dirs;
    bool is_frontier;
    bool is_navpoint;
    std::vector<std::size_t> connect_idxs;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;

    std::vector<std::size_t> contour_idxs;
    std::vector<std::shared_ptr<NavNode>> contour_connects;

    std::vector<std::size_t> traj_idxs;
    std::vector<std::shared_ptr<NavNode>> traj_connects;
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

    visibility_graph_msg::Graph shared_graph_;

    void LoadParmas();


    void SetMarker(const VizColor& color, 
                const std::string& ns,
                const float scale,
                const float alpha, 
                Marker& scan_marker);
    
    void SetColor(const VizColor& color, const float alpha, Marker& scan_marker);

    void GraphCallBack(const visibility_graph_msg::GraphConstPtr& msg);

    bool SaveGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool ReadGraphFromFile(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void VisualizeGraph();

    void CreateNavNode(std::string str, NavNodePtr& node_ptr);

    void CreateNavNode(const visibility_graph_msg::Node& msg, NavNodePtr& node_ptr);

    void AssignConnectNodes(const std::unordered_map<std::size_t, std::size_t>& idxs_map,
                            const NodePtrStack& graph,
                            std::vector<std::size_t>& node_idxs,
                            std::vector<NavNodePtr>& connects);

    template <typename Point>
    geometry_msgs::Point inline ToGeoMsgP(const Point& p) {
        geometry_msgs::Point geo_p;
        geo_p.x = p.x;
        geo_p.y = p.y; 
        geo_p.z = p.z;
        return geo_p;
    }

    template <typename T>
    bool IsTypeInStack(const T& elem, const std::vector<T>& T_stack) {
        if (std::find(T_stack.begin(), T_stack.end(), elem) != T_stack.end()) {
            return true;
        }
        return false;
    }

    inline void ResetGraph() {
        graph_nodes_.clear();
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