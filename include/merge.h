#ifndef PYSKELETON2GRAPH_INCLUDE_MERGE_H_
#define PYSKELETON2GRAPH_INCLUDE_MERGE_H_

#include "enum.h"
#include "pixel.h"
#include "graph.h"
#include "graph_helper.h"
#include "linear_cluster.h"

typedef Node<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphNode;
typedef Edge<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphEdge;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> *SkeletonGraphHelperPtr;
typedef Node<LinearCluster, float> ClusterGraphNode;
typedef Edge<LinearCluster, float> ClusterGraphEdge;
typedef Graph<LinearCluster, float> ClusterGraph;

/// TODO: use smart pointers instead of raw pointers
class ClusterMergeHelper
{
public:
    ClusterMergeHelper(SkeletonGraphHelperPtr graph_helper, const std::vector<LinearCluster> &linear_cluster_list, const float &angular_threshold_cluster_merge) : n_nodes(0), n_edges(0), m_angular_threshold_cluster_merge_(angular_threshold_cluster_merge)
    {
        int32_t n_cluster = linear_cluster_list.size();
        for (int32_t i = 0; i < n_cluster; i++)
        {
            std::shared_ptr<LinearCluster> cluster_ptr = std::make_shared<LinearCluster>(linear_cluster_list[i]);
            int32_t cluster_label = linear_cluster_list[i].label();
            m_map_label_to_cluster_ptr_.insert(
                std::pair<int32_t, std::shared_ptr<LinearCluster>>{cluster_label, cluster_ptr});
        }
    }

    void addNode(LinearCluster linear_cluster)
    {
        ClusterGraphNode *node_ptr = graph.addNode(linear_cluster);
        int32_t cluster_label = linear_cluster.label();
        m_map_label_to_node_ptr_.insert(std::pair<Hash, ClusterGraphNode *>(cluster_label, node_ptr));
        m_map_node_ptr_to_label_.insert(std::pair<ClusterGraphNode *, Hash>(node_ptr, cluster_label));
        n_nodes++;
    }

    void addEdge(int32_t label_cluster_source, int32_t label_cluster_target)
    {
        ClusterGraphEdge *edge_ptr = graph.addEdge(
            m_map_label_to_cluster_ptr_.at(label_cluster_source)->getDiffDegree(m_map_label_to_cluster_ptr_.at(label_cluster_target).get()),
            m_map_label_to_node_ptr_.at(label_cluster_source),
            m_map_label_to_node_ptr_.at(label_cluster_target));
        m_map_hash_pair_to_edge_ptr_.insert({{label_cluster_source, label_cluster_target}, edge_ptr});
        n_edges++;
    }

    void merge()
    {
        std::unordered_map<int32_t, bool> map_label_to_check_visited;
        for (auto kv : m_map_label_to_cluster_ptr_)
        {
            int32_t label = kv.first;
            map_label_to_check_visited.insert({label, false});
        }

        std::unordered_map<int32_t, std::vector<int32_t>> map_label_to_merged;
        std::vector<int32_t> label_list_end_point_cluster = get_end_points_cluster_label();
        for (int32_t label : label_list_end_point_cluster)
        {
            if (map_label_to_check_visited.at(label))
                continue;
            std::vector<int32_t> label_list_to_merge;
            depth_first_search_cluster_merge(label, -1, label_list_to_merge, map_label_to_check_visited);
            map_label_to_merged.insert({label, label_list_to_merge});
        }

        // Rewrite label information
        for (auto kv : map_label_to_merged)
        {
            int32_t label = kv.first;
            int32_t size = kv.second.size();
        }
    }

    void depth_first_search_cluster_merge(
        const int32_t &label_current, const int32_t &label_parent,
        std::vector<int32_t> &label_list_to_merge,
        std::unordered_map<int32_t, bool> &map_label_to_check_visited)
    {
        if (map_label_to_check_visited.at(label_current))
            return;

        // to accept duplicated labelling of specified cluster
        if (m_map_label_to_cluster_ptr_.at(label_current)->type() != LinearClusterType::kJunctionPointCluster)
            map_label_to_check_visited[label_current] = true;

        label_list_to_merge.push_back(label_current);
        std::vector<int32_t> label_list_neighbors = getNeighborLabelListfromClusterLabel(label_current);
        for (int32_t label_neighbor : label_list_neighbors)
        {
            if (label_neighbor == label_parent)
                continue;

            /// check merge condition
            std::shared_ptr<LinearCluster> cluster_current_label = m_map_label_to_cluster_ptr_.at(label_current);
            std::shared_ptr<LinearCluster> cluster_neighbor_label = m_map_label_to_cluster_ptr_.at(label_neighbor);
            if (cluster_current_label->getDiffDegree(cluster_neighbor_label.get()) < m_angular_threshold_cluster_merge_)
            {
                depth_first_search_cluster_merge(label_neighbor, label_current, label_list_to_merge, map_label_to_check_visited);
            }
        }
    }

private:
    std::vector<int32_t> getNeighborLabelListfromClusterLabel(const int32_t &label)
    {
        std::vector<ClusterGraphNode *> cluster_ptr_list_neighbors = m_map_label_to_node_ptr_.at(label)->getNeighborNodes();
        std::vector<int32_t> label_list_neighbors;
        std::transform(
            cluster_ptr_list_neighbors.begin(), cluster_ptr_list_neighbors.end(),
            std::back_inserter(label_list_neighbors),
            [](ClusterGraphNode *node)
            {
                return node->data.label();
            });
        return label_list_neighbors;
    }

    std::vector<int32_t> get_end_points_cluster_label()
    {
        std::vector<int32_t> labels_end_point_cluster;
        for (auto kv : m_map_label_to_cluster_ptr_)
        {
            int32_t label = kv.first;
            std::shared_ptr<LinearCluster> cluster_ptr = kv.second;
            if (cluster_ptr->type() == LinearClusterType::kEndPointCluster)
            {
                labels_end_point_cluster.push_back(label);
            }
        }
        return labels_end_point_cluster;
    }

    float m_angular_threshold_cluster_merge_;
    std::unordered_map<int32_t, std::shared_ptr<LinearCluster>> m_map_label_to_cluster_ptr_;

    // TODO; use smart pointer
    std::unordered_map<int32_t, ClusterGraphNode *> m_map_label_to_node_ptr_;
    std::unordered_map<ClusterGraphNode *, int32_t> m_map_node_ptr_to_label_;
    std::map<std::pair<Hash, Hash>, ClusterGraphEdge *> m_map_hash_pair_to_edge_ptr_;

    int32_t n_nodes, n_edges;
    ClusterGraph graph;
};

#endif // PYSKELETON2GRAPH_INCLUDE_MERGE_H_

//std::copy(linear_cluster_list.begin(), linear_cluster_list.end(), std::back_inserter(m_linear_cluster_list_));
//std::vector<LinearCluster> m_linear_cluster_list_;
