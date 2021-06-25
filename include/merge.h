#ifndef PYSKELETON2GRAPH_INCLUDE_MERGE_H_
#define PYSKELETON2GRAPH_INCLUDE_MERGE_H_

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

class ClusterMergeHelper
{
public:
    ClusterMergeHelper(SkeletonGraphHelperPtr graph_helper, const std::vector<LinearCluster> &linear_cluster_list)
    {
        int32_t n_cluster = linear_cluster_list.size();
        for(int32_t i=0; i< n_cluster; i++){
            std::shared_ptr<LinearCluster> cluster_ptr = std::make_shared<LinearCluster>(linear_cluster_list[i]);
            int32_t cluster_label = linear_cluster_list[i].label();
            m_map_label_to_cluster_ptr_.insert(
                std::pair<int32_t, std::shared_ptr<LinearCluster>>{cluster_label, cluster_ptr}
            );
        }
    }

    void addNode(LinearCluster linear_cluster, int32_t cluster_label)
    {
        ClusterGraphNode *node_ptr = graph.addNode(linear_cluster);
        m_map_label_to_node_ptr_.insert(std::pair<Hash, ClusterGraphNode *>(cluster_label, node_ptr));
        m_map_node_ptr_to_label_.insert(std::pair<ClusterGraphNode *, Hash>(node_ptr, cluster_label));
    }

    void addEdge(int32_t label_cluster_source, int32_t label_cluster_target)
    {
        //FIXME
        ClusterGraphEdge *edge_ptr = graph.addEdge(
            m_map_label_to_cluster_ptr_.at(label_cluster_source)->getDiffDegree(
                m_map_label_to_cluster_ptr_.at(label_cluster_target).get()),
            m_map_label_to_node_ptr_.at(label_cluster_source),
            m_map_label_to_node_ptr_.at(label_cluster_target));
        m_map_hash_pair_to_edge_ptr_.insert({{label_cluster_source, label_cluster_target}, edge_ptr});
    }

private:
    std::unordered_map<int32_t, std::shared_ptr<LinearCluster>> m_map_label_to_cluster_ptr_;

    // TODO; use smart pointer
    std::unordered_map<int32_t, ClusterGraphNode *> m_map_label_to_node_ptr_;
    std::unordered_map<ClusterGraphNode *, int32_t> m_map_node_ptr_to_label_;
    std::map<std::pair<Hash, Hash>, ClusterGraphEdge *> m_map_hash_pair_to_edge_ptr_;

    ClusterGraph graph;
};

#endif // PYSKELETON2GRAPH_INCLUDE_MERGE_H_

//std::copy(linear_cluster_list.begin(), linear_cluster_list.end(), std::back_inserter(m_linear_cluster_list_));
//std::vector<LinearCluster> m_linear_cluster_list_;
