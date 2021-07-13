#ifndef PYSKELETON2GRAPH_INCLUDE_PURUNING_H_
#define PYSKELETON2GRAPH_INCLUDE_PURUNING_H_

#include <algorithm>
#include <iostream>
#include "enum.h"
#include "pixel.h"
#include "graph.h"
#include "graph_helper.h"
#include "typedef_graph.h"

class PruningHelper
{
public:
    PruningHelper(const float &simplification_threshold, const SkeletonGraphHelperPtr graph_helper_ptr) : m_simplification_threshold_(simplification_threshold), m_graph_helper_ptr_(graph_helper_ptr) {}
    void setup()
    {
        setEndPoints();
        std::vector<Hash> hash_list = m_graph_helper_ptr_->getHashList();

        int node_count = 0;
        for (auto it = hash_list.begin(); it != hash_list.end(); ++it)
        {
            m_map_hash2index_.insert(std::pair<Hash, int32_t>{*it, node_count});
            node_count++;
        }
    }

    /*
    Pruning graph node to reduce skeleton density
    */
    void searchPruneTarget()
    {
        int32_t n_node = m_graph_helper_ptr_->size();
        std::vector<bool> is_visited(n_node, false);
        std::vector<bool> is_prune_target(n_node, false);

        float cumulative_distance;
        for (Hash hash_end_point : m_hash_list_end_points_)
        {
            if (is_visited[hash2index(hash_end_point)])
                continue;
            cumulative_distance = 0;
            depth_search_prune_target(hash_end_point, -1, is_visited, is_prune_target, cumulative_distance);
        }
        m_is_prune_target_.clear();
        std::copy(is_prune_target.begin(), is_prune_target.end(), std::back_inserter(m_is_prune_target_));
    }

    std::vector<Hash> getHashListForPruning()
    {
        std::vector<Hash> hash_list_pruning_target;
        std::vector<Hash> hash_list = m_graph_helper_ptr_->getHashList();
        for (Hash hash : hash_list)
        {
            if (m_is_prune_target_[hash2index(hash)])
                hash_list_pruning_target.push_back(hash);
        }
        return hash_list_pruning_target;
    }

private:
    int32_t hash2index(Hash hash)
    {
        return m_map_hash2index_.at(hash);
    }

    void depth_search_prune_target(const Hash &hash_v, const Hash &hash_parent, std::vector<bool> &is_visited, std::vector<bool> &is_prune_target, float &cumulative_distance)
    {
        if (is_visited[hash2index(hash_v)])
            return;

        is_visited[hash2index(hash_v)] = true;
        if (hash_parent >= 0)
        {
            SkeletonGraphEdge *edge_ptr = m_graph_helper_ptr_->getEdgePtr(hash_v, hash_parent);
            PointType node_point_type = m_graph_helper_ptr_->getNodePtr(hash_v)->data.getPointType();
            cumulative_distance += edge_ptr->data.getEdgeLength();
            if (
                cumulative_distance >= m_simplification_threshold_ ||
                node_point_type != PointType::kBridgePoint)
            {
                cumulative_distance = 0;
            }
            else
            {
                is_prune_target[hash2index(hash_v)] = true;
            }
        }

        std::vector<Hash> neighbor_hash_list_v = m_graph_helper_ptr_->getNeighborHashList(hash_v);
        for (Hash hash_n : neighbor_hash_list_v)
        {
            if (hash_n != hash_parent)
            {
                depth_search_prune_target(
                    hash_n, hash_v, is_visited, is_prune_target, cumulative_distance);
            }
        }
    }

    void setEndPoints()
    {
        m_hash_list_end_points_.clear();
        std::vector<Hash> hash_list_temp = m_graph_helper_ptr_->getHashList();
        std::copy_if(
            hash_list_temp.begin(), hash_list_temp.end(),
            std::back_inserter(m_hash_list_end_points_),
            [&](Hash hash)
            {
                return m_graph_helper_ptr_->getNodePtr(hash)->data.getPointType() == PointType::kEndPoint;
            });
    }

    float m_simplification_threshold_;
    std::unordered_map<Hash, int32_t> m_map_hash2index_;
    SkeletonGraphHelperPtr m_graph_helper_ptr_;
    std::vector<Hash> m_hash_list_end_points_;
    std::vector<bool> m_is_prune_target_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_PURUNING_H_
