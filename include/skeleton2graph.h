#ifndef PYSKELETON2GRAPH_INCLUDE_SKELETON2GRAPH_H_
#define PYSKELETON2GRAPH_INCLUDE_SKELETON2GRAPH_H_

#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "typedef.h"
#include "pixel.h"
#include "frame.h"
#include "enum.h"
#include "graph.h"
#include "graph_helper.h"
#include "biconnected_component.h"
#include "pruning.h"
#include "connected_component.h"
#include "linear_cluster.h"
#include "merge.h"

typedef SkeletonPixel *SkeletonPixelPtr;
typedef std::pair<Hash, SkeletonPixel> Hash2Pixel;
typedef std::pair<Hash, SkeletonPixelPtr> Hash2PixelPtr;
typedef std::unordered_map<Hash, SkeletonPixel> MapHash2Pixel;
typedef std::unordered_map<Hash, SkeletonPixelPtr> MapHash2PixelPtr;
typedef Graph<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraph;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphHelper;

class Skeleton2Graph
{
public:
  Skeleton2Graph(const float &simplification_threshold, const float &directional_threshold) : m_simplification_threshold_(simplification_threshold), m_directional_threshold_(directional_threshold)
  {
    if (simplification_threshold < 1.415)
    {
      std::cerr << "Please assign simplification_threshold in the range of > 1.415" << std::endl;
      exit(EXIT_FAILURE);
    }
    if ((0 > m_directional_threshold_) || (m_directional_threshold_ > 90))
    {
      std::cerr << "Please assign directional_threshold in the range of 0 <= directional_threshold < 90" << std::endl;
      exit(EXIT_FAILURE);
    }

    m_cluster_proximity_threshold_ = m_simplification_threshold_ * 1.2;
  }

  ~Skeleton2Graph() {}

  void setFrame(const SkeletonFrame &frame)
  {
    // Convert input skeleton image to node set, and set connectivity
    m_image_width_ = frame.image_width;
    m_image_height_ = frame.image_height;
    initializeNodeList(frame);
  }

  void Simplification()
  {
    /*
    Step 1. Make junction points unique
    */
    std::vector<Hash> hash_list_articular_node;
    std::vector<Hash> hash_list_gh = m_graph_helper_ptr_->getHashList();
    std::vector<std::vector<Hash>> hash_list_clique_to_compress;

    /// TODO: Rename function
    articulationPoint<SkeletonPixel, EdgeSkeletonPixels>(m_graph_helper_ptr_, hash_list_articular_node, hash_list_clique_to_compress);

    /// Remove redundant node
    for (std::vector<Hash> hash_list_clique : hash_list_clique_to_compress)
    {
      /// Search locally maximum connectivity node, and merge around node to it
      int32_t max_connectivity = 0;
      Hash hash_arg_max_connectivity = -1;
      for (Hash hash_tmp : hash_list_clique)
      {
        int32_t connectivity_tmp = m_graph_helper_ptr_->getNodePtr(hash_tmp)->data.getConnectivity();
        if (max_connectivity < connectivity_tmp)
        {
          max_connectivity = connectivity_tmp;
          hash_arg_max_connectivity = hash_tmp;
        }
      }
      m_graph_helper_ptr_->compressNodeSet(hash_arg_max_connectivity, hash_list_clique);
    }
    m_graph_helper_ptr_->refreshGraphInfo();
    m_graph_helper_ptr_->validateGraphInfo();

    /*
    Step 2. Bridge Link Pruning
    */
    PruningHelper pruning = PruningHelper(m_simplification_threshold_, m_graph_helper_ptr_);
    pruning.setup();
    pruning.searchPruneTarget();
    std::vector<int32_t> hash_list_pruning_target = pruning.getHashListForPruning();
    for (Hash hash_pruning_target : hash_list_pruning_target)
    {
      m_graph_helper_ptr_->removeNode(hash_pruning_target);
    }
    m_graph_helper_ptr_->refreshGraphInfo();
    m_graph_helper_ptr_->validateGraphInfo();

    /// Removing too small connected component cluster points
    GraphConnectedComponent cc = GraphConnectedComponent(m_graph_helper_ptr_);
    cc.setup();
    cc.compute();
    std::vector<std::vector<Hash>> hash_list_each_cc = cc.getConnectedComponent();
    for (std::vector<Hash> hash_list_cc : hash_list_each_cc)
    {
      /// node size is too small
      if (hash_list_cc.size() < 2)
      {
        for (Hash hash_cc_elem : hash_list_cc)
          m_graph_helper_ptr_->removeNode(hash_cc_elem);
        m_graph_helper_ptr_->refreshGraphInfo();
        m_graph_helper_ptr_->validateGraphInfo();
      }
    }

    /// Labelling with
    m_graph_helper_ptr_->setupOutputGraph();
    m_node_position_list_output_ = m_graph_helper_ptr_->getNodePositions();
    m_edge_list_output_ = m_graph_helper_ptr_->getEdges();
  }

  void computeDirectionalConnectedComponent()
  {
    GraphConnectedComponent cc = GraphConnectedComponent(m_graph_helper_ptr_);
    cc.setup();
    cc.compute(ConnectedComponent::kDirectional, m_directional_threshold_);
    std::vector<std::vector<Hash>> hash_list_each_cc = cc.getConnectedComponent();
    m_graph_helper_ptr_->setConnectedComponentLabels(hash_list_each_cc);
    m_node_labels_output_ = m_graph_helper_ptr_->getNodeLabels();

    int32_t n_cluster = hash_list_each_cc.size();

    // Setup for cluster merging

    int32_t cluster_index = 1;
    m_linear_cluster_list_.clear();
    for (std::vector<Hash> hash_list_cc : hash_list_each_cc)
    {
      LinearCluster linear_cluster(cluster_index, m_graph_helper_ptr_, m_cluster_proximity_threshold_);
      for (Hash hash : hash_list_cc)
      {
        linear_cluster.addNodePtr(m_graph_helper_ptr_->getNodePtr(hash));
      }
      m_linear_cluster_list_.push_back(linear_cluster);
      cluster_index++;
    }
  }

  void mergeClusters()
  {
    for (LinearCluster m_linear_cluster_ : m_linear_cluster_list_)
    {
      m_linear_cluster_.fitLine();
    }
    ClusterMergeHelper merge_helper(m_graph_helper_ptr_, m_linear_cluster_list_);
  }

  std::vector<int32_t> getNodeLabels() const
  {
    return m_node_labels_output_;
  }

  std::vector<std::vector<int32_t>> getNodePositions()
  {
    return m_node_position_list_output_;
  }

  std::vector<std::vector<int32_t>> getEdges()
  {
    return m_edge_list_output_;
  }

private:
  inline int32_t pos2hash(const int32_t &x_pos, const int32_t &y_pos, const int32_t &image_width, const int32_t &image_height)
  {
    return y_pos * image_width + x_pos;
  }

  inline void hash2pos(const int32_t &hash, int32_t &x_pos, int32_t &y_pos, const int32_t &image_width, const int32_t &image_height)
  {
    x_pos = hash % image_width;
    y_pos = hash / image_width;
  }

  void initializeNodeList(const SkeletonFrame &frame)
  {
    int32_t node_count = 0;
    std::unordered_map<Hash, std::vector<Hash>> temp_map_pixel_hash_to_neighbor;
    m_graph_helper_ptr_ = new SkeletonGraphHelper(m_graph_initial_);
    for (int32_t v = 0; v < frame.image_height; v++)
    {
      for (int32_t u = 0; u < frame.image_width; u++)
      {
        Hash hash = pos2hash(u, v, frame.image_width, frame.image_height);
        if (frame.data[hash] == 0)
          continue;

        // Calculate pixel connectivity
        std::vector<Hash> neighbor_hash_list_uv = getPixelNeighborList(u, v, frame);
        int8_t connectivity = neighbor_hash_list_uv.size();
        if (connectivity == 0)
          continue;
        temp_map_pixel_hash_to_neighbor.insert(std::pair<Hash, std::vector<Hash>>{hash, neighbor_hash_list_uv});

        // Set pixel attributes
        SkeletonPixel pixel_uv = SkeletonPixel();
        pixel_uv.setPosition(u, v);
        pixel_uv.setHash(hash);
        pixel_uv.setLabel(hash);                // initial label <- initial node hash
        pixel_uv.setConnectivity(connectivity); // and point type
        pixel_uv.setPointType();

        // Add node (skeleton pixel)
        m_graph_helper_ptr_->addNode(hash, pixel_uv);
        m_map_hash2pixel_initial_.insert(Hash2Pixel{hash, pixel_uv});
        m_map_hash2index_initial_.insert(Hash2Index{hash, node_count});
        node_count++;
      }
    }

    for (auto kv : temp_map_pixel_hash_to_neighbor)
    {
      Hash hash = kv.first;
      std::vector<Hash> hash_list_neighbors = kv.second;
      for (Hash hash_neighbor : hash_list_neighbors)
      {
        EdgeSkeletonPixels edge_attributes = EdgeSkeletonPixels(
            m_map_hash2pixel_initial_.at(hash),
            m_map_hash2pixel_initial_.at(hash_neighbor));
        m_graph_helper_ptr_->addEdge(edge_attributes, hash, hash_neighbor);
      }
    }
    m_graph_helper_ptr_->validateGraphInfo();

    m_graph_helper_ptr_->setupOutputGraph();
    m_node_position_list_output_ = m_graph_helper_ptr_->getNodePositions();
    m_edge_list_output_ = m_graph_helper_ptr_->getEdges();
  }

  /*
    Calculate number of 8-nearest neighbor pixel on skeleton image;
  */
  std::vector<Hash> getPixelNeighborList(const int32_t &u, const int32_t &v, const SkeletonFrame &frame)
  {
    int32_t kernel_start_pos_x = std::max(0, u - 1);
    int32_t kernel_start_pos_y = std::max(0, v - 1);
    int32_t kernel_end_pos_x = std::min(u + 1, frame.image_width - 1);
    int32_t kernel_end_pos_y = std::min(v + 1, frame.image_height - 1);
    std::vector<Hash> neighbor_hash_list;
    for (int32_t x = kernel_start_pos_x; x <= kernel_end_pos_x; x++)
    {
      for (int32_t y = kernel_start_pos_y; y <= kernel_end_pos_y; y++)
      {
        if ((x == u) && (y == v))
          continue;
        Hash hash = pos2hash(x, y, frame.image_width, frame.image_height);
        if (frame.data[hash] > 0)
          neighbor_hash_list.push_back(hash); // when skeleton pixel exists
      }
    }
    return neighbor_hash_list;
  }

  int32_t m_image_width_, m_image_height_;

  std::vector<SkeletonPixel> m_vector_skeleton_pixels_;
  MapHash2Pixel m_map_hash2pixel_initial_;
  MapHash2Index m_map_hash2index_initial_;
  SkeletonGraph m_graph_initial_;
  SkeletonGraphHelper *m_graph_helper_ptr_;

  float m_cluster_proximity_threshold_;
  float m_simplification_threshold_;
  float m_directional_threshold_;

  std::vector<int32_t> m_node_labels_output_;
  std::vector<std::vector<int32_t>> m_node_position_list_output_;
  std::vector<std::vector<int32_t>> m_edge_list_output_;
  std::vector<LinearCluster> m_linear_cluster_list_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_SKELETON2GRAPH_H_