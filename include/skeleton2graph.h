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

typedef SkeletonPixel* SkeletonPixelPtr;
typedef std::pair<Hash, SkeletonPixel> Hash2Pixel;
typedef std::pair<Hash, SkeletonPixelPtr> Hash2PixelPtr;
typedef std::unordered_map<Hash, SkeletonPixel> MapHash2Pixel;
typedef std::unordered_map<Hash, SkeletonPixelPtr> MapHash2PixelPtr;
typedef Graph<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraph;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphHelper;

class Skeleton2Graph
{
public:
  Skeleton2Graph(const float &simplification_threshold) : m_simplification_threshold_(simplification_threshold)
  {
    if (simplification_threshold < 1.415)
    {
      std::cerr << "Please assign simplification_threshold > 1.415" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  ~Skeleton2Graph() {}

  void setFrame(const SkeletonFrame &frame)
  {
    // Convert input skeleton image to node set, and set connectivity
    m_image_width_ = frame.image_width;
    m_image_height_ = frame.image_height;
    initializeNodeList(frame);
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
    m_graph_helper_ptr_initial_ = new SkeletonGraphHelper(m_graph_initial_);
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
        m_graph_helper_ptr_initial_->addNodeWithHash(hash, pixel_uv);
        //std::cout << m_map_hash2pixel_ptr_initial_.at(hash)->getHash() << std::endl;
        m_map_hash2pixel_initial_.insert(Hash2Pixel{hash, pixel_uv});
        m_map_hash2index_initial_.insert(Hash2Index{hash, node_count});
        
        node_count++;
      }
    }

    for(auto kv: temp_map_pixel_hash_to_neighbor){
      Hash hash = kv.first;
      
      //std::cout << pixel_ptr->getHash() << std::endl;
      std::vector<Hash> hash_list_neighbors = kv.second;
      for(Hash hash_neighbor: hash_list_neighbors){
        /*
        SkeletonPixel pixel_ = m_map_hash2pixel_initial_.at(hash);
        SkeletonPixel pixel_n = m_map_hash2pixel_initial_.at(hash_neighbor);
        Hash test = m_graph_helper_ptr_initial_->getNodePtr(hash)->data.getHash();
        Hash testn = m_graph_helper_ptr_initial_->getNodePtr(hash_neighbor)->data.getHash();
        std::cout << hash << " " << hash_neighbor << " " << pixel_.getHash() << " " << pixel_n.getHash() << std::endl;
        */;
        //std::cout << hash << " " << hash_neighbor << " " << test << " " << testn << std::endl; <- OK

        //SkeletonPixelPtr m_map_hash2pixel_ptr_initial_.at(hash);
        //SkeletonPixelPtr m_map_hash2pixel_ptr_initial_.at(hash)
        
        EdgeSkeletonPixels edge_attributes = EdgeSkeletonPixels(
          m_map_hash2pixel_initial_.at(hash),
          m_map_hash2pixel_initial_.at(hash_neighbor)
        );
        m_graph_helper_ptr_initial_->addEdgeWithHash(edge_attributes, hash, hash_neighbor);
      }
    }

    //coco
    std::vector<Hash> hash_list_articular_node;
    std::vector<Hash> hash_list_gh = m_graph_helper_ptr_initial_->getHashList();
    articulationPoint<SkeletonPixel, EdgeSkeletonPixels>(m_graph_helper_ptr_initial_, hash_list_articular_node);
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
  //MapHash2PixelPtr m_map_hash2pixel_ptr_initial_;
  MapHash2Pixel m_map_hash2pixel_initial_;
  MapHash2Index m_map_hash2index_initial_;
  SkeletonGraph m_graph_initial_;
  SkeletonGraphHelper* m_graph_helper_ptr_initial_;

  float m_simplification_threshold_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_SKELETON2GRAPH_H_