#ifndef PYSKELETON2GRAPH_INCLUDE_TYPEDEF_H_
#define PYSKELETON2GRAPH_INCLUDE_TYPEDEF_H_
#include <cstdint>
#include <unordered_map>

typedef int32_t Hash;
typedef std::pair<Hash, int32_t> Hash2Index;
typedef std::pair<int32_t, Hash> Index2Hash;
typedef std::unordered_map<Hash, int32_t> MapHash2Index;
typedef std::unordered_map<int32_t, Hash> MapIndex2Hash;

#endif  // PYSKELETON2GRAPH_INCLUDE_TYPEDEF_H_