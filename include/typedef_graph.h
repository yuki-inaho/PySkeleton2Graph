#ifndef PYSKELETON2GRAPH_INCLUDE_TYPEDEF_GRAPH_H_
#define PYSKELETON2GRAPH_INCLUDE_TYPEDEF_GRAPH_H_

typedef Node<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphNode;
typedef Edge<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphEdge;
typedef Graph<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraph;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphHelper;
typedef std::shared_ptr<GraphHelper<SkeletonPixel, EdgeSkeletonPixels>> SkeletonGraphHelperPtr;

#endif //PYSKELETON2GRAPH_INCLUDE_TYPEDEF_GRAPH_H_
