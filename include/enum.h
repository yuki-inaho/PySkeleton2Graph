#ifndef PYSKELETON2GRAPH_INCLUDE_ENUM_H_
#define PYSKELETON2GRAPH_INCLUDE_ENUM_H_

enum struct PointType
{
    kEndPoint,
    kBridgePoint,
    kJunctionPoint
};

enum struct LinearClusterType
{
    kEndPointCluster,
    kBridgePointCluster,
    kJunctionPointCluster
};

enum struct ConnectedComponent
{
    kSimple,
    kDirectional
};

#endif