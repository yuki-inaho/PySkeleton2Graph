#ifndef PYSKELETON2GRAPH_INCLUDE_ENUM_H_
#define PYSKELETON2GRAPH_INCLUDE_ENUM_H_

enum struct PointType
{
    kEndPoint,
    kBridgePoint,
    kJunctionPoint
};

enum struct ConnectedComponent
{
    kOnlyBridge,
    kDirectional
};

enum struct NeighborGetterTypeJP //JP: junction point
{
    kFromTo,
    kDirectional
};

#endif