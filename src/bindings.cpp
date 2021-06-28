#include <string>
#include <pybind11/stl.h>
#include "ndarray_converter.h"
#include "pybind11/pybind11.h"

#include "graph.h"
#include "frame.h"
#include "skeleton2graph.h"
#include "linear_cluster.h"

namespace py = pybind11;

PYBIND11_MODULE(pys2g, m)
{
    NDArrayConverter::init_numpy();
    py::enum_<LinearClusterType>(m, "LinearClusterType")
        .value("EndPointCluster", LinearClusterType::kEndPointCluster)
        .value("BridgePointCluster", LinearClusterType::kBridgePointCluster)
        .value("JunctionPointCluster", LinearClusterType::kJunctionPointCluster)
        .export_values();
    py::class_<SkeletonFrame>(m, "SkeletonFrame")
        .def(
            py::init<const cv::Mat &>(),
            py::arg("skeleton_image"));
    py::class_<LinearCluster>(m, "LinearCluster")
        .def("label", &LinearCluster::label)
        .def("type", &LinearCluster::type)
        .def("size", &LinearCluster::size)
        .def("line", &LinearCluster::line)
        .def("points", &LinearCluster::points)
        .def("edges", &LinearCluster::edges);
    py::class_<Skeleton2Graph>(m, "Skeleton2Graph")
        .def(
            py::init<const float &, const float &>(),
            py::arg("simplification_threshold"),
            py::arg("directional_threshold"))
        .def("set_frame", &Skeleton2Graph::setFrame)
        .def("simplify", &Skeleton2Graph::Simplification)
        .def("clustering", &Skeleton2Graph::clustering)
        .def("get_linear_clusters", &Skeleton2Graph::getLinearClusters)
        .def("get_node_labels", &Skeleton2Graph::getNodeLabels)
        .def("get_node_positions", &Skeleton2Graph::getNodePositions)
        .def("get_edges", &Skeleton2Graph::getEdges);
}
