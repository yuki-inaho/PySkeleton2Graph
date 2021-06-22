#include <string>
#include <pybind11/stl.h>
#include "ndarray_converter.h"
#include "pybind11/pybind11.h"
#include "graph.h"
#include "frame.h"
#include "skeleton2graph.h"

namespace py = pybind11;

PYBIND11_MODULE(pys2g, m)
{
    NDArrayConverter::init_numpy();
    py::class_<SkeletonFrame>(m, "SkeletonFrame")
        .def(
            py::init<const cv::Mat &>(),
            py::arg("skeleton_image"));
    py::class_<Skeleton2Graph>(m, "Skeleton2Graph")
        .def(
            py::init<const float &>(),
            py::arg("simplification_threshold"))
        .def("set_frame", &Skeleton2Graph::setFrame);
}

