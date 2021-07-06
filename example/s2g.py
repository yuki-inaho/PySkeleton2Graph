from typing import List, Optional
from pys2g import SkeletonFrame, Skeleton2Graph, LinearCluster
from decorator import timeit

# TODO: Write explanation
class Skeleton2GraphHelper(object):
    def __init__(self, simplification_threshold: float = 10, directional_threshold: float = 30):
        self._s2g: Skeleton2Graph = Skeleton2Graph(simplification_threshold=simplification_threshold, directional_threshold=directional_threshold)

        self._frame: Optional[SkeletonFrame] = None
        self._node_init: Optional[List[int]] = None
        self._edge_init: Optional[List[int]] = None

        self._node_simplified: Optional[List[int]] = None
        self._edge_simplified: Optional[List[int]] = None

        self._linear_clusters: Optional[List[LinearCluster]] = None
        self._node_labels_simplified: Optional[List[int]] = None
        self._mutual_cluster_index_pairs: Optional[List[int]] = None
        self._point_index_pairs_mutual_clusters: Optional[List[int]] = None

    @timeit
    def set_frame(self, skeleton_image):
        self._frame = SkeletonFrame(skeleton_image)
        self._s2g.set_frame(self._frame)

    @timeit
    def initialize_graph_state(self):
        self._node_init = self._s2g.get_node_positions()
        self._edge_init = self._s2g.get_edges()

    @timeit
    def analyze_graph(self):
        self._s2g.simplify()
        self._s2g.clustering()
        self._node_simplified = self._s2g.get_node_positions()
        self._edge_simplified = self._s2g.get_edges()
        self._node_labels_simplified = self._s2g.get_node_labels()

        self._linear_clusters = self._s2g.get_linear_clusters()
        self._mutual_cluster_index_pairs = self._s2g.get_mutual_cluster_index_pair()
        self._point_index_pairs_mutual_clusters = self._s2g.get_point_index_pair_mutual_clusters()

    @property
    def initial_graph_elements(self):
        return self._node_init, self._edge_init

    @property
    def simplified_graph_elements(self):
        return self._node_simplified, self._edge_simplified

    @property
    def node_labels(self):
        return self._node_labels_simplified

    @property
    def linear_clusters(self):
        return self._linear_clusters

    @property
    def cluster_connection_information(self):
        return self._mutual_cluster_index_pairs, self._point_index_pairs_mutual_clusters
