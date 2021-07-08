import cv2
import numpy as np
from pys2g import LinearCluster
from s2g import Skeleton2GraphHelper
from typing import List, Optional


class SkeletonGraphPainter(object):
    def __init__(self, point_diameter: int = 2, edge_thickness: int = 2):
        self._point_diameter: int = point_diameter
        self._edge_thickness: int = edge_thickness
        self._image_to_draw: Optional[np.ndarray] = None
        self._s2g: Optional[Skeleton2GraphHelper] = None
        self._image_width = 0
        self._image_height = 0

    def __call__(self, image: np.ndarray, s2g: Skeleton2GraphHelper, draw_simplified=False) -> np.ndarray:
        self.set_image(image)
        self.set_converter(s2g, draw_simplified)
        self.draw_edges()
        self.draw_nodes()
        return self.image

    def set_image(self, image: np.ndarray):
        if len(image.shape) < 3:
            self._image_to_draw = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            self._image_to_draw = image

    def set_converter(self, s2g: Skeleton2GraphHelper, draw_simplified=False):
        self._s2g = s2g
        if not draw_simplified:
            self._nodes, self._edges = s2g.initial_graph_elements
        else:
            self._nodes, self._edges = s2g.simplified_graph_elements

    def draw_nodes(self):
        for point in self._nodes:
            cv2.circle(self._image_to_draw, (point[0], point[1]), self._point_diameter, (0, 0, 255), -1)

    def draw_edges(self):
        for edge in self._edges:
            p_from = self._nodes[edge[0]]
            p_to = self._nodes[edge[1]]
            cv2.line(self._image_to_draw, (p_from[0], p_from[1]), (p_to[0], p_to[1]), (255, 0, 0), self._edge_thickness)

    @property
    def image(self):
        return self._image_to_draw


class LinearClusterPainter(object):
    def __init__(self, point_diameter: int = 2, edge_thickness: int = 2):
        self._point_diameter: int = point_diameter
        self._edge_thickness: int = edge_thickness
        self._image_to_draw: Optional[np.ndarray] = None
        self._s2g: Optional[Skeleton2GraphHelper] = None
        self._linear_clusters: Optional[List[LinearCluster]] = None
        self._image_width = 0
        self._image_height = 0

    def __call__(
        self,
        image: np.ndarray,
        linear_clusters: List[LinearCluster],
        s2g: Skeleton2GraphHelper,
        draw_fitted_line: bool = False,
        draw_mutual_cluster_connections: bool = False,
    ) -> np.ndarray:
        self.set_image(image)
        self.set_clusters(linear_clusters)
        self.set_converter(s2g)
        self.draw_categorized_edges()
        if draw_mutual_cluster_connections:
            self.draw_cluster_connection_bridge()
        self.draw_end_points()
        if draw_fitted_line:
            self.draw_line_models()
        return self.image

    def set_image(self, image: np.ndarray):
        if len(image.shape) < 3:
            self._image_to_draw = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            self._image_to_draw = image

    def set_converter(self, s2g: Skeleton2GraphHelper):
        self._s2g = s2g
        self._nodes, self._edges = s2g.simplified_graph_elements

    def set_clusters(self, linear_clusters: List[LinearCluster]):
        self._linear_clusters = linear_clusters

    def draw_end_points(self):
        for linear_cluster in self._linear_clusters:
            cluster_points = linear_cluster.points()
            end_points = [cluster_points[end_point_index] for end_point_index in linear_cluster.indices_end_points()]
            for point in end_points:
                cv2.circle(self._image_to_draw, (point[0], point[1]), int(self._point_diameter * 1.5), (0, 0, 255), -1)

    def draw_categorized_edges(self):
        """
        Draw line and points
        """
        for label_m1, line_segment in enumerate(self._linear_clusters):
            # convert {0,255}-binary image to labelled image
            label_var = label_m1 + 1
            label_img_temp = (line_segment.binary_mask(self._edge_thickness)).astype(np.float) / 255 * label_var
            label_img_normalized = label_img_temp / self.n_clusters
            label_img_colorized = cv2.applyColorMap((255.0 * label_img_normalized).astype(np.uint8), cv2.COLORMAP_HSV)
            label_img_colorized[np.where(label_img_temp == 0)[0], np.where(label_img_temp == 0)[1], :] = 0
            self._image_to_draw[np.where(label_img_colorized > 0)] = label_img_colorized[np.where(label_img_colorized > 0)]

    def draw_line_models(self):
        for linear_cluster in self._linear_clusters:
            cluster_points = np.array(linear_cluster.points())
            line_direction = np.array(linear_cluster.direction())
            point_highest_y = cluster_points[linear_cluster.point_index_lowest_y()]
            point_lowest_y = cluster_points[linear_cluster.point_index_highest_y()]
            point_mean = (point_highest_y + point_lowest_y) / 2
            half_length_segment = linear_cluster.length() / 2
            p1 = point_mean - line_direction * half_length_segment
            p2 = point_mean + line_direction * half_length_segment
            cv2.line(self._image_to_draw, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 0, 0), 2)
            cv2.line(self._image_to_draw, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 0), 1)

    def draw_cluster_connection_bridge(self):
        mutual_cluster_index_pairs, point_index_pairs_mutual_clusters = self._s2g.cluster_connection_information
        for cluster_index_pair, point_index_pair in zip(mutual_cluster_index_pairs, point_index_pairs_mutual_clusters):
            cluster_a = self._linear_clusters[cluster_index_pair[0]]
            cluster_b = self._linear_clusters[cluster_index_pair[1]]
            p_a = np.asarray(cluster_a.points())[point_index_pair[0]]
            p_b = np.asarray(cluster_b.points())[point_index_pair[1]]
            cv2.line(self._image_to_draw, (p_a[0], p_a[1]), (p_b[0], p_b[1]), (0, 192, 200), 2)

    @property
    def image(self):
        return self._image_to_draw

    @property
    def n_clusters(self):
        return len(self._linear_clusters)


def show_image(image, title="image", scale=1.0):
    while cv2.waitKey(10) & 0xFF not in [ord("q"), 27]:
        image_resize = cv2.resize(image, None, fx=scale, fy=scale)
        cv2.imshow(title, image_resize)
    cv2.destroyAllWindows()


def write_image(image, save_path, scale=1.0):
    image_resize = cv2.resize(image, None, fx=scale, fy=scale)
    cv2.imwrite(save_path, image_resize)
    cv2.waitKey(10)
