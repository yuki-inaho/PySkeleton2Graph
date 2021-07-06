import cv2
import numpy as np
from pathlib import Path
import time
from typing import List
from pys2g import SkeletonFrame, Skeleton2Graph, LinearCluster

SCRIPT_DIR = str(Path().parent)


def draw_graph(image, points, edges, label=None, circle_diameter=2, edge_bold=1, draw_cluster_info=False):
    assert (len(image.shape) == 3) and (image.shape[-1] == 3)
    draw_frame = image.copy()
    if (label is None) or (~draw_cluster_info):
        for edge in edges:
            p_from = points[edge[0]]
            p_to = points[edge[1]]
            cv2.line(draw_frame, (p_from[0], p_from[1]), (p_to[0], p_to[1]), (255, 0, 0), edge_bold)
        for point in points:
            cv2.circle(draw_frame, (point[0], point[1]), circle_diameter, (0, 0, 255), -1)
    else:
        label_img_temp = np.zeros((draw_frame.shape[0], draw_frame.shape[1]), dtype=np.uint8)
        for i, point in enumerate(points):
            cv2.circle(label_img_temp, (point[0], point[1]), circle_diameter, (label[i]), -1)
        for i, edge in enumerate(edges):
            p_from = points[edge[0]]
            p_to = points[edge[1]]
            cv2.line(label_img_temp, (p_from[0], p_from[1]), (p_to[0], p_to[1]), (label[edge[0]]), edge_bold)
        label_img_normalized = (label_img_temp - label_img_temp.min()) / (label_img_temp.max() - label_img_temp.min())
        label_img_colorized = cv2.applyColorMap((255.0 * label_img_normalized).astype(np.uint8), cv2.COLORMAP_HSV)
        label_img_colorized[np.where(label_img_temp == 0)[0], np.where(label_img_temp == 0)[1], :] = 0
        draw_frame[np.where(label_img_colorized > 0)] = label_img_colorized[np.where(label_img_colorized > 0)]

    return draw_frame


def draw_line_segments(image, line_segments: List[LinearCluster], circle_diameter=2, edge_bold=1, with_end_point=False, with_fitted_line=False):
    n_segments = len(line_segments)
    draw_frame = image.copy()
    for label_m1, line_segment in enumerate(line_segments):
        """
        Draw line and points
        """
        points = line_segment.points()
        edges = line_segment.edges()
        label = label_m1 + 1
        label_img_temp = np.zeros((draw_frame.shape[0], draw_frame.shape[1]), dtype=np.uint8)
        for i, point in enumerate(points):
            cv2.circle(label_img_temp, (point[0], point[1]), circle_diameter, label, -1)
        for i, edge in enumerate(edges):
            p_from = points[edge[0]]
            p_to = points[edge[1]]
            cv2.line(label_img_temp, (p_from[0], p_from[1]), (p_to[0], p_to[1]), label, edge_bold)
        label_img_normalized = label_img_temp / n_segments
        label_img_colorized = cv2.applyColorMap((255.0 * label_img_normalized).astype(np.uint8), cv2.COLORMAP_HSV)
        label_img_colorized[np.where(label_img_temp == 0)[0], np.where(label_img_temp == 0)[1], :] = 0
        draw_frame[np.where(label_img_colorized > 0)] = label_img_colorized[np.where(label_img_colorized > 0)]

        """
        Draw end points
        """
        if with_end_point:
            end_points = [points[end_point_index] for end_point_index in line_segment.indices_end_points()]
            for point in end_points:
                cv2.circle(draw_frame, (point[0], point[1]), int(circle_diameter * 1.5), (0, 0, 255), -1)

        if with_fitted_line:
            nx, ny, nconst = line_segment.line()
            points_ary = np.asarray(points)
            dist_mat = np.sqrt(np.sum((points_ary[np.newaxis, :, :] - points_ary[:, np.newaxis, :]) ** 2, axis=-1))
            point_mean = points_ary.mean(0)
            half_length_segment = np.max(dist_mat) / 2
            line_direction = np.asarray([ny, -nx])
            p1 = point_mean - line_direction * half_length_segment
            p2 = point_mean + line_direction * half_length_segment
            cv2.line(draw_frame, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 0, 0), 2)
            cv2.line(draw_frame, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 0), 1)

    return draw_frame


def draw_mutual_cluster_connection(
    image_to_show, line_segments: List[LinearCluster], mutual_cluster_index_pairs: List[int], point_index_pairs_mutual_clusters: List[int]
):
    n_mutual_cluster_edge = 0
    for cluster_index_pair, point_index_pair in zip(mutual_cluster_index_pairs, point_index_pairs_mutual_clusters):
        cluster_a = line_segments[cluster_index_pair[0]]
        cluster_b = line_segments[cluster_index_pair[1]]

        p_a = np.asarray(cluster_a.points())[point_index_pair[0]]
        p_b = np.asarray(cluster_b.points())[point_index_pair[1]]
        # print(f"{p_a}, {p_b}")
        cv2.line(image_to_show, (p_a[0], p_a[1]), (p_b[0], p_b[1]), (0, 192, 200), 2)
        n_mutual_cluster_edge += 1
    print(f"n_mutual_cluster_edge: {int(n_mutual_cluster_edge)}")
    return image_to_show


def show_image(image, title="image", scale=1.0):
    while cv2.waitKey(10) & 0xFF not in [ord("q"), 27]:
        image_resize = cv2.resize(image, None, fx=scale, fy=scale)
        cv2.imshow(title, image_resize)
    cv2.destroyAllWindows()


def write_image(image, save_path, scale=1.0):
    image_resize = cv2.resize(image, None, fx=scale, fy=scale)
    cv2.imwrite(save_path, image_resize)
    cv2.waitKey(10)


"""
Skeleton to Graph convertion
"""
skeleton = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton.png", cv2.IMREAD_ANYDEPTH)
frame = SkeletonFrame(skeleton)
s2g = Skeleton2Graph(simplification_threshold=10, directional_threshold=30)

start = time.time()
s2g.set_frame(frame)
node_init = s2g.get_node_positions()
edge_init = s2g.get_edges()
s2g.simplify()
s2g.clustering()
node_labels_simplified = s2g.get_node_labels()
node_simplified = s2g.get_node_positions()
edge_simplified = s2g.get_edges()
end = time.time()
print(end - start)

print(f"num edge(init): {len(edge_init)}")
print(f"num edge(simplified): {len(edge_simplified)}")


"""
Output results
"""
image_to_show = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

"""
### Output raw skeleton
show_image(image_to_show, scale=2)
write_image(skeleton, f"{SCRIPT_DIR}/results/input.png", scale=2)

### Graph extraction result
#show_image(draw_graph(cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR), node_init, edge_init))
#show_image(
write_image(
    draw_graph(image_to_show, node_simplified, edge_simplified, node_labels_simplified, edge_bold=2),
    f"{SCRIPT_DIR}/results/graph.png",
    scale=2,
)
"""

line_segments = s2g.get_linear_clusters()
mutual_cluster_index_pairs: List[int] = s2g.get_mutual_cluster_index_pair()
point_index_pairs_mutual_clusters: List[int] = s2g.get_point_index_pair_mutual_clusters()

### Postprocessing result
image_to_show = draw_mutual_cluster_connection(image_to_show, line_segments, mutual_cluster_index_pairs, point_index_pairs_mutual_clusters)
image_to_show = draw_line_segments(image_to_show, line_segments, circle_diameter=3, edge_bold=3, with_end_point=True, with_fitted_line=True)

show_image(image_to_show, scale=2.0)
#write_image(image_to_show, f"{SCRIPT_DIR}/results/parsed.png", scale=2.0)
