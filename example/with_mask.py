import cv2
import numpy as np
from pathlib import Path
from s2g import Skeleton2GraphHelper
from utils import draw_mutual_cluster_connection, draw_line_segments, draw_graph, show_image, write_image
from decorator import timeit

SCRIPT_DIR = str(Path().parent)


@timeit
def distance_transform(mask_image):
    return cv2.distanceTransform(mask_image, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def colorize_distance_image(distance_image):
    distance_image_normed = cv2.normalize(distance_image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F) * 255
    dt_image_uc = distance_image_normed.astype(np.uint8)
    return cv2.applyColorMap(dt_image_uc, cv2.COLORMAP_PARULA)


"""
Skeleton to Graph convertion
"""
skeleton_image = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton.png", cv2.IMREAD_ANYDEPTH)
mask_image = cv2.imread(f"{SCRIPT_DIR}/example/data/mask.png", cv2.IMREAD_ANYDEPTH)
print("<distance tranform process (for mask combined process)>")
distance_image = distance_transform(mask_image)

print("\n <skeleton2graph process>")
s2g = Skeleton2GraphHelper(simplification_threshold=15.0, directional_threshold=30.0)
s2g.set_frame(skeleton_image)
s2g.initialize_graph_state()
node_init, edge_init = s2g.initial_graph_elements

s2g.analyze_graph()
node_list_simplified, edge_list_simplified = s2g.simplified_graph_elements
node_labels_simplified = s2g.node_labels
mutual_cluster_index_pairs, point_index_pairs_mutual_clusters = s2g.cluster_connection_information
linear_clusters = s2g.linear_clusters

skeleton_viz_image = cv2.cvtColor(skeleton_image, cv2.COLOR_GRAY2BGR)
mask_viz_image = np.zeros_like(skeleton_viz_image)
nonzero_elem = np.where(mask_image > 0)
mask_viz_image[nonzero_elem[0], nonzero_elem[1], :] = (255, 0, 0)
skel_plus_mask_viz = cv2.addWeighted(skeleton_viz_image, 0.7, mask_viz_image, 0.3, 1.0)
for node in node_list_simplified:
    cv2.circle(skel_plus_mask_viz, node, int(distance_image[node[1], node[0]]), (0, 0, 255), thickness=1, lineType=cv2.LINE_AA)

show_image(skel_plus_mask_viz, scale=2.0)
show_image(colorize_distance_image(distance_image), scale=2.0)

"""
write_image(cv2.cvtColor(mask_image, cv2.COLOR_GRAY2BGR), "results/mask_resized.png", scale=2.0)
write_image(skel_plus_mask_viz, "results/skeleton_with_mask.png", scale=2.0)
write_image(colorize_distance_image(distance_image), "results/distance_transform.png", scale=2.0)
"""
