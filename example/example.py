from os import write
import cv2
import numpy as np
from pathlib import Path
from s2g import Skeleton2GraphHelper
from utils import SkeletonGraphPainter, LinearClusterPainter, show_image, write_image

# from skimage.morphology import thin, skeletonize

SCRIPT_DIR = str(Path().parent)


"""
Skeleton to Graph convertion
"""
scale = 2.0
skeleton_image = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton_test.png", cv2.IMREAD_ANYDEPTH)
skeleton_image = cv2.resize(skeleton_image, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST)
image_height, image_width = skeleton_image.shape

N = 1000
y = (np.random.rand(N) * image_height).astype(int)
x = (np.random.rand(N) * image_width).astype(int)
skeleton_image[y, x] = 255

s2g = Skeleton2GraphHelper(simplification_threshold=30.0, directional_threshold=90.0)
s2g.set_frame(skeleton_image)
s2g.initialize_graph_state()
node_init, edge_init = s2g.initial_graph_elements

s2g.analyze_graph()
node_simplified, edge_simplified = s2g.simplified_graph_elements
node_labels_simplified = s2g.node_labels
mutual_cluster_index_pairs, point_index_pairs_mutual_clusters = s2g.cluster_connection_information
linear_clusters = s2g.linear_clusters

print(f"num edge(init): {len(edge_init)}")
print(f"num edge(simplified): {len(edge_simplified)}")

"""
Output results
"""
image_to_show = cv2.cvtColor(skeleton_image, cv2.COLOR_GRAY2BGR)

### Output raw skeleton
graph_painter = SkeletonGraphPainter(point_diameter=2, edge_thickness=2)

show_image(graph_painter(skeleton_image, s2g, draw_simplified=False), scale=2)
# write_image(graph_painter(skeleton_image, s2g, draw_simplified=False), f"{SCRIPT_DIR}/results/input.png", scale=scale)

### Output graph extraction result
show_image(graph_painter(skeleton_image, s2g, draw_simplified=True), scale=scale)
# write_image(graph_painter(skeleton_image, s2g, draw_simplified=True), f"{SCRIPT_DIR}/results/graph.png", scale=scale)

show_image(graph_painter(s2g.skeleton_simplified, s2g, draw_simplified=True), scale=scale)
### Output post-processing result
cluster_painter = LinearClusterPainter(point_diameter=2, edge_thickness=2)
# show_image(cluster_painter(skeleton_image, linear_clusters, s2g, draw_fitted_line=True, draw_mutual_cluster_connections=True), scale=scale)
"""
write_image(
    cluster_painter(skeleton_image, linear_clusters, s2g, draw_fitted_line=True, draw_mutual_cluster_connections=True),
    f"{SCRIPT_DIR}/results/parsed.png",
    scale=scale,
)
"""

### rescale check
"""
image_to_show = cv2.resize(cv2.cvtColor(skeleton_image, cv2.COLOR_GRAY2BGR), None, fx=scale, fy=scale)
image_height_scale = int(image_height * scale)
image_width_scale = int(image_width * scale)
[linear_cluster.rescale(image_width_scale, image_height_scale) for linear_cluster in linear_clusters]
cluster_painter = LinearClusterPainter(point_diameter=4, edge_thickness=3)
# show_image(cluster_painter(image_to_show, linear_clusters, s2g, draw_fitted_line=True, draw_mutual_cluster_connections=True), scale=1.0)
"""

"""
write_image(
    cluster_painter(image_to_show, linear_clusters, s2g, draw_fitted_line=True, draw_mutual_cluster_connections=True),
    f"{SCRIPT_DIR}/results/parsed.png",
    scale=1.0,
)
"""
