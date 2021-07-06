import cv2
from pathlib import Path
from s2g import Skeleton2GraphHelper
from utils import draw_mutual_cluster_connection, draw_line_segments, draw_graph, show_image, write_image


SCRIPT_DIR = str(Path().parent)


"""
Skeleton to Graph convertion
"""
skeleton_image = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton.png", cv2.IMREAD_ANYDEPTH)
s2g = Skeleton2GraphHelper(simplification_threshold=15.0, directional_threshold=30.0)
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
# show_image(image_to_show, scale=2)
write_image(skeleton_image, f"{SCRIPT_DIR}/results/input.png", scale=2)

### Output graph extraction result
show_image(draw_graph(image_to_show, node_init, edge_init), scale=2)
show_image(draw_graph(image_to_show, node_simplified, edge_simplified), scale=2)
"""
write_image(
    draw_graph(image_to_show, node_simplified, edge_simplified, node_labels_simplified, edge_bold=2),
    f"{SCRIPT_DIR}/results/graph.png",
    scale=2,
)
"""

### Output post-processing result
image_to_show = draw_mutual_cluster_connection(image_to_show, linear_clusters, mutual_cluster_index_pairs, point_index_pairs_mutual_clusters)
image_to_show = draw_line_segments(image_to_show, linear_clusters, circle_diameter=3, edge_bold=3, with_end_point=True, with_fitted_line=True)
show_image(image_to_show, scale=2.0)
# write_image(image_to_show, f"{SCRIPT_DIR}/results/parsed.png", scale=2.0)
