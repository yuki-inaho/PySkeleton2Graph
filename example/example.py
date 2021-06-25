import cv2
import numpy as np
from pathlib import Path
import time
from pys2g import SkeletonFrame, Skeleton2Graph

SCRIPT_DIR = str(Path().parent)


def draw_graph(image, points, edges, label=None, circle_diameter=2, edge_bold=1):
    assert (len(image.shape) == 3) and (image.shape[-1] == 3)
    if label is None:
        draw_frame = image.copy()
        for point in points:
            cv2.circle(draw_frame, (point[0], point[1]), circle_diameter, (0, 0, 255), -1)
        for edge in edges:
            p_from = points[edge[0]]
            p_to = points[edge[1]]
            cv2.line(draw_frame, (p_from[0], p_from[1]), (p_to[0], p_to[1]), (255, 0, 0), edge_bold)
    else:
        draw_frame = image.copy()
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


def show_image(image, title="image", scale=1.0):
    while cv2.waitKey(10) & 0xFF not in [ord("q"), 27]:
        image_resize = cv2.resize(image, None, fx=scale, fy=scale)
        cv2.imshow(title, image_resize)
    cv2.destroyAllWindows()


skeleton = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton.png", cv2.IMREAD_ANYDEPTH)
frame = SkeletonFrame(skeleton)
s2g = Skeleton2Graph(simplification_threshold=5, directional_threshold=10)
start = time.time()
s2g.set_frame(frame)
node_init = s2g.get_node_positions()
edge_init = s2g.get_edges()
s2g.simplify()
s2g.compute_directional_connected_component()
s2g.merge_clusters()
node_labels_simplified = s2g.get_node_labels()
node_simplified = s2g.get_node_positions()
edge_simplified = s2g.get_edges()
end = time.time()
print(end - start)
print(f"num edge(init): {len(edge_init)}")
print(f"num edge(simplified): {len(edge_simplified)}")

#show_image(draw_graph(cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR), node_init, edge_init))
"""
show_image(
    draw_graph(
        cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR),
        node_simplified,
        edge_simplified
    ),
    scale=3.0,
)
"""

"""
show_image(
    draw_graph(
        cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR),
        node_simplified,
        edge_simplified,
        node_labels_simplified,
        edge_bold=2,
    ),
    scale=3.0,
)
"""