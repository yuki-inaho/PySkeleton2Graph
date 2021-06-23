import cv2
import numpy as np
from pathlib import Path
import time
from pys2g import SkeletonFrame, Skeleton2Graph

SCRIPT_DIR = str(Path().parent)


def draw_graph(image, points, edges, circle_diameter=2, edge_bold=1):
    assert (len(image.shape) == 3) and (image.shape[-1] == 3)
    draw_frame = image.copy()
    for point in points:
        cv2.circle(draw_frame, (point[0], point[1]), circle_diameter, (0, 0, 255), -1)
    for edge in edges:
        p_from = points[edge[0]]
        p_to = points[edge[1]]
        cv2.line(draw_frame, (p_from[0], p_from[1]), (p_to[0], p_to[1]), (255, 0, 0), edge_bold)
    return draw_frame


def show_image(image, title="image", scale=1.0):
    while cv2.waitKey(10) & 0xFF not in [ord("q"), 27]:
        image_resize = cv2.resize(image, None, fx=scale, fy=scale)
        cv2.imshow(title, image_resize)
    cv2.destroyAllWindows()


skeleton = cv2.imread(f"{SCRIPT_DIR}/example/data/skeleton.png", cv2.IMREAD_ANYDEPTH)
frame = SkeletonFrame(skeleton)
s2g = Skeleton2Graph(simplification_threshold=10)
start = time.time()
s2g.set_frame(frame)
end = time.time()
print(end - start)
node_init = s2g.get_node_positions()
edge_init = s2g.get_edges()
#show_image(draw_graph(cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR), node_init, edge_init))