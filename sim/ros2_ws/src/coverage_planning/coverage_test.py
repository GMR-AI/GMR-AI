import math
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from PIL import Image, ImageDraw
from scipy.spatial import KDTree

from grid_map_lib import GridMap, FloatGrid

import numpy as np

import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString

def generate_zigzag_points(vertices, radius):
    def interpolate(p1, p2, t):
        return (p1[0] + (p2[0] - p1[0]) * t, p1[1] + (p2[1] - p1[1]) * t)

    # Create the polygon from vertices
    poly = Polygon(vertices)
    
    # Determine the bounding box
    minx, miny, maxx, maxy = poly.bounds
    
    # Generate lines
    lines = []
    y = miny
    while y <= maxy:
        lines.append(LineString([(minx, y), (maxx, y)]))
        y += radius
    
    # Clip lines to the polygon and extract points
    points = []
    for line in lines:
        clipped_line = poly.intersection(line)
        if isinstance(clipped_line, LineString):
            coords = list(clipped_line.coords)
            points.extend(coords)
        elif isinstance(clipped_line, MultiLineString):
            for subline in clipped_line:
                points.extend(list(subline.coords))
    
    # Arrange points in zigzag order
    ordered_points = []
    toggle = False
    for i in range(0, len(points), 2):
        if i + 1 < len(points):
            if toggle:
                ordered_points.append(points[i + 1])
                ordered_points.append(points[i])
            else:
                ordered_points.append(points[i])
                ordered_points.append(points[i + 1])
            toggle = not toggle
    
    return ordered_points


def planning(ax, ay, res):
    # Example usage
    vertices = [(x, y) for x, y in zip(ax, ay)]
    radius = res

    points = generate_zigzag_points(vertices, radius)
    print(points)
    return [p[0] for p in points], [p[1] for p in points]




def plot_path_on_image(area_coords, path_coords, background_image_path):
    img = Image.open(background_image_path)
    draw = ImageDraw.Draw(img)

    # Draw the area boundary
    draw.line(list(zip(*area_coords)), fill="blue", width=2)

    # Draw the sweep path
    draw.line(list(zip(*path_coords)), fill="red", width=2)

    img.save("output_image.png")

def check_within_bounds(coords, img_width, img_height):
    for x, y in zip(*coords):
        if not (0 <= x < img_width and 0 <= y < img_height):
            return False
    return True

def load_image_mask(image_path):
    img = Image.open(image_path).convert("L")  # Convert to grayscale
    img_array = np.array(img)
    free_space = np.where(img_array == 0)  # Black pixels
    obstacles = np.where(img_array != 0)  # Non-black pixels (grey)
    return img_array, free_space, obstacles

def find_nearest_free_space(point, free_space):
    tree = KDTree(np.c_[free_space[0], free_space[1]])
    _, idx = tree.query(point)
    return free_space[0][idx], free_space[1][idx]

def post_process_path(path_x, path_y, free_space):
    adjusted_x, adjusted_y = [], []
    for x, y in zip(path_x, path_y):
        if (x, y) not in zip(free_space[0], free_space[1]):
            nx, ny = find_nearest_free_space((x, y), free_space)
            adjusted_x.append(nx)
            adjusted_y.append(ny)
        else:
            adjusted_x.append(x)
            adjusted_y.append(y)
    return adjusted_x, adjusted_y


def coverage_planning(area_x, area_y, background_image_path, res):
    img = Image.open(background_image_path)
    img_width, img_height = img.size
    img_width, img_height = img.size
    img_array, free_space, _ = load_image_mask(background_image_path)

    if check_within_bounds((area_x, area_y), img_width, img_height):
        rx, ry = planning(area_x, area_y, res)
        rx = [int(x) for x in rx]
        ry = [int(y) for y in ry]
        #rx, ry = post_process_path(rx, ry, free_space)
        plot_path_on_image((area_x, area_y), (rx, ry), background_image_path)
        print("Points are:")
        print("x: ", rx[:5])
        print("y: ", ry[:5])
    else:
        print("Error: Some coordinates in Test 2 are out of image bounds. Skipping this test.")

def main():
    print("start!!")
    background_image_path = "image.png"


    # Test 1: Coordinates that may be out of image bounds
    ox1 = [0, 200, 500, 1000, 1300, 400, 0]
    oy1 = [0, 200, 0, 300, 600, 800, 0]

    coverage_planning(ox1, oy1, background_image_path, 5.0)

    # Test 2: Coordinates that fit within the 256x256 image bounds
    ox2 = [44, 185, 178, 44]
    oy2 = [53, 57, 197, 196]

    coverage_planning(ox2, oy2, background_image_path, 5.0)

    # Test 3: ROMBO
    ox2 = [120, 180, 120, 125]
    oy2 = [50, 125, 200, 45]

    coverage_planning(ox2, oy2, background_image_path, 5.0)

    print("done!!")

if __name__ == '__main__':
    main()
    #planning()

