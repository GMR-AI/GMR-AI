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

from zig_zag import planning

def plot_path_on_image(area_coords, path_coords, background_image_path, save_name):
    img = Image.open(background_image_path)
    draw = ImageDraw.Draw(img)

    # Draw the area boundary
    draw.polygon(list(zip(*area_coords)), outline="blue", fill=None, width=2)

    #draw.line(list(zip(*area_coords)), fill="blue", width=2)

    # Draw the sweep path
    draw.line(list(zip(*path_coords)), fill="red", width=2)

    img.save(save_name)

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


def coverage_planning(area_x, area_y, diameter, background_image_path, save_name="output.png"):
    img = Image.open(background_image_path)
    img_width, img_height = img.size
    img_width, img_height = img.size
    img_array, free_space, _ = load_image_mask(background_image_path)

    if check_within_bounds((area_x, area_y), img_width, img_height):
        rx, ry = planning(area_x, area_y, diameter, background_image_path)
        if not rx:
            return
        rx = [int(x) for x in rx]
        ry = [int(y) for y in ry]
        #rx, ry = post_process_path(rx, ry, free_space)
        plot_path_on_image((area_x, area_y), (rx, ry), background_image_path, save_name)
        print("Points are:")
        print("x: ", rx[:5])
        print("y: ", ry[:5])
    else:
        print("Error: Some coordinates in Test 2 are out of image bounds. Skipping this test.")

def main():
    print("start!!")
    background_image_path = "src/coverage_planning/image.png"
    diameter=10

    # Test 1: Coordinates that may be out of image bounds
    ox1 = [0, 200, 500, 1000, 1300, 400, 0]
    oy1 = [0, 200, 0, 300, 600, 800, 0]

    coverage_planning(ox1, oy1, diameter, background_image_path)

    # Test 2: Coordinates that fit within the 256x256 image bounds but has a vertical line
    ox2 = [44, 185, 178, 44]
    oy2 = [53, 57, 197, 196]

    coverage_planning(ox2, oy2, diameter, background_image_path, "test_square.png")

    # Test 3: Test 2 without a vertical line
    ox3 = [44, 185, 178, 43]
    oy3 = [53, 57, 197, 196]

    coverage_planning(ox3, oy3, diameter, background_image_path, "test_square.png")

    # Test 4: ROMBO
    ox4 = [120, 180, 120, 125]
    oy4 = [50, 125, 200, 45]

    coverage_planning(ox4, oy4, diameter, background_image_path, "test_rombo.png")

    print("done!!")

if __name__ == '__main__':
    main()
    #planning()

