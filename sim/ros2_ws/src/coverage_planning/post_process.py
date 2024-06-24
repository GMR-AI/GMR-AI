import numpy as np
from PIL import Image

class Mask:
    def __init__(self, m, image_path):
        self.m = m
        img = Image.open(image_path).convert("L")  # Convert to grayscale
        self.mask = np.array(img)
    
    def __call__(self, x, y):
        if 0 <= x < self.mask.shape[1] and 0 <= y < self.mask.shape[0]:
            return self.mask[y, x]
        return 255  # Assume white (obstacle) for out-of-bounds

    def process_points(self, P1, P2, padding):
        Pi1 = [int(P1[0]), int(P1[1])]
        Pi2 = [int(P2[0]), int(P2[1])]

        if self.__call__(Pi1[0], Pi1[1]) == 0 and self.__call__(Pi2[0], Pi2[1]) == 0:
            return Pi1, Pi2
        
        line_points = bresenham_line(Pi1[0], Pi1[1], Pi2[0], Pi2[1])
        
        def is_free_with_padding(x, y, padding):
            for dx in range(-padding, padding + 1):
                for dy in range(-padding, padding + 1):
                    if 0 <= x + dx < self.mask.shape[1] and 0 <= y + dy < self.mask.shape[0]:
                        if self.__call__(x + dx, y + dy) != 0:
                            return False
            return True

        for x, y in line_points:
            if self.__call__(x, y) == 0 and is_free_with_padding(x, y, padding):
                Pi1[0], Pi1[1] = x, y
                break

        for x, y in reversed(line_points):
            if self.__call__(x, y) == 0 and is_free_with_padding(x, y, padding):
                Pi2[0], Pi2[1] = x, y
                break
        
        if self.__call__(Pi1[0], Pi1[1]) != 0 or self.__call__(Pi2[0], Pi2[1]) != 0:
            return None, None

        return Pi1, Pi2



def bresenham_line(x0, y0, x1, y1):
    """
    Bresenham's Line Algorithm to compute the discrete values of a line given two points.
    
    :param x0: Start x-coordinate
    :param y0: Start y-coordinate
    :param x1: End x-coordinate
    :param y1: End y-coordinate
    :return: List of (x, y) tuples representing the line pixels
    """
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points