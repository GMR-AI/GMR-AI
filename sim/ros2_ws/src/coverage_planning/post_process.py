import numpy as np
from PIL import Image, ImageDraw

class mask:
    def __init__(self, m, image_path):
        self.m = m
        img = Image.open(image_path).convert("L")  # Convert to grayscale
        img.save("grayscale.png")
        self.mask = np.array(img)
    
    def __call__(self, x, y):
        return self.mask[y, x]

    def process_points(self, P1,P2, padding):
        Pi1 = [int(P1[0]), int(P1[1])]
        Pi2 = [int(P2[0]), int(P2[1])]

        if self.mask[Pi1[0]][Pi1[1]] == 0 and self.mask[Pi2[0]][Pi2[1]] == 0:
            return Pi1, Pi2
        
        line_points = bresenham_line(Pi1[0], Pi1[1], Pi2[0], Pi2[1])
        for x, y in line_points:
            if Pi2[0] == x and Pi2[1] == y:
                if self.mask[x, y] == 0: # Edge case donde P2 es el unico hueco libre
                    return None, Pi2
                else:
                    return None, None
            
            if self.mask[y, x] == 0: # if Free is found then break
                Pi1[0] = x
                Pi1[1] = y
                break
        
        for x, y in line_points[::-1]:
            if Pi1[0] == x and Pi1[1] == y:
                return Pi1, None
                         
            if self.mask[y, x] == 0: # if Free is found then break
                Pi2[0] = x
                Pi2[1] = y
                break


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