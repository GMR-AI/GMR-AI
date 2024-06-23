class Line:
    def __init__(self, m, n):
        self.m = m
        self.n = n

    def __call__(self, x):
        return self.m * x + self.n

def get_line_from_two_points(p1, p2):
    if p1[0] == p2[0]:  # Vertical line
        raise ValueError("Vertical line - slope is undefined.")
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    n = p1[1] - m * p1[0]
    return Line(m, n)

def is_point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False

    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

import math

def get_point_given_distance(distance, line, origin, area):
    # Calculate the two possible points at the given distance
    dx = distance / math.sqrt(1 + line.m ** 2)
    dy = line.m * dx

    point1 = (origin[0] + dx, origin[1] + dy)
    point2 = (origin[0] - dx, origin[1] - dy)

    if is_point_in_polygon(point1, area):
        return point1
    if is_point_in_polygon(point2, area):
        return point2

    return None  # This should not happen if distance is correct and within bounds

import numpy as np

def generate_zigzag_coverage(area, diameter):
    p1, p2, p3, p4 = area
    width = np.linalg.norm(np.array(p2) - np.array(p1))
    height = np.linalg.norm(np.array(p4) - np.array(p1))
    num_lines = int(height // diameter) + 1

    coverage_points = []
    direction = 1  # 1 for right, -1 for left

    for i in range(num_lines):
        y = p1[1] + i * diameter
        if direction == 1:
            start = (p1[0], y)
            end = (p1[0] + width, y)
        else:
            start = (p1[0] + width, y)
            end = (p1[0], y)

        coverage_points.append(start)
        coverage_points.append(end)
        direction *= -1

    return coverage_points

# Example usage
area = [(0, 0), (10, 0), (10, 5), (0, 5)]
diameter = 1.0
zigzag_points = generate_zigzag_coverage(area, diameter)
print(zigzag_points)
