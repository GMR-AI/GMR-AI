import math

class Line:
    def __init__(self, m, n):
        self.m = m
        self.n = n

    def __call__(self, x):
        return self.m * x + self.n

def distance_point_to_line(point, line):
    """
    Calculate the shortest distance between a point and a line.

    :param point: Tuple (x, y) representing the point.
    :param line: Line class instance representing the line.
    :return: Shortest distance between the point and the line.
    """
    x0, y0 = point
    m, n = line.m, line.n

    distance = abs(m * x0 - y0 + n) / math.sqrt(m**2 + 1)
    return distance


def get_point_given_distance(distance, line, origin, reference_point):
    """
    Calculate the point at a given distance along a line from the origin point,
    and return the point closest to the given reference point.
    
    :param distance: Distance to travel along the line.
    :param line: Line class instance representing the line.
    :param origin: Tuple (x, y) representing the starting point.
    :param reference_point: Tuple (x, y) representing the point to which the closest point is determined.
    :return: Tuple (x, y) representing the point closest to the reference point.
    """
    # Calculate the two possible points at the given distance
    dx = distance / math.sqrt(1 + line.m ** 2)
    dy = line.m * dx

    point1 = [origin[0] + dx, origin[1] + dy]
    point2 = [origin[0] - dx, origin[1] - dy]

    dist1 = math.sqrt((point1[0] - reference_point[0]) ** 2 + (point1[1] - reference_point[1]) ** 2)
    dist2 = math.sqrt((point2[0] - reference_point[0]) ** 2 + (point2[1] - reference_point[1]) ** 2)

    # Return the point that is closer to the reference point
    if dist1 < dist2:
        return point1
    else:
        return point2
    

def get_line_from_two_points(p1, p2):
    if p1[0] == p2[0]:  # Vertical line, move one line a little
        p2[0]+=1
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    n = p1[1] - m * p1[0]
    return Line(m, n)


def generate_parallel_line(line, point):
    """
    Generate a line parallel to the given line that passes through the given point.

    :param line: Line class instance representing the original line.
    :param point: Tuple (x, y) representing the point through which the new line passes.
    :return: Line class instance representing the new parallel line.
    """
    m = line.m
    x, y = point
    n = y - m * x
    return Line(m, n)

def intersection_point(line1, line2):
    """
    Calculate the intersection point between two lines.

    :param line1: First line (Line class instance).
    :param line2: Second line (Line class instance).
    :return: Tuple (x, y) representing the intersection point.
    """
    m1, n1 = line1.m, line1.n
    m2, n2 = line2.m, line2.n
    
    if m1 == m2:
        raise ValueError("The lines are parallel and do not intersect.")
    
    x = (n2 - n1) / (m1 - m2)
    y = m1 * x + n1
    
    return [x, y]

def angle_between_lines(line1, line2):
    """
    Calculate the angle between two lines.
    
    :param line1: First line (Line class instance).
    :param line2: Second line (Line class instance).
    :return: Angle in degrees between the two lines.
    """
    m1, m2 = line1.m, line2.m
    
    if (1 + m1 * m2) == 0:
        return 90.0
    
    angle_rad = math.atan(abs((m1 - m2) / (1 + m1 * m2)))
    angle_deg = math.degrees(angle_rad)
    
    return angle_deg