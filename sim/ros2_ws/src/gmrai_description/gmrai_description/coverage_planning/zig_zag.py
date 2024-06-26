from gmrai_description.coverage_planning.functions import *
from gmrai_description.coverage_planning.post_process import Mask

from PIL import Image, ImageDraw
import IPython.display as display

def reorder_points(xs, ys):
    points = list(zip(xs, ys))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    
    points = sorted(points)
    
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    
    convex_hull = lower[:-1] + upper[:-1]
    
    if len(convex_hull) != 4:
        raise ValueError("The points do not form a convex quadrilateral.")
    
    return [c[0] for c in convex_hull], [c[1] for c in convex_hull]

def get_area_lines(ox, oy):
    ## Obtener lineas de la area
    Ltop = get_line_from_two_points([ox[0],oy[0]], [ox[1], oy[1]])
    LRight = get_line_from_two_points([ox[1],oy[1]], [ox[2], oy[2]])
    LBot = get_line_from_two_points([ox[2],oy[2]], [ox[3], oy[3]])
    LLeft = get_line_from_two_points([ox[3],oy[3]], [ox[0], oy[0]])
    return (Ltop, LRight, LBot, LLeft) if Ltop and LRight and LBot and LLeft else (None, None, None, None)

def get_points(ox, oy):
    return [ox[0],oy[0]], [ox[1],oy[1]], [ox[2],oy[2]], [ox[3],oy[3]]

def gen_image(background_image_path, ox, oy):
    area_coords= list(zip(ox, oy))
    img = Image.open(background_image_path)

    img = Image.open(background_image_path)
    draw = ImageDraw.Draw(img)

    # Draw the area boundary
    draw.polygon(area_coords, outline="blue", fill=None, width=2)

    display.display(img)
    return draw, img

def planning(ox1, oy1, diameter, image):

    # Pre-process
    ox1, oy1 = reorder_points(ox1, oy1)

    solution=[]
    (LTop, LRight, LBot, LLeft) = get_area_lines(ox1, oy1)
    if not LTop:
        return None, None
    Po1, Po2, Po3, Po4 = get_points(ox1, oy1)
    m = Mask(LTop.m, image)

    ## Obtener puntos dentro de las lineas para conseguir rectas generadoras
    Pgen1=get_point_given_distance(diameter, LTop, Po1, Po2)
    gen1=generate_parallel_line(LLeft, Pgen1)

    Pgen2=get_point_given_distance(diameter, LTop, Po2, Po1)
    gen2=generate_parallel_line(LRight, Pgen2)

    # Gen first two points (from top)
    P1=intersection_point(gen1, LTop)
    P2=intersection_point(gen2, LTop)

    switch=False

    # Generacion quadrada
    while True:

        # P3 a partir de la recta paralela a LLeft
        P1=get_point_given_distance(diameter, gen1, P1, Po4)
        if distance_point_to_line(P1, LBot) < diameter:
            Pi1 = [int(P1[0]), int(P1[1])]
            if m(Pi1[0], Pi1[1]) == 0:
                solution.append(tuple(Pi1))
            break

        # Fem el mateix per a P4
        P2=get_point_given_distance(diameter, gen2, P2, Po3)

        P1_aux, P2_aux = m.process_points(P1.copy(), P2.copy(), diameter)
        if P1_aux:
            solution.append(tuple(P1_aux))
        if P2_aux:
            solution.append(tuple(P2_aux))

        if P1_aux and P2_aux and switch:
            solution[-1], solution[-2] = solution[-2], solution[-1]

        if distance_point_to_line(P2, LBot) < diameter:
            break
        
        switch = not switch

    rx = [s[0] for s in solution]
    ry = [s[1] for s in solution]

    return rx, ry, ox1, oy1