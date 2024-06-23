from functions import *
from PIL import Image, ImageDraw
import IPython.display as display

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

## Parameters
background_image_path = "./src/coverage_planning/image.png"
ox1 = [44, 185, 178, 43]
oy1 = [53, 57, 197, 196]
diameter=5

## Return value
solution=[]

draw, img = gen_image(background_image_path, ox1, oy1)

LTop, LRight, LBot, LLeft = get_area_lines(ox1, oy1)
Po1, Po2, Po3, Po4 = get_points(ox1, oy1)

## Obtener puntos dentro de las lineas para conseguir rectas generadoras
Pgen1=get_point_given_distance(diameter, LTop, Po1, Po4)
gen1=generate_parallel_line(LLeft, Pgen1)

Pgen2=get_point_given_distance(diameter, LTop, Po2, Po3)
gen2=generate_parallel_line(LRight, Pgen2)

# Gen first two points (from top)
P1=intersection_point(gen1, LTop)
P2=intersection_point(gen2, LTop)

# TODO: tratar edge case donde Ltop este muy cerca de LBot en algun lado (por parte de Pi1 o Pi2), mas cerca que el diametro

EndGen=True # Este valor representa en que linea generadora nos hemos quedado al acabar la generacion quadrada True si esta en 1 i False si esta en 2

# Generacion quadrada
while True:

    # P3 a partir de la recta paralela a LLeft
    P1=get_point_given_distance(diameter, gen1, P1, Po4)
    solution.append(tuple(P1))
    if distance_point_to_line(P1, LBot) < diameter:
        break

    # Fem el mateix per a P4
    P2=get_point_given_distance(diameter, gen2, P2, Po3)
    solution.append(tuple(P2))

    if distance_point_to_line(P2, LBot) < diameter:
        EndGen=False
        break


draw.line(solution, fill="red", width=2)
#display.display(img)

rx = [s[0] for s in solution]
ry = [s[1] for s in solution]

print("rx: ", rx)
print("ry: ", ry)
