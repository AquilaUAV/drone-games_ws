import numpy as np
from numpy import linalg
from math import *

# Радиус
radius = 0.5
# Xn Yn Wn Hn
free_spaces1 = [[-5.0, 5.0, 6.0, 6.0], [5.0, 5.0, 6.0, 6.0], [0.0, 0.0, 1.0, 1.0], [-5.0, -5.0, 6.0, 6.0],
                [5.0, -5.0, 6.0, 6.0]]
free_spaces2 = [[0.0, 0.0, 0.8, 0.8], [2.0, 0.0, 0.8, 0.8], [-2.0, 0.0, 0.8, 0.8], [0.0, 2.0, 0.8, 0.8],
                [0.0, -2.0, 0.8, 0.8]]
free_spaces3 = [[0.0, 0.0, 2.8, 1.8], [6.0, 0.0, 3.8, 2.8], [-6.0, 0.0, 1.8, 5.8], [0.0, 6.0, 4.8, 2.8],
                [0.0, -6.0, 2.8, 4.8]]

free_spaces = free_spaces3

maximum_free_space = max([min(*spaces[2:]) for spaces in free_spaces])
if maximum_free_space < radius * 2:
    radius = maximum_free_space / 2
print(radius)


def try_sqared(free_spaces):
    circles = []
    for space in free_spaces:
        if min(space[2:]) < 2 * radius:
            continue
        x_capacity = int(space[2] / (2 * radius))
        y_capacity = int(space[3] / (2 * radius))
        x_start = radius
        y_start = radius
        x_delta = 2 * radius
        y_delta = 2 * radius
        if x_capacity != 1:
            x_delta += (space[2] - x_capacity * 2 * radius) / (x_capacity - 1)
        else:
            x_start += (space[2] - x_capacity * 2 * radius) / 2
        if y_capacity != 1:
            y_delta += (space[3] - y_capacity * 2 * radius) / (y_capacity - 1)
        else:
            y_start += (space[3] - y_capacity * 2 * radius) / 2
        for x_num in range(x_capacity):
            for y_num in range(y_capacity):
                circles.append([x_start + x_num * x_delta + space[0] - space[2] / 2,
                                y_start + y_num * y_delta + space[1] - space[3] / 2])
    return circles


found_points = try_sqared(free_spaces)

# ==========
# ==========
# ==========
# DEBUG BELOW
import pygame

pygame.init()
space_multiplier = 50
size = 1000
screen = pygame.display.set_mode([size, size])

running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))

    for space in free_spaces:
        space = space_multiplier * np.array(space)
        space[0] = size / 2 + space[0] - space[2] / 2
        space[1] = size / 2 + space[1] - space[3] / 2
        pygame.draw.rect(screen, (255, 0, 0), pygame.Rect(space))

    for circle in found_points:
        space = space_multiplier * np.array(circle)
        space[0] = size / 2 + space[0]
        space[1] = size / 2 + space[1]
        pygame.draw.circle(screen, (0, 255, 0), tuple(space), space_multiplier * radius)

    pygame.display.flip()

# Done! Time to quit.
pygame.quit()
