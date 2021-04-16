from FreeSpaceFinder2D import FreeSpaceFinder2D
import numpy as np
import pygame

# Xn Yn Wn Hn
free_spaces1 = [[-5.0, 5.0, 6.0, 6.0], [5.0, 5.0, 6.0, 6.0], [0.0, 0.0, 1.0, 1.0], [-5.0, -5.0, 6.0, 6.0],
                [5.0, -5.0, 6.0, 6.0]]
free_spaces2 = [[0.0, 0.0, 0.8, 0.8], [2.0, 0.0, 0.8, 0.8], [-2.0, 0.0, 0.8, 0.8], [0.0, 2.0, 0.8, 0.8],
                [0.0, -2.0, 0.8, 0.8]]
free_spaces3 = [[0.0, 0.0, 2.8, 1.8], [6.0, 0.0, 3.8, 2.8], [-6.0, 0.0, 1.8, 5.8], [0.0, 6.0, 4.8, 2.8],
                [0.0, -6.0, 2.8, 4.8]]

free_spaces0 = free_spaces3
radius0 = 0.6

space_finder = FreeSpaceFinder2D()
found_points = space_finder.find_optimal_points(free_spaces0, radius0)

# ==========
# ==========
# DEBUG BELOW
# ==========
# ==========

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

    for space in free_spaces0:
        space = space_multiplier * np.array(space)
        space[0] = size / 2 + space[0] - space[2] / 2
        space[1] = size / 2 + space[1] - space[3] / 2
        pygame.draw.rect(screen, (255, 0, 0), pygame.Rect(space))

    for circle in found_points:
        space = space_multiplier * np.array(circle)
        space[0] = size / 2 + space[0]
        space[1] = size / 2 + space[1]
        pygame.draw.circle(screen, (0, 255, 0), tuple(space), space_multiplier * radius0)

    pygame.display.flip()

# Done! Time to quit.
pygame.quit()
