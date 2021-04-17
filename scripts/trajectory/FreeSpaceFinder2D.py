import numpy as np
from math import *
from random import choice

class FreeSpaceFinder2D(object):

    def __single_try_sqared(self, space, radius):
        circles = []
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

    def __single_try_hexed_horizontal(self, space, radius):
        circles = []
        x_line_capacity = int(space[2] / (2 * radius))
        if x_line_capacity == 1:
            return circles
        x_delta = 2 * radius + (space[2] - x_line_capacity * 2 * radius) / (x_line_capacity - 1)
        x_start = radius
        y_height = 2 * radius * sin(acos(x_delta / (4 * radius)))
        y_capacity = int((space[3] - 2 * radius) / y_height) + 1
        if y_capacity == 1:
            return circles
        y_delta = y_height + (space[3] - 2 * radius - (y_capacity - 1) * y_height) / (y_capacity - 1)
        y_start = radius
        for x_num in range(x_line_capacity):
            for y_num in range(y_capacity):
                if y_num % 2 == 0:
                    circles.append([x_start + x_num * x_delta + space[0] - space[2] / 2,
                                    y_start + y_num * y_delta + space[1] - space[3] / 2])
                else:
                    if x_num >= x_line_capacity - 1:
                        continue
                    circles.append([x_start + (x_delta / 2) + x_num * x_delta + space[0] - space[2] / 2,
                                    y_start + y_num * y_delta + space[1] - space[3] / 2])
        return circles

    def __single_try_hexed_vertical(self, space, radius):
        circles = []
        y_line_capacity = int(space[3] / (2 * radius))
        if y_line_capacity == 1:
            return circles
        y_delta = 2 * radius + (space[3] - y_line_capacity * 2 * radius) / (y_line_capacity - 1)
        y_start = radius
        x_height = 2 * radius * sin(acos(y_delta / (4 * radius)))
        x_capacity = int((space[2] - 2 * radius) / x_height) + 1
        if x_capacity == 1:
            return circles
        x_delta = x_height + (space[2] - 2 * radius - (x_capacity - 1) * x_height) / (x_capacity - 1)
        x_start = radius
        for y_num in range(y_line_capacity):
            for x_num in range(x_capacity):
                if x_num % 2 == 0:
                    circles.append([x_start + x_num * x_delta + space[0] - space[2] / 2,
                                    y_start + y_num * y_delta + space[1] - space[3] / 2])
                else:
                    if y_num >= y_line_capacity - 1:
                        continue
                    circles.append([x_start + x_num * x_delta + space[0] - space[2] / 2,
                                    y_start + (y_delta / 2) + y_num * y_delta + space[1] - space[3] / 2])
        return circles

    def find_optimal_points(self, free_spaces, radius, num=-1):
        circles = []
        circles_num = []
        maximum_free_space = max([min(*spaces[2:]) for spaces in free_spaces])
        if maximum_free_space < radius * 2:
            radius = maximum_free_space / 2
        for space in free_spaces:
            if min(space[2:]) < 2 * radius:
                continue
            new_circles = []
            for test_circles in [self.__single_try_sqared(space, radius),
                                 self.__single_try_hexed_horizontal(space, radius),
                                 self.__single_try_hexed_vertical(space, radius)]:
                if len(test_circles) > len(new_circles):
                    new_circles = test_circles
            circles.append(new_circles)
        for space in circles:
            circles_num.extend(space)
        if num != -1:
            if len(circles_num) > num:
                circles_num = []
                circles_used = {}
                for i in range(len(circles)):
                    circles_used[i] = {}
                space_num = -1
                inspace_num = 0
                while num > 0:
                    space_num += 1
                    if space_num >= len(circles):
                        space_num %= len(circles)
                        inspace_num += 1
                    if inspace_num > len(circles[space_num]) - 1:
                        continue
                    new_key_candidats = set([i for i in range(len(circles[space_num]))])
                    new_key_candidats = list(new_key_candidats.difference(set(circles_used[space_num].keys())))
                    new_key = choice(new_key_candidats)
                    circles_num.append(circles[space_num][new_key])
                    circles_used[space_num][new_key] = True
                    num -= 1
        return circles_num
