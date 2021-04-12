import numpy as np
from numpy import linalg

# Xn Yn Wn Hn
free_spaces = [[-2.0, 0.0, 2.0, 6.0], [2.0, -2.5, 2.0, 1.5], [2.0, 0.0, 2.0, 1.5], [2.0, 2.5, 2.0, 1.5]]
# X Y
previous = [0.0, 0.0]
maximum_free_space = max([min(*spaces[2:]) for spaces in free_spaces])
maximum_spaces = [spaces for spaces in free_spaces if min(*spaces[2:]) == maximum_free_space]
optimal_lines = [[spaces[0], spaces[1], spaces[2] - maximum_free_space, spaces[3] - maximum_free_space] for spaces in
                 maximum_spaces]

good_vectors = []
for line in optimal_lines:
    if line[2] == 0:
        # Вертикальная
        down_y = line[1] - line[3] / 2
        up_y = line[1] + line[3] / 2
        both_x = line[0]
        down_vector = [both_x - previous[0], down_y - previous[1]]
        up_vector = [both_x - previous[0], up_y - previous[1]]
        if linalg.norm(down_vector) < linalg.norm(up_vector):
            good_vector = down_vector
        else:
            good_vector = up_vector
        if down_y <= previous[1] <= up_y and both_x < linalg.norm(good_vector):
            good_vector = [both_x, 0.0]
        good_vectors.append(good_vector)
    elif line[3] == 0:
        # Горизонтальная
        left_x = line[0] - line[2] / 2
        right_x = line[0] + line[2] / 2
        both_y = line[1]
        left_vector = [left_x - previous[0], both_y - previous[1]]
        right_vector = [right_x - previous[0], both_y - previous[1]]
        if linalg.norm(left_vector) < linalg.norm(right_vector):
            good_vector = left_vector
        else:
            good_vector = right_vector
        if left_x <= previous[0] <= right_x and both_y < linalg.norm(good_vector):
            good_vector = [0.0, both_y]
        good_vectors.append(good_vector)
min_vector_len = min([linalg.norm(np.array(vector)) for vector in good_vectors])
best_vectors = [vector for vector in good_vectors if linalg.norm(np.array(vector)) == min_vector_len]
best_vector = best_vectors[0]
print(best_vector)
