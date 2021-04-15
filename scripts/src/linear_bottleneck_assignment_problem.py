import numpy as np

a = np.array([[0.1, 1.0, 1.0],
              [-0.4, 2.0, -1.2],
              [0.4, 2.0, 1.2]])

has_one_positive = True
for column in a:
    print(column)
    has_one_positive = has_one_positive and (column > 0).any()

print(has_one_positive)



