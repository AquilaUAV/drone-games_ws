from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import maximum_bipartite_matching

graph = csr_matrix([[1, 1, 1], [1, 1, 0], [1, 0, 0]])

print(maximum_bipartite_matching(graph))
print((maximum_bipartite_matching(graph) != -1).all())
