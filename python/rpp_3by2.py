from pathlib import Path
import sys
import numpy as np

path = Path(__file__)
path_lib = path.parent.parent.joinpath('lib')
path_lib_abs = str(path_lib.resolve())
sys.path.insert(1, path_lib_abs)

import py_lclibrary

dt = np.dtype('int32')
vertex_ids = np.array([5, 1, 2, 4], dt)


edges_tail_ids = np.array([5, 1, 2], dt)
edges_head_ids = np.array([2, 4, 1], dt)
edges_req = np.array([True, True, False], np.dtype(bool))
edges_costs = np.array([10, 12.5, 8])


route_edges = py_lclibrary.SolveRPP(vertex_ids, edges_tail_ids, edges_head_ids, edges_req, edges_costs)
print(type(route_edges))
print(route_edges.size)
print(route_edges.dtype)
print(route_edges)
