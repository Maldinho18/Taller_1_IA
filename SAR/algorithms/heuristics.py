from typing import Any, Tuple
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    """
    # TODO: Add your code here
    x, y = state
    gx, gy = problem.goal
    return abs(x - gx) + abs(y - gy)


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    """
    # TODO: Add your code here
    x, y = state
    gx, gy = problem.goal
    dx = x - gx
    dy = y - gy
    return (dx * dx + dy * dy) ** 0.5


def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    position, survivors_grid = state
    survivors = survivors_grid.asList(True)
    
    if not survivors:
        return 0
    
    dist_cache = problem.heuristicInfo.setdefault("pairDist", {})
    
    def manhattan(a, b):
        llave = (a, b) if a <= b else (b, a)
        if llave not in dist_cache:
            dist_cache[llave] = abs(a[0] - b[0]) + abs(a[1] - b[1])
        return dist_cache[llave]
    
    cercano = min(manhattan(position, s) for s in survivors)
    
    def costo_mst(puntos):
        if len(puntos) <= 1:
            return 0
        visitados = set()
        inicio = puntos[0]
        visitados.add(inicio)
        total = 0
        mejor = {p: manhattan(inicio, p) for p in puntos[1:]}
        while len(visitados) < len(puntos):
            nxt, c= min(mejor.items(), key=lambda kv: kv[1])
            total += c
            visitados.add(nxt)
            del mejor[nxt]
            for p in list (mejor.keys()):
                mejor[p] = min(mejor[p], manhattan(nxt, p))
        return total
    mst_cache = problem.heuristicInfo.setdefault("mst", {})
    survivors_llave = tuple(sorted(survivors))
    if survivors_llave not in mst_cache:
        mst_cache[survivors_llave] = costo_mst(list(survivors_llave))
    return cercano + mst_cache[survivors_llave]