"""Simple A* search implementation.

Run this file directly to see a small graph-search example:

    python astar_search.py
"""

from heapq import heappop, heappush
from math import inf


def reconstruct_path(came_from, current):
    """Build the path by walking backward from the goal."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))


def astar_search(graph, start, goal, heuristic):
    """Find the cheapest path from start to goal with A* search.

    Args:
        graph: Mapping of nodes to neighbor-cost mappings.
        start: Starting node.
        goal: Goal node.
        heuristic: Function that estimates cost from a node to the goal.

    Returns:
        A tuple of (path, cost). If no path exists, returns (None, inf).
    """
    open_set = []
    heappush(open_set, (heuristic(start, goal), 0, start))

    came_from = {}
    best_cost = {start: 0}
    closed = set()

    while open_set:
        _, current_cost, current = heappop(open_set)

        if current in closed:
            continue

        if current == goal:
            return reconstruct_path(came_from, current), current_cost

        closed.add(current)

        for neighbor, step_cost in graph.get(current, {}).items():
            new_cost = current_cost + step_cost
            if new_cost < best_cost.get(neighbor, inf):
                best_cost[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor, goal)
                heappush(open_set, (priority, new_cost, neighbor))

    return None, inf


def manhattan_distance(a, b):
    """Estimate distance between two grid points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


if __name__ == "__main__":
    graph = {
        (0, 0): {(1, 0): 1, (0, 1): 4},
        (1, 0): {(2, 0): 1, (1, 1): 2},
        (2, 0): {(2, 1): 1},
        (0, 1): {(1, 1): 1},
        (1, 1): {(2, 1): 1},
        (2, 1): {(2, 2): 1},
        (2, 2): {},
    }

    path, cost = astar_search(graph, (0, 0), (2, 2), manhattan_distance)
    print(f"Path: {path}")
    print(f"Cost: {cost}")
