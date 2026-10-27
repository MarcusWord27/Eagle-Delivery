from collections import deque
import heapq
from itertools import count


class SearchResult:
    def __init__(self, path, expanded):
        self.path = path
        self.expanded = expanded


def reconstruct_path(parents, goal):
    path = []
    state = goal
    # follow parent links backward until the route reaches its starting point.
    while state is not None:
        path.append(state)
        state = parents[state]
    # reverse the recovered route so it proceeds from start to destination.
    path.reverse()
    return path


def bfs(campus, start, goal):
    frontier = deque([start])
    parents = {start: None}
    expanded = 0
    # breadth-first search expands states in arrival order to preserve shortest paths.
    while frontier:
        state = frontier.popleft()
        # stop when the destination is reached; the recorded parents define the route.
        if state == goal:
            return SearchResult(reconstruct_path(parents, goal), expanded)
        # count states whose neighbors are explored, excluding the destination.
        expanded += 1
        # expand only traversable neighboring cells; walls are excluded by the map.
        for action, successor in campus.neighbors(state):
            # prevent revisiting states already discovered by breadth-first search.
            if successor not in parents:
                parents[successor] = state
                frontier.append(successor)
    return SearchResult(None, expanded)


def manhattan(state, goal):
    """TODO 1: Return Manhattan distance between two (row, column) tuples."""
    # add the absolute row difference and the absolute column difference.
    raise NotImplementedError("Complete TODO 1: manhattan")


def astar(campus, start, goal):
    """TODO 2: Return SearchResult(path, expanded) for A* graph search."""
    # use heapq as a priority queue and manhattan() as the heuristic.
    # track best known g-costs and parent states in dictionaries.
    raise NotImplementedError("Complete TODO 2: astar")


def epsilon_greedy(q_values, state, actions, epsilon, rng):
    """Choose a random action with probability epsilon, otherwise a best action."""
    # explore with probability epsilon instead of following the current best policy.
    if rng.random() < epsilon:
        return rng.choice(actions)

    # compare every action; unseen actions have an implicit value of zero.
    best_value = max(q_values.get((state, action), 0.0) for action in actions)
    # retain all tied best actions to avoid favoring a fixed action order.
    best_actions = [
        action for action in actions
        if q_values.get((state, action), 0.0) == best_value
    ]
    # break exploitation ties randomly, including ties among unseen actions.
    return rng.choice(best_actions)


def q_learning_update(q_values, state, action, reward, next_state,
                      actions, alpha, gamma, terminal):
    """TODO 4: Update exactly one Q-table entry in place; return nothing."""
    # get q(state, action), defaulting to 0.0.
    # if terminal is true, the target is reward with no continuation term.
    # otherwise target = reward + gamma * max q(next_state, next_action).
    # consider every action, including unseen entries with value 0.0.
    # apply q <- q + alpha * (target - q).
    raise NotImplementedError("Complete TODO 4: q_learning_update")
