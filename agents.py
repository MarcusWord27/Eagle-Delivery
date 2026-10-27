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
    """TODO 3: Choose an action with epsilon-greedy exploration."""
    # q_values is a dictionary. its keys are (state, action) pairs.
    # example key: ((1, 1), "right"). unseen entries have value 0.0.
    raise NotImplementedError("Complete TODO 3: epsilon_greedy")


def q_learning_update(q_values, state, action, reward, next_state,
                      actions, alpha, gamma, terminal):
    """Update the Q-value for a single state-action pair in place."""
    current = q_values.get((state, action), 0.0)
    # terminal transitions have no future reward to add to the immediate reward.
    if terminal:
        target = reward
    else:
        # bootstrap from the highest next-state value, including unseen actions.
        best_next = max(q_values.get((next_state, next_action), 0.0)
                        for next_action in actions)
        # discount the best expected continuation before updating the current estimate.
        target = reward + gamma * best_next
    # move only this state-action estimate toward the target by the learning rate.
    q_values[(state, action)] = current + alpha * (target - current)
