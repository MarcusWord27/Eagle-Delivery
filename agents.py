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
    """Return the Manhattan distance between two (row, column) tuples."""
    # estimate remaining grid steps without accounting for walls or detours.
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])


def astar(campus, start, goal):
    """Find a shortest path with A* and report expanded nodes."""
    frontier = []
    tie = count()
    # prioritize estimated total route cost; the counter breaks equal-priority ties.
    heapq.heappush(frontier, (manhattan(start, goal), next(tie), 0, start))
    best_cost = {start: 0}
    parents = {start: None}
    expanded = 0

    # repeatedly examine the most promising available search state.
    while frontier:
        _, _, cost, state = heapq.heappop(frontier)
        # discard obsolete queue entries when a cheaper route was found afterward.
        if cost != best_cost[state]:
            continue
        # stop when the destination is reached; the recorded parents define the route.
        if state == goal:
            return SearchResult(reconstruct_path(parents, goal), expanded)

        # count states whose neighbors are explored, excluding the destination.
        expanded += 1
        for _, successor in campus.neighbors(state):
            new_cost = cost + 1
            # only improve a route when its actual traveled cost is lower.
            if new_cost < best_cost.get(successor, float("inf")):
                best_cost[successor] = new_cost
                parents[successor] = state
                # combine distance traveled and estimated distance remaining for a*.
                priority = new_cost + manhattan(successor, goal)
                heapq.heappush(frontier, (priority, next(tie), new_cost, successor))

    # report failure after exhausting every reachable candidate.
    return SearchResult(None, expanded)


def epsilon_greedy(q_values, state, actions, epsilon, rng):
    """TODO 3: Choose an action with epsilon-greedy exploration."""
    # q_values is a dictionary. its keys are (state, action) pairs.
    # example key: ((1, 1), "right"). unseen entries have value 0.0.
    raise NotImplementedError("Complete TODO 3: epsilon_greedy")


def q_learning_update(q_values, state, action, reward, next_state,
                      actions, alpha, gamma, terminal):
    """TODO 4: Update exactly one Q-table entry in place; return nothing."""
    # get q(state, action), defaulting to 0.0.
    # if terminal is true, the target is reward with no continuation term.
    # otherwise target = reward + gamma * max q(next_state, next_action).
    # consider every action, including unseen entries with value 0.0.
    # apply q <- q + alpha * (target - q).
    raise NotImplementedError("Complete TODO 4: q_learning_update")
