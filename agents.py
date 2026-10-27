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
    while state is not None:
        path.append(state)
        state = parents[state]
    path.reverse()
    return path


def bfs(campus, start, goal):
    frontier = deque([start])
    parents = {start: None}
    expanded = 0
    while frontier:
        state = frontier.popleft()
        if state == goal:
            return SearchResult(reconstruct_path(parents, goal), expanded)
        expanded += 1
        for action, successor in campus.neighbors(state):
            if successor not in parents:
                parents[successor] = state
                frontier.append(successor)
    return SearchResult(None, expanded)


def manhattan(state, goal):
    """Return the Manhattan distance between two (row, column) tuples."""
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])


def astar(campus, start, goal):
    """Find a shortest path with A* and report expanded nodes."""
    frontier = []
    tie = count()
    heapq.heappush(frontier, (manhattan(start, goal), next(tie), 0, start))
    best_cost = {start: 0}
    parents = {start: None}
    expanded = 0

    while frontier:
        _, _, cost, state = heapq.heappop(frontier)
        if cost != best_cost[state]:
            continue
        if state == goal:
            return SearchResult(reconstruct_path(parents, goal), expanded)

        expanded += 1
        for _, successor in campus.neighbors(state):
            new_cost = cost + 1
            if new_cost < best_cost.get(successor, float("inf")):
                best_cost[successor] = new_cost
                parents[successor] = state
                priority = new_cost + manhattan(successor, goal)
                heapq.heappush(frontier, (priority, next(tie), new_cost, successor))

    return SearchResult(None, expanded)


def epsilon_greedy(q_values, state, actions, epsilon, rng):
    """Choose a random action with probability epsilon, otherwise a best action."""
    if rng.random() < epsilon:
        return rng.choice(actions)

    best_value = max(q_values.get((state, action), 0.0) for action in actions)
    best_actions = [
        action for action in actions
        if q_values.get((state, action), 0.0) == best_value
    ]
    return rng.choice(best_actions)


def q_learning_update(q_values, state, action, reward, next_state,
                      actions, alpha, gamma, terminal):
    """Update the Q-value for a single state-action pair in place."""
    current = q_values.get((state, action), 0.0)
    if terminal:
        target = reward
    else:
        best_next = max(q_values.get((next_state, next_action), 0.0)
                        for next_action in actions)
        target = reward + gamma * best_next
    q_values[(state, action)] = current + alpha * (target - current)
