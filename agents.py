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
