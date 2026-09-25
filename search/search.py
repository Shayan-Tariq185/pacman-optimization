# search.py
# ---------

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).

=====================================================================
Merged file - combines both contributors' work into a single module:

Person 1 contribution:
    - depthFirstSearch, breadthFirstSearch, uniformCostSearch
    - _graphSearch: shared bookkeeping/frontier-management helper used by
      DFS, BFS, UCS, and also reused below by GBFS and A*.
    - CSVTraceLogger / _frontierStates / _reconstructPath: the automated
      CSV trace-logging framework (Section 3 of the assignment spec).

Person 2 contribution:
    - greedyBestFirstSearch, aStarSearch (now implemented on top of the
      shared _graphSearch helper, so they get the same strict graph-search
      behaviour and CSV trace logging as DFS/BFS/UCS, per Person 1's note).

Nothing needs to change in searchAgents.py for either half's code to work.
=====================================================================
"""

import csv
import os
import time

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


# =====================================================================
# CSV Trace Logging Framework (Section 3 of the assignment)
#
# Mandatory columns:
#   iteration, expanded_state, parent, action, generated_successors,
#   frontier_before, frontier_after, explored, g, h, f
#
# Every graph-search run (DFS/BFS/UCS/GBFS/A*) goes through this logger so
# every CSV in evidence/ has an identical, gradeable schema.
# =====================================================================

CSV_HEADERS = [
    "iteration", "expanded_state", "parent", "action", "generated_successors",
    "frontier_before", "frontier_after", "explored", "g", "h", "f",
]


def _evidenceDir():
    """evidence/ directory, created relative to the current working
    directory (i.e. wherever pacman.py is launched from)."""
    evidence_dir = os.path.join(os.getcwd(), "evidence")
    if not os.path.exists(evidence_dir):
        os.makedirs(evidence_dir)
    return evidence_dir


class CSVTraceLogger:
    """
    Buffers one row per expanded state and writes a single CSV file per
    search run into evidence/. Reusable by any of the 5 algorithms --
    algo_name just becomes (part of) the output filename.
    """

    def __init__(self, algo_name):
        timestamp = int(time.time() * 1000)
        filename = "%s_%d.csv" % (algo_name, timestamp)
        self.path = os.path.join(_evidenceDir(), filename)
        self.rows = []
        self.iteration = 0

    def log(self, expanded_state, parent, action, generated_successors,
            frontier_before, frontier_after, explored_set, g, h=0, f=None):
        self.iteration += 1
        if f is None:
            f = g + h
        self.rows.append([
            self.iteration,
            expanded_state,
            parent,
            action,
            list(generated_successors),
            list(frontier_before),
            list(frontier_after),
            len(explored_set),
            g,
            h,
            f,
        ])

    def write(self):
        with open(self.path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_HEADERS)
            writer.writerows(self.rows)
        return self.path


def _frontierStates(frontier):
    """Snapshot of the states currently sitting in a Stack/Queue/PriorityQueue,
    for the frontier_before / frontier_after CSV columns."""
    if isinstance(frontier, util.PriorityQueue):
        return [entry[2] for entry in frontier.heap]
    return list(frontier.list)


def _reconstructPath(parent_of, action_of, goal_state):
    """Walks the parent pointers built during the search back to the start
    state and returns the ordered list of actions."""
    actions = []
    state = goal_state
    while parent_of[state] is not None:
        actions.append(action_of[state])
        state = parent_of[state]
    actions.reverse()
    return actions


# =====================================================================
# Shared graph-search helper (bookkeeping used by DFS, BFS, UCS, GBFS, A*)
# =====================================================================

def _graphSearch(problem, frontier, algo_name, priority_fn=None, heuristic=None):
    """
    Generic graph-search driver.

      frontier:    an *empty* util.Stack() (-> DFS), util.Queue() (-> BFS),
                   or util.PriorityQueue() (-> UCS / GBFS / A*).
      priority_fn: only used when frontier is a PriorityQueue.
                   priority_fn(g, h) -> priority used to order the queue.
                     UCS   : lambda g, h: g
                     GBFS  : lambda g, h: h
                     A*    : lambda g, h: g + h
      heuristic:   optional heuristic(state, problem) -> number. Defaults
                   to 0 for uninformed search (DFS/BFS/UCS).

    Behaviour / requirements this satisfies for all five algorithms:
      - Strict GRAPH search: an explicit `explored` set stops states from
        being expanded twice, avoiding infinite loops (DFS requirement).
      - A state is only (re)placed on the frontier the first time it is
        discovered, or later if a strictly cheaper g(n) to it is found --
        so states already on the frontier/explored are never blindly
        re-enqueued (BFS requirement), and when a PriorityQueue is used,
        frontier.update(...) rewrites that state's priority in place
        (UCS/GBFS/A* "cheaper path -> update priority" requirement).
      - Every expansion is written to a CSVTraceLogger row with the
        mandatory columns from Section 3.

    Note on GBFS: since priority_fn(g, h) = h for GBFS, a "cheaper" g(n)
    to an already-frontiered state does not necessarily change its
    priority, but frontier.update() is still safe to call -- it just
    rewrites the entry with the same h-based priority.

    Returns the list of actions from start to goal (empty list if no
    solution).
    """
    logger = CSVTraceLogger(algo_name)

    start_state = problem.getStartState()
    is_priority_queue = isinstance(frontier, util.PriorityQueue)

    g_cost = {start_state: 0}
    parent_of = {start_state: None}
    action_of = {start_state: None}

    start_h = heuristic(start_state, problem) if heuristic else 0
    if is_priority_queue:
        frontier.push(start_state, priority_fn(0, start_h))
    else:
        frontier.push(start_state)

    explored = set()

    while not frontier.isEmpty():
        frontier_before = _frontierStates(frontier)
        state = frontier.pop()

        # Lazy deletion: a state can be sitting on the frontier more than
        # once (pushed at an earlier, worse g). Skip anything already
        # expanded -- this is what keeps the search a strict graph search.
        if state in explored:
            continue
        explored.add(state)

        g = g_cost[state]
        h = heuristic(state, problem) if heuristic else 0
        f = g + h
        parent = parent_of[state]
        action = action_of[state]
        generated_successors = []

        if problem.isGoalState(state):
            logger.log(state, parent, action, generated_successors,
                       frontier_before, _frontierStates(frontier),
                       explored, g, h, f)
            logger.write()
            return _reconstructPath(parent_of, action_of, state)

        # Successors are expanded in exactly the order getSuccessors()
        # returns them (searchAgents.py is responsible for returning them
        # North -> East -> South -> West per the assignment spec).
        for successor, step_action, step_cost in problem.getSuccessors(state):
            generated_successors.append(successor)

            if successor in explored:
                continue

            new_g = g + step_cost
            if successor not in g_cost or new_g < g_cost[successor]:
                g_cost[successor] = new_g
                parent_of[successor] = state
                action_of[successor] = step_action

                if is_priority_queue:
                    new_h = heuristic(successor, problem) if heuristic else 0
                    # PriorityQueue.update() finds the existing entry for
                    # this state (if any) and lowers its priority, or
                    # pushes it fresh if it wasn't on the frontier yet.
                    frontier.update(successor, priority_fn(new_g, new_h))
                else:
                    frontier.push(successor)

        logger.log(state, parent, action, generated_successors,
                   frontier_before, _frontierStates(frontier),
                   explored, g, h, f)

    # Frontier exhausted, no goal found.
    logger.write()
    return []


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Strict graph search: uses a LIFO Stack (util.Stack) as the frontier and
    an explicit explored set (inside _graphSearch) so no state is expanded
    twice. Logs every expansion to evidence/dfs_<timestamp>.csv.
    """
    return _graphSearch(problem, util.Stack(), "dfs")


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.

    Uses a FIFO Queue (util.Queue) as the frontier. Because every step
    costs 1, the first time BFS reaches a state is guaranteed to be via a
    shallowest/optimal path. States already discovered (on the frontier or
    already expanded) are never re-enqueued. Logs every expansion to
    evidence/bfs_<timestamp>.csv.
    """
    return _graphSearch(problem, util.Queue(), "bfs")


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.

    Uses a util.PriorityQueue ordered by accumulated path cost g(n). If a
    newly generated successor offers a cheaper g(n) to a state already on
    the frontier, frontier.update(...) rewrites its priority in place
    rather than leaving a stale duplicate entry. Logs every expansion to
    evidence/ucs_<timestamp>.csv.
    """
    return _graphSearch(problem, util.PriorityQueue(), "ucs",
                         priority_fn=lambda g, h: g)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def greedyBestFirstSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest heuristic value h(n) first.
    Priority is based ONLY on h(n) -- no path cost is considered.

    Built on the shared _graphSearch helper, so it gets the same strict
    graph-search (explored set) behaviour and CSV trace logging as
    DFS/BFS/UCS. Logs every expansion to evidence/gbfs_<timestamp>.csv.
    """
    return _graphSearch(problem, util.PriorityQueue(), "gbfs",
                         priority_fn=lambda g, h: h,
                         heuristic=heuristic)


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest f(n) = g(n) + h(n) first.
    g(n) is the actual path cost from start; h(n) is the heuristic estimate.

    Built on the shared _graphSearch helper: strict graph search with lazy
    deletion (an entry already expanded via a cheaper path is skipped) and
    goal testing at expansion (dequeue) time, not generation time. Logs
    every expansion to evidence/astar_<timestamp>.csv.
    """
    return _graphSearch(problem, util.PriorityQueue(), "astar",
                         priority_fn=lambda g, h: g + h,
                         heuristic=heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
gbfs = greedyBestFirstSearch