# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from Node import *
import sys

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
    return  [s, s, w, s, w, w, s, w]

def nullHeuristic(state, problem=None):   
    return 0

def uniformCostSearch(problem):
    n=Node(None,None,0,problem.getStartState())
    fringe=[n]
    generated = { n.state : [n,'F'] }
    while True:
        if len(fringe) == 0:
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop(0)
        if generated[n.state][1] != 'E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state] = [n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns = Node(n, action, cost, state)
                if ns.state not in generated: 
                    fringe.append(ns)
                    generated[ns.state]=[ns, 'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and ns.path_cost < generated[ns.state][0].path_cost:
                    fringe.insert(0,ns)
                    generated[ns.state] = [ns, 'F']


def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

def greedyBestFirstSearch(problem, heuristic=nullHeuristic):
    n=Node(None,None,0,problem.getStartState())
    fringe=[n]
    generated = { n.state : [n, 'F'] }
    while True:
        if len(fringe)==0:
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop(0)
        if generated[n.state][1]!='E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state]=[n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns=Node(n,action,cost,state)
                if ns.state not in generated:
                    fringe.append(ns)
                    generated[ns.state]=[ns,'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and heuristic(ns.state, problem) < heuristic(generated[ns.state][0].state,problem):
                    fringe.insert(0,ns)
                    generated[ns.state] = [ns, 'F']

def aStarSearch(problem, heuristic=nullHeuristic):
    n=Node(None,None,0,problem.getStartState())
    fringe=[n]
    generated = { n.state : [n, 'F'] }
    while True:
        if len(fringe)==0:
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop(0)
        if generated[n.state][1]!='E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state]=[n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns=Node(n,action,cost,state)
                if ns.state not in generated:
                    fringe.append(ns)
                    generated[ns.state]=[ns,'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and (ns.path_cost+heuristic(ns.state, problem)) < (generated[ns.state][0].path_cost+heuristic(generated[ns.state][0].state,problem)):
                    fringe.insert(0,ns)
                    generated[ns.state] = [ns, 'F']

def bidirectionalSearch(problem):
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviations
ucs = uniformCostSearch
bfsh = greedyBestFirstSearch
astar = aStarSearch
bds = bidirectionalSearch
mandH = manhattanHeuristic
eucdH = euclideanHeuristic
