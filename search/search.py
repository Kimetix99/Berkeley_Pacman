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

import math
import util
from Node import *
import sys
import numpy as np

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
    fringe = util.PriorityQueue()
    fringe.push(n,n.path_cost)
    generated = { n.state : [n,'F'] }
    while True:
        if fringe.isEmpty():
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop()
        if generated[n.state][1] != 'E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state] = [n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns = Node(n, action, cost, state)
                if ns.state not in generated: 
                    fringe.push(ns,ns.path_cost)
                    generated[ns.state]=[ns, 'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and ns.path_cost < generated[ns.state][0].path_cost:
                    fringe.push(ns,ns.path_cost)
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

def moovingFoodDirection(position, problem, info={}):  
    xy1 = position
    xy2 = problem.goal
    closerNorth=False
    closerSouth=False
    closerEast=False
    closerWest=False
    for state, action, cost in problem.getSuccessors(xy1): 
        if action=='North' and state[0]+state[1] <= xy1[0]+xy1[1]:
            closerNorth=True
        if action=='South' and state[0]+state[1] <= xy1[0]+xy1[1]:
            closerSouth=True
        if action=='East' and state[0]+state[1] <= xy1[0]+xy1[1]:
            closerEast=True
        if action=='West' and state[0]+state[1] <= xy1[0]+xy1[1]:
            closerWest=True

    if closerNorth or closerSouth or closerEast or closerWest:
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
    else:
        return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])+1

def greedyBestFirstSearch(problem, heuristic=nullHeuristic):
    n=Node(None,None,0,problem.getStartState())
    fringe = util.PriorityQueue()
    fringe.push(n,heuristic(n.state, problem))
    generated = { n.state : [n, 'F'] }
    while True:
        if fringe.isEmpty():
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop()
        if generated[n.state][1]!='E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state]=[n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns=Node(n,action,cost,state)
                if ns.state not in generated:
                    fringe.push(ns,heuristic(n.state, problem))
                    generated[ns.state]=[ns,'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and heuristic(ns.state, problem) < heuristic(generated[ns.state][0].state, problem):
                    fringe.push(ns,heuristic(ns.state, problem))
                    generated[ns.state] = [ns, 'F']

def aStarSearch(problem, heuristic=nullHeuristic):
    n=Node(None,None,0,problem.getStartState())
    fringe = util.PriorityQueue()
    fringe.push(n,heuristic(n.state, problem))
    generated = { n.state : [n, 'F'] }
    while True:
        if fringe.isEmpty():
            print "No s'ha trobat solucio"
            sys.exit()
        n = fringe.pop()
        if generated[n.state][1]!='E':
            if problem.isGoalState(n.state):
                return n.path()
            generated[n.state]=[n, 'E']
            for state, action, cost in problem.getSuccessors(n.state):
                ns=Node(n,action,cost,state)
                if ns.state not in generated:
                    fringe.push(ns,heuristic(n.state, problem)+ns.path_cost)
                    generated[ns.state]=[ns,'F']
                elif ns.state in generated and generated[ns.state][1] == 'F' and (ns.path_cost+heuristic(ns.state, problem)) < (generated[ns.state][0].path_cost+heuristic(generated[ns.state][0].state,problem)):
                    fringe.push(ns,heuristic(ns.state, problem)+ns.path_cost)
                    generated[ns.state] = [ns, 'F']

def bidirectionalSearch(problem):
    n1=Node(None,None,0,problem.getStartState())
    n2=Node(None,None,0,problem.goal)
    fringe1=[n1]
    fringe2=[n2]
    generated1 = { n1.state : [n1,'F'] }
    generated2 = { n2.state : [n2,'F'] }
    while True:
        if len(fringe1) == 0 or len(fringe2) == 0:
            print "No s'ha trobat solucio"
            sys.exit()
        for i in fringe1:
            for j in fringe2:
                if i.state==j.state:
                    fromGoalList=np.asarray(j.path())
                    fromGoalList=reversed(fromGoalList)
                    return i.path()+list(fromGoalList)
        n1 = fringe1.pop(0)
        n2 = fringe2.pop(0)
        generated1[n1.state] = [n1, 'E']
        generated2[n2.state] = [n2, 'E']
        for state, action, cost in problem.getSuccessors(n1.state):
            ns = Node(n1, action, cost, state)
            if ns.state not in generated1: 
                fringe1.append(ns)
                generated1[ns.state]=[ns, 'F']
        for state, action, cost in problem.getSuccessors(n2.state):
            ns = Node(n2, anthagonicAction(action), cost, state)
            if ns.state not in generated2: 
                fringe2.append(ns)
                generated2[ns.state]=[ns, 'F']

def anthagonicAction(action):
    if action=='North':
        return 'South'
    if action=='East':
        return 'West'
    if action=='West':
        return 'East'
    if action=='South':
        return 'North'

# Abbreviations
ucs = uniformCostSearch
bfsh = greedyBestFirstSearch
astar = aStarSearch
bds = bidirectionalSearch
mandH = manhattanHeuristic
eucdH = euclideanHeuristic
custH = moovingFoodDirection
