#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  7 15:54:31 2020

@author: ulrikbd
"""

from Map import *
import numpy as np


class Node():
    """Search node"""

    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent
        self.kids = []

        self.g = 0
        self.h = 0
        self.f = 0

    """Operator for checking if two nodes are the same"""
    def __eq__(self, other):
        if other == None: #Needed for the backtracking
            return False

        return self.pos == other.pos

    """Print out information about a node, useful for debugging"""
    def print_info(self):
        print("Pos:", self.pos, "f:", self.f, "g:", self.g, "h:", self.h)


def A_star(map_obj, moveable_goal=False):
    """Implementation of the A_star search algorithm"""

   #Initializing empty closed and open list of nodes
    closed_nodes = []
    open_nodes = []

    #Initializing the start node n0
    n0 = Node(pos = map_obj.get_start_pos())
    n0.h = manhattan_distance(n0.pos, map_obj.get_goal_pos())
    n0.f = n0.g + n0.h

    open_nodes.append(n0)

    #Beginning agenda loop
    while len(open_nodes) > 0:
        #Choose the new node X and push it to the closed node list
        X = open_nodes.pop(0)
        closed_nodes.append(X)

        if moveable_goal:
            map_obj.tick()

        #Check if we have reached the goal
        if X.pos == map_obj.get_goal_pos():
            return backtrack_path(X)

        #Generate successors and add them as kids
        successors = get_successors(X.pos, map_obj)
        for S in successors:
            X.kids.append(S)

        for C in X.kids:
            #Check if C has been explored before
            if C not in open_nodes and C not in closed_nodes:
                if moveable_goal:
                    attach_and_eval(C, X, map_obj, moveable_goal_heuristic)
                else:
                    attach_and_eval(C, X, map_obj)
                open_nodes.append(C)
                open_nodes.sort(key=lambda n: n.f)

            #Check if X is a better parent for C
            elif X.g + 1 < C.g:
                if moveable_goal:
                    attach_and_eval(C, X, map_obj, moveable_goal_heuristic)
                else:
                    attach_and_eval(C, X, map_obj)

                if C in closed_nodes:
                    propagate_path_improvements(C)


def manhattan_distance(pos, goal):
    """Calculates the manhatten distance between the position and the goal"""
    return np.abs(goal[0] - pos[0]) + np.abs(goal[1] - pos[1])


def moveable_goal_heuristic(pos, goal):
    """Heuristic function used in task 5 with a moveable goal"""
    return np.abs(goal[0] - pos[0])  + np.abs(goal[1] - pos[1]) * 2 #Multiplying the x-distance by a factor 2 to adjust for moving target


def get_successors(pos, map_obj):
    """Returns a list of the surrounding possible nodes"""
    successors = []
    #iterate through adjecent squares
    for step in [(0, -1), (0, 1), (-1, 0), (1, 0)]: #, (-1, -1), (-1, 1), (1, -1), (1, 1)]:
        new_pos = [pos[0] + step[0], pos[1] + step[1]]

        #Check if it is a legal step
        if map_obj.get_cell_value(new_pos) != -1:
            new_node = Node(new_pos)
            successors.append(new_node)

    return successors


def arc_cost(node, map_obj):
    """Calculates the cost of taking the step"""
    return map_obj.get_cell_value(node.pos)


def attach_and_eval(C, P, map_obj, heuristic_function=manhattan_distance):
    C.parent = P
    C.g = P.g + arc_cost(C, map_obj)
    C.h = heuristic_function(C.pos, map_obj.get_goal_pos())
    C.f = C.g + C.h


def propagate_path_improvements(P, map_obj):
    for C in P.kids:
        if P.g + arc_cost(C, map_obj) < C.g:
            C.parent = P
            C.g = P.g + arc_cost(C, map_obj)
            C.f = C.g + C.h
            propagate_path_improvements(C)


def backtrack_path(N):
    """Backtracking through the parents to find the best route from the starting position"""
    seq = [N.pos]

    while N.parent != None:
        N = N.parent
        seq.append(N.pos)

    return seq[::-1]


def get_map_with_path(path_to_goal, map_obj):
    """Returns the string map where the path to goal is highlighted"""
    int_map, str_map = map_obj.get_maps()
    for i in range(1, len(path_to_goal) - 1):
        str_map[path_to_goal[i][0], path_to_goal[i][1]] = 'P'

    return str_map

def show_map_with_path(path_to_goal, map_obj):
    """Shows map where the path to goal is highlighted"""
    int_map, str_map = map_obj.get_maps()
    for i in range(1, len(path_to_goal) - 1):
        str_map[path_to_goal[i][0], path_to_goal[i][1]] = 'P'

    map_obj.show_map(map=str_map)


def main():
    """Main function where all tasks are excecuted and the images presented"""

    task = int(input("Task: ")) #Get task from user
    map_obj = Map_Obj(task=task)

    if task == 5:
        path_to_goal = A_star(map_obj, moveable_goal=True)
    else:
        path_to_goal = A_star(map_obj)

    show_map_with_path(path_to_goal, map_obj)


if __name__ == '__main__':
    main()

