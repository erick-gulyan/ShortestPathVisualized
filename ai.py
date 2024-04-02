from __future__ import print_function
from heapq import *

ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]

class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            #frontier stack
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            #frontier queue
            self.frontier = [self.grid.start]
            self.explored = []
            
            pass
        elif self.type == "ucs":
            #frontier priority queue (G, node[0], node[1])
            self.frontier = [(0 , self.grid.start[0], self.grid.start[1])]
            self.explored = []
            #Dictionary of distances
            self.distances = {self.grid.start: 0 }
            pass
        elif self.type == "astar":
            #frontier holds tuple of (G+H, node[0], node[1])
            self.frontier = [(0 + (abs(self.grid.start[0]-self.grid.goal[0]) + abs(self.grid.start[1] - self.grid.goal[1])), self.grid.start[0], self.grid.start[1])]
            self.explored = []
            #Dictionary of distances, G values only
            self.distances = {self.grid.start: 0}
            pass

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True #This turns the color of the node to red
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    #DFS: BUGGY, fix it first
    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        current = self.frontier.pop()
        self.explored.append(current)
        # print(visited)
        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        
        for n in children:
            
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.explored:
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.explored.append(n)
                    self.grid.nodes[n].color_frontier = True
                    if n == self.grid.goal:
                        self.finished = True
                        return

    #Implement BFS here (Don't forget to implement initialization at line 23)
    def bfs_step(self):
        #Nothing in frontier, no path exists
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop(0)
        self.explored.append(current)
        if current == self.grid.goal:
            self.finished = True
            return
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        for n in children:
            
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.explored:
                    self.previous[n] = current
                    self.frontier.append(n)
                    self.explored.append(n)
                    self.grid.nodes[n].color_frontier = True
                    if n == self.grid.goal:
                        self.finished = True
                        return

    #Implement UCS here (Don't forget to implement initialization at line 23)
    def ucs_step(self):
        #empty PQ, no path
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        current = heappop(self.frontier)
        currCost = current[0]
        current = (current[1], current[2])
        self.explored.append(current)
        if current == self.grid.goal:
            self.finished = True
            return
        
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored:
                        costToN = currCost + self.grid.nodes[n].cost()
                        self.previous[n] = current
                        self.distances[n] = costToN
                        newTuple = (self.distances[n], n[0], n[1])
                        heappush(self.frontier, newTuple)
                        self.explored.append(n)
                        self.grid.nodes[n].color_frontier = True
                        if n == self.grid.goal:
                            self.finished = True
                            return
                    else:
                        costToN = currCost + self.grid.nodes[n].cost()
                        if(costToN < self.distances[n]):
                            self.distances[n] = costToN
                            newTuple = (costToN, n[0], n[1])
                            heappush(self.frontier,newTuple)
                            self.grid.nodes[n].color_frontier = True
                            if n == self.grid.goal:
                                self.finished = True
                                return
    #Implement Astar here (Don't forget to implement initialization at line 23)
    def astar_step(self):
         #empty PQ, no path
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = heappop(self.frontier)
        currCost = current[0] - ((abs(current[1] - self.grid.goal[0]) + abs(current[2] - self.grid.goal[1])))
        current = (current[1], current[2])
        self.explored.append(current)
        if current == self.grid.goal:
            self.finished = True
            return
        
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored:
                        costToN = currCost + self.grid.nodes[n].cost() + (abs(n[0] - self.grid.goal[0]) + abs(n[1] - self.grid.goal[1]))
                        self.previous[n] = current
                        self.distances[n] = currCost + self.grid.nodes[n].cost()
                        newTuple = (costToN, n[0], n[1])
                        heappush(self.frontier, newTuple)
                        self.explored.append(n)
                        self.grid.nodes[n].color_frontier = True
                        if n == self.grid.goal:
                            self.finished = True
                            return
                    else:
                        costToN = currCost + self.grid.nodes[n].cost() + (abs(n[0] - self.grid.goal[0]) + abs(n[1] - self.grid.goal[1]))
                        if(costToN < self.distances[n] + ((abs(n[0] - self.grid.goal[0]) + abs(n[1] - self.grid.goal[1])))):
                            self.previous[n] = current
                            self.distances[n] = currCost + self.grid.nodes[n].cost()
                            newTuple = (costToN, n[0], n[1])
                            heappush(self.frontier,newTuple)
                            self.grid.nodes[n].color_frontier = True
                            if n == self.grid.goal:
                                self.finished = True
                                return
