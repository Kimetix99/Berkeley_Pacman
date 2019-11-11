class Node:

    def __init__(self,parent,action,path_cost,state):
        self.parent = parent
        self.action = action
        self.path_cost = 0
        if self.parent != None:
            self.path_cost = path_cost + self.parent.path_cost
        self.state = state

    def path(self):
        path = []
        curr = self
        while curr.parent!=None:
            path.append(curr.action)
            curr = curr.parent
        return list(reversed(path))

    def equals(self,node):
        return self.state==node.state
