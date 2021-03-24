import numpy as np
from queue import Queue

def bfs(matrix, start, end_cond, not_neighbour_cond):
    row = len(matrix)
    col = len(matrix[0])
    visited = np.empty((row, col))
    for i in range(row):
      for j in range(col):
        visited[i][j] = 0
    parent = np.empty((row, col, 2))
    for i in range(row):
      for j in range(col):
        for k in range(2):
          parent[i][j][k] = -1
    queue = Queue(row * col)
    end = None

    def valid(node):
        return node[0] >= 0 and node[0] < row and node[1] >= 0 and node[1] < col
    
    def getNeighbours(node):
        neighbours = []
        #top_left = (node[0]-1, node[1]-1)
        #top = (node[0]-1, node[1])
        #top_right = (node[0]-1, node[1]+1)
        #left = (node[0], node[1]-1)
        #right = (node[0], node[1]+1)
        #botton_left = (node[0]+1, node[1]-1)
        #botton = (node[0]+1, node[1])
        #bototm_right = (node[0]+1, node[1]+1)
        for i in range(-1, 2):
            for j in range(-1, 2):
                test_node = [node[0]+i, node[1]+j]
                if test_node != node and valid(test_node) and matrix[test_node[0]][test_node[1]] != not_neighbour_cond and visited[test_node[0]][test_node[1]] == 0:
                    neighbours.append(test_node)
        return neighbours


    def backtrack():
        path = []
        curr = [end[0], end[1]]
        while curr[0]!= -2.0 and curr[1] != -2.0:
            path.append(curr)
            par = [parent[int(curr[0])][int(curr[1])][0], parent[int(curr[0])][int(curr[1])][1]]
            curr = par
        return path
    
    if start[0] < 0 or start[1] < 0 or start[0] >= row or start[1] >= col:
        return []

    visited[start[0]][start[1]] = 1
    parent[start[0]][start[1]] = [-2, -2]
    queue.put(start)

    while not queue.empty():
        curr = queue.get()
      
        if matrix[curr[0]][curr[1]] == end_cond:
            end = curr
            break

        neighbours = getNeighbours(curr)
        for i in range(len(neighbours)):
            visited[neighbours[i][0]][neighbours[i][1]] = 1
            parent[neighbours[i][0]][neighbours[i][1]] = curr
            queue.put(neighbours[i])

    if end != None:
        return backtrack()
    else:
        return []
def vvs:
    newpath = [path[0]]
    for elem in path[1:len(path)]:
        for g in getNeighbours(matrix[elem[0]][elem[1]]):
            if int(round(matrix[g[0]][g[1]])) == 3:
                counter += 1
                continue
            else:
                continue
        if counter >= 4:
            newpath.append(elem)
        newpath.append(path[-1])
     return newpath
