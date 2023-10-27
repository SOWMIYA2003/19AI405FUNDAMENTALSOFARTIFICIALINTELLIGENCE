# 19AI405 FUNDAMENTALS OF ARTIFICIALINTELLIGENCE 
# Laboratory Experiments

# ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph
## Name : Sowmiya N
## Register No : 212221230106

 

## Aim:
To Implement Depth First Search Traversal of a Graph using Python 3.

## Theory:
Depth First Traversal (or DFS) for a graph is like Depth First Traversal of a tree. The only catch here is that, unlike trees, graphs may contain cycles (a node may be visited twice). Use a Boolean visited array to avoid processing a node more than once. A graph can have more than one DFS traversal. Depth-first search is an algorithm for traversing or searching trees or graph data structures. The algorithm starts at the root node (selecting some arbitrary node as the root node in the case of a graph) and explores as far as possible along each branch before backtracking. Step 1: Initially, stack and visited arrays are empty. 

![277147664-640b3c6f-3ac1-49a2-a955-68da9a71f446](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/57367246-ee54-4319-b5fc-cc7b786b17cf)

Queue and visited arrays are empty initially. Stack and visited arrays are empty initially. Step 2: Visit 0 and put its adjacent nodes which are not visited yet into the stack. 

![277147583-86dcf7d9-1f9d-49b0-a821-5976a6e77606](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/cb8c6a82-6677-4ad4-87eb-04572122bb0c)

Visit node 0 and put its adjacent nodes (1, 2, 3) into the stack Visit node 0 and put its adjacent nodes (1, 2, 3) into the stack

Step 3: Now, Node 1 at the top of the stack, so visit node 1 and pop it from the stack and put all of its adjacent nodes which are not visited in the stack. 

![277147597-e6017942-08b1-4742-87ad-c97eb97bf985](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/70a9094b-66db-480f-8951-f865c9820745)

Visit node 1 Visit node 1

Step 4: Now, Node 2 at the top of the stack, so visit node 2 and pop it from the stack and put all of its adjacent nodes which are not visited (i.e, 3, 4) in the stack. 

![277147603-6e6d123c-60ae-4f9c-a27c-c4fc7e57d57c](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/04007a30-b42d-4c02-9bc7-a288add398ae)

Visit node 2 and put its unvisited adjacent nodes (3, 4) into the stack Visit node 2 and put its unvisited adjacent nodes (3, 4) into the stack

Step 5: Now, Node 4 at the top of the stack, so visit node 4 and pop it from the stack and put all of its adjacent nodes which are not visited in the stack. 

![277147620-20b76a05-5668-4da5-8189-e10fb1bb7238](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/a5135d93-8018-482b-a353-2bc7d771dcc2)

Visit node 4 Visit node 4

Step 6: Now, Node 3 at the top of the stack, so visit node 3 and pop it from the stack and put all of its adjacent nodes which are not visited in the stack. 
![277147623-3b88f04a-7846-4f75-89b4-22bbd5b48e52](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/3ab75fe2-ead7-4d08-8034-b173a538d2d0)

Visit node 3 Visit node 3

Now, the Stack becomes empty, which means we have visited all the nodes, and our DFS traversal ends.
## Algorithm:

1. Construct a Graph with Nodes and Edges
2. Depth First Search Uses Stack and Recursion
3. Insert a START node to the STACK
4. Find its Successors Or neighbors and Check whether the node is visited or not
5. If Not Visited, add it to the STACK. Else Call The Function Again Until No more nodes needs to be visited.
## Program : 
```
#import defaultdict
from collections import defaultdict
def dfs(graph,start,visited,path):
    path.append(start)
    visited[start]=True
    for neighbour in graph[start]:
        if visited[neighbour]==False:
            dfs(graph,neighbour,visited,path)
            visited[neighbour]=True
    return path
graph=defaultdict(list)
n,e=map(int,input().split())
for i in range(e):
    u,v=map(str,input().split())
    graph[u].append(v)
    graph[v].append(u)
#print(graph)
start='A' 
visited=defaultdict(bool)
path=[]
traversedpath=dfs(graph,start,visited,path)
print(traversedpath)
```
## Output 1 :
![image](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/32d454ac-fbd4-46cf-ba8a-884659c7184e)
## Program :
```
#import defaultdict
from collections import defaultdict
def dfs(graph,start,visited,path):
    path.append(start)
    visited[start]=True
    for neighbour in graph[start]:
        if visited[neighbour]==False:
            dfs(graph,neighbour,visited,path)
            visited[neighbour]=True
    return path
graph=defaultdict(list)
n,e=map(int,input().split())
for i in range(e):
    u,v=map(str,input().split())
    graph[u].append(v)
    graph[v].append(u)
#print(graph)
start='0'   # The starting node is 0
visited=defaultdict(bool)
path=[]
traversedpath=dfs(graph,start,visited,path)
print(traversedpath)
```
## Output 2 :
![image](https://github.com/SOWMIYA2003/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/93427443/ee6875bd-9afe-4736-b15d-51759cbfe939)

## Result:
Thus,a Graph was constructed and implementation of Depth First Search for the same graph was done


# ExpNo 2 : Implement Breadth First Search Traversal of a Graph
## Name : Sowmiya N
## Register No : 212221230106

## Aim:

To Implement Breadth First Search Traversal of a Graph using Python 3.
## Theory:

Breadth-First Traversal (or Search) for a graph is like the Breadth-First Traversal of a tree. The only catch here is that, unlike trees, graphs may contain cycles so that we may come to the same node again. To avoid processing a node more than once, we divide the vertices into two categories:

1. Visited
2. Not Visited

A Boolean visited array is used to mark the visited vertices. For simplicity, it is assumed that all vertices are reachable from the starting vertex. BFS uses a queue data structure for traversal.

### How does BFS work?
Starting from the root, all the nodes at a particular level are visited first, and then the next level nodes are traversed until all the nodes are visited. To do this, a queue is used. All the adjacent unvisited nodes of the current level are pushed into the queue, and the current-level nodes are marked visited and popped from the queue. Illustration: Let us understand the working of the algorithm with the help of the following example. Step1: Initially queue and visited arrays are empty. 
![277148319-8acdebf8-ecc2-4d10-a208-45cce441f059](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/08741553-9b8b-4f1d-a697-606914149871)

Queue and visited arrays are empty initially. Step2: Push node 0 into queue and mark it visited.

![277148352-0e9ce012-8e1f-43d7-b7b9-c0fb19fe0c3f](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/9a2361f0-76ed-44b2-978b-eded12605407)

Push node 0 into queue and mark it visited. Step 3: Remove node 0 from the front of queue and visit the unvisited neighbours and push them into queue.

![277148379-67d8fa3b-ce9e-46c2-9dd7-089e204e667a](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/cf1224e2-f711-4163-ae8c-33335a02d14f)


Step 4: Remove node 1 from the front of queue and visit the unvisited neighbours and push them into queue.

![277148430-b0cf0fde-8a86-41cb-a054-36875ac24ab0](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/49d596da-60ed-4961-bf03-d2a4db00542b)

Step 5: Remove node 2 from the front of queue and visit the unvisited neighbours and push them into queue.

![277148459-8968a163-6b3a-4f7e-8ad4-bbf24f326b9b](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/ec57922f-9fa4-4d6c-a798-26c023b7d095)

Step 6: Remove node 3 from the front of queue and visit the unvisited neighbours and push them into queue. As we can see that every neighbours of node 3 is visited, so move to the next node that are in the front of the queue.

![277148500-7a1c1b16-ea69-497f-a099-8440200f6dc0](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/31a4abe5-2ec8-4b13-a821-5bff394d488e)

Steps 7: Remove node 4 from the front of queue and visit the unvisited neighbours and push them into queue. As we can see that every neighbours of node 4 are visited, so move to the next node that is in the front of the queue.

![277148529-8e16ffa3-c3d6-4774-822b-6eb84adedad9](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/55b2ded8-9f49-4424-811c-1439ad165783)

Remove node 4 from the front of queue and visit the unvisited neighbours and push them into queue. Now, Queue becomes empty, So, terminate these process of iteration.

## Algorithm:

1. Construct a Graph with Nodes and Edges
2. Breadth First Uses Queue and iterates through the Queue for Traversal.
3. Insert a Start Node into the Queue.
4. Find its Successors Or neighbors and Check whether the node is visited or not.
5. If Not Visited, add it to the Queue. Else Continue.
6. Iterate steps 4 and 5 until all nodes get visited, and there are no more unvisited nodes.

## Program :
```
from collections import deque
from collections import defaultdict
def bfs(graph,start,visited,path):
    queue = deque()
    path.append(start)
    queue.append(start)
    visited[start] = True
    while len(queue) != 0:
        tmpnode = queue.popleft()
        for neighbour in graph[tmpnode]:
            if visited[neighbour] == False:
                path.append(neighbour)
                queue.append(neighbour)
                visited[neighbour] = True
    return path

graph = defaultdict(list)
v,e = map(int,input().split())
for i in range(e):
    u,v = map(str,input().split())
    graph[u].append(v)
    graph[v].append(u)

start = '0'
path = []
visited = defaultdict(bool)
traversedpath = bfs(graph,start,visited,path)
print(traversedpath)
```
## Output :
![image](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/421bf2bd-42ac-4760-87a7-3d1bc9ef3028)

## Result:

Thus,a Graph was constructed and implementation of Breadth First Search for the same graph was done successfully.

# ExpNo 3 : Implement A* search algorithm for a Graph
## Name : Sowmiya N
## Register No : 212221230106

## Aim:

To ImplementA * Search algorithm for a Graph using Python 3.
## Algorithm:
A* Search Algorithm 
1. Initialize the open list
2. Initialize the closed list put the starting node on the open list (you can leave its f at zero)
3. while the open list is not empty
   
    a) find the node with the least f on the open list, call it "q"

    b) pop q off the open list

    c) generate q's 8 successors and set their parents to q

    d) for each successor
 ```
 i) if successor is the goal, stop search
 ii) else, compute both g and h for successor
  successor.g = q.g + distance between 
                      successor and q
  successor.h = distance from goal to 
  successor (This can be done using many 
  ways, we will discuss three heuristics- 
  Manhattan, Diagonal and Euclidean 
  Heuristics)
  
  successor.f = successor.g + successor.h

iii) if a node with the same position as 
    successor is in the OPEN list which has a 
   lower f than successor, skip this successor

iV) if a node with the same position as 
    successor  is in the CLOSED list which has
    a lower f than successor, skip this successor
    otherwise, add  the node to the open list
```
end (for loop)

    e) push q on the closed list end (while loop)
    
## Sample Graph 
![277152712-acbb09cb-ed39-48e5-a59b-2f8d61b978a3](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/1f77c3dc-487e-46ad-967c-ae33cc42942c)

## Program :
```
from collections import defaultdict
H_dist ={}
def aStarAlgo(start_node, stop_node):
    open_set = set(start_node)
    closed_set = set()
    g = {}               #store distance from starting node
    parents = {}         # parents contains an adjacency map of all nodes
    #distance of starting node from itself is zero
    g[start_node] = 0
    #start_node is root node i.e it has no parent nodes
    #so start_node is set to its own parent node
    parents[start_node] = start_node
    while len(open_set) > 0:
        n = None
        #node with lowest f() is found
        for v in open_set:
            if n == None or g[v] + heuristic(v) < g[n] + heuristic(n):
                n = v
        if n == stop_node or Graph_nodes[n] == None:
            pass
        else:
            for (m, weight) in get_neighbors(n):
                #nodes 'm' not in first and last set are added to first
                #n is set its parent
                if m not in open_set and m not in closed_set:
                    open_set.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight
                #for each node m,compare its distance from start i.e g(m) to the
                #from start through n node
                else:
                    if g[m] > g[n] + weight:
                        #update g(m)
                        g[m] = g[n] + weight
                        #change parent of m to n
                        parents[m] = n
                        #if m in closed set,remove and add to open
                        if m in closed_set:
                            closed_set.remove(m)
                            open_set.add(m)
        if n == None:
            print('Path does not exist!')
            return None
        
        # if the current node is the stop_node
        # then we begin reconstructin the path from it to the start_node
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()
            print('Path found: {}'.format(path))
            return path
        # remove n from the open_list, and add it to closed_list
        # because all of his neighbors were inspected
        open_set.remove(n)
        closed_set.add(n)
    print('Path does not exist!')
    return None

#define fuction to return neighbor and its distance
#from the passed node
def get_neighbors(v):
    if v in Graph_nodes:
        return Graph_nodes[v]
    else:
        return None
def heuristic(n):
    return H_dist[n]


#Describe your graph here
'''Graph_nodes = {
    'A': [('B', 6), ('F', 3)],
    'B': [('A', 6), ('C', 3), ('D', 2)],
    'C': [('B', 3), ('D', 1), ('E', 5)],
    'D': [('B', 2), ('C', 1), ('E', 8)],
    'E': [('C', 5), ('D', 8), ('I', 5), ('J', 5)],
    'F': [('A', 3), ('G', 1), ('H', 7)],
    'G': [('F', 1), ('I', 3)],
    'H': [('F', 7), ('I', 2)],
    'I': [('E', 5), ('G', 3), ('H', 2), ('J', 3)],
}
'''
graph = defaultdict(list)
n,e = map(int,input().split())
for i in range(e):
    u,v,cost = map(str,input().split())
    t=(v,float(cost))
    graph[u].append(t)
    t1=(u,float(cost))
    graph[v].append(t1)
for i in range(n):
    node,h=map(str,input().split())
    H_dist[node]=float(h)
print(H_dist)

Graph_nodes=graph
print(graph)
aStarAlgo('A', 'G')
```
## Output :
![image](https://github.com/SOWMIYA2003/ExpNo-1-Implement-Depth-First-Search-Traversal-of-a-Graph/assets/93427443/24c8f868-9e5e-4851-8e9d-74955efad2c5)

## Result :
Thus the implementation of A* Search algorithm is done successfully.

# ExpNo 4 : Implement Simple Hill Climbing Algorithm

## Name : Sowmiya N
## Register No : 212221230106

## Aim:

Implement Simple Hill Climbing Algorithm and Generate a String by Mutating a Single Character at each iteration
## Theory:

Hill climbing is a variant of Generate and test in which feedback from test procedure is used to help the generator decide which direction to move in search space. Feedback is provided in terms of heuristic function
## Algorithm:

1. Evaluate the initial state.If it is a goal state then return it and quit. Otherwise, continue with initial state as current state.
2. Loop until a solution is found or there are no new operators left to be applied in current state:
      a. Select an operator that has not yet been applied to the current state and apply it to produce a new state
      b. Evaluate the new state:
            i. if it is a goal state, then return it and quit
            ii. if it is not a goal state but better than current state then make new state as current state
            iii. if it is not better than current state then continue in the loop

## Steps Applied:
### Step-1
Generate Random String of the length equal to the given String
### Step-2
Mutate the randomized string each character at a time
### Step-3
Evaluate the fitness function or Heuristic Function
### Step-4:
Lopp Step -2 and Step-3 until we achieve the score to be Zero to achieve Global Minima.


## Program :
```
import random
import string
def generate_random_solution(answer):
    l=len(answer)
    return [random.choice(string.printable) for _ in range(l)]
def evaluate(solution,answer):
    print(solution)
    target=list(answer)
    diff=0
    for i in range(len(target)):
        s=solution[i]
        t=target[i]
        diff +=abs(ord(s)-ord(t))
    return diff
def mutate_solution(solution):
    ind=random.randint(0,len(solution)-1)
    solution[ind]=random.choice(string.printable)
    return solution
def SimpleHillClimbing():
    answer="Artificial Intelligence"
    best=generate_random_solution(answer)
    best_score=evaluate(best,answer)
    while True:
        print("Score:",best_score," Solution : ","".join(best))  
        if best_score==0:
            break
        new_solution=mutate_solution(list(best))
        score=evaluate(new_solution,answer)   
        if score<best_score:
            best=new_solution
            best_score=score
#answer="Artificial Intelligence"
#print(generate_random_solution(answer))
#solution=generate_random_solution(answer)
#print(evaluate(solution,answer))
SimpleHillClimbing()
```
## Output :
![image](https://github.com/SOWMIYA2003/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE-/assets/93427443/6bbb11b5-1448-4709-8a03-11d0b00fc941)

![image](https://github.com/SOWMIYA2003/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE-/assets/93427443/a6324832-7a8f-48ce-9e7f-4b19d2a3e082)

## Result :

Thus the Implementation of Simple Hill Climbing Algorithm and Generating a String by Mutating a Single Character at each iteration is done successfully.

















