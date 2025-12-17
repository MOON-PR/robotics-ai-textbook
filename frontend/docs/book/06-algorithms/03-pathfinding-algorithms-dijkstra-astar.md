---
id: book-06-algorithms-03-pathfinding-algorithms-dijkstra-astar
title: 03-pathfinding-algorithms-dijkstra-astar
sidebar_position: 3
---

--- 
sidebar_position: 3
title: Pathfinding Algorithms (Dijkstra, A*)
---

## 03-Pathfinding Algorithms (Dijkstra, A*)

For a robot to navigate autonomously from one point to another in an environment, it needs to find a path. **Pathfinding algorithms** are a class of algorithms designed to determine the shortest or most optimal route between a starting point and a destination, considering obstacles and costs associated with traversing different parts of the environment. This chapter introduces two of the most fundamental and widely used pathfinding algorithms: Dijkstra's Algorithm and the A* (A-star) Search Algorithm.

### 3.1 Representing the Environment (Graphs)

Pathfinding algorithms typically operate on a **graph** representation of the environment.

*   **Nodes (Vertices):** Represent locations, waypoints, intersections, or discrete cells in a grid.
*   **Edges (Arcs):** Represent possible connections or paths between nodes.
*   **Weights/Costs:** Each edge can have a weight or cost associated with traversing it (e.g., distance, time, energy consumption, risk).

Common ways to construct a graph from a robot's environment:
*   **Grid Map:** Discretizing the environment into a grid of cells (e.g., 1m x 1m squares), where each cell is a node, and adjacent free cells have edges between them.
*   **Waypoint Graph:** Manually defining key locations (waypoints) and connections between them.
*   **Visibility Graph:** Connecting vertices of obstacles that are mutually visible, or points around obstacles.

**Diagram 3.1: Grid Map Representation for Pathfinding**

```mermaid
graph TD
    subgraph Grid Map
        NW["(0,2)"] --- N["(1,2)"] --- NE["(2,2)"]
        |             |             |
        W["(0,1)"] --- C["(1,1)"] --- E["(2,1)"]
        |             |             | 
        SW["(0,0)"] --- S["(1,0)"] --- SE["(2,0)"]
        
        style N fill:#f9f,stroke:#333,stroke-width:2px
        style E fill:#f9f,stroke:#333,stroke-width:2px
        style C fill:#f9f,stroke:#33,stroke-width:2px
        style S fill:#f9f,stroke:#333,stroke-width:2px
        style W fill:#f9f,stroke:#333,stroke-width:2px
        style NW fill:#f9f,stroke:#333,stroke-width:2px
        style NE fill:#f9f,stroke:#333,stroke-width:2px
        style SW fill:#f9f,stroke:#333,stroke-width:2px
        style SE fill:#f9f,stroke:#333,stroke-width:2px
    end
    
    style C fill:#fcc,stroke:#333,stroke-width:2px
    style N fill:#cff,stroke:#333,stroke-width:2px
    style S fill:#cff,stroke:#333,stroke-width:2px
    style W fill:#cff,stroke:#333,stroke-width:2px
    style E fill:#cff,stroke:#333,stroke-width:2px
    style NW fill:#cff,stroke:#333,stroke-width:2px
    style NE fill:#cff,stroke:#333,stroke-width:2px
    style SW fill:#cff,stroke:#333,stroke-width:2px
    style SE fill:#cff,stroke:#333,stroke-width:2px
    
    NW -.- N
    N -.- NE
    W -.- C
    C -.- E
    SW -.- S
    S -.- SE
    NW -.- W
    N -.- C
    NE -.- E
    W -.- SW
    C -.- S
    E -.- SE
```

*Description: A 3x3 grid map where each cell represents a node. Edges connect adjacent cells, with potential costs representing difficulty of traversal. Obstacles would be nodes with infinite cost or no outgoing edges.*

### 3.2 Dijkstra's Algorithm

**Dijkstra's Algorithm** is an algorithm for finding the shortest paths between nodes in a graph, which may have positive edge weights. It is a **greedy algorithm** that explores outward from the starting node, always expanding to the node with the currently known shortest distance.

#### 3.2.1 How it Works

1.  **Initialization:**
    *   Assign a distance of 0 to the starting node and infinity to all other nodes.
    *   Maintain a set of "visited" nodes and a priority queue (or min-heap) of "unvisited" nodes, ordered by their current shortest distance.
2.  **Iteration:**
    *   While the priority queue is not empty:
        *   Extract the node `u` with the smallest distance from the priority queue.
        *   Mark `u` as visited.
        *   For each unvisited neighbor `v` of `u`:
            *   Calculate the distance from the start to `v` through `u` (current distance to `u` + weight of edge `u-v`).
            *   If this new distance is less than the current recorded distance to `v`, update `v`'s distance and its predecessor (how it was reached).
3.  **Termination:** The algorithm terminates when the destination node is visited, or when the priority queue is empty (meaning no path exists).

#### 3.2.2 Efficiency

*   **Time Complexity:** O(E log V) or O(E + V log V) with a Fibonacci heap, where V is the number of vertices (nodes) and E is the number of edges. More practically, with a binary min-priority queue (like `std::priority_queue` in C++), it's O(E log V).
*   **Space Complexity:** O(V + E) for storing the graph and distances.
*   **Limitations:** Does not work with negative edge weights. Explores in all directions, which can be inefficient for very large maps if the goal is far away.

### 3.3 A* (A-star) Search Algorithm

The **A* Search Algorithm** is an extension of Dijkstra's Algorithm that improves efficiency by using a **heuristic function** to guide its search towards the goal. It's often referred to as an "informed search" algorithm.

#### 3.3.1 How it Works

A* evaluates each node `n` using a cost function `f(n) = g(n) + h(n)`:
*   `g(n)`: The actual cost of the path from the start node to node `n`. (Same as Dijkstra's distance).
*   `h(n)`: The estimated cost of the path from node `n` to the goal node (the **heuristic function**).
*   `f(n)`: The estimated total cost of the path through node `n` to the goal.

The algorithm prioritizes exploring nodes that are likely to be on the shortest path by picking the node with the lowest `f(n)` value from the priority queue.

#### 3.3.2 Heuristic Function (`h(n)`)

The choice of heuristic is critical for A* performance:
*   **Admissible Heuristic:** Never overestimates the actual cost to reach the goal (i.e., `h(n) le` actual_cost(n, goal)). An admissible heuristic guarantees that A* will find the optimal path.
    *   **Examples:** Euclidean distance (straight-line distance), Manhattan distance (sum of absolute differences of coordinates).
*   **Consistent Heuristic:** A stronger condition than admissible. If `h(n)` is consistent, it is also admissible.
    *   **Effect:** A consistent heuristic (or admissible one, if goal is far) reduces the number of nodes explored, making the search faster.

#### 3.3.3 Efficiency

*   **Time Complexity:** Depends on the quality of the heuristic. In the worst case (e.g., non-admissible heuristic or a very complex search space), it can degrade to O(E log V) or even O(E). With a good heuristic, it's often much faster than Dijkstra's.
*   **Space Complexity:** O(V + E) in the worst case.
*   **Advantages:** Optimal (if heuristic is admissible), much faster than Dijkstra's for large maps when a good heuristic is available.
*   **Disadvantages:** Requires an appropriate heuristic, which might not always be easy to design.

### 3.4 Applications in Robotics

*   **Mobile Robot Navigation:** Planning routes in known or partially known environments (warehouses, homes, outdoor areas).
*   **Autonomous Driving:** Finding optimal driving paths while considering traffic, speed limits, and vehicle dynamics.
*   **Game AI:** Pathfinding for non-player characters.
*   **Robotic Arm Motion Planning:** Searching for collision-free joint configurations to reach a target pose.

Dijkstra's and A* algorithms are foundational for enabling robots to autonomously plan their movements, serving as critical components in any navigation stack.

--- 

### C++ Example: Dijkstra's Algorithm Implementation

This C++ example implements Dijkstra's algorithm to find the shortest path in a conceptual robot map represented as an adjacency list.

```cpp
#include <iostream>
#include <vector>
#include <queue> // For std::priority_queue
#include <limits> // For std::numeric_limits
#include <map>    // For string-to-int node mapping
#include <string>

// Structure to represent an edge in the graph
struct Edge {
    int to_node;
    int weight;

    // Custom comparator for priority_queue to act as a min-heap
    // Priority queue stores {distance, node_id}
    bool operator>(const Edge& other) const {
        return weight > other.weight;
    }
};

// Function to find shortest paths using Dijkstra's algorithm
std::map<int, int> dijkstra(int start_node, int num_nodes, 
                            const std::vector<std::vector<Edge>>& adj_list,
                            std::map<int, int>& predecessors) {
    
    std::map<int, int> distances; // Stores shortest distance from start_node to each node
    for (int i = 0; i < num_nodes; ++i) {
        distances[i] = std::numeric_limits<int>::max(); // Initialize with infinity
    }
    distances[start_node] = 0;

    // Priority queue to store {distance, node_id}
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.push({start_node, 0});

    while (!pq.empty()) {
        int current_distance = pq.top().weight;
        int current_node = pq.top().to_node;
        pq.pop();

        // If we found a shorter path to current_node already, skip
        if (current_distance > distances[current_node]) {
            continue;
        }

        // Explore neighbors
        for (const Edge& edge : adj_list[current_node]) {
            int neighbor = edge.to_node;
            int weight = edge.weight;

            if (distances[current_node] + weight < distances[neighbor]) {
                distances[neighbor] = distances[current_node] + weight;
                predecessors[neighbor] = current_node; // Store predecessor for path reconstruction
                pq.push({neighbor, distances[neighbor]});
            }
        }
    }
    return distances;
}

// Function to reconstruct the path from predecessors map
std::vector<int> reconstructPath(int start_node, int end_node, const std::map<int, int>& predecessors) {
    std::vector<int> path;
    int current = end_node;
    while (current != start_node) {
        path.insert(path.begin(), current);
        if (predecessors.find(current) == predecessors.end()) { // No path found
            return {};
        }
        current = predecessors.at(current);
    }
    path.insert(path.begin(), start_node);
    return path;
}


int main() {
    std::cout << "--- Dijkstra\'s Algorithm for Robot Pathfinding ---" << std::endl;

    // Map string names to integer IDs for easier graph representation
    std::map<std::string, int> node_names_to_id;
    std::map<int, std::string> node_id_to_names;
    int next_id = 0;

    auto add_node = [&](const std::string& name) {
        if (node_names_to_id.find(name) == node_names_to_id.end()) {
            node_names_to_id[name] = next_id;
            node_id_to_names[next_id] = name;
            next_id++;
        }
        return node_names_to_id[name];
    };

    int cs = add_node("Charging_Station");
    int wa = add_node("Workstation_A");
    int wb = add_node("Workstation_B");
    int sa = add_node("Storage_Area");
    int ep = add_node("Entry_Point");

    int num_nodes = next_id;
    std::vector<std::vector<Edge>> adj_list(num_nodes);

    // Add edges (node1, node2, weight)
    adj_list[cs].push_back({ep, 5}); adj_list[ep].push_back({cs, 5});
    adj_list[ep].push_back({wa, 10}); adj_list[wa].push_back({ep, 10});
    adj_list[ep].push_back({wb, 8}); adj_list[wb].push_back({ep, 8});
    adj_list[wa].push_back({sa, 7}); adj_list[sa].push_back({wa, 7});
    adj_list[wb].push_back({sa, 6}); adj_list[sa].push_back({wb, 6});
    adj_list[wa].push_back({wb, 3}); adj_list[wb].push_back({wa, 3});

    int start_node_id = add_node("Charging_Station");
    int goal_node_id = add_node("Storage_Area");

    std::map<int, int> predecessors;
    std::map<int, int> shortest_distances = dijkstra(start_node_id, num_nodes, adj_list, predecessors);

    std::cout << "\nShortest distances from " << node_id_to_names[start_node_id] << ":" << std::endl;
    for (const auto& pair : shortest_distances) {
        std::cout << "  To " << node_id_to_names[pair.first] << ": " << pair.second << std::endl;
    }

    std::vector<int> path = reconstructPath(start_node_id, goal_node_id, predecessors);
    if (!path.empty()) {
        std::cout << "\nPath from " << node_id_to_names[start_node_id] << " to " << node_id_to_names[goal_node_id] << ": ";
        for (int i = 0; i < path.size(); ++i) {
            std::cout << node_id_to_names[path[i]];
            if (i < path.size() - 1) std::cout << " -> ";
        }
        std::cout << " (Total cost: " << shortest_distances[goal_node_id] << ")" << std::endl;
    } else {
        std::cout << "\nNo path found from " << node_id_to_names[start_node_id] << " to " << node_id_to_names[goal_node_id] << "." << std::endl;
    }

    std::cout << "\nSimulation finished." << std::endl;
    return 0;
}
```

--- 

### Python Example: A* Search Algorithm Implementation

This Python example implements the A* search algorithm for pathfinding on a 2D grid map, including a heuristic function.

```python
import heapq # For priority queue
import math

class Node:
    def __init__(self, x, y, cost=0, heuristic=0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost      # g(n): cost from start to current node
        self.heuristic = heuristic # h(n): estimated cost from current node to end
        self.f_cost = cost + heuristic # f(n) = g(n) + h(n)
        self.parent = parent  # For reconstructing path

    def __lt__(self, other): # For comparison in priority queue
        return self.f_cost < other.f_cost

    def __eq__(self, other): # For checking equality (important for visited set)
        return self.x == other.x and self.y == other.y

    def __hash__(self): # For using Node objects in sets/dictionaries
        return hash((self.x, self.y))

def heuristic_distance(node_a, node_b):
    """
    Manhattan distance heuristic (admissible for grid with 4-directional movement).
    For 8-directional movement, Chebyshev or Euclidean distance is better.
    """
    return abs(node_a.x - node_b.x) + abs(node_a.y - node_b.y)

def a_star_search(grid, start, end):
    """
    Finds the shortest path from start to end on a grid using A* algorithm.
    grid: 2D list where 0 is traversable, 1 is obstacle.
    start: (x, y) tuple for start coordinates.
    end: (x, y) tuple for end coordinates.
    """
    rows, cols = len(grid), len(grid[0])
    
    # Check if start or end are obstacles
    if grid[start[0]][start[1]] == 1 or grid[end[0]][end[1]] == 1:
        print("Start or End node is an obstacle!")
        return None

    # Priority queue (min-heap) to store (f_cost, node_object)
    open_list = []
    start_node = Node(start[0], start[1], 0, heuristic_distance(Node(start[0], start[1]), Node(end[0], end[1])))
    heapq.heappush(open_list, (start_node.f_cost, start_node))

    # Dictionary to store the cheapest path found to each node
    # {node: node_object}
    came_from = {}
    
    # Dictionary to store g_cost (cost from start) for each node
    # {node: g_cost}
    g_costs = {start_node: 0}

    # Possible movements (4-directional: up, down, left, right)
    # For 8-directional: add (1,1), (1,-1), (-1,1), (-1,-1) with cost sqrt(2)
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)] # (dx, dy)
    move_cost = 1 # Assuming uniform cost for movement

    while open_list:
        current_f_cost, current_node = heapq.heappop(open_list)

        if current_node.x == end[0] and current_node.y == end[1]:
            # Path found! Reconstruct and return
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = came_from.get(current_node)
            return path[::-1] # Reverse path to get from start to end

        for dx, dy in movements:
            neighbor_x, neighbor_y = current_node.x + dx, current_node.y + dy

            # Check if neighbor is within grid bounds
            if not (0 <= neighbor_x < rows and 0 <= neighbor_y < cols):
                continue
            
            # Check if neighbor is an obstacle
            if grid[neighbor_x][neighbor_y] == 1:
                continue

            # Calculate tentative g_cost for neighbor
            tentative_g_cost = g_costs.get(current_node, float('inf')) + move_cost
            
            neighbor_node = Node(neighbor_x, neighbor_y) # Create a neighbor node object

            # If this path to neighbor is better than any previous one
            if tentative_g_cost < g_costs.get(neighbor_node, float('inf')):
                came_from[neighbor_node] = current_node
                g_costs[neighbor_node] = tentative_g_cost
                
                neighbor_node.cost = tentative_g_cost
                neighbor_node.heuristic = heuristic_distance(neighbor_node, Node(end[0], end[1]))
                neighbor_node.f_cost = neighbor_node.cost + neighbor_node.heuristic
                
                # Push neighbor to open list if not already there with lower cost
                # For simplicity, we just push it, heapq will handle the lowest f_cost
                heapq.heappush(open_list, (neighbor_node.f_cost, neighbor_node))

    return None # No path found

def print_path_on_grid(grid, path):
    """Prints the grid with the found path."""
    if path:
        path_set = set(path)
    else:
        path_set = set()

    for r_idx, row in enumerate(grid):
        line = ""
        for c_idx, cell in enumerate(row):
            if (r_idx, c_idx) in path_set:
                line += " * " # Path
            elif cell == 1:
                line += " X " # Obstacle
            else:
                line += " . " # Free space
        print(line)

if __name__ == "__main__":
    # Example Grid Map: 0 = free, 1 = obstacle
    grid_map = [
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start_coords = (0, 0)
    end_coords = (4, 4)

    print("--- A* Search Algorithm for Grid Map Pathfinding ---")
    print("Grid Map ('.' = Free, 'X' = Obstacle):")
    print_path_on_grid(grid_map, None)
    
    path = a_star_search(grid_map, start_coords, end_coords)

    if path:
        print(f"\nPath found from {start_coords} to {end_coords}:")
        print(path)
        print("\nVisualized Path ('*' = Path):")
        print_path_on_grid(grid_map, path)
    else:
        print(f"\nNo path found from {start_coords} to {end_coords}.")

    # Another scenario with a blocked path
    grid_map_blocked = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start_coords_blocked = (0, 0)
    end_coords_blocked = (4, 4)
    print("\n--- A* Search Algorithm for Blocked Path ---")
    print("Grid Map ('.' = Free, 'X' = Obstacle):")
    print_path_on_grid(grid_map_blocked, None)
    path_blocked = a_star_search(grid_map_blocked, start_coords_blocked, end_coords_blocked)
    if path_blocked:
        print(f"\nPath found from {start_coords_blocked} to {end_coords_blocked}:")
        print(path_blocked)
        print("\nVisualized Path ('*' = Path):")
        print_path_on_grid(grid_map_blocked, path_blocked)
    else:
        print(f"\nNo path found from {start_coords_blocked} to {end_coords_blocked}.")

    print("\nSimulation finished.")
```

--- 

### Arduino Example: Maze Solver (Simplified DFS Concept)

This Arduino sketch demonstrates a very simplified maze-solving concept using a Depth-First Search (DFS) like approach, suitable for a conceptual robot with simple movement. This is more about exploration than shortest path.

```arduino
// Simplified Maze Solver (Conceptual DFS-like approach) for Arduino
// This doesn't implement a full graph, but a conceptual traversal logic.

// Robot movement commands (replace with actual motor controls)
void moveForward() {
  Serial.println("Robot: Move Forward");
  // Implement actual motor control here
  delay(500);
}

void turnLeft() {
  Serial.println("Robot: Turn Left");
  // Implement actual motor control here
  delay(500);
}

void turnRight() {
  Serial.println("Robot: Turn Right");
  // Implement actual motor control here
  delay(500);
}

// Sensor readings (replace with actual sensor reads)
// Assuming front sensor is a distance sensor, left/right are proximity
bool isPathClearFront() {
  // Simulate
  return (random(0, 100) < 70); // 70% chance clear
}

bool isPathClearLeft() {
  // Simulate
  return (random(0, 100) < 40); // 40% chance clear
}

bool isPathClearRight() {
  // Simulate
  return (random(0, 100) < 40); // 40% chance clear
}

// Simple stack to keep track of decisions for backtracking
// (Conceptual, not a full stack implementation)
int decisionStack[10]; // Max 10 decisions
int stackPointer = 0;

void pushDecision(int decision) {
  if (stackPointer < 10) {
    decisionStack[stackPointer++] = decision;
  }
}

int popDecision() {
  if (stackPointer > 0) {
    return decisionStack[--stackPointer];
  }
  return -1; // Indicate empty
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0)); // Seed random with analog noise
  Serial.println("Arduino Simplified Maze Solver Demo Ready.");
}

void loop() {
  static bool mazeSolved = false;
  if (mazeSolved) {
    Serial.println("Maze solved! Robot is idle.");
    delay(1000);
    return;
  }

  Serial.println("\n--- New Step ---");

  if (isPathClearFront()) {
    Serial.println("Front is clear. Moving forward.");
    moveForward();
    pushDecision(0); // 0 = moved forward
  } else {
    Serial.println("Front is blocked. Looking for alternative path.");
    if (isPathClearRight()) {
      Serial.println("Right is clear. Turning Right.");
      turnRight();
      pushDecision(1); // 1 = turned right
    } else if (isPathClearLeft()) {
      Serial.println("Left is clear. Turning Left.");
      turnLeft();
      pushDecision(2); // 2 = turned left
    } else {
      Serial.println("All paths blocked. Backtracking.");
      int lastDecision = popDecision();
      if (lastDecision == 0) { // If last decision was move forward
        Serial.println("Backtracked from forward. Turning 180 (left twice).");
        turnLeft();
        turnLeft();
      } else if (lastDecision == 1) { // If last decision was turn right
        Serial.println("Backtracked from turn right. Turning left.");
        turnLeft();
      } else if (lastDecision == 2) { // If last decision was turn left
        Serial.println("Backtracked from turn left. Turning right.");
        turnRight();
      } else {
        Serial.println("No more decisions to backtrack. Stuck!");
        mazeSolved = true; // Or declare failure
      }
    }
  }

  // Simple condition to "solve" maze for demo purposes
  if (random(0, 100) < 5) { // 5% chance to randomly solve
    Serial.println("--- Goal Reached! (Simulated) ---");
    mazeSolved = true;
  }
  delay(100);
}
```

--- 

### Equations in LaTeX: A* Search Cost Function

The A* search algorithm prioritizes nodes based on the cost function `f(n)`: 

```latex
f(n) = g(n) + h(n)
``` 

Where:
*   `g(n)` is the cost of the path from the start node to node `n`.
*   `h(n)` is the estimated cost of the path from node `n` to the goal node (the heuristic function).

For A* to guarantee an optimal path, the heuristic `h(n)` must be **admissible** (never overestimates the true cost to the goal) and typically **consistent**.

--- 

### MCQs with Answers

1.  Which pathfinding algorithm uses a heuristic function to guide its search towards the goal, making it an "informed search" algorithm?
    a) Dijkstra's Algorithm
    b) Breadth-First Search (BFS)
    c) Depth-First Search (DFS)
    d) A* Search Algorithm
    *Answer: d) A* Search Algorithm*

2.  What is a key requirement for using Dijkstra's Algorithm in a graph?
    a) All edge weights must be negative.
    b) The graph must be acyclic.
    c) All edge weights must be positive (or zero).
    d) The graph must be directed.
    *Answer: c) All edge weights must be positive (or zero).*

3.  In the A* search algorithm's cost function `f(n) = g(n) + h(n)`, what does `h(n)` represent?
    a) The actual cost from the start node to node `n`.
    b) The total estimated cost of the path from start to goal through node `n`.
    c) The estimated cost from node `n` to the goal node.
    d) The weight of the edge from `n` to its parent.
    *Answer: c) The estimated cost from node `n` to the goal node.*

--- 

### Practice Tasks

1.  **Heuristic Comparison:** For a grid-based robot navigation problem where movement is restricted to 4 directions (up, down, left, right), compare the Manhattan distance heuristic with the Euclidean distance heuristic. Which one is admissible? Which one is a better heuristic in terms of speeding up the search for a truly shortest path? Explain why.
2.  **Obstacle Grid Pathfinding:** Modify the Python A* example to include a new obstacle: a 1x3 wall at `(1,2), (2,2), (3,2)`. Visualize the grid and then run the A* search from `(0,0)` to `(4,4)`. Print the path found.
3.  **Dijkstra's vs. A* Scenario:** You have a very large, static map of a warehouse (millions of potential nodes) and a robot needs to go from a loading dock to a specific storage shelf.
    *   If you don't have a good estimate of the remaining distance to the goal, which algorithm (Dijkstra's or A*) would be more suitable? 
    *   If you *do* have a good estimate (e.g., straight-line distance), which would be better? Explain your reasoning.

--- 

### Notes for Teachers

*   **Visualizations are Key:** Use visual aids (animations, online tools) to demonstrate the step-by-step execution of Dijkstra's and A* algorithms.
*   **Graph Theory Foundation:** Reinforce the importance of graph theory as the underlying data structure for pathfinding.
*   **Heuristic Impact:** Emphasize how the choice and quality of the heuristic function dramatically impact A* performance.

### Notes for Students

*   **Practice Graph Representation:** Get comfortable converting a real-world environment (like a room or a maze) into a graph or grid map.
*   **Manual Tracing:** Manually trace Dijkstra's and A* on small graphs to solidify your understanding of their mechanics.
*   **Admissibility Matters:** Remember that an admissible heuristic is crucial for A* to guarantee finding the optimal (shortest) path.
*   **Computational Cost:** Be aware that even efficient pathfinding algorithms can be computationally intensive for very large maps, requiring careful optimization.
