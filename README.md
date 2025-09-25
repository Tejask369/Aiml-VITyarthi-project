I. Modeling the Simulation Environment
The foundation of this project is a virtual world designed to emulate real-world navigation challenges. This world is modeled as a two-dimensional grid, a discretized Cartesian plane where each coordinate (x,y) represents a distinct location.

Within this grid, certain cells are designated as impassable barriers, representing permanent structures like buildings or walls. These are marked with a prohibitive value to ensure the agent's pathfinding logic avoids them entirely. To reflect the complexity of a real city, navigable cells feature heterogeneous traversal costs. A standard paved road might incur a base movement cost of 1 unit, whereas more challenging terrain like a gravel path or a congested area could impose higher costs (e.g., 2 or 3 units). This cost system simulates the varying expenditure of resources like time or fuel.

To introduce an element of unpredictability, the environment also supports time-dependent obstacles. These are transient blockages, such as a passing vehicle or a temporary construction zone, defined by a set of coordinates and a specific time, (x,y,t). The agent's navigation system must be intelligent enough to detect and reroute its path if a planned route is obstructed by one of these dynamic events.

The agent's condition, or state, is primarily defined by its current grid position (x,y). In scenarios involving dynamic obstacles, time becomes a critical component of the state, represented as (x,y,t). From any given cell, the agent can execute a set of actions, namely moving to any of the four cardinally adjacent cells (north, south, east, west). The cost of performing an action is determined by the traversal cost of the destination cell.

II. Pathfinding and Adaptation Strategies
Uninformed Search Algorithms
To establish a performance baseline, two fundamental uninformed search algorithms were implemented.

Breadth-First Search (BFS): This algorithm systematically explores the grid layer by layer from the starting point. By employing a queue data structure, BFS guarantees finding the path with the fewest steps. It is highly effective in environments with uniform costs but becomes inefficient when traversal costs vary.

Uniform-Cost Search (UCS): An evolution of BFS, UCS prioritizes paths based on their cumulative cost rather than the number of steps. Using a priority queue, it always expands the lowest-cost path from the origin. This ensures it finds the optimal (least-cost) path in a weighted grid, though its exhaustive exploration can be computationally intensive.

Heuristic-Guided Search
A* Search: To improve upon the efficiency of uninformed methods, the A* algorithm was implemented. A* combines the optimality of UCS with a heuristic function, h(n), to guide its search intelligently toward the destination. The algorithm prioritizes nodes based on the formula f(n)=g(n)+h(n), where g(n) is the known cost from the start to node n. The Manhattan distance, h(n)=∣n 
x
​
 −goal 
x
​
 ∣+∣n 
y
​
 −goal 
y
​
 ∣, was chosen as the heuristic. This function is computationally cheap and, crucially, admissible—it never overestimates the actual cost to the goal, thus preserving A*'s guarantee of finding the optimal path.

Dynamic Replanning with Local Search
Real-world environments are rarely static. To equip the agent with the ability to adapt, a replanning mechanism using a local search strategy was developed.

Hill-Climbing with Random Restarts: This strategy is invoked when the agent's pre-calculated path is invalidated by a dynamic obstacle. The agent performs a hill-climbing search by exploring alternative path segments in its immediate vicinity, greedily selecting the option that offers the best improvement in cost. However, this can lead to settling for a sub-optimal solution (a local minimum). To counteract this, random restarts are incorporated. If the agent finds itself unable to improve its path, it can jump to a different valid state and initiate a new search, increasing the chances of discovering a more globally optimal route.

III. Experimental Methodology
Test Scenarios 🗺️
To rigorously evaluate the algorithms, a suite of diverse map scenarios was designed, each intended to probe different aspects of performance:

A simple map with low, uniform costs and few obstacles.

A complex map featuring a labyrinth of static obstacles.

A variable-cost map with significant differences in terrain difficulty.

A dynamic map where a new obstacle appears after the agent has started its journey, forcing a replan.

Performance Metrics 📊
For each combination of algorithm and map, the following key performance indicators were meticulously recorded:

Path Cost: The final, total cost accumulated by the agent to travel from the start to the goal.

Nodes Expanded: The total number of unique grid cells evaluated by the algorithm during its search. This metric is a direct measure of computational effort and search efficiency.

Execution Time: The real-world clock time required for the algorithm to compute the final path.

IV. Analysis and Interpretation
Results Summary
The empirical data gathered will be consolidated and presented in a structured table, allowing for a direct comparison of each algorithm across all map instances and performance metrics.

Comparative Performance Insights
Informed vs. Uninformed Search: The results are expected to show that while BFS is optimal on unweighted grids, its ignorance of cost makes it unsuitable for complex terrains. UCS, while always finding the optimal path, tends to explore a large search space, resulting in higher computational overhead. In contrast, A* is hypothesized to be the most balanced algorithm. By leveraging its heuristic, it intelligently prunes the search space, drastically reducing the number of expanded nodes and execution time compared to UCS, all while retaining the guarantee of optimality.

Static vs. Dynamic Planning: The replanning scenario will highlight the critical need for adaptability. While A* produces an excellent initial plan, its solution is brittle and fails when the environment changes unexpectedly. A local search replanning strategy demonstrates the agent's robustness. It allows the agent to dynamically adapt to unforeseen events, ensuring mission completion even if the new path is not the absolute global optimum.

Algorithm Selection Rationale 🤔
The analysis will conclude with a clear rationale for selecting the appropriate algorithm based on the problem context:

For simple, unweighted environments, BFS provides a straightforward and efficient solution.

For complex, static environments with variable costs, A* search is the unequivocal choice, offering optimal paths with superior performance.

For dynamic, unpredictable environments, a hybrid approach is most effective: using A* for initial pathfinding combined with a rapid local search strategy for on-the-fly replanning. This framework provides a powerful blend of optimality and real-time adaptability.
