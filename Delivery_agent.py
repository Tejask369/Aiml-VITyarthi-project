import heapq
import random
import math
import time
import argparse
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from collections import deque
from typing import Tuple, List, Dict, Any, Optional

# Defines a single square on our map, including its terrain properties.
class MapTile:
    """Represents a single cell in the grid, defining its properties."""
    def __init__(self, move_difficulty: int = 1, is_blocked: bool = False):
        self.move_difficulty = move_difficulty
        self.is_blocked = is_blocked

    def get_traversal_cost(self, current_time: int) -> float:
        """
        Calculates the effort required to move through this tile.
        Can be used to model dynamic obstacles that appear over time.
        """
        if self.is_blocked:
            return float('inf')
        return self.move_difficulty

# Manages the overall simulation environment, including the grid and tasks.
class CourierGrid:
    """Holds the entire map layout, delivery tasks, and simulation constraints."""
    def __init__(self):
        self.width: int = 0
        self.height: int = 0
        self.tiles: List[List[MapTile]] = []
        self.tasks: List[Tuple[Tuple[int, int], Tuple[int, int]]] = []
        self.start_pos: Tuple[int, int] = (0, 0)
        self.max_energy: int = 1000
        self.max_time: int = 1000

    def load_map_from_spec(self, map_name: str):
        """Loads a predefined map configuration based on its name."""
        map_loaders = {
            "small.map": self._configure_small_map,
            "medium.map": self._configure_medium_map,
            "large.map": self._configure_large_map,
            "dynamic.map": self._configure_dynamic_map,
        }
        loader = map_loaders.get(map_name)
        if loader:
            loader()
        else:
            raise ValueError(f"Unknown map name provided: {map_name}")

    def _configure_small_map(self):
        self.width, self.height = 5, 5
        self.tiles = [[MapTile() for _ in range(self.width)] for _ in range(self.height)]
        self.tiles[2][2].is_blocked = True
        self.tiles[3][1].is_blocked = True
        self.tasks = [((1, 1), (4, 4))]
        self.start_pos = (0, 0)

    def _configure_medium_map(self):
        self.width, self.height = 10, 10
        self.tiles = [[MapTile() for _ in range(self.width)] for _ in range(self.height)]
        # Create a patch of difficult terrain (e.g., mud)
        for r in range(3, 7):
            for c in range(3, 7):
                self.tiles[r][c].move_difficulty = 3
        self.tiles[5][5].is_blocked = True
        self.tiles[6][2].is_blocked = True
        self.tiles[7][7].is_blocked = True
        self.tasks = [((0, 1), (9, 9)), ((2, 2), (8, 8))]
        self.start_pos = (0, 0)

    def _configure_large_map(self):
        self.width, self.height = 10, 10
        self.tiles = [[MapTile() for _ in range(self.width)] for _ in range(self.height)]
        # Create a pattern of obstacles
        for r in range(self.height):
            for c in range(self.width):
                is_task_related = (r, c) in [(0, 0), (1, 1), (9, 9)]
                if (r + c) % 5 == 0 and not is_task_related:
                    self.tiles[r][c].is_blocked = True
        self.tasks = [((1, 1), (9, 9)), ((2, 2), (8, 8))]
        self.start_pos = (0, 0)

    def _configure_dynamic_map(self):
        self.width, self.height = 8, 8
        self.tiles = [[MapTile() for _ in range(self.width)] for _ in range(self.height)]
        self.tiles[2][2].is_blocked = True
        self.tiles[2][3].is_blocked = True
        self.tiles[3][3].is_blocked = True
        self.tasks = [((1, 1), (7, 7))]
        self.start_pos = (0, 0)

# An abstract base class for different pathfinding algorithms.
class PathfindingStrategy:
    """Base class defining the interface for all pathfinding algorithms."""
    FOUR_DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    def __init__(self, grid: CourierGrid):
        self.grid = grid

    def get_valid_moves(self, pos: Tuple[int, int], time_step: int) -> List[Tuple[Tuple[int, int], float]]:
        """Finds all reachable, 4-directionally connected neighbors and their costs."""
        r, c = pos
        potential_moves = []
        for dr, dc in self.FOUR_DIRECTIONS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.grid.height and 0 <= nc < self.grid.width:
                cost = self.grid.tiles[nr][nc].get_traversal_cost(time_step)
                if cost != float('inf'):
                    potential_moves.append(((nr, nc), cost))
        return potential_moves

# A simple BFS-based planner that finds the shortest path in terms of steps.
class UninformedPathfinder(PathfindingStrategy):
    """Implements Breadth-First Search to find the path with the fewest steps."""
    def discover_route(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        """Uses BFS to find a path from start to goal."""
        q = deque([(start_pos, [start_pos])])
        explored = {start_pos}
        nodes_checked = 0
        while q:
            curr, path = q.popleft()
            nodes_checked += 1
            if curr == goal_pos:
                return path[1:], nodes_checked
            for neighbor, _ in self.get_valid_moves(curr, len(path)):
                if neighbor not in explored:
                    explored.add(neighbor)
                    q.append((neighbor, path + [neighbor]))
        return None, nodes_checked

# A UCS-based planner that minimizes the total traversal cost.
class CostAwarePathfinder(PathfindingStrategy):
    """Implements Uniform-Cost Search to find the cheapest path."""
    def discover_route(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        """Uses UCS to find the lowest-cost path."""
        p_queue = [(0, start_pos, [start_pos])]
        min_costs = {start_pos: 0}
        nodes_checked = 0
        while p_queue:
            cost, curr, path = heapq.heappop(p_queue)
            nodes_checked += 1
            if curr == goal_pos:
                return path[1:], nodes_checked
            if cost > min_costs.get(curr, float('inf')):
                continue
            for neighbor, move_cost in self.get_valid_moves(curr, len(path)):
                new_cost = cost + move_cost
                if new_cost < min_costs.get(neighbor, float('inf')):
                    min_costs[neighbor] = new_cost
                    heapq.heappush(p_queue, (new_cost, neighbor, path + [neighbor]))
        return None, nodes_checked

# An A*-based planner using Manhattan distance for efficient, optimal pathfinding.
class HeuristicPathfinder(PathfindingStrategy):
    """Implements A* Search, using a heuristic for efficient, optimal pathfinding."""
    def _manhattan_distance(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
        """Calculates the Manhattan distance between two points."""
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def discover_route(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        """Uses A* search to find an optimal path efficiently."""
        node_frontier = [(self._manhattan_distance(start_pos, goal_pos), start_pos)]
        came_from = {}
        g_cost = {start_pos: 0}
        f_cost = {start_pos: self._manhattan_distance(start_pos, goal_pos)}
        nodes_checked = 0

        while node_frontier:
            _, curr = heapq.heappop(node_frontier)
            nodes_checked += 1
            if curr == goal_pos:
                # Reconstruct path by backtracking
                final_path = deque()
                while curr in came_from:
                    final_path.appendleft(curr)
                    curr = came_from[curr]
                return list(final_path), nodes_checked

            for neighbor, cost in self.get_valid_moves(curr, int(g_cost.get(curr, 0))):
                tentative_g = g_cost.get(curr, float('inf')) + cost
                if tentative_g < g_cost.get(neighbor, float('inf')):
                    came_from[neighbor] = curr
                    g_cost[neighbor] = tentative_g
                    f_cost[neighbor] = tentative_g + self._manhattan_distance(neighbor, goal_pos)
                    heapq.heappush(node_frontier, (f_cost[neighbor], neighbor))
        return None, nodes_checked

# A Hill-Climbing based planner for finding a fast, non-optimal solution.
class GreedyPathfinder(PathfindingStrategy):
    """Implements Hill-Climbing with Random Restarts for quick, good-enough paths."""
    EXPLORATION_CHANCE = 0.3

    def __init__(self, grid: CourierGrid, restarts: int = 5):
        super().__init__(grid)
        self.restarts = restarts

    def _manhattan_distance(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def _calculate_path_cost(self, path: List[Tuple[int, int]]) -> float:
        cost = 0.0
        for i, pos in enumerate(path):
            if i > 0:
                step_cost = self.grid.tiles[pos[0]][pos[1]].get_traversal_cost(i)
                if step_cost == float('inf'): return float('inf')
                cost += step_cost
        return cost

    def discover_route(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        best_solution = None
        best_cost = float('inf')
        nodes_checked = 0
        for _ in range(self.restarts):
            current = start_pos
            path = [start_pos]
            visited = {start_pos}
            while current != goal_pos:
                neighbors = [n for n, _ in self.get_valid_moves(current, len(path)) if n not in visited]
                nodes_checked += len(neighbors)
                if not neighbors: break
                
                neighbors.sort(key=lambda n: self._manhattan_distance(n, goal_pos))
                
                # Introduce randomness to escape local optima
                if random.random() < self.EXPLORATION_CHANCE:
                    next_node = random.choice(neighbors)
                else:
                    next_node = neighbors[0]

                path.append(next_node)
                visited.add(next_node)
                current = next_node

            if current == goal_pos:
                cost = self._calculate_path_cost(path)
                if cost < best_cost:
                    best_cost = cost
                    best_solution = path[1:]
        return best_solution, nodes_checked

# A Simulated Annealing based planner that can escape local minima.
class AnnealingPathfinder(PathfindingStrategy):
    """Implements Simulated Annealing to find a good path by avoiding local minima."""
    def __init__(self, grid: CourierGrid, start_temp: float = 100.0, cooling_factor: float = 0.97):
        super().__init__(grid)
        self.start_temp = start_temp
        self.cooling_factor = cooling_factor

    def _generate_random_walk(self, start: Tuple[int, int], end: Tuple[int, int], limit: int = 50) -> List[Tuple[int, int]]:
        path = [start]
        current = start
        for _ in range(limit):
            if current == end: break
            neighbors = self.get_valid_moves(current, len(path))
            if not neighbors: break
            current = random.choice(neighbors)[0]
            path.append(current)
        return path

    def _tweak_path(self, path: List[Tuple[int, int]], end: Tuple[int, int]) -> List[Tuple[int, int]]:
        new_path = path[:]
        if len(new_path) > 2 and random.random() < 0.5:
            idx = random.randint(1, len(new_path) - 2)
            neighbors = self.get_valid_moves(new_path[idx - 1], idx)
            if neighbors: new_path[idx] = random.choice(neighbors)[0]
        elif new_path[-1] != end:
            neighbors = self.get_valid_moves(new_path[-1], len(new_path))
            if neighbors: new_path.append(random.choice(neighbors)[0])
        return new_path

    def _calculate_path_cost(self, path: List[Tuple[int, int]]) -> float:
        cost = 0.0
        for i, pos in enumerate(path):
            if i > 0:
                step_cost = self.grid.tiles[pos[0]][pos[1]].get_traversal_cost(i)
                if step_cost == float('inf'): return float('inf')
                cost += step_cost
        return cost

    def discover_route(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        current_solution = self._generate_random_walk(start_pos, goal_pos)
        if not current_solution or current_solution[-1] != goal_pos:
            return None, 0
            
        current_energy = self._calculate_path_cost(current_solution)
        temp = self.start_temp
        iterations = 0

        while temp > 1 and current_solution and current_solution[-1] == goal_pos:
            new_solution = self._tweak_path(current_solution, goal_pos)
            iterations += 1
            new_energy = self._calculate_path_cost(new_solution)
            
            # Acceptance probability function
            if new_energy < current_energy or random.random() < math.exp((current_energy - new_energy) / temp):
                current_solution = new_solution
                current_energy = new_energy
                
            temp *= self.cooling_factor

        return (current_solution[1:], iterations) if current_solution and current_solution[-1] == goal_pos else (None, iterations)

# The agent that performs deliveries using a selected pathfinding strategy.
class DeliveryBot:
    """The agent that navigates the grid to complete delivery tasks."""
    def __init__(self, grid: CourierGrid, strategy_key: str = 'astar', on_update: Optional[callable] = None):
        self.grid = grid
        self.pos = grid.start_pos
        self.pending_tasks = grid.tasks[:]
        self.completed_tasks = []
        self.energy = grid.max_energy
        self.time = 0
        self.strategy_key = strategy_key
        self.planner = self._create_planner(strategy_key)
        self.mission_log = []
        self.on_update = on_update

    def _create_planner(self, key: str) -> PathfindingStrategy:
        """Factory method to instantiate the chosen pathfinding strategy."""
        strategies = {
            'bfs': UninformedPathfinder,
            'ucs': CostAwarePathfinder,
            'astar': HeuristicPathfinder,
            'hillclimb': GreedyPathfinder,
            'simanneal': AnnealingPathfinder,
        }
        if key not in strategies:
            raise ValueError(f"Strategy '{key}' is not a valid option.")
        return strategies[key](self.grid)

    def _find_route(self, origin: Tuple[int, int], dest: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], int]:
        """Wrapper for calling the planner's route discovery method."""
        return self.planner.discover_route(origin, dest)

    def start_delivery_cycle(self) -> bool:
        """Main loop to process all assigned delivery tasks."""
        self.mission_log.append(f"Dispatching bot with '{self.strategy_key}' strategy.")
        self.mission_log.append(f"Starting at {self.pos} with {self.energy} energy.")

        while self.pending_tasks and self.energy > 0 and self.time < self.grid.max_time:
            pickup_loc, dropoff_loc = self.pending_tasks[0]
            
            # --- Path to Pickup ---
            self.mission_log.append(f"Planning route to pickup at {pickup_loc}")
            path_to_pickup, nodes = self._find_route(self.pos, pickup_loc)
            self.mission_log.append(f"Planner checked {nodes} nodes.")
            if not path_to_pickup:
                self.mission_log.append("Failed to find path to package.")
                return False
            if not self._follow_path(path_to_pickup, pickup_loc):
                return False
            self.mission_log.append(f"Package collected from {pickup_loc}.")
            
            # --- Path to Dropoff ---
            self.pending_tasks.pop(0)
            self.mission_log.append(f"Planning route to dropoff at {dropoff_loc}.")
            path_to_dropoff, nodes = self._find_route(self.pos, dropoff_loc)
            self.mission_log.append(f"Planner checked {nodes} nodes for dropoff.")
            if not path_to_dropoff:
                self.mission_log.append("Failed to find path to destination.")
                return False
            if not self._follow_path(path_to_dropoff, dropoff_loc):
                return False
            self.mission_log.append(f"Package delivered to {dropoff_loc}!")
            self.completed_tasks.append((pickup_loc, dropoff_loc))

        is_success = not self.pending_tasks
        self.mission_log.append(f"Delivery cycle finished. Success: {is_success}")
        self.mission_log.append(f"Delivered: {len(self.completed_tasks)}, Energy: {self.energy}, Time: {self.time}")
        return is_success

    def _follow_path(self, path: List[Tuple[int, int]], final_dest: Tuple[int, int]) -> bool:
        """Moves the bot along a given path, handling energy, time, and obstacles."""
        if not path: return False

        for next_pos in path:
            if self.energy <= 0 or self.time >= self.grid.max_time:
                self.mission_log.append("Mission aborted due to lack of energy or time.")
                return False
            
            cost = self.grid.tiles[next_pos[0]][next_pos[1]].get_traversal_cost(self.time)
            
            # Dynamic obstacle detected, requires re-planning
            if cost == float('inf'):
                self.mission_log.append(f"Obstacle at {next_pos}! Re-planning...")
                new_path, nodes = self._find_route(self.pos, final_dest)
                self.mission_log.append(f"Re-plan checked {nodes} nodes.")
                if not new_path:
                    self.mission_log.append("No alternative route found. Mission failed.")
                    return False
                return self._follow_path(new_path, final_dest) # Recursive call with new path
            
            self.energy -= cost
            self.time += 1
            self.pos = next_pos
            
            # Update GUI if available
            if self.on_update:
                self.on_update(self)
                time.sleep(0.3)
            
            if self.time % 5 == 0:
                self.mission_log.append(f"T={self.time}: At {next_pos}, E={self.energy}")
        return True

# The graphical user interface for running and visualizing the simulation.
class SimulationGUI:
    """Provides the Tkinter GUI for controlling and viewing the simulation."""
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Delivery Bot Simulation")
        self.root.geometry("1000x700")
        self.grid_env: Optional[CourierGrid] = None
        self.bot: Optional[DeliveryBot] = None
        self.is_simulating = False
        self.total_task_count = 0
        self._setup_widgets()

    def _setup_widgets(self):
        """Creates and arranges all the GUI components."""
        top_frame = ttk.Frame(self.root, padding="10")
        top_frame.grid(row=0, column=0, sticky="nsew")

        self._create_control_panel(top_frame)
        self._create_grid_display(top_frame)
        self._create_log_panel(top_frame)

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        top_frame.columnconfigure(0, weight=3)
        top_frame.columnconfigure(1, weight=1)
        top_frame.rowconfigure(1, weight=1)
        
    def _create_control_panel(self, parent):
        settings_frame = ttk.LabelFrame(parent, text="Configuration", padding="5")
        settings_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)

        ttk.Label(settings_frame, text="Map:").grid(row=0, column=0, sticky=tk.W)
        self.map_var = tk.StringVar(value="small.map")
        map_cb = ttk.Combobox(settings_frame, textvariable=self.map_var, values=["small.map", "medium.map", "large.map", "dynamic.map"], width=15)
        map_cb.grid(row=0, column=1, padx=5)

        ttk.Label(settings_frame, text="Algorithm:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.strategy_var = tk.StringVar(value="astar")
        strategy_cb = ttk.Combobox(settings_frame, textvariable=self.strategy_var, values=["bfs", "ucs", "astar", "hillclimb", "simanneal"], width=15)
        strategy_cb.grid(row=0, column=3, padx=5)

        ttk.Button(settings_frame, text="Run", command=self.run_simulation).grid(row=0, column=4, padx=10)
        ttk.Button(settings_frame, text="Compare All", command=self.run_batch_comparison).grid(row=0, column=5, padx=10)
        ttk.Button(settings_frame, text="Clear", command=self.reset_simulation).grid(row=0, column=6, padx=10)

    def _create_grid_display(self, parent):
        grid_frame = ttk.LabelFrame(parent, text="Live Map", padding="5")
        grid_frame.grid(row=1, column=0, sticky="nsew", pady=5)
        self.canvas = tk.Canvas(grid_frame, width=600, height=600, bg="ivory")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        grid_frame.columnconfigure(0, weight=1)
        grid_frame.rowconfigure(0, weight=1)

    def _create_log_panel(self, parent):
        log_frame = ttk.LabelFrame(parent, text="Mission Log", padding="5")
        log_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        self.log_display = tk.Text(log_frame, width=40, height=35, wrap=tk.WORD)
        self.log_display.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_display.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_display.configure(yscrollcommand=scrollbar.set)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)

    def _update_gui_callback(self, bot_state: 'DeliveryBot'):
        """The function passed to the bot to update the GUI from its thread."""
        self.log_display.delete(1.0, tk.END)
        for entry in bot_state.mission_log[-20:]:
            self.log_display.insert(tk.END, entry + "\n")
        self.log_display.see(tk.END)
        self._draw_grid(bot_state)
        self.root.update_idletasks()
        self.root.update()

    def _draw_grid(self, bot_state: 'DeliveryBot'):
        """Renders the current state of the grid on the canvas."""
        self.canvas.delete("all")
        if not self.grid_env: return

        cell_dim = min(600 // self.grid_env.width, 600 // self.grid_env.height)
        for r in range(self.grid_env.height):
            for c in range(self.grid_env.width):
                x1, y1 = c * cell_dim, r * cell_dim
                x2, y2 = x1 + cell_dim, y1 + cell_dim
                
                tile = self.grid_env.tiles[r][c]
                color = "black" if tile.is_blocked else "darkseagreen" if tile.move_difficulty > 1 else "ivory"
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                
                if tile.move_difficulty > 1:
                    self.canvas.create_text(x1 + cell_dim/2, y1 + cell_dim/2, text=str(tile.move_difficulty))

        for pickup, dropoff in self.grid_env.tasks + bot_state.completed_tasks:
            px, py = pickup[1] * cell_dim + cell_dim/2, pickup[0] * cell_dim + cell_dim/2
            self.canvas.create_oval(px-5, py-5, px+5, py+5, fill="dodgerblue", outline="dodgerblue")
            dx, dy = dropoff[1] * cell_dim + cell_dim/2, dropoff[0] * cell_dim + cell_dim/2
            self.canvas.create_rectangle(dx-5, dy-5, dx+5, dy+5, fill="crimson", outline="crimson")

        bot_x, bot_y = bot_state.pos[1] * cell_dim + cell_dim/2, bot_state.pos[0] * cell_dim + cell_dim/2
        self.canvas.create_oval(bot_x-8, bot_y-8, bot_x+8, bot_y+8, fill="gold", outline="black")

        stats = f"E: {bot_state.energy} | T: {bot_state.time} | Done: {len(bot_state.completed_tasks)}/{self.total_task_count}"
        self.canvas.create_text(300, 10, text=stats, anchor=tk.N, fill="black", font=("Helvetica", 10))

    def run_simulation(self):
        """Initializes and runs a single simulation in a new thread."""
        if self.is_simulating: return
        self.is_simulating = True
        self.log_display.delete(1.0, tk.END)
        
        self.grid_env = CourierGrid()
        self.grid_env.load_map_from_spec(self.map_var.get())
        
        # Special handling for the dynamic map to introduce a change
        if self.map_var.get() == "dynamic.map":
            def trigger_event():
                if self.bot and self.bot.time >= 20:
                    self.grid_env.tiles[4][4].is_blocked = True
                    if self.bot.on_update: self.bot.on_update(self.bot)
            self.grid_env.tiles[4][4].is_blocked = False
            self.root.after(100, trigger_event) # Check for event trigger periodically
        
        self.total_task_count = len(self.grid_env.tasks)
        self.bot = DeliveryBot(self.grid_env, self.strategy_var.get(), self._update_gui_callback)

        def simulation_task():
            try:
                start_t = time.perf_counter()
                success = self.bot.start_delivery_cycle()
                end_t = time.perf_counter()
                
                summary = (f"\nStrategy: {self.strategy_var.get()}\n"
                           f"Success: {success}\n"
                           f"Delivered: {len(self.bot.completed_tasks)}\n"
                           f"Energy Left: {self.bot.energy}\n"
                           f"Time Taken: {self.bot.time}\n"
                           f"Compute Time: {end_t - start_t:.6f}s\n")
                           
                self.log_display.insert(tk.END, summary)
                self.log_display.see(tk.END)
            except Exception as e:
                messagebox.showerror("Simulation Error", str(e))
            finally:
                self.is_simulating = False
        
        threading.Thread(target=simulation_task, daemon=True).start()

    def run_batch_comparison(self):
        """Runs all algorithms on all static maps and displays a comparison table."""
        if self.is_simulating: return
        self.is_simulating = True
        self.log_display.delete(1.0, tk.END)
        self.log_display.insert(tk.END, "Starting batch comparison...\n")
        
        maps = ["small.map", "medium.map", "large.map"]
        strategies = ['bfs', 'ucs', 'astar', 'hillclimb', 'simanneal']
        
        def comparison_task():
            results = []
            try:
                for map_name in maps:
                    for strat in strategies:
                        self.log_display.insert(tk.END, f"\nTesting '{strat}' on '{map_name}'...")
                        self.log_display.see(tk.END)
                        
                        grid = CourierGrid()
                        grid.load_map_from_spec(map_name)
                        
                        start_t = time.perf_counter()
                        bot = DeliveryBot(grid, strat)
                        success = bot.start_delivery_cycle()
                        end_t = time.perf_counter()
                        
                        results.append({
                            'map': map_name, 'strat': strat, 'success': success,
                            'delivered': len(bot.completed_tasks), 'energy': bot.energy,
                            'time': bot.time, 'compute': end_t - start_t
                        })
                
                # Format and display results
                summary = "\n" + "="*80 + "\nCOMPARISON SUMMARY\n" + "="*80 + "\n"
                header = f"{'Map':<15}{'Strategy':<15}{'Success':<10}{'Done':<8}{'Energy':<10}{'Time':<10}{'Compute (s)':<15}\n"
                summary += header
                for res in results:
                    line = (f"{res['map']:<15}{res['strat']:<15}{str(res['success']):<10}"
                            f"{res['delivered']:<8}{res['energy']:<10}{res['time']:<10}"
                            f"{res['compute']:.6f}\n")
                    summary += line
                self.log_display.insert(tk.END, summary)
                self.log_display.see(tk.END)
            except Exception as e:
                messagebox.showerror("Comparison Error", str(e))
            finally:
                self.is_simulating = False
        
        threading.Thread(target=comparison_task, daemon=True).start()

    def reset_simulation(self):
        """Resets the GUI and simulation state."""
        if self.is_simulating:
            messagebox.showwarning("Reset", "Cannot reset while a simulation is running.")
            return
        self.grid_env = None
        self.bot = None
        self.canvas.delete("all")
        self.log_display.delete(1.0, tk.END)
        messagebox.showinfo("Reset", "Environment has been cleared.")

def main():
    """Application entry point."""
    app_root = tk.Tk()
    gui = SimulationGUI(app_root)
    app_root.mainloop()

if __name__ == '__main__':
    main()
