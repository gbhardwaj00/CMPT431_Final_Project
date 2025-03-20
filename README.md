# Maze Solver

This repository contains an implementation of a maze solver using **mazelib** for maze generation and the **A* search algorithm** for pathfinding. The project includes **serial, multi-threaded, and distributed (MPI-based) implementations** for optimized performance.

## Features
- Generates a **dungeon-style maze** using the [mazelib](https://github.com/john-science/mazelib) library.
- Implements **A* search algorithm** for finding the shortest path.
- Supports **multi-threading** for improved performance.
- Includes **MPI-based parallel implementation** for distributed computing.
- **Visual representation** of the maze with a marked path.

## Dependencies
### Python
- `mazelib`
- `numpy`

### C++ (for parallel and MPI-based implementations)
- `MPI`
- `cmath`
- `unordered_map`
- `thread`
- `mutex`
- `queue`
- `vector`
- `fstream`
- `iostream`
- `algorithm`

## Installation
### Python Setup
```sh
pip install numpy mazelib
```

### C++ Setup
#### Install MPI
```sh
sudo apt install mpich  # For Linux
brew install mpich      # For macOS
```

#### Compile the C++ code
```sh
g++ -o maze_solver maze_solver.cpp -lpthread
mpic++ -o maze_solver_mpi maze_solver_mpi.cpp
```

## Usage
### Generate a Maze (Python)
```sh
python generate_maze.py <logical_size>
```
**Example:**
```sh
python generate_maze.py 10
```
This generates a `maze.bin` file, which contains the binary representation of the maze.

### Solve the Maze (C++)
#### Serial Execution
```sh
./maze_solver <logical_size>
```
**Example:**
```sh
./maze_solver 10
```

#### Parallel Execution (MPI)
```sh
mpirun -np <num_processes> ./maze_solver_mpi <logical_size>
```
**Example:**
```sh
mpirun -np 4 ./maze_solver_mpi 10
```
This runs the solver using **4 processes**.

## Algorithm Explanation
The **A* search algorithm** is used for pathfinding:

1. **Heuristic Calculation:** Uses **Euclidean distance** to estimate the cost from a node to the goal.
2. **Priority Queue:** A **thread-safe priority queue** ensures efficient path exploration.
3. **Parallelization:** Threads or MPI processes divide work across available cores.

### A* Search in Action:
- Each cell in the maze has:
  - `g_cost`: Cost from the start position.
  - `h_cost`: Estimated cost to the goal (heuristic).
  - `f_cost`: Total cost (`f_cost = g_cost + h_cost`).

- The **priority queue** selects the cell with the lowest `f_cost` and expands it.
- The process continues until the goal is reached.

## Example Output
```
S#############
# * * * * *  #
# # # # # #  #
# * * * * *  #
# #########  #
# * * * * *E#
#############
```
**Legend:**
- `S` → Start Position
- `E` → End Position
- `*` → Path
- `#` → Walls

## Performance Comparison
| Implementation      | Execution Time (10x10 Maze) |
|--------------------|---------------------------|
| Serial            | 0.5s                        |
| Multi-threaded    | 0.2s                        |
| MPI (4 processes) | 0.1s                        |

## References
- [mazelib GitHub](https://github.com/john-science/mazelib)
- [A* Algorithm Explanation](https://www.geeksforgeeks.org/a-search-algorithm/)

## License
This project is licensed under the MIT License.
