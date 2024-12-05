//https://www.geeksforgeeks.org/a-search-algorithm/
//this link is where we took the basic functioning idea for the serial A* implementation of the algorithm

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <mpi.h>
#include <cstdint>
#include <limits>

using namespace std;

#define TIME_PRECISION 10

struct Cell
{
    uint16_t x,y;
    int g_cost, f_cost;
    Cell() : x(0), y(0), g_cost(0), f_cost(0) {}
    Cell(uint16_t x, uint16_t y, int g_cost, int f_cost)
        : x(x), y(y), g_cost(g_cost), f_cost(f_cost) {}
    //overloadded_operator to compare the f_cost cells
    bool operator<(const Cell& rhs) const {
        return f_cost > rhs.f_cost;
    }
};

// Using the Euclidean distance as the heuristic
int calculate_heuristic(int x1, int y1, int x2, int y2) {
    return static_cast<int>(sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}

void master_process(vector<vector<uint16_t>>& maze, const Cell& start, const Cell& goal, int num_processes) {
    const int rows = maze.size();
    const int cols = maze[0].size();

    priority_queue<Cell> open_list;
    unordered_map<int, int> g_cost_map;
    unordered_map<int, pair<int, int>> parent_map;

    // Initialize the open list with start cell
    open_list.push(start);
    g_cost_map[start.x * cols + start.y] = start.g_cost;
    parent_map[start.x * cols + start.y] = {-1, -1};

    // Directions for neighbor cells
    const vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};


    bool goal_found = false;
    // To store goal cell when found
    Cell goal_cell(0, 0, 0, 0);

    // Main Loop
    while (!open_list.empty() && !goal_found) {
        // Get cell with lowest f_cost
        Cell current = open_list.top();
        open_list.pop();

        int current_index = current.x * cols + current.y;

        if (current.x == goal.x && current.y == goal.y) {
            goal_found = true;
            goal_cell = current;
            break;
        }

        // Send the current cell to all worker processes
        for (int proc = 1; proc < num_processes; ++proc) {
            int current_data[5] = {current.x, current.y, current.g_cost, current.f_cost, goal.x * cols + goal.y};
            MPI_Send(current_data, 5, MPI_INT, proc, 0, MPI_COMM_WORLD);
        }

        // Send the current cell to all worker processes
        for (int proc = 1; proc < num_processes; proc++) {
            int num_neighbours;
            MPI_Recv(&num_neighbours, 1, MPI_INT, proc, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            // Receive data for each neighbout
            for (int i = 0; i < num_neighbours; i++) {
                int neighbour_data[4];
                MPI_Recv(neighbour_data, 4, MPI_INT, proc, 0,  MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                int nx = neighbour_data[0];
                int ny = neighbour_data[1];
                int ng_cost = neighbour_data[2];
                int nf_cost = neighbour_data[3];
                int neighbour_index = nx * cols + ny;

                // If neighbour has a better g_cost or is not in g_cost map
                if (g_cost_map.find(neighbour_index) == g_cost_map.end() || ng_cost < g_cost_map[neighbour_index]) {
                    g_cost_map[neighbour_index] = ng_cost;
                    open_list.push(Cell(nx, ny, ng_cost, nf_cost));
                    parent_map[neighbour_index] = {current.x, current.y};
                }
            }
        }
    }
    // Inform worker processes to terminate
    for (int proc = 1; proc < num_processes; ++proc) {
        int terminate_signal[1] = {-1};
        MPI_Send(terminate_signal, 1, MPI_INT, proc, 0, MPI_COMM_WORLD);
    }

    // Reconstruct the path if goal is found
    if (goal_found) {
        vector<pair<int, int>> path;
        int idx = goal_cell.x * cols + goal_cell.y;
        while (idx != -1) {
            int x = idx / cols;
            int y = idx % cols;
            path.push_back({x, y});
            auto parent = parent_map[idx];
            idx = parent.first == -1 ? -1 : parent.first * cols + parent.second;
        }
        reverse(path.begin(), path.end());

        // Output the path
        cout << "Path found:\n";
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found!" << endl;
    }
}

void worker_process(vector<vector<uint16_t>>& maze) {
    const int rows = maze.size();
    const int cols = maze[0].size();

    const vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (true) {
        // Receive the current cell from the master
        int data[5];
        MPI_Recv(data, 5, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

        // Check for termination signal
        if (data[0] == -1) {
            break;
        }

        Cell current(data[0], data[1], data[2], data[3]);
        int goal_index = data[4];
        int goal_x = goal_index / cols;
        int goal_y = goal_index % cols;

        vector<Cell> neighbors;

        // Explore neighbors
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // Check bounds
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols) {
                // Check if the cell is not a wall
                if (maze[nx][ny] != 1) {
                    int ng_cost = current.g_cost + 1;
                    int nh_cost = calculate_heuristic(nx, ny, goal_x, goal_y);
                    int nf_cost = ng_cost + nh_cost;

                    neighbors.emplace_back(nx, ny, ng_cost, nf_cost);
                }
            }
        }

        // Send the number of neighbors to the master
        int num_neighbors = neighbors.size();
        MPI_Send(&num_neighbors, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);

        // Send each neighbor's data to the master
        for (const auto& neighbor : neighbors) {
            int neighbor_data[4] = {neighbor.x, neighbor.y, neighbor.g_cost, neighbor.f_cost};
            MPI_Send(neighbor_data, 4, MPI_INT, 0, 0, MPI_COMM_WORLD);
        }
    }
}


int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);
    
    int rank, num_processes;
    int size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    
    if (argc != 2) {
        if (rank == 0) {
            cerr << "Usage: " << argv[0] << " <logical_size>" << endl;
        }
        MPI_Finalize();
        return 1;
    }

    uint16_t logical_size = stoi(argv[1]); // Logical size of the maze
    const uint16_t rows = 2 * logical_size + 1;
    const uint16_t cols = 2 * logical_size + 1; 
    
    vector<vector<uint16_t>> maze(rows, vector<uint16_t>(cols));
    
    if (rank == 0) {
        // Open the binary file
        ifstream inputFile("maze.bin", ios::binary);
        if (!inputFile) {
            cerr << "Error: Could not open the binary file." << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
        for (int i = 0; i < rows; i++) {
            inputFile.read(reinterpret_cast<char*>(maze[i].data()), cols * sizeof(uint16_t));
        }
        inputFile.close();
    }
    
    // Broadcast the maze to all processes
    for (int i = 0; i < rows; ++i) {
        MPI_Bcast(maze[i].data(), cols, MPI_UINT16_T, 0, MPI_COMM_WORLD);
    }

    if (rank == 0) {
        // Master process
        Cell start(1, 1, 0, calculate_heuristic(1, 1, rows - 2, cols - 2));
        Cell goal(rows - 2, cols - 2, 0, 0);
        double start_time = MPI_Wtime();
        master_process(maze, start, goal, num_processes);
        double end_time = MPI_Wtime();
        cout << "Time taken: " << fixed << end_time - start_time << " seconds" << endl;
    } else {
        // Worker processes
        worker_process(maze);
    }

    MPI_Finalize();
    return 0;
}