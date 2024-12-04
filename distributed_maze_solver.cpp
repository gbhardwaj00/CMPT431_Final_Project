//https://www.geeksforgeeks.org/a-search-algorithm/
//this link is where we took the basic functioning idea for the serial A* implementation of the algorithm

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include "core/get_time.h"
#include <mpi>
using namespace std;

#define TIME_PRECISION 10

pair <int,int> = st;

struct Cell
{
    uint16_t x,y;
    int g_cost, f_cost;
    Cell() : x(0), y(0), g_cost(0), f_cost(0) {}
    Cell(uint16_t x, uint16_t y, int g_cost, int f_cost)
        : x(x), y(y), g_cost(g_cost), f_cost(f_cost) {}
    //overloadded_operator to compare the f_cost cells
    bool operator<(const Cell& rhs) const
    {
        return f_cost > rhs.f_cost;
    }
};
// Using the Euclidean distance as the heuristic
int calculate_heuristic(int x1, int y1, int x2, int y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> a_star(vector<vector<uint16_t>>& maze,
                                        const pair<uint16_t, uint16_t>& start,
                                        const pair<uint16_t, uint16_t>& goal, const int end_row) {
    const uint16_t VISITED_FLAG = 0b1000000000000000; // Bit 16 it is used to mark the cell as vistied or not
    const int rows = maze.size();
    const int cols = maze[0].size();
    priority_queue<Cell> open_list;
    unordered_map<int, pair<int, int>> parent; // Will help to reconstruct the path
    unordered_map<int, int> g_cost_map; // Cost from start to current cell
    
    // Push the start cell into the open list
    int start_heuristic = calculate_heuristic(start.first, start.second, goal.first, goal.second);
    open_list.push({start.first, start.second, 0, start_heuristic});
    parent[start.first * cols + start.second] = {-1, -1}; //start node has no parent
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; //up, down, left, right

    while(!open_list.empty()) {
        //get the top of the current cell array
        Cell current = open_list.top();
        open_list.pop();
        
        // If the current cell is the goal cell, reconstruct the path
        if ((current.x == goal.first && current.y == goal.second) || (current.x == end_row)) {
            vector<pair<int, int>> path;
            st = current;
            for (pair<int, int> at = {current.x, current.y};
                 at.first != -1 && at.second != -1;
                 at = parent[at.first * cols + at.second]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end()); // Reverse to get start-to-goal order
            return path;
        }

        maze[current.x][current.y] |= VISITED_FLAG; // Mark the cell as visited i.e. setting the most significant bit to 1
        // Check the neighbors of the current cell
        for(const auto& dir : directions){
            uint16_t nx = current.x + dir.first;
            uint16_t ny = current.y + dir.second;
            // Check if the cell is within the bounds, is not a wall, and is not visited
            if(nx < 0 || ny <0 || nx >= rows || ny >= cols || maze[nx][ny] == 1 || maze[nx][ny] & VISITED_FLAG) {
                continue;
            }
            int new_g_cost = current.g_cost + 1; // Cost to move to the neighbor cell
            int h_cost = calculate_heuristic(nx, ny, goal.first, goal.second);
            int new_f_cost = new_g_cost + h_cost; // f_cost = g_cost + h_cost
            
            // If the new path to the neighbor is shorter or the neighbor is not in the open list
            int neighbor_index = nx * cols + ny;
            if (g_cost_map.find(neighbor_index) == g_cost_map.end() || new_g_cost < g_cost_map[neighbor_index]) {
                g_cost_map[neighbor_index] = new_g_cost;
                open_list.push({nx, ny, new_g_cost, new_f_cost});
                parent[neighbor_index] = {current.x, current.y}; // Store the parent of the current cell
            }

        }
    }
    return {}; // Return an empty path if no path found
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);
    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    uint16_t logical_size = stoi(argv[1]); // Logical size of the maze
    const uint16_t rows = 2 * logical_size + 1;
    const uint16_t cols = 2 * logical_size + 1; 
    int rows_per_process = rows / size;
    int extra_rows = rows % size;   
    int end_row = rank == size - 1 ? rows - 1 : (rank + 1) * rows_per_process - 1;
    // Open the binary file
    ifstream inputFile("maze.bin", ios::binary);
    if (!inputFile) {
        cerr << "Error: Could not open the binary file." << endl;
        return 1;
    }

    // Read the binary file into a 2D vector
    vector<vector<uint16_t>> maze(rows, vector<uint16_t>(cols));
    for (int i = 0; i < rows; ++i) {
        inputFile.read(reinterpret_cast<char*>(maze[i].data()), cols * sizeof(uint16_t));
    }
    inputFile.close();
    if(rank == 0)
   { 
    pair<int, int> start = {1, 1}; 
    pair<int, int> goal = {rows - 2, cols - 2}; // Goal is the bottom-right cell
   }
   else
   {
    pair<int,int> goal = {rows - 2, cols - 2};
   }
    timer serial_timer;
    double time_taken = 0.0;
    serial_timer.start();
    if(rank ==  0)
    {auto path = a_star(maze, start, goal, end_row);
   if (!path.empty()) {
    printf("Path found:\n");
    for (const auto& p : path) {
        printf("(%d, %d) ", p.first, p.second);
    }
    printf("\n");
} else {
    printf("No path found!\n");
}
    MPI_SEND(&st, 2, MPI_INT, rank + 1, 0, MPI_COMM_WORLD);
    }
    else if(rank == size - 1)
    {
        pair <int,int> start;
        MPI_RECV(&start, 2, MPI_INT, rank - 1, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        auto path = a_star(maze, start, goal, end_row);
        if (!path.empty()) {
    printf("Path found:\n");
    for (const auto& p : path) {
        printf("(%d, %d) ", p.first, p.second);
    }
    printf("\n");
} else {
    printf("No path found!\n");
}
        MPI_SEND(&path, 2, MPI_INT, 0, 0, MPI_COMM_WORLD);
    }
    else
    {
        pair <int,int> start;
        //receive the starting point from the previous process
        MPI_RECV(&start, 2, MPI_INT, rank - 1, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        auto path = a_star(maze, start, goal, end_row);
       if (!path.empty()) {
    printf("Path found:\n");
    for (const auto& p : path) {
        printf("(%d, %d) ", p.first, p.second);
    }
    printf("\n");
} else {
    printf("No path found!\n");
}
        MPI_SEND(&st, 2, MPI_INT, rank + 1, 0, MPI_COMM_WORLD);
        //send your ending point to the next process.
    }
    time_taken = serial_timer.stop();
   printf("Time taken: %.*g seconds\n", TIME_PRECISION, time_taken);

    
    MPI_Finalize();
    return 0;
}