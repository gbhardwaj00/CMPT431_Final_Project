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
                                        const pair<uint16_t, uint16_t>& goal) {
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
        if (current.x == goal.first && current.y == goal.second) {
            vector<pair<int, int>> path;
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
    // Logical size of the maze (excluding in-between wall positions)
    uint16_t logical_size = stoi(argv[1]); 
    const uint16_t rows = 2 * logical_size + 1;
    const uint16_t cols = 2 * logical_size + 1; 

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

    // Assigned start and goal positions
    pair<int, int> start = {1, 1}; 
    pair<int, int> goal = {2 * logical_size - 1, 2 * logical_size - 1}; // Goal is the bottom-right cell
    
    timer serial_timer;
    double time_taken = 0.0;
    serial_timer.start();
    auto path = a_star(maze, start, goal);
    time_taken = serial_timer.stop();
    cout << "Time taken: " << fixed << setprecision(TIME_PRECISION) << time_taken << " seconds" << endl;

    if (!path.empty()) {
        cout << "Path taken: ";
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ") ";
        }
        cout << endl;

        // Render the maze with the path
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                if (make_pair(y, x) == start) {
                    cout << "S"; // Start
                } else if (make_pair(y, x) == goal) {
                    cout << "E"; // Goal
                } else if (find(path.begin(), path.end(), make_pair(y, x)) != path.end()) {
                    cout << "*"; // Mark path
                } else if (maze[y][x] & 1) {
                    cout << "#"; // Wall
                } else {
                    cout << " "; // Empty path
                }
            }
            cout << endl;
        }
    } else {
        cout << "No path found!" << endl;
    }
    return 0;
}