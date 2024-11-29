#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>




struct Cell
{
    uint8_t x,y;
    uint8_t f_cost;
    Cell() : x(0), y(0), f_cost(0) {}
    Cell(uint8_t x, uint8_t y, uint8_t f_cost)
        : x(x), y(y), f_cost(f_cost) {}
    //overloadded_operator to compare the f_cost cells
    bool operator<(const Cell& rhs) const
    {
        return f_cost > rhs.f_cost;
    }
};


std::vector<std::pair<uint8_t, uint8_t>> bfs(std::vector<std::vector<uint8_t>>& maze,
                                        const std::pair<uint8_t, uint8_t>& start,
                                        const std::pair<uint8_t, uint8_t>& goal) {
    const uint8_t VISITED_FLAG = 0b10000000; // Bit 8 it is used to mark the cell as vistied or not
    const uint8_t rows = maze[0].size();
    const uint8_t cols = maze.size();
    std::queue<std::pair<uint8_t, uint8_t>> queue;  // Queue for BFS
    std::unordered_map<uint8_t, std::pair<uint8_t, uint8_t>> parent; // Path reconstruction

    std::vector<std::pair<uint8_t, uint8_t>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    queue.push(start);
    maze[start.first][start.second] |= VISITED_FLAG;
    parent[start.first * cols + start.second] = {-1, -1};  // Start has no parent

    while(!queue.empty())
    {
        //get the top of the current cell array
        auto current = queue.front();
        queue.pop();
        if (current == goal) {
            std::vector<std::pair<uint8_t, uint8_t>> path;
            for (std::pair<uint8_t, uint8_t> at = current; at.first != -1 && at.second != -1;
                 at = parent[at.first * cols + at.second]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());  // Reverse to get start-to-goal order
            return path;
        }
        for (const auto& dir : directions) {
            uint8_t nx = current.first + dir.first;
            uint8_t ny = current.second + dir.second;

            // Check bounds, walls, and visited status
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && 
                !(maze[nx][ny] & VISITED_FLAG) && maze[nx][ny] != 1) {
                queue.push({nx, ny});
                maze[nx][ny] |= VISITED_FLAG;  // Mark as visited
                parent[nx * cols + ny] = current;  // Store parent
            }
        }
}
                return {};  
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <maze_size>" << std::endl;
        return 1;
    }
    uint8_t logical_size = std::stoi(argv[1]); // Logical size of the maze
    const uint8_t rows = 2 * logical_size + 1;
    const uint8_t cols = 2 * logical_size + 1; 

    // Open the binary file
    std::ifstream inputFile("maze.bin", std::ios::binary);
    if (!inputFile) {
        std::cerr << "Error: Could not open the binary file." << std::endl;
        return 1;
    }

    // Read the binary file uint8_to a 2D vector
    std::vector<std::vector<uint8_t>> maze(rows, std::vector<uint8_t>(cols));
    for (uint8_t i = 0; i < rows; ++i) {
        inputFile.read(reinterpret_cast<char*>(maze[i].data()), cols * sizeof(uint8_t));
    }

    inputFile.close();

    std::pair<uint8_t, uint8_t> start = {0, 1}; 
    std::pair<uint8_t, uint8_t> goal = {39, 39};
    
    auto path = bfs(maze, start, goal);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& p : path) {
            std::cout << "(" << p.first << ", " << p.second << ") ";
        }
        std::cout << std::endl;

        // Render the maze with the path
        for (uint8_t y = 0; y < rows; ++y) {
            for (uint8_t x = 0; x < cols; ++x) {
                if (std::make_pair(y, x) == start) {
                    std::cout << "S"; // Start
                } else if (std::make_pair(y, x) == goal) {
                    std::cout << "E"; // Goal
                } else if (std::find(path.begin(), path.end(), std::make_pair(y, x)) != path.end()) {
                    std::cout << "*"; // Mark path
                } else if (maze[y][x] & 1) {
                    std::cout << "#"; // Wall
                } else {
                    std::cout << " "; // Empty path
                }
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "No path found!" << std::endl;
    }
    return 0;
}