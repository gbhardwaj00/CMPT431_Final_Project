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
    int cost;
    Cell() : x(0), y(0), cost(0) {}
    Cell(uint8_t x, uint8_t y, int f_cost)
        : x(x), y(y), cost(f_cost) {}
    //overloadded_operator to compare the f_cost cells
    bool operator<(const Cell& rhs) const
    {
        return cost > rhs.cost;
    }
};
//we need a predermined goal location to calculate the heuristic
int calculate_heuristic(int x, int y, int goal_x, int goal_y) {
    return std::sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y));    
}


std::vector<std::pair<int, int>> a_star(std::vector<std::vector<uint8_t>>& maze,
                                        const std::pair<uint8_t, uint8_t>& start,
                                        const std::pair<uint8_t, uint8_t>& goal) {
    const uint8_t VISITED_FLAG = 0b10000000; // Bit 7 it is used to mark the cell as vistied or not
    const int rows = maze[0].size();
    const int cols = maze.size();
    std::priority_queue<Cell> open_list;
    std::unordered_map<int, std::pair<int, int>> parent;                    //path reconstruction

    
    // Push the start cell into the open list
    int start_heuristic = calculate_heuristic(start.first, start.second, goal.first, goal.second);
    open_list.push({start.first, start.second, start_heuristic});
    parent[start.first * cols + start.second] = {-1, -1}; //start node has no parent
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; //up, down, left, right

    while(!open_list.empty())
    {
        //get the top of the current cell array
        Cell current = open_list.top();
        open_list.pop();

        //exit condition
        if (current.x == goal.first && current.y == goal.second) {
            std::vector<std::pair<int, int>> path;
            for (std::pair<int, int> at = {current.x, current.y};
                 at.first != -1 && at.second != -1;
                 at = parent[at.first * cols + at.second]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());                         // Reverse to get start-to-goal order
            return path;
        }

        maze[current.x][current.y] |= VISITED_FLAG;             // Mark the cell as visited i.e. setting the most significant bit to 1
        //iterate over the directions vector
        for(const auto& dir : directions){
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;
            //check if the cell is within the bounds of the maze
            //check if the cell is not a wall
            //check if the cell is not visited
            if(new_x < 0 || new_y <0 || new_x >= rows || new_y >= cols || maze[new_x][new_y] == 1 || maze[new_x][new_y] & VISITED_FLAG)
            {
                continue;
            }
            
            int heuristic = calculate_heuristic(new_x, new_y, goal.first, goal.second);   

            Cell neighbor(new_x, new_y, heuristic + current.cost + 1); //calculate the f_cost of the neighbor cell
            open_list.push(neighbor);

            parent[new_x * cols + new_y] = {current.x, current.y}; //store the parent of the current cell
        }

    }
    return {}; // Return an empty path if no path found
}



int main(int argc, char* argv[]) {
    uint8_t logical_size = std::stoi(argv[1]); // Logical size of the maze
    const uint8_t rows = 2 * logical_size + 1;
    const uint8_t cols = 2 * logical_size + 1; 

    // Open the binary file
    std::ifstream inputFile("maze.bin", std::ios::binary);
    if (!inputFile) {
        std::cerr << "Error: Could not open the binary file." << std::endl;
        return 1;
    }

    // Read the binary file into a 2D vector
    std::vector<std::vector<uint8_t>> maze(rows, std::vector<uint8_t>(cols));
    for (int i = 0; i < rows; ++i) {
        inputFile.read(reinterpret_cast<char*>(maze[i].data()), cols * sizeof(uint8_t));
    }

    inputFile.close();
    
    std::pair<int, int> goal = {1, 1}; //hardcoded start
    std::pair<int, int> start = {79, 79}; //hardcoded end
    auto path = a_star(maze, start, goal);
    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (const auto& p : path) {
            std::cout << "(" << p.first << ", " << p.second << ") ";
        }
        std::cout << std::endl;

        // Render the maze with the path
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
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