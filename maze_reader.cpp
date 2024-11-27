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
    int f_cost;
    Cell() : x(0), y(0), cost(0), f_cost(0) {}
    Cell(uint8_t x, uint8_t y, int cost, int f_cost)
        : x(x), y(y), cost(cost), f_cost(f_cost) {}
    //overloadded_operator to compare the f_cost cells
    bool operator<(const Cell& rhs) const
    {
        return f_cost > rhs.f_cost;
    }
};
//we need a predermined goal location to calculate the heuristic
int calculate_heuristic(int x, int y, int goal_x, int goal_y, int max_dist) {
    int raw_heuristic = abs(x - goal_x) + abs(y - goal_y);   
    return std::round(raw_heuristic * 7.0 / max_dist);     //scaling in the range of 0 to 7
}

int extract_heurestic(uint8_t cell_value)
{
    if(cell_value != 1)
    {
        return (cell_value & 0b00001110) >> 1;
    }
    return -1; 
}

std::vector<std::pair<int, int>> a_star(std::vector<std::vector<uint8_t>>& maze,
                                        const std::pair<uint8_t, uint8_t>& start,
                                        const std::pair<uint8_t, uint8_t>& goal) {
    const uint8_t VISITED_FLAG = 0b10000000; // Bit 7 it is used to mark the cell as vistied or not
    const int rows = maze[0].size();
    const int cols = maze.size();
    std::priority_queue<Cell> open_list;
    std::unordered_map<int, std::pair<int, int>> parent;                    //path reconstruction

    
    open_list.push({start.first, start.second, 0, extract_heurestic(maze[start.first][start.second])});
    parent [start.first * cols + start.second] = {-1, -1}; //start node has no parent
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; //up, down, left, right

    while(!open_list.empty())
    {
        //get the top of the current cell array
        Cell current = open_list.top();
        open_list.pop();
        if (current.x == goal.first && current.y == goal.second) {
            std::vector<std::pair<int, int>> path;
            for (std::pair<int, int> at = {current.x, current.y};
                 at.first != -1 && at.second != -1;
                 at = parent[at.first * cols + at.second]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end()); // Reverse to get start-to-goal order
            return path;
        }

        maze[current.x][current.y] |= VISITED_FLAG; // Mark the cell as visited i.e. setting the most significant bit to 1
        //iterate over the directions vector
        for(const auto& dir : directions){
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            //check if the cell is within the bounds of the maze
            //check if the cell is not a wall
            //check if the cell is not visited
            if(nx < 0 || ny <0 || nx >= rows || ny >= cols || maze[nx][ny] == 1 || maze[nx][ny] & VISITED_FLAG)
            {
                continue;
            }
            
            int heuristic = extract_heurestic(maze[nx][ny]);
            int g_cost = current.cost + 1;
            int f_cost = g_cost + heuristic;

            Cell neighbor(nx, ny, g_cost, f_cost);  
            open_list.push(neighbor);

            parent[nx * cols + ny] = {current.x, current.y}; //store the parent of the current cell
        }

    }
    return {}; // Return an empty path if no path found
}



int main() {
    const uint8_t rows = 20+21;
    const uint8_t cols = 20+21;
    const uint8_t goal_x = 2 * 17 + 1;
    const uint8_t goal_y = 2* 15 + 1; 
    const uint8_t max_distance = rows + cols - 2;

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

    // print the maze
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            std::cout << static_cast<int>(maze[y][x]) << " ";
        }
        std::cout << std::endl;
    }

    inputFile.close();
    //calculate the heuristic for each cell in the maze
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (maze[y][x] == 0){
                int heuristic = calculate_heuristic(x, y, goal_x, goal_y, max_distance);
                maze[y][x] = (heuristic << 1) | 0; // Store the heuristic in the maze
            }
        }
    }

    std::pair<int, int> start = {2 * 0 + 1, 2 * 1 + 1}; 
    std::pair<int, int> goal = {2 * 17 + 1, 2 * 15 + 1};
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
                if (std::find(path.begin(), path.end(), std::make_pair(y, x)) != path.end()) {
                    std::cout << "*"; // Mark path
                } else if (maze[y][x] & 1) {
                    std::cout << "W"; // Wall
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