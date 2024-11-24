#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

//this code was taken from chatgpt


//we need a predermined goal location to calculate the heuristic
int calculate_heuristic(int x, int y, int goal_x, int goal_y, int max_dist) {
    // Raw Manhattan distance
    int raw_heuristic = abs(x - goal_x) + abs(y - goal_y);
    // Scale to range [0, 7]
    return std::round(raw_heuristic * 7.0 / max_dist);
}


int main() {
    const uint8_t rows = 20, 
    cols = 20, 
    max_distance = rows + cols - 2, 
    goal_x = 19, 
    goal_y = 19; 

    // Open the binary file
    std::ifstream inputFile("maze.bin", std::ios::binary);
    if (!inputFile) {
        std::cerr << "Error: Could not open the binary file." << std::endl;
        return 1;
    }

    // Read the binary file into a 2D vector
    std::vector<std::vector<uint8_t> > maze(rows, std::vector<uint8_t>(cols));
    for (int i = 0; i < rows; ++i) {
        inputFile.read(reinterpret_cast<char*>(maze[i].data()), cols * sizeof(uint8_t));
    }

    inputFile.close();

    // Print the maze
    std::cout << "Maze with Heuristics:" << std::endl;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (maze[y][x] == 0){
                int heuristic = calculate_heuristic(x, y, goal_x, goal_y, max_distance);
                maze[y][x] = (heuristic << 1) | 0; // Store the heuristic in the maze
            }
        }
    }
    std::cout << "Maze with Encoded Heuristics:" << std::endl;

for (int y = 0; y < rows; ++y) {
    std::cout<< "|";
    for (int x = 0; x < cols; ++x) {
        if (maze[y][x] & 1) { 
            std::cout << " W ";
        } else {
            // Path: Extract the heuristic using &0b1110
            //int heuristic = (maze[y][x] >>1); 
            //std::cout << heuristic << " ";
            std::cout << "   ";
        }
    }
    std::cout<< "|" << std::endl; // Move to the next row
}


    return 0;
}
