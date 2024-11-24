#this was taken from chatgpt and this api is used to create a maze
#the link is as follows
#https://github.com/john-science/mazelib/blob/main/docs/API.md
from mazelib import Maze
from mazelib.generate.DungeonRooms import DungeonRooms
from mazelib.generate.GrowingTree import GrowingTree
# Import other required generators or components as needed

import numpy as np

def add_loops(maze, num_loops):
    rows, cols = maze.shape
    for _ in range(num_loops):
        x, y = np.random.randint(1, rows - 1), np.random.randint(1, cols - 1)
        if maze[x, y] == 1:  # If it's a wall, break it to create a loop
            maze[x, y] = 0
    return maze
#
# This function was called because the first two rows of the maze were completely blocked
def randomize_first_rows(maze, num_clear_per_row):
    rows, cols = maze.shape
    for row in range(2):  # First two rows
        clear_indices = np.random.choice(cols, num_clear_per_row, replace=False)
        for col in clear_indices:
            maze[row, col] = 0  # Clear cell to create a path
    return maze


m = Maze()
m.generator = DungeonRooms(20, 20)
m.generate()
dungeon_maze = np.array(m.grid, dtype=np.uint8)  

m.generator = GrowingTree(20, 20)
m.generate()
growing_tree_maze = np.array(m.grid, dtype=np.uint8)  

final_maze = np.minimum(dungeon_maze, growing_tree_maze) 

final_maze = add_loops(final_maze, num_loops=5)  
final_maze = randomize_first_rows(final_maze, 20)

final_maze.tofile('maze.bin') 
