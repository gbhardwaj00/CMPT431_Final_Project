# Using mazelib API to generate a maze for our inputs
# Source : https://github.com/john-science/mazelib
from mazelib import Maze
from mazelib.solve.RandomMouse import RandomMouse
from mazelib.generate.DungeonRooms import DungeonRooms
import argparse
import numpy as np

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Generate a dungeon maze and print its grid size.")
    parser.add_argument("size", type=int, help="Logical size of the maze (n x n).")
    args = parser.parse_args()
    logical_size = args.size
    actual_size = 2 * logical_size + 1
    m = Maze()
    m.generator = DungeonRooms(logical_size, logical_size)
    m.generate()    
    maze = np.array(m.grid, dtype=np.uint16)
    print(maze)
    maze.tofile('maze.bin')
