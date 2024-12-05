# Using mazelib API to generate a maze for our inputs
# Source : https://github.com/john-science/mazelib
from mazelib import Maze
from mazelib.solve.BacktrackingSolver import BacktrackingSolver
from mazelib.generate.DungeonRooms import DungeonRooms
import argparse
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate a dungeon maze and print its grid size.")
    parser.add_argument("size", type=int, help="Logical size of the maze (n x n).")

    # Parse the command line arguments
    args = parser.parse_args()

     # Validate the maze size argument
    if args.size <= 0:
        parser.error("The size of the maze must be a positive integer greater than 0.")

    # The actual size of the maze is 2 * logical size + 1 because it includes positions for the walls
    # The logic used will take care of that so that the maze is generated correctly
    logical_size = args.size
    
    m = Maze()
    # Generate a maze using the DungeonRooms algorithm with size logical_size x logical_size
    m.generator = DungeonRooms(logical_size, logical_size)
    m.generate()    

    # Comment out the line to print the maze
    # print(m.tostring(True))

    # Comment out the lines to solve the maze using mazelib library and print the solution
    # m.solver = BacktrackingSolver()
    # m.start = (1, 1)
    # m.end = (2 * logical_size - 1, 2 * logical_size - 1)
    # m.solve()
    # print(m.tostring(True, True))

    # Using numpy to convert the maze grid to a 2D numpy array
    maze = np.array(m.grid, dtype=np.uint16)

    # Save the maze to a binary file
    maze.tofile('maze.bin')
