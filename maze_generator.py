# Using mazelib API to generate a maze for our inputs
# Source : https://github.com/john-science/mazelib
from mazelib import Maze
from mazelib.solve.RandomMouse import RandomMouse
from mazelib.generate.DungeonRooms import DungeonRooms

import numpy as np

if __name__ == '__main__':
    m = Maze()
    m.generator = DungeonRooms(20, 20)
    m.generate()

    # print("Maze")
    # print(m.tostring())

    m.solver = RandomMouse()
    m.start = (0, 1)
    m.end = (17, 15)
    # print("Maze")
    # print(m.tostring(True))
    
    m.solve()
    
    print("Solution")
    print(m.tostring(True, True))
    
    maze = np.array(m.grid, dtype=np.uint8)
    # print(maze)
    maze.tofile('maze.bin')
