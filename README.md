# CMPT431_Final_Project
Implementation for serial, parallel(using threads) and Distributed(using MPI) versions for a pathfinding algorithm
System Requirements.
Python==3.0.11
mazelib==0.9.16
numpy==2.1.3
C++ == g++ compiler with version 14
MPI == mpic++

The seq and parallel versions were tested on MAC devices.
Each time a new maze is generated and solved so we didn't provided with only a single test case.

In order to run generate the maze, the command line to execute the function is :-
python maze_generator.py size
[e.g python maze_generator.py 20] size can be only positive integer values, Anything above 100 takes substantial increase in time to generate a maze.
where size is the size of the whole grid, It generates a mazew of size (2*size + 1) by (2*size + 1), which is saved as maze.bin

Run the Make command on the command line to generate all the solutions and they run as follows:-
Execute make to build all the executables
./seq_maze_solver size  [e.g ./seq_maze_solver 20] size can be only positive integer values
./parallel_maze_solver size [e.g ./parallel_maze_solver 20] size can be only positive integer values
Execute make clean to clear out the executables and the maze generated.

