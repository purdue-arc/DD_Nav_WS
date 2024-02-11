#Variables:

#Executables:
PLAN_EXEC = plan

#Source Files:
SRC_FILES = src/*.cpp # Wildcard used to get every file with .cpp extension in the src folder
UTIL_FILES = util/*.cpp
MAPS_FILES = maps2/*.cpp
OCPNCY_FILES = ocpncy/*.cpp
Header = header.hh
Dijkstra = dijkstra.cpp
AStar = a_star.cpp
DStar = d_star.cpp
Node = node.cpp
Node_Map = node_map.cpp
Maze_Gen = random_maze_generator.cpp
PQue = priority_queue.cpp

#Test Files:
AStar_Test = tests/a_star_tests.cpp

#System:
CC = g++
CFLAGS = -O3 -std=c++2b -Wall -Wextra -Wshadow

#
#----------------------
#

#RULES:
$(PLAN_EXEC) : $(SRC_FILES) $(UTIL_FILES) $(Header) $(AStar_Test)
	$(CC) $(SRC_FILES) $(UTIL_FILES) $(AStar_Test) $(CFLAGS) -o $(PLAN_EXEC)

test_AStar: $(PLAN_EXEC) 
	./$(PLAN_EXEC) -a*

test_DStar: $(PLAN_EXEC)
	./$(PLAN_EXEC) -d*

clean: rm -f *.pbm

#Telling the Makefile which rules are not named after files
.PHONY: test_AStar test_DStar clean
