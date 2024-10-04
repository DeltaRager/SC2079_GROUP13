import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
from math import sqrt
from collections import deque
import itertools

class Algorithm:

    START_SIZE = 4 # 4x4 cells [40x40]
    ROBOT_SIZE = 3 # 3x3 cells [30x30]
    GRID_SIZE = 20
    OBSTACLE_AVOID = 1
    TURN_COST = 20000
    TURN_RADIUS = 3

    visited = []
    path = []

    class CellType(Enum):
        EMPTY_SPACE = 0
        OBSTACLE = 1
        OBSTACLE_BOUNDARY = 2
        OBSTACLE_END = 3
        ROBOT = 4
        ROBOT_FRONT = 10
        START = 5
        PATH = 6
    
    class ImageDirection(Enum):
        NORTH = (0, 1)
        EAST = (1, 0)
        SOUTH = (0, -1)
        WEST = (-1, 0)

    def __init__(self, obstacles) -> None:
        self.robot_state = (1, 1, self.ImageDirection.NORTH)
        self.grid = [[self.CellType.EMPTY_SPACE.value for x in range(self.GRID_SIZE)] for x in range(self.GRID_SIZE)]

        for i in range(self.START_SIZE):
            for j in range(self.START_SIZE):
                self.grid[i][j] = self.CellType.START.value

        # self.obstacles = obstacles
        new_obstacle = []
        for obstacle in obstacles:
            x, y, face = obstacle
            y = 19 - y
            new_obstacle.append((x, y, face))

        self.obstacles = new_obstacle
        # print(self.obstacles)


        for obstacle in self.obstacles:
            x, y, face = obstacle
            self.grid[x][y] = self.CellType.OBSTACLE.value
            self.grid[x + self.OBSTACLE_AVOID][y] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x - self.OBSTACLE_AVOID][y] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x + self.OBSTACLE_AVOID][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x + self.OBSTACLE_AVOID][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x - self.OBSTACLE_AVOID][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x - self.OBSTACLE_AVOID][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            self.grid[x][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            dx, dy = face.value
            x += dx*2
            y += dy*2
            self.grid[x][y] = self.CellType.OBSTACLE_END.value

    def reset_robot(self):
        self.robot_state = (1, 1, self.ImageDirection.NORTH)
    
    def is_obstacle(self, coord):
        x, y, _ = coord
        if self.grid[x][y] == self.CellType.OBSTACLE.value:
            return True

        return False

    def is_obstacle_bfs(self, coord):
        x, y = coord
        if self.grid[x][y] == self.CellType.OBSTACLE.value:
            return True

        return False

    def get_obstacle(self, coord):
        x, y = coord
        for obstacle in self.obstacles:
            x_i, y_i, face = obstacle
            if x == x_i and y == y_i:
                return obstacle

        return None

    def get_eucledian(self, coord1, coord2):
        x1, y1 = coord1
        x2, y2 = coord2
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_total_distance(self, path):
        distance = 0
        for i in range(len(path) - 1):
            distance += self.get_eucledian(path[i], path[i + 1])

        return distance

    def is_obstacle_avoid(self, coord):

        x, y = coord
        if self.grid[x][y] == self.CellType.OBSTACLE_BOUNDARY.value:
            return False

        return True

    def is_obstacle_end(self, coord):
        x, y = coord

        return self.grid[x][y] == self.CellType.OBSTACLE_END.value

    def get_obstacle_end(self, coord):

        if self.is_obstacle_bfs(coord) == False:
            print('Not an obstacle')
            return

        obstacle = self.get_obstacle(coord)
        x, y, face = obstacle
        dx, dy = face.value
        
        if face.value == self.ImageDirection.NORTH.value:
            face = self.ImageDirection.SOUTH
        elif face.value == self.ImageDirection.SOUTH.value:
            face = self.ImageDirection.NORTH
        elif face.value == self.ImageDirection.EAST.value:
            face = self.ImageDirection.WEST
        elif face.value == self.ImageDirection.WEST.value:
            face = self.ImageDirection.EAST

        return (x + dx*3, y + dy*3, face)

    def bfs2(self, coord, i=0):
        if self.visited.count(coord) != 0:
            return

        if self.is_obstacle_bfs(coord) is True:
            self.visited.append(coord)
            self.path.append(coord)
            return None

        self.visited.append(coord)

        x, y = coord
        i += 1

        if x > 0:
           self.bfs2((x - 1, y), i)

        if y > 0:
            self.bfs2((x, y - 1), i)

        if x > 0 and y > 0:
            self.bfs2((x - 1, y - 1), i)

        if x < self.GRID_SIZE - 1:
            self.bfs2((x + 1, y), i)

        if y < self.GRID_SIZE - 1:
            self.bfs2((x, y + 1), i)

        if x < self.GRID_SIZE - 1 and y < self.GRID_SIZE - 1:
            self.bfs2((x + 1, y + 1), i)

    def is_obstacle_avoid(self, coord):

        x, y, _ = coord

        for i in range(-1, 2, 1):
            for j in range(-1, 2, 1):
                if not (0 <= x + i < self.GRID_SIZE and 0 <= y + j < self.GRID_SIZE):
                    return False
                if self.grid[x + i][y + j] == self.CellType.OBSTACLE.value:
                    return False
                if self.grid[x + i][y + j] == self.CellType.OBSTACLE_BOUNDARY.value:
                    return False

        return True

    def move_forward(self, state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x, y + 1, direction)
        elif direction == self.ImageDirection.EAST:
            return (x + 1, y, direction)
        elif direction == self.ImageDirection.SOUTH:
            return (x, y - 1, direction)
        elif direction == self.ImageDirection.WEST:
            return (x - 1, y, direction)

    def move_reverse(self, state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x, y - 1, direction)
        elif direction == self.ImageDirection.EAST:
            return (x - 1, y, direction)
        elif direction == self.ImageDirection.SOUTH:
            return (x, y + 1, direction)
        elif direction == self.ImageDirection.WEST:
            return (x + 1, y, direction)

    def turn_left(self, state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.EAST:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.NORTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.WEST:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.SOUTH)
    
    def turn_right(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.EAST:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.SOUTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.WEST:
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.NORTH)

    def back_left(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.EAST:
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.SOUTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.WEST:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.NORTH)
    
    def back_right(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.EAST:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS, self.ImageDirection.NORTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.WEST:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS, self.ImageDirection.SOUTH)

    def is_valid_state(self, new_state, current_state):
        x, y, direction = new_state
        # Check boundaries
        if not (0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE):
            return False

        if self.is_obstacle_avoid((x, y, direction)) == False:
            return False

        return True

    def get_eucledian(self, coord1, coord2):
        x1, y1, d1 = coord1
        x2, y2, d2 = coord2
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_neighbors(self, state):
        neighbors = []
        for movement in [self.move_forward, self.turn_left, self.turn_right, self.move_reverse, self.back_left, self.back_right]:
            new_state = movement(state)
            if self.is_valid_state(new_state, state):
                distance_cost = self.get_eucledian(state, new_state) * self.TURN_COST if movement == self.turn_left or movement == self.turn_right or movement == self.back_left or movement == self.back_right else self.get_eucledian(state, new_state)
                neighbors.append([distance_cost, new_state])
        return neighbors

    def bfsNew(self, start_state, end_state):
        queue = deque([start_state])
        visited = set([start_state])
        path = {start_state: None}

        while queue:
            current_state = queue.popleft()

            if (current_state[0], current_state[1], current_state[2]) == (end_state[0], end_state[1], end_state[2]):
                shortest_path = []
                while current_state:
                    shortest_path.append(current_state)
                    current_state = path[current_state]
                return shortest_path[::-1]

            neighbors = self.get_neighbors(current_state)
            neighbors.sort(key=lambda x: x[0])

            for neighbor in neighbors:
                neighbor = neighbor[1]
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    path[neighbor] = current_state

        return None

    def computePath(self):
        self.visited = []

        start = (self.robot_state[0], self.robot_state[1])

        self.path = [start]
        self.bfs2(start)
        
        instructions = []
        maxInstructions = float("inf")
        chosenPath = []

        
        allPaths = list(itertools.permutations(self.path[1:]))

        for path in allPaths:
            start = self.robot_state
            calc_instructions = []
            hasNone = False

            for i in range(len(path)):
                end = self.get_obstacle_end(path[i])
                current_path = self.bfsNew(start, end)
                start = self.get_obstacle_end(path[i])
                if current_path is None:
                    hasNone = True
                    continue
                calc_instructions = calc_instructions + current_path

            if len(calc_instructions) < maxInstructions and hasNone == False:
                instructions = calc_instructions.copy()
                maxInstructions = len(calc_instructions)
                chosenPath = path

            
        # print(chosenPath, instructions)
        print("Found path")
        return instructions