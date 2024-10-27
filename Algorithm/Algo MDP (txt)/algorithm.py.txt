from enum import Enum
from math import sqrt
from collections import deque
import itertools
import configparser

config = configparser.ConfigParser()
config.read('configs/Algorithm_config.ini')

class Algorithm:

    START_SIZE = int(config['algorithm']['START_SIZE'])
    ROBOT_SIZE = int(config['algorithm']['ROBOT_SIZE'])
    GRID_SIZE = int(config['algorithm']['GRID_SIZE'])
    OBSTACLE_AVOID = int(config['algorithm']['OBSTACLE_AVOID'])
    TURN_COST = int(config['algorithm']['TURN_COST'])
    TURN_RADIUS = int(config['algorithm']['TURN_RADIUS'])
    TURN_RADIUS_Y = int(config['algorithm']['TURN_RADIUS_Y'])
    PLAY_IT_SAFE = True if config['algorithm']['PLAY_IT_SAFE'] == 'True' else False

    visited = []
    path = []
    blacklist = []

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

    def __init__(self, obstacles, isSimulator=True) -> None:
        self.robot_state = (1, 1, self.ImageDirection.NORTH)
        self.grid = [[self.CellType.EMPTY_SPACE.value for x in range(self.GRID_SIZE)] for x in range(self.GRID_SIZE)]

        for i in range(self.START_SIZE):
            for j in range(self.START_SIZE):
                self.grid[i][j] = self.CellType.START.value

        if isSimulator:
            # self.obstacles = obstacles
            new_obstacle = []
            for obstacle in obstacles:
                x, y, face = obstacle
                y = 19 - y
                new_obstacle.append((x, y, face))

            self.obstacles = new_obstacle
            # print(self.obstacles)'
        else:
            self.obstacles = []

            for obstacle in obstacles:
                obstacle = obstacle.split(",")
                x, y, direction = int(obstacle[1]), int(obstacle[2]), obstacle[3]
                direction = self.ImageDirection[direction]
                self.obstacles.append((x, y, direction))


        for obstacle in self.obstacles:
            x, y, face = obstacle
            self.grid[x][y] = self.CellType.OBSTACLE.value

            for i in range(-self.OBSTACLE_AVOID, self.OBSTACLE_AVOID + 1):
                for j in range(-self.OBSTACLE_AVOID, self.OBSTACLE_AVOID + 1):
                    if i == 0 and j == 0:
                        continue
                    if x + i >= self.GRID_SIZE or y + j >= self.GRID_SIZE:
                        continue
                    if x + i < 0 or y + j < 0:
                        continue
                    self.grid[x + i][y + j] = self.CellType.OBSTACLE_BOUNDARY.value

            #self.grid[x + self.OBSTACLE_AVOID][y] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x - self.OBSTACLE_AVOID][y] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x + self.OBSTACLE_AVOID][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x + self.OBSTACLE_AVOID][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x - self.OBSTACLE_AVOID][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x - self.OBSTACLE_AVOID][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x][y + self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value
            #self.grid[x][y - self.OBSTACLE_AVOID] = self.CellType.OBSTACLE_BOUNDARY.value

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

    def get_obstacle_index(self, coord):
        x, y = coord
        index = 1
        for obstacle in self.obstacles:
            x_i, y_i, face = obstacle
            if x == x_i and y == y_i:
                return index
            index += 1

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

    #def is_obstacle_avoid(self, coord):
#
      #  x, y = coord
     #   if self.grid[x][y] == self.CellType.OBSTACLE_BOUNDARY.value:
     #       return False
#
      #  return True

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
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS_Y, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.EAST:
            return (x + self.TURN_RADIUS_Y, y + self.TURN_RADIUS, self.ImageDirection.NORTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS_Y, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.WEST:
            return (x - self.TURN_RADIUS_Y, y - self.TURN_RADIUS, self.ImageDirection.SOUTH)

    def turn_right(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS_Y, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.EAST:
            return (x + self.TURN_RADIUS_Y, y - self.TURN_RADIUS, self.ImageDirection.SOUTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS_Y, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.WEST:
            return (x - self.TURN_RADIUS_Y, y + self.TURN_RADIUS, self.ImageDirection.NORTH)

    def back_left(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x - self.TURN_RADIUS_Y, y - self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.EAST:
            return (x - self.TURN_RADIUS, y + self.TURN_RADIUS_Y, self.ImageDirection.SOUTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x + self.TURN_RADIUS_Y, y + self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.WEST:
            return (x + self.TURN_RADIUS, y - self.TURN_RADIUS_Y, self.ImageDirection.NORTH)

    def back_right(self,state):
        x, y, direction = state
        if direction == self.ImageDirection.NORTH:
            return (x + self.TURN_RADIUS_Y, y - self.TURN_RADIUS, self.ImageDirection.WEST)
        elif direction == self.ImageDirection.EAST:
            return (x - self.TURN_RADIUS, y - self.TURN_RADIUS_Y, self.ImageDirection.NORTH)
        elif direction == self.ImageDirection.SOUTH:
            return (x - self.TURN_RADIUS_Y, y + self.TURN_RADIUS, self.ImageDirection.EAST)
        elif direction == self.ImageDirection.WEST:
            return (x + self.TURN_RADIUS, y + self.TURN_RADIUS_Y, self.ImageDirection.SOUTH)

    def is_valid_state(self, new_state, current_state):
        x, y, direction = new_state
        cur_x, cur_y, cur_dir = current_state
        
        if new_state in self.blacklist:
            return False
        
        if self.blockingCache.get(str(new_state)) is None:
            self.blockingCache[str(new_state)] = {}
        
        if self.blockingCache[str(new_state)].get(str(current_state)) is None:
            self.blockingCache[str(new_state)][str(current_state)] = False
        else:
            return self.blockingCache[str(new_state)][str(current_state)] 
        
        # Check boundaries

        #for i in range(-1, 2, 1):
         #   for j in range(-1, 2, 1):
         #       if not (0 <= x + i < self.GRID_SIZE and 0 <= y + j < self.GRID_SIZE):
          #          return False

        if not (0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE):
            return False

        sign_x = 1 if x - cur_x >= 0 else -1
        sign_y = 1 if y - cur_y >= 0 else -1

        if self.PLAY_IT_SAFE:
            if (x - cur_x) == 0:
                for dy in range(cur_y + sign_x, y + sign_y, sign_y):
                    if self.is_obstacle_avoid((x, dy, direction)) == False:
                        return False
            elif (y - cur_y) == 0:
                for dx in range(cur_x + sign_x, x + sign_x, sign_x):
                    if self.is_obstacle_avoid((dx, y, direction)) == False:
                        return False
            else:
                for dx in range(cur_x + sign_x, x + sign_x, sign_x):
                    for dy in range(cur_y + sign_x, y + sign_y, sign_y):
                        if self.is_obstacle_avoid((dx, dy, direction)) == False:
                            return False
        else:
            if self.is_obstacle_avoid((x, y, direction)) == False:
                return False
        
        self.blockingCache[str(new_state)][str(current_state)] = True
        
        return True

    def get_eucledian(self, coord1, coord2):
        x1, y1, d1 = coord1
        x2, y2, d2 = coord2
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_neighbors(self, state):
        neighbors = []
        #, self.back_left, self.back_right
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

            #if len(neighbors) == 0:
               # print("zero neighbor")

            for neighbor in neighbors:
                neighbor = neighbor[1]
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
                    path[neighbor] = current_state

        return None

    def generateMovementCommands(self, instructions):
        commands = []

        for i in range(1, len(instructions) - 1):
            prev_x, prev_y, prev_direction = instructions[i - 1] if not isinstance(instructions[i - 1], str) else instructions[i - 2]
            curr_x, curr_y, curr_direction = instructions[i] if not isinstance(instructions[i], str) else instructions[i + 1]

            if isinstance(instructions[i], str):
                commands.append(instructions[i])

            delta_x = curr_x - prev_x
            delta_y = curr_y - prev_y

            if prev_direction == self.ImageDirection.NORTH:
                if delta_x == 0 and delta_y > 0:
                    commands.append("FW10")
                elif delta_x < 0 and delta_y > 0:
                    commands.append("FL30")
                elif delta_x > 0 and delta_y > 0:
                    commands.append("FR30")
                elif delta_x == 0 and delta_y < 0:
                    commands.append("BW10")
                elif delta_x < 0 and delta_y < 0:
                    commands.append("BL30")
                elif delta_x > 0 and delta_y < 0:
                    commands.append("BR30")

            elif prev_direction == self.ImageDirection.SOUTH:
                if delta_x == 0 and delta_y < 0:
                    commands.append("FW10")
                elif delta_x > 0 and delta_y < 0:
                    commands.append("FL30")
                elif delta_x < 0 and delta_y < 0:
                    commands.append("FR30")
                elif delta_x == 0 and delta_y > 0:
                    commands.append("BW10")
                elif delta_x > 0 and delta_y > 0:
                    commands.append("BL30")
                elif delta_x < 0 and delta_y > 0:
                    commands.append("BR30")

            elif prev_direction == self.ImageDirection.EAST:
                if delta_x > 0 and delta_y == 0:
                    commands.append("FW10")
                elif delta_x > 0 and delta_y > 0:
                    commands.append("FL30")
                elif delta_x > 0 and delta_y < 0:
                    commands.append("FR30")
                elif delta_x < 0 and delta_y == 0:
                    commands.append("BW10")
                elif delta_x < 0 and delta_y > 0:
                    commands.append("BL30")
                elif delta_x < 0 and delta_y < 0:
                    commands.append("BR30")

            elif prev_direction == self.ImageDirection.WEST:
                if delta_x < 0 and delta_y == 0:
                    commands.append("FW10")
                elif delta_x < 0 and delta_y < 0:
                    commands.append("FL30")
                elif delta_x < 0 and delta_y > 0:
                    commands.append("FR30")
                elif delta_x > 0 and delta_y == 0:
                    commands.append("BW10")
                elif delta_x > 0 and delta_y < 0:
                    commands.append("BL30")
                elif delta_x > 0 and delta_y > 0:
                    commands.append("BR30")

        #commands.append(instructions[len(instructions) - 1])

        return commands
    
    MAX_CHAIN = 4
    
    def optimizeMovement(self, instructions):
        
        new_instruction = []
        len_instr = len(instructions)
        i = 0
        
        curr_lock = ""
        chain = 0
        
        while (i < len_instr):
            curr_instr = instructions[i]
            i += 1
            if curr_instr == "FW10":
                if curr_lock == "":
                    curr_lock = "FW10"
                    chain += 1
                    continue
                
                if curr_lock == "FW10":
                    chain += 1
                    if chain >= self.MAX_CHAIN:
                        new_instruction.append(f'FW{chain}0')
                        chain = 0
                        curr_lock = ""
                    continue
                    
                if curr_lock == "BW10":
                    new_instruction.append(f'BW{chain}0')
                    curr_lock = "FW10"
                    chain = 1
                    continue
                    
            if curr_instr == "BW10":
                if curr_lock == "":
                    curr_lock = "BW10"
                    chain += 1
                    continue
                
                if curr_lock == "BW10":
                    chain += 1
                    if chain >= self.MAX_CHAIN:
                        new_instruction.append(f'BW{chain}0')
                        chain = 0
                        curr_lock = ""
                    continue
                    
                if curr_lock == "FW10":
                    new_instruction.append(f'FW{chain}0')
                    curr_lock = "BW10"
                    chain = 1
                    continue
            
            if curr_lock == "FW10":
                new_instruction.append(f'FW{chain}0')
                
            if curr_lock == "BW10":
                new_instruction.append(f'BW{chain}0')

            chain = 0
            curr_lock = ''
            
            new_instruction.append(curr_instr)
            
        return new_instruction
                
        
        

    def computeTime(self, instructions):

        time = 0

        for i in range(1, len(instructions) - 1):

            prevIns = instructions[i - 1] if not isinstance(instructions[i - 1], str) else instructions[i - 2]
            currIns = instructions[i] if not isinstance(instructions[i], str) else instructions[i + 1]

            prevX, prevY, _ = prevIns
            currX, currY, _ = currIns

            delta_x, delta_y = abs(currX - prevX), abs(currY - prevY)

            if delta_x != 0 and delta_y != 0:
                time += ((delta_x + delta_y) * 2) / 2.2624
            else:
                time += (delta_x + delta_y) / 1.667

            time += 3

        return time


    def computePath(self):
        self.visited = []

        start = (self.robot_state[0], self.robot_state[1])

        self.path = [start]
        self.bfs2(start)

        instructions = []
        maxTime = float("inf")
        chosenPath = []

        numOfPaths = len(self.path)
        pathCache = {}
        self.blockingCache = {}

        allPaths = list(itertools.permutations(self.path[1:]))

        for path in allPaths:
            start = self.robot_state
            calc_instructions = []
            hasNone = False

            for i in range(len(path)):
                obstacle_index = self.get_obstacle_index(path[i])
                end = self.get_obstacle_end(path[i])

                if pathCache.get(str(start)) is None:
                    pathCache[str(start)] = {}

                if pathCache[str(start)].get(str(end)) is None:
                    pathCache[str(start)][str(end)] = self.bfsNew(start, end)

                if pathCache[str(start)].get(str(end)) is None:
                    hasNone = True
                    start = self.get_obstacle_end(path[i])
                    self.blacklist = [start]
                    start = self.move_reverse(start)
                    continue

                calc_instructions = calc_instructions + pathCache[str(start)][str(end)] + [f'C,{obstacle_index}']
                start = self.get_obstacle_end(path[i])
                self.blacklist = [start]
                calc_instructions = calc_instructions + [start]
                start = self.move_reverse(start)

            time = self.computeTime(calc_instructions)
            if time < maxTime and hasNone == False:
                instructions = calc_instructions.copy()
                maxTime = time
                chosenPath = path


        # print(chosenPath, instructions)
        #print(maxTime)
        #print("Found path")

        commands = self.generateMovementCommands(instructions)
        new_commands = self.optimizeMovement(commands)
        #print(new_commands)
        #print(commands)

        return new_commands
