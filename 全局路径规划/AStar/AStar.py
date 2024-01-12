import pygame
import sys
import random
import threading
import time

# 初始化 Pygame
pygame.init()


class AStar:
    def __init__(self):
        self.current_node = None
        self.order = 1
        self.running = True
        self.open_list = []    # [["13",15]]   13: 表示第1行， 第3列。15表示F值
        self.closed_list = []  # ["11", "12", "13"]
        self.g_value_list = {} # {"13": 20}  表示索引为1 和 3 的格子的G值 为20
        self.path = []
        self.obstacles = []
        self.start = []
        self.end = []
        self.father_list = {} # {"12": "13"}, 表示索引为1 2 的格子的父节点索引为 1 3
        self.level = 2
        self.width = 1200
        self.hight = 800
        self.grid_size = 20
        self.rows = self.hight // self.grid_size
        self.columns = self.width // self.grid_size

        # 创建窗口
        self.screen = pygame.display.set_mode((self.width, self.hight))
        pygame.display.set_caption("A*")

        # 定义颜色
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        self.blue = (0, 0, 255)

        # 创建一个表示网格的二维数组
        self.grid = [[self.white for _ in range(self.width // self.grid_size)] for _ in range(self.hight // self.grid_size)]
        self.generate_obstacles(self.level)
        self.thread1 = threading.Thread(target=self.work)
        self.thread1.start()
    
    def find_neighbors(self, node, g_value):

        # 字符格式 转成int 格式  “12” -->1 2
        row = int(node[0]) # 行
        col = int(node[1]) # 列
        neighbors = {}     # 保存临时的相邻方格
        obstacles = []     # 保存相邻方格中的障碍物

        """
            查看该索引相邻的8个点, x 为可通行方格， o为障碍物
            x  x  o
            o  s  x
            x  x  x
        """

        for i in range(row -1, row + 2):
            if i < 0 or i >= self.rows: # 说明超出了 行的边界
                continue
            for j in range(col -1, col + 2):
                if j < 0 or j >= self.columns:
                    continue
                if i == row and j == col:
                    continue
                
                if self.is_obstacle(i, j):
                    obstacles.append(str(i) + str(j))
                    continue

                if self.is_goal(i, j):
                            return True 

                # 左上角， 右下角， 左下角， 右上角
                if (i < row and j < col) or (i > row and j > col) or (i > row and j < col) or (i < row and j > col):
                    neighbors[str(i) + str(j)] = g_value + 14
                else:
                    neighbors[str(i) + str(j)] = g_value + 10

        # 遍历障碍物
        for obstacle in obstacles:
            if (int(obstacle[0]) == row -1) and (int(obstacle[1]) == col):   # 障碍物在该点的正上方
                for j in [col -1, col +1]:
                    index = str(row) + str(j)
                    if index  in neighbors:
                        del neighbors[index]
            elif (int(obstacle[0]) == row +1) and (int(obstacle[1]) == col): # 障碍物在该点的正下方
                for j in [col -1, col +1]:
                    index = str(row) + str(j)
                    if index  in neighbors:
                        del neighbors[index]
            elif (int(obstacle[0]) == row) and (int(obstacle[1]) == col -1): # 障碍物在该点的左侧
                for i in [row -1, row +1]:
                    index = str(i) + str(col)
                    if index  in neighbors:
                        del neighbors[index]
            elif (int(obstacle[0]) == row) and (int(obstacle[1]) == col +1): # 障碍物在该点的右侧
                for i in [row -1, row +1]:
                    index = str(i) + str(col)
                    if index  in neighbors:
                        del neighbors[index]
                
            # 判断周围点的有没有更小的G值
                           


        

    def find_paths(self, start):
        pass
        str_start  = self.get_str_index(start)
        start_list = []
        start_list.append(str_start)
        start_list.append(0)              # 起点的F值暂时设置为0
        self.g_value_list[str_start] = 0  # 起点时的G值先设置为0
        self.open_list.append(start_list)

        while len(self.open_list) > 0:
            if len(self.open_list) == 1:
                self.current_node = self.open_list[0][0]   # 取出字符串格式的索引值 “13”
                self.open_list.clear()
                self.closed_list.append(self.current_node) # 把已经选取过的方格放入到 close_list中

                # 查找相邻的点，将该点的索引和G值传进去
                self.find_neighbors(self.current_node,  self.g_value_list[self.current_node])  
       
    def get_str_index(self, start):
         return str(start[1]) + str(start[0])

    def work(self):
        while(self.running):
            while(self.order <= 2 and self.running):

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                        return 
                    elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        
                        # 鼠标左键点击事件
                        x, y = event.pos
                        grid_x = x // self.grid_size # 列
                        grid_y = y // self.grid_size # 行

                        if self.is_obstacle(grid_y, grid_x):
                            print("障碍物，请重新选择")
                            continue

                        if self.order == 1:
                            self.start.append(grid_y)
                            self.start.append(grid_x)
                        elif self.order == 2:
                            self.end.append(grid_y)
                            self.end.append(grid_x)
                            
                        # 更改对应栅格的颜色为蓝色
                        self.grid[grid_y][grid_x] = self.blue
                        self.order +=1

            if self.order == 3:
                self.find_paths(self.start)
                self.grid = [[self.white for _ in range(self.width // self.grid_size)] for _ in range(self.hight // self.grid_size)]
                self.generate_obstacles(self.level)
                self.order = 1  
                self.start = []
                self.end = []
    
    def is_obstacle(self, x, y):
        return self.grid[x][y] == self.black
    
    def is_goal(self, x, y):
        return x == self.end[0] and y == self.end[1]

    def draw_grid(self):
        # 根据二维数组绘制方格颜色
        for i, row in enumerate(self.grid):
            for j, color in enumerate(row):
                rect = pygame.Rect(j * self.grid_size, i * self.grid_size, self.grid_size, self.grid_size)
                pygame.draw.rect(self.screen, color, rect)

        # 绘制栅格线
        for x in range(0, self.width, self.grid_size):
            pygame.draw.line(self.screen, self.black, (x, 0), (x, self.hight))
        for y in range(0, self.hight, self.grid_size):
            pygame.draw.line(self.screen, self.black, (0, y), (self.width, y))

    def generate_obstacles(self, level):
        if level == 0:
            num_obstacles = 0
        elif level == 1:
            num_obstacles = 50
        elif level == 2:
            num_obstacles = 100
        else:
            num_obstacles = 150
        
        for i in range(num_obstacles):
            obstacle = (random.randint(0, self.rows-1), random.randint(0, self.columns-1))
            self.grid[obstacle[0]][obstacle[1]] = self.black
            self.obstacles.append(obstacle)

    def loop(self):
        # 主循环
        while self.running:
            
            # 重新绘制栅格
            self.draw_grid()
            time.sleep(0.05)
            # 刷新显示
            pygame.display.flip()
        

if __name__ == "__main__":
    a_star = AStar()
    a_star.loop()