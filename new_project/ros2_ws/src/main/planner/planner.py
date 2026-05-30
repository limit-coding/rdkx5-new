from map.map import Map
from planner.a_star import a_star
class Pathplanner():
    def __init__(self):
        self.grid_map = Map()
        self.path = []
        # self.landpath = []
        self.sparse_path = []
        self.way_point = []
        self.corner_index = []
        self.start_point = (8,0)
    
    def plan(self):
        blocked = False
        if(self.grid_map.blocks[0][1]-self.grid_map.blocks[1][1]==0):#x_0-x_1,y_0-y_1
            for y in range(7):
                if (y+1) % 2 == 0:
                    for x in range(9):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y,0))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                for point in temp_path:
                                    self.path.append((point[0],point[1],1))
                            
                                blocked = False
                        else:
                            blocked = True
                            continue
                else:
                    for x in reversed(range(9)):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                for point in temp_path:
                                    self.path.append((point[0],point[1],1))
                                blocked = False
                        else:
                            blocked = True
                            continue
        else: 
            for x in reversed(range(9)):
                if x % 2 == 0:
                    for y in range(7):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                for point in temp_path:
                                    self.path.append((point[0],point[1],1))
                                blocked = False
                        else:
                            blocked = True
                            continue
                else:
                    for y in reversed(range(7)):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                for point in temp_path:
                                    self.path.append((point[0],point[1],1))
                                blocked = False
                        else:
                            blocked = True
                            continue
        self.plan_landing()
        return True
    
    def plan_stop(self):
        blocked = False
        if(self.grid_map.blocks[0][1]-self.grid_map.blocks[1][1]==0):#x_0-x_1,y_0-y_1
            for y in range(7):
                if (y+1) % 2 == 0:
                    for x in range(9):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y,1))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                temp_path.pop(-1)
                                if (len(temp_path)>1):
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                    self.path.append((temp_path[-1][0],temp_path[-1][1],0))
                                else :
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                self.path.append((x,y,1))
                                blocked = False
                        else:
                            blocked = True
                            continue
                else:
                    for x in reversed(range(9)):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y,1))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                temp_path.pop(-1)
                                if (len(temp_path)>1):
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                    self.path.append((temp_path[-1][0],temp_path[-1][1],0))
                                else :
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                self.path.append((x,y,1))
                                blocked = False
                        else:
                            blocked = True
                            continue
        else: 
            for x in reversed(range(9)):
                if x % 2 == 0:
                    for y in range(7):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y,1))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                temp_path.pop(-1)
                                if (len(temp_path)>1):
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                    self.path.append((temp_path[-1][0],temp_path[-1][1],0))
                                else :
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                self.path.append((x,y,1))
                                blocked = False
                        else:
                            blocked = True
                            continue
                else:
                    for y in reversed(range(7)):
                        if self.grid_map.map[x][y] == 0:
                            if not blocked:
                                self.path.append((x,y,1))
                            else:
                                temp_path = a_star(self.grid_map.map,self.path[-1],(x,y))
                                temp_path.pop(0)
                                temp_path.pop(-1)
                                if (len(temp_path)>1):
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                    self.path.append((temp_path[-1][0],temp_path[-1][1],0))
                                else :
                                    self.path.append((temp_path[0][0],temp_path[0][1],0))
                                self.path.append((x,y,1))
                                blocked = False
                        else:
                            blocked = True
                            continue
        self.plan_landing()
        return True
    
    # 检测最后一个航点所在列与最下面边是否有禁飞区
    def check_landing_obstacles(self, last_point):
        last_col = last_point[0]  # 最后一个点的x坐标（列）
        bottom_blocked = False
        column_blocked = False

        # 检测最后一列（x=last_col）从最后点到最下面边（y=0）是否有禁飞区
        for y in range(last_point[1], 0):
            if self.grid_map.map[last_col][y] == 1:
                column_blocked = True
                break

        # 检测最下面边（y=0）从最后列到起飞点列（x=8）是否有禁飞区
        for x in range(last_col, self.start_point[0] + 1):
            if self.grid_map.map[x][0] == 1:
                bottom_blocked = True
                break

        return column_blocked or bottom_blocked

    def plan_landing(self):
        
        # if (col or bot):
        #     self.landpath=[[8,self.path[-1][1]],[8,0]]
        # else:
        #     self.landpath=[[self.path[-1][0],0],[8,0]]
        if (self.check_landing_obstacles(self.path[-1])):
            self.path+=((8,self.path[-1][1],0),(8,0,0))
        else:
            self.path+=((self.path[-1][0],0,0),(8,0,0))
        
            
            
            
            
            
            
            
    