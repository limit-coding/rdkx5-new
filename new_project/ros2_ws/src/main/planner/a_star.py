import heapq  # 用于实现优先队列（开放列表）
class Node:
    def __init__(self, x, y):
        self.x = x  # 行坐标
        self.y = y  # 列坐标
        self.g = 0  # 起点到当前节点的实际成本
        self.h = 0  # 到终点的估计成本
        self.f = 0  # f = g + h
        self.parent = None  # 父节点

    # 用于优先队列比较（按f值排序）
    def __lt__(self, other):
        return self.f < other.f

def manhattan_distance(node, end):
    """曼哈顿距离（四方向启发函数）"""
    return abs(node.x - end.x) + abs(node.y - end.y)

def a_star(grid, start, end):
    # 初始化起点和终点节点
    start_node = Node(start[0], start[1])
    end_node = Node(end[0], end[1])

    # 开放列表（优先队列）和关闭列表（集合，存坐标元组）
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_list = set()

    while open_list:
        # 取出f值最小的节点
        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        # 到达终点，回溯路径
        if (current_node.x, current_node.y) == (end_node.x, end_node.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]  # 反转路径（从起点到终点）

        # 四方向邻居（上、下、左、右）
        neighbors = [
            (current_node.x - 1, current_node.y),  # 上
            (current_node.x + 1, current_node.y),  # 下
            (current_node.x, current_node.y - 1),  # 左
            (current_node.x, current_node.y + 1)   # 右
        ]

        for nx, ny in neighbors:
            # 检查邻居是否在网格内且可通行（非障碍物）
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 0:
                # 若邻居在关闭列表，跳过
                if (nx, ny) in closed_list:
                    continue

                # 创建邻居节点
                neighbor_node = Node(nx, ny)
                neighbor_node.parent = current_node
                neighbor_node.g = current_node.g + 1  # 成本+1
                neighbor_node.h = manhattan_distance(neighbor_node, end_node)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                # 检查邻居是否已在开放列表
                in_open = False
                for node in open_list:
                    if (node.x, node.y) == (nx, ny):
                        in_open = True
                        # 若当前路径更优（g更小），更新
                        if neighbor_node.g < node.g:
                            node.g = neighbor_node.g
                            node.f = neighbor_node.f
                            node.parent = current_node
                        break
                if not in_open:
                    heapq.heappush(open_list, neighbor_node)

    # 开放列表为空，无路径
    return None