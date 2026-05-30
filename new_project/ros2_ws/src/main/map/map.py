from interfaces.msg import Pair
class Map():
    def __init__(self):
        
        self.blocks = []
        self.map = [[0 for _ in range(7)] for _ in range(9)]
        self.x_char = {
            0: 'A1',
            1: 'A2',
            2: 'A3',
            3: 'A4',
            4: 'A5',
            5: 'A6',
            6: 'A7',
            7: 'A8',
            8: 'A9'
        }

        # y 对应 B1 到 B7
        self.y_char = {
            0: 'B1',
            1: 'B2',
            2: 'B3',
            3: 'B4',
            4: 'B5',
            5: 'B6',
            6: 'B7'
        }
