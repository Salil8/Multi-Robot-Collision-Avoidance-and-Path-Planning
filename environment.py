from typing import Tuple, Set

DIRECTIONS = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}

class BatteryModel:
    def __init__(self, capacity_ah: float, nominal_voltage: float):
        self.capacity_wh = capacity_ah * nominal_voltage
        self.idle_drain = 0.05      
        self.forward_drain = 0.50   
        self.turn_drain = 1.20      
        
    def get_action_cost(self, action: str, current_soc: float) -> float:
        ocv_penalty = 1.0 + (0.5 * (1.0 - current_soc))
        if action == 'FORWARD': cost = self.forward_drain * ocv_penalty
        elif action in ['TURN_LEFT', 'TURN_RIGHT']: cost = self.turn_drain * ocv_penalty
        elif action == 'WAIT': cost = self.idle_drain * ocv_penalty
        else: cost = 0.0
        return (cost / self.capacity_wh)

class GridEnvironment:
    def __init__(self, width: int, height: int, obstacles: Set[Tuple[int, int]]):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        
    def is_valid(self, x: int, y: int) -> bool:
        if x < 0 or x >= self.width or y < 0 or y >= self.height: return False
        if (x, y) in self.obstacles: return False
        return True