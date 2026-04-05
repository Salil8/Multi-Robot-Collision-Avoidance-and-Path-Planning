import heapq
from environment import GridEnvironment, BatteryModel, DIRECTIONS

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

class RCNode:
    def __init__(self, x, y, theta, time, soc, parent=None, action=None, g_score=0.0):
        self.x, self.y, self.theta, self.time, self.soc = x, y, theta, time, soc
        self.parent, self.action, self.g_score, self.f_score = parent, action, g_score, 0.0

    def __lt__(self, other):
        if self.f_score == other.f_score:
            if self.soc == other.soc: return self.time < other.time
            return self.soc > other.soc
        return self.f_score < other.f_score

class PrioritizedSpaceTimeAStar:
    """Computes paths while avoiding dynamic reservations from higher-priority agents."""
    def __init__(self, env: GridEnvironment, battery: BatteryModel):
        self.env = env
        self.battery = battery
        
    def search(self, start, goal, init_theta, reservations, edge_reservations, parked_agents, critical_soc_limit=0.05, max_time=150, heuristic_weight=2.0):
        open_list = []
        dominance_dict = {} 
        
        start_node = RCNode(start[0], start[1], init_theta, 0, 1.0, g_score=0.0)
        start_node.f_score = start_node.g_score + (heuristic_weight * manhattan_distance(start[0], start[1], goal[0], goal[1]))
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current = heapq.heappop(open_list)
            if (current.x, current.y) == goal: return self.reconstruct_path(current)
            if current.time >= max_time: continue
                
            state_key = (current.x, current.y, current.theta, current.time)
            if state_key in dominance_dict and dominance_dict[state_key] >= current.soc: continue 
            dominance_dict[state_key] = current.soc

            transitions = [
                ('FORWARD', current.x + DIRECTIONS[current.theta][0], current.y + DIRECTIONS[current.theta][1], current.theta),
                ('TURN_LEFT', current.x, current.y, (current.theta - 1) % 4),
                ('TURN_RIGHT', current.x, current.y, (current.theta + 1) % 4),
                ('WAIT', current.x, current.y, current.theta)
            ]
            
            for action, nx, ny, ntheta in transitions:
                if not self.env.is_valid(nx, ny): continue
                ntime = current.time + 1
                
                if (nx, ny, ntime) in reservations: continue 
                if (nx, ny, current.x, current.y, ntime) in edge_reservations: continue
                if (nx, ny) in parked_agents and ntime >= parked_agents[(nx, ny)]: continue
                    
                nsoc = current.soc - self.battery.get_action_cost(action, current.soc)
                if nsoc <= critical_soc_limit: continue 
                
                action_cost = 5.0 if action == 'WAIT' else (2.0 if action in ['TURN_LEFT', 'TURN_RIGHT'] else 1.0)
                
                neighbor = RCNode(nx, ny, ntheta, ntime, nsoc, current, action, current.g_score + action_cost)
                neighbor.f_score = neighbor.g_score + (heuristic_weight * manhattan_distance(nx, ny, goal[0], goal[1]))
                heapq.heappush(open_list, neighbor)
                
        return None

    def reconstruct_path(self, node):
        path = []
        while node:
            path.append({'pos': (node.x, node.y), 'theta': node.theta, 'time': node.time, 'soc': node.soc, 'action': node.action})
            node = node.parent
        return path[::-1]