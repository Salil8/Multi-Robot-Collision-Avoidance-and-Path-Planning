import heapq
from typing import List, Dict
from environment import GridEnvironment, BatteryModel
from astar_solver import CBSSpaceTimeAStar

class CTNode:
    """Represents a High-Level 'Universe' in the CBS Tree."""
    def __init__(self, constraints, paths):
        self.constraints = constraints # Dict: {agent_id: [list of constraint tuples]}
        self.paths = paths             # Dict: {agent_id: [path data]}
        
        # The cost of this universe is the total battery drained by ALL agents combined
        self.cost = sum(path[-1]['g_score'] for path in paths.values() if path)

    def __lt__(self, other):
        # The High-Level CBS queue always explores the universe with the lowest total battery drain first
        return self.cost < other.cost

def get_first_conflict(paths: Dict):
    """Scans all current paths and finds the very first temporal/spatial crash."""
    agents = list(paths.keys())
    # Find the longest path to know how far in time we must check
    max_time = max(len(p) for p in paths.values()) if paths else 0
    
    for t in range(max_time):
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                a1, a2 = agents[i], agents[j]
                
                # If an agent finished its path, it remains parked at its final position
                pos1 = paths[a1][t]['pos'] if t < len(paths[a1]) else paths[a1][-1]['pos']
                pos2 = paths[a2][t]['pos'] if t < len(paths[a2]) else paths[a2][-1]['pos']
                
                # 1. VERTEX CONFLICT: Two robots on the same tile at the same time
                if pos1 == pos2:
                    return {'type': 'vertex', 'a1': a1, 'a2': a2, 'pos': pos1, 'time': t}
                
                # 2. EDGE CONFLICT: Two robots swapping tiles simultaneously
                if t > 0 and t < len(paths[a1]) and t < len(paths[a2]):
                    prev_pos1 = paths[a1][t-1]['pos']
                    prev_pos2 = paths[a2][t-1]['pos']
                    if pos1 == prev_pos2 and pos2 == prev_pos1:
                        return {'type': 'edge', 'a1': a1, 'a2': a2, 'pos1': prev_pos1, 'pos2': pos1, 'time': t}
    return None

class CBSPlanner:
    def __init__(self, env: GridEnvironment, agents: List[dict]):
        self.env = env
        self.agents = agents
        self.battery = BatteryModel(capacity_ah=5.0, nominal_voltage=24.0)
        self.solver = CBSSpaceTimeAStar(self.env, self.battery)
        
    def solve(self):
        print("Initializing High-Level Conflict-Based Search (CBS)...")
        
        # Initialize empty constraints for all agents
        root_constraints = {agent['id']: [] for agent in self.agents}
        root_paths = {}
        
        # ROOT NODE: Calculate ideal paths completely ignoring other robots
        for agent in self.agents:
            path = self.solver.search(agent['start'], agent['goal'], agent['init_theta'], set(), set())
            if not path:
                print(f"FATAL: Agent {agent['id']} cannot even reach the goal in an empty warehouse.")
                return None
            root_paths[agent['id']] = path
            
        root = CTNode(root_constraints, root_paths)
        tree = [root]
        heapq.heapify(tree)
        
        nodes_expanded = 0
        
        while tree:
            current_node = heapq.heappop(tree)
            nodes_expanded += 1
            
            # Look for crashes in this universe
            conflict = get_first_conflict(current_node.paths)
            
            # IF NO CRASHES: We found the globally optimal, collision-free solution!
            if not conflict:
                print(f"SUCCESS! Optimal collision-free paths found. (Universes explored: {nodes_expanded})")
                return current_node.paths
                
            # IF CRASH: Split into two parallel universes to resolve it
            a1, a2 = conflict['a1'], conflict['a2']
            t = conflict['time']
            
            if conflict['type'] == 'vertex':
                pos = conflict['pos']
                new_constraints = [
                    (a1, ('vertex', pos[0], pos[1], t)), # Universe A: Ban Agent 1 from this tile
                    (a2, ('vertex', pos[0], pos[1], t))  # Universe B: Ban Agent 2 from this tile
                ]
            else: # Edge conflict
                p1, p2 = conflict['pos1'], conflict['pos2']
                new_constraints = [
                    (a1, ('edge', p1[0], p1[1], p2[0], p2[1], t)), # Ban Agent 1 from traversing p1->p2
                    (a2, ('edge', p2[0], p2[1], p1[0], p1[1], t))  # Ban Agent 2 from traversing p2->p1
                ]
                
            # Generate the child universes
            for agent_id, constraint in new_constraints:
                # Deep copy constraints so universes don't contaminate each other
                child_constraints = {k: list(v) for k, v in current_node.constraints.items()}
                child_constraints[agent_id].append(constraint)
                
                # Separate constraints for the low-level solver
                v_cons = set((c[1], c[2], c[3]) for c in child_constraints[agent_id] if c[0] == 'vertex')
                e_cons = set((c[1], c[2], c[3], c[4], c[5]) for c in child_constraints[agent_id] if c[0] == 'edge')
                
                # Find the agent config to replan
                agent_config = next(a for a in self.agents if a['id'] == agent_id)
                
                # Ask the low-level solver for a new path under this new rule
                new_path = self.solver.search(
                    agent_config['start'], agent_config['goal'], agent_config['init_theta'], v_cons, e_cons
                )
                
                # If a valid path still exists, lock in this universe and add it to the queue
                if new_path:
                    child_paths = dict(current_node.paths)
                    child_paths[agent_id] = new_path
                    child_node = CTNode(child_constraints, child_paths)
                    heapq.heappush(tree, child_node)
                    
        print("FAILED: CBS exhausted all possible universes without finding a safe solution.")
        return None
