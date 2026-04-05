from typing import List
from environment import GridEnvironment, BatteryModel
from astar_solver import PrioritizedSpaceTimeAStar, manhattan_distance

class PrioritizedPlanner:
    def __init__(self, env: GridEnvironment, agents: List[dict]):
        self.env = env
        self.agents = agents
        self.battery = BatteryModel(capacity_ah=5.0, nominal_voltage=24.0)
        self.solver = PrioritizedSpaceTimeAStar(self.env, self.battery)
        
    def solve(self):
        sorted_agents = sorted(self.agents, key=lambda a: manhattan_distance(a['start'][0], a['start'][1], a['goal'][0], a['goal'][1]), reverse=True)
        
        paths = {}
        reservations = set()       
        edge_reservations = set()  
        parked_agents = {}         
        
        for agent in sorted_agents:
            print(f"Planning route for Agent {agent['id']}...")
            path = self.solver.search(
                agent['start'], agent['goal'], agent['init_theta'], 
                reservations, edge_reservations, parked_agents
            )
            
            if not path:
                print(f"FAILED: Agent {agent['id']} could not find a valid space-time route.")
                return None
                
            paths[agent['id']] = path
            
            for i in range(len(path)):
                loc = path[i]['pos']
                t = path[i]['time']
                reservations.add((loc[0], loc[1], t))
                
                if i > 0:
                    prev_loc = path[i-1]['pos']
                    edge_reservations.add((prev_loc[0], prev_loc[1], loc[0], loc[1], t))
                    
            final_loc = path[-1]['pos']
            final_time = path[-1]['time']
            parked_agents[final_loc] = final_time

        print("All paths successfully mapped and synchronized.")
        return paths