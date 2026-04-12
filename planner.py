from typing import List
from itertools import permutations
from environment import GridEnvironment, BatteryModel
from astar_solver import PrioritizedSpaceTimeAStar, manhattan_distance

class PrioritizedPlanner:
    def __init__(self, env: GridEnvironment, agents: List[dict]):
        self.env = env
        self.agents = agents
        self.battery = BatteryModel(capacity_ah=5.0, nominal_voltage=24.0)
        self.solver = PrioritizedSpaceTimeAStar(self.env, self.battery)
        
    def solve(self):
        # 1. Sort by difficulty first so the permutations generator tries our "best guess" first
        initial_sorted_agents = sorted(
            self.agents, 
            key=lambda a: manhattan_distance(a['start'][0], a['start'][1], a['goal'][0], a['goal'][1]), 
            reverse=True
        )
        
        # 2. Iterate through every possible priority ordering (N! permutations)
        attempt_count = 0
        for current_order in permutations(initial_sorted_agents):
            attempt_count += 1
            order_ids = [a['id'] for a in current_order]
            print(f"\n--- Attempt #{attempt_count} | Priority Order: {order_ids} ---")
            
            # Reset the shared memory for each new permutation attempt
            paths = {}
            reservations = set()       
            edge_reservations = set()  
            parked_agents = {}         
            success = True
            
            for agent in current_order:
                path = self.solver.search(
                    agent['start'], agent['goal'], agent['init_theta'], 
                    reservations, edge_reservations, parked_agents
                )
                
                if not path:
                    print(f"  -> FAILED: Agent {agent['id']} deadlocked. Abandoning this order.")
                    success = False
                    break # Break the inner loop and immediately try the next permutation
                    
                paths[agent['id']] = path
                
                # Log the reservations for the next agents in this specific timeline
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

            # If the inner loop finishes without breaking, we found a globally valid solution!
            if success:
                print(f"\nSUCCESS! All paths synchronized using priority order {order_ids}.")
                return paths

        # If the outer loop finishes entirely, no solution exists in any universe
        print("\nFATAL: Exhausted all possible priority orderings. No feasible solution exists.")
        return None
