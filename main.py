from environment import GridEnvironment
from planner import PrioritizedPlanner
from visualizer import Visualizer
from IPython.display import display

if __name__ == "__main__":
    print("Generating 20x20 Warehouse Environment...")
    
    obstacles = set()
    for x in [2, 3, 8, 9, 14, 15]:
        for y in range(2, 18):
            if y not in [9, 10]: 
                obstacles.add((x, y))
                
    env = GridEnvironment(20, 20, obstacles)
    
    agents = [
        {'id': 1, 'start': (0, 5), 'goal': (19, 9), 'init_theta': 1},  
        {'id': 2, 'start': (19, 9), 'goal': (0, 9), 'init_theta': 3}, 
        {'id': 3, 'start': (5, 2), 'goal': (5, 19), 'init_theta': 0},  
        {'id': 4, 'start': (11, 3), 'goal': (11, 0), 'init_theta': 2}  
    ]
    
    print("Initializing Decoupled Prioritized Planner...")
    planner = PrioritizedPlanner(env, agents)
    optimal_paths = planner.solve()
    
    if optimal_paths:
        print("Rendering animation widget...")
        vis = Visualizer(env, optimal_paths)
        display(vis.animate())
    else:
        print("No feasible solution exists.")