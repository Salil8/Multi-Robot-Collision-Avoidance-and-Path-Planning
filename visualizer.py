import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IPython.display import HTML

class Visualizer:
    def __init__(self, env, paths):
        self.env = env
        self.paths = paths
        self.max_time = max(len(p) for p in paths.values())
        
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-0.5, env.width - 0.5)
        self.ax.set_ylim(-0.5, env.height - 0.5)
        self.ax.set_xticks(np.arange(-0.5, env.width, 1))
        self.ax.set_yticks(np.arange(-0.5, env.height, 1))
        self.ax.grid(True, alpha=0.3)
        
        for ox, oy in env.obstacles:
            self.ax.add_patch(plt.Rectangle((ox - 0.5, oy - 0.5), 1, 1, color='#555555'))
            
        self.colors = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd']
        self.agent_patches = {}
        self.soc_texts = {}
        
        for i, agent_id in enumerate(paths.keys()):
            color = self.colors[(agent_id - 1) % len(self.colors)]
            patch = plt.Circle((-1, -1), 0.4, color=color, zorder=3)
            self.ax.add_patch(patch)
            self.agent_patches[agent_id] = patch
            
            txt = self.ax.text(0, 0, '', color=color, fontweight='bold', transform=self.ax.transAxes)
            self.soc_texts[agent_id] = txt

    def init_anim(self):
        for patch in self.agent_patches.values(): patch.center = (-1, -1)
        return list(self.agent_patches.values()) + list(self.soc_texts.values())

    def update(self, frame):
        self.ax.set_title(f"Warehouse Time Step: {frame}", fontsize=14, pad=20)
        y_offset = 1.05
        
        for i, (agent_id, path) in enumerate(self.paths.items()):
            state_idx = min(frame, len(path) - 1)
            state = path[state_idx]
            self.agent_patches[agent_id].center = state['pos']
            
            action = state['action'] if state['action'] else "START"
            self.soc_texts[agent_id].set_text(f"Robo {agent_id}: {state['soc']*100:.1f}% [{action}]")
            self.soc_texts[agent_id].set_position((0.01 + (i * 0.22), y_offset))
            
        return list(self.agent_patches.values()) + list(self.soc_texts.values())

    def animate(self):
        anim = animation.FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            frames=self.max_time + 5, interval=400, blit=False, repeat=False
        )
        plt.close(self.fig)
        return HTML(anim.to_jshtml())