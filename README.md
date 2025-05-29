# AI Pathfinding Algorithms

![Pathfinding Visualization](results/screenshots/demo.gif)

Implementation of BFS, DFS, and A* search algorithms for grid-based pathfinding with obstacle avoidance.

##  Features

- **Algorithms Implemented**:
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
  - A* Search with Euclidean/Manhattan heuristics
  - **Bonus**: Safety-optimized A* variant

- **Visualization**:
  - Interactive Pygame visualization
  - Color-coded nodes (visited vs. path)
  - Real-time statistics display

- **Customization**:
  - Interactive map editor
  - Adjustable start/goal positions
  - Configurable movement patterns (4-directional/8-directional)

##  Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/mehrsamiz/AI-Pathfinding-Algorithms.git
   cd AI-Pathfinding-Algorithms/
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   
## Usage
Run the main script:
Toggle between algorithms by changing line 58 in main.py:
```python
agent = DFS_Agent(s_start, s_goal, map_name)  # Change to BFS_Agent or AStar_Agent
