
motion_planning.py
This script controls an autonomous drone using a state machine that handles different flight phases like arming, takeoff, waypoint navigation, landing, and disarming. It reads obstacle data from a CSV file and builds a 2D grid map with safety margins using the drone's altitude. The drone’s global position is converted to local coordinates, and A\* search is used to find a collision-free path on the grid. The path is then simplified using `prune_path` to remove unnecessary waypoints, which are sent to the simulator for execution. The drone transitions through each state based on position and velocity updates received via callbacks.


backyard_flyer_solution.py
This script defines a `BackyardFlyer` drone class that autonomously flies in a square pattern using a finite state machine. The drone starts in `MANUAL` mode, then transitions through `ARMING`, `TAKEOFF`, `WAYPOINT`, `LANDING`, and `DISARMING` based on sensor callbacks. During takeoff, it reaches a target altitude and then follows a sequence of predefined waypoints forming a box shape. Once the waypoints are completed and the drone slows down, it initiates landing and disarms upon touching down. The mission flow is handled through state transitions triggered by position and velocity updates.

Comparison:
The **MotionPlanning** script is a more advanced drone control program that uses A\* search and obstacle data to plan safe, optimized paths in complex environments. In contrast, **BackyardFlyer** follows a fixed square route without considering obstacles, making it ideal for simple flight tests. Both use a state machine, but MotionPlanning includes a PLANNING state for pathfinding, while BackyardFlyer transitions directly to takeoff. MotionPlanning suits real-world autonomous navigation, while BackyardFlyer is best for learning basic flight control.


Steps:

1. Implemented a function called read_colliders to estimate the globakl home position
2. Get the global position and based on that estimate the local position
3. Set a start and goal position with offsets
4. Extended the Astar lagotihtm with diagonal action
5. Implemented  a prune_path fubction with colinearity to remove redundant waypoints
6. sent waypoints