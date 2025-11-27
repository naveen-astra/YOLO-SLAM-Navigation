# YOLO-SLAM-Navigation

A real-time navigation system integrating YOLO for object detection, ORB-SLAM3 for mapping and localization, and ADMM for trajectory optimization. The project demonstrates obstacle detection, mapping, and collision-free path planning in dynamic environments.

## Project Overview

YOLO-SLAM-Navigation combines real-time object detection, SLAM-based localization, and optimization-driven path planning. YOLO identifies obstacles, ORB-SLAM3 constructs a map and tracks the agent's pose, and an ADMM-based planner generates optimal collision-free trajectories. The system adapts to dynamic changes by re-planning when new obstacles appear.

## Key Features

- Object detection using YOLO for obstacle identification
- Mapping and localization via ORB-SLAM3
- ADMM-based optimized path planning
- Dynamic obstacle handling and re-planning
- Scene and trajectory visualization
- Modular MATLAB-based implementation

## Workflow

1. Capture frames from a mono, stereo, or RGB-D camera.
2. Use YOLO to detect objects and extract obstacle information.
3. Use ORB-SLAM3 to build/update the environment map and track position.
4. Feed detected obstacles and current state to the ADMM planner.
5. Generate and visualize an optimal collision-free trajectory.
6. Re-run planning if obstacles or map conditions change.

## Example Usage

Clone the repository:
git clone https://github.com/naveen-astra/YOLO-SLAM-Navigation.git
cd YOLO-SLAM-Navigation

Run the main script:
matlab -batch "run('work.m')"

Primary workflow scripts include:
- slam + detection → obstacle map
- admm_planner.m → optimized path
- visualize_scene.m → environment and trajectory visualization

## Project Structure

YOLO-SLAM-Navigation/
├── admm_planner.asv / admm_planner.m       # ADMM-based path planner
├── admm_solver.m                           # Solver for trajectory optimization
├── bicycle_model.m / bicycle_update.m      # Vehicle dynamics (bicycle model)
├── collision_penalty.m                     # Collision cost function
├── get_obstacles.m / transform_obstacles.m # Converts detections to obstacle list
├── update_trajectory.m                     # Adjusts path based on environment
├── visualize_scene.m                       # Map, obstacles, and path visualization
├── work.m                                  # Main script to run the entire pipeline
├── data/                                   # Optional datasets or inputs
└── README.md                               

## Requirements

- MATLAB (or GNU Octave where compatible)
- YOLO model output or integration script
- ORB-SLAM3 (native or wrapped for use with your pipeline)
- Basic image processing and optimization toolboxes

## Use Cases

- Robot navigation in dynamic environments
- Autonomous vehicle path planning experiments
- Drone navigation simulations
- Research in SLAM, robotics, and optimization
- Integration prototype for perception + mapping + planning systems

## Notes

- Replace paths or parameter settings in work.m as required for your system.
- Ensure YOLO detections and ORB-SLAM3 outputs are synchronized correctly.

## Contributors

Naveen Babu
Kishore
Koushal Reddy
Sai Charan


