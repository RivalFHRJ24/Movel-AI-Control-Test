# Movel AI - Robotics Intern Control Systems Assignment

This repository contains the complete solution for the Control Systems Test (Part A, B, and C), built inside the specified **Docker (Ubuntu 20.04/ROS-Noetic)** environment.

## 1. Environment Setup and Launch

The solution requires the provided Movel AI Docker environment.

1.  **Launch the Environment:** Navigate to the directory containing your modified `docker-compose.yaml` (e.g., `.../files/catkin_ws/movel_ai/`) and execute:
    ```bash
    docker compose up -d
    ```
2.  **Start ROS Master (Terminal 1):** Access the container and start the core system manually (as the default entrypoint was overridden for stability).
    ```bash
    docker exec -it seirios-ros /bin/bash
    roscore
    ```

## 2. Program Execution Instructions

All program execution must be done in new Docker sessions after sourcing the workspace (`source /home/movel/catkin_ws/devel/setup.bash`).

### A. PART C: Algorithmic Test (Path Simplification)

*(Goal: Simplify the 992-point path to N=50 points while mitigating noise.)*

1.  **Execute Command:** Run the program with the required parameters (Epsilon and Target N).
    ```bash
    # In Terminal 2 (after sourcing workspace)
    rosrun movel_ai_ctrl_test path_simplifier.py /path_test.bag 0.005 50
    ```
    *Proof:* The output log provides immediate proof of path reduction, and the `simplified_path_N_50.csv` file is generated.

### B. PART B: Programming Test (Waypoint Following - 2 Controllers)

*(Goal: Send 5 complex waypoints, verifying DWA and TEB configurations.)*

1.  **Test 1 (DWA Planner):** Launch the navigation stack configured for DWA, then launch the waypoint sequence.
    ```bash
    # In Terminal 3
    roslaunch movel_ai_ctrl_test nav_test.launch local_planner:=dwa_local_planner
    # In Terminal 4 (Execute Publisher)
    rosrun movel_ai_ctrl_test waypoint_publisher.py
    ```
2.  **Test 2 (TEB Planner):** Stop the DWA launch (`Ctrl+C`), then restart the stack configured for TEB.
    ```bash
    # In Terminal 3 (Stop previous launch, then run new launch)
    roslaunch movel_ai_ctrl_test nav_test.launch local_planner:=teb_local_planner
    # In Terminal 4 (Re-execute Publisher)
    rosrun movel_ai_ctrl_test waypoint_publisher.py
    ```

## 3. Documentation and Analysis

* **Comprehensive Technical Analysis:** Refer to **[SOLUTION_DETAILS.md]** for full details on **Part A** Prerequisite Checks, **Part B** Controller Analysis (Pros/Cons, Tuning), **Part C** Algorithmic Design (DP vs. Moving Average), and documentation of all difficulties faced.
