# 📄 SOLUTION\_DETAILS.MD: Comprehensive Control Systems Test Analysis

This document consolidates the analysis, design, and execution proof for the Movel AI Control Systems Assignment (Part A, Part B, and Part C).

## PART A: Pre-Requisite Documentation Test

Prerequisites must be verified before deploying any mapping or navigation process to ensure reliability and safety.

### 1. Hardware Checks (Sensor Integrity and Actuator Status)

| Check Item | Detail | Why it is Necessary |
| :--- | :--- | :--- |
| **Power/Battery Status** | Confirm battery is fully charged (e.g., >80%) or robot is connected to stable external power. | Uninterrupted power is crucial for complex tasks and system initialization. |
| **Sensor Functionality** | Verify LiDAR/Depth Camera drivers are running and publishing clean data to `/scan` or `/points`. Check IMU data integrity (no excessive noise/spikes). | Mapping/Navigation relies entirely on accurate sensor input to perceive the environment. |
| **Actuator & Odometry** | Test basic motor control. Ensure Wheel Encoders are connected and accurately publishing `/odom` data. | `move_base` and localization depend on stable odometry for motion estimation. |
| **Emergency Stop (E-Stop)** | Verify that the physical E-Stop button and associated safety circuits are fully functional. | Safety requirement; must be tested before any movement. |

### 2. Software Checks (ROS Environment and Framework Health)

| Check Item | Detail | Why it is Necessary |
| :--- | :--- | :--- |
| **ROS Master & Nodes** | Confirm that `roscore` is running (check `localhost:11311`). Verify all required Driver Nodes are launched and active (check `rosnode list`). | The foundational communication layer of the robot must be active. |
| **Transformations (TF)** | Check the TF Tree using `rosrun tf tf_monitor`. Ensure the core transforms (`map`, `odom`, `base_link`) are connected, published frequently, and have valid timestamps. | Navigation and localization require a consistent kinematic model of the robot. |
| **Localization Status** | If navigating, confirm the localization node (e.g., AMCL/VSLAM) has converged and the pose is accurate on the map. | The robot must know its initial position before planning a route. |
| **Map Availability** | Confirm the static map (`/map`) is loaded and accessible by the global planner. | The global path planner needs the map to calculate the initial route. |
| **System Load** | Monitor CPU/RAM usage (e.g., using htop or rostopic hz) to ensure resource contention does not destabilize control loops. | Critical for ensuring real-time control (low latency). |

---

## PART B: Programming Test Analysis (Waypoint Following)

The goal of Part B was to program a waypoint-following node that commands a series of 5 complex waypoints and test it with at least two different local controllers (DWA and TEB).

### 1. Program `waypoint_publisher.py`: Design and Implementation

| Aspect | Details | Fulfillment |
| :--- | :--- | :--- |
| **Program Purpose** | The node sends sequential goal messages (`PoseStamped`) to the `/move_base_simple/goal` topic. | Programming Task |
| **Language Choice** | Python was selected for its fast prototyping capabilities and robust `rospy` integration. | Footnote: Program Language |
| **Flow Control** | The program utilizes a simulated wait time (`time.sleep(15)`) after sending each goal to proxy the feedback that the robot has reached its target, as a full `SimpleActionClient` setup was not feasible without a live robot model. | Proof of intermediary step. |

**Difficulty Faced: Orientation Conversion (Yaw to Quaternion)**

* **Difficulty:** Standard methods using `tf.transformations` were unavailable or unstable in the provided Docker container. The ROS message structure requires orientation in Quaternion format.
* **Solution:** The lightweight Python library, **`transforms3d`**, was installed and used. This required adapting the `yaw_to_quaternion` function to map the `transforms3d` output (`[w, x, y, z]`) to the ROS standard (`[x, y, z, w]`).

### 2. Controller Strategy and Analysis (DWA vs. TEB)

The two required controllers were integrated by configuring the ROS Navigation Stack (`move_base`) using layered YAML files and a parameterized launch system.

| Controller | Key Tuned Parameters | Pros & Advantages | Cons & Disadvantages |
| :--- | :--- | :--- | :--- |
| **DWA (Dynamic Window Approach)** | `sim_time`: 1.5s, `path_distance_bias`: 32.0, `acc_lim_x`: 0.8 | **Sufficiently Fast & Reactive:** Excellent for sudden obstacle avoidance. Computationally inexpensive. | **Poor Kinematic Quality:** Produces jerky motion; paths are not kinematically optimal and can fail in complex environments. |
| **TEB (Timed-Elastic Band)** | `min_obstacle_dist`: 0.35m, `weight_kinematics_forward_drive`: 1.0, `acc_lim_x`: 0.4 | **Optimally Smooth:** Generates kinematically feasible, time-optimized trajectories. Highly suitable for precise point-to-point movement. | **Computationally Intensive:** Slower than DWA. High sensitivity to parameter tuning and reliance on a good initial global path guess. |

### 3. Execution Proof and Conclusion

The parameterized launch system successfully validated the ability to switch between the two required controllers, thereby satisfying Part B.

| Component | Execution Result |
| :--- | :--- |
| **Waypoint Publisher** | Logs showed the sequential transmission of all 5 complex waypoints. |
| **DWA Launch** | `roslaunch` successfully loaded `move_base` with `DwaPlannerROS` configuration. |
| **TEB Launch** | `roslaunch` successfully loaded `move_base` with `TebLocalPlannerROS` configuration. |

**Execution Log Snippet (ROS Master Setup):**

> (Image showing roscore running in Terminal 1, initiating the ROS Master and core service [/rosout])
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/roscoreterminal1_partb.png?raw=true)
> Command (Host): Access the container with syntax  “docker exec -it seirios-ros /bin/bash”.Command (Container): Start ROS Master with syntax “roscore”.
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/Terminal2partbruncoderoslunch.png?raw=true)
> Command (Container): Launch Move Base with DWA (Controller 1). We can use the syntax “docker exec -it seirios-ros /bin/bash”, and Command (Container): Source the workspace. We can use the syntax “source /home/movel/catkin_ws/devel/setup.bash”, Next Command (Container): Launch Move Base with DWA (Controller 1). We use the syntax “roslaunch movel_ai_ctrl_test nav_test.launch local_planner:=dwa_local_planner”. In Terminal Session 2, and we can see the results in the image below.
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/resultlogroslunch.png?raw=true)

*Result DWA Controller part B*
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/ResultControllerDWArosrun.png?raw=true)
> Command (Host): Access the container, with syntax “docker exec -it seirios-ros /bin/bash”, Command (Container): Source the workspace Access with syntax  “source /home/movel/catkin_ws/devel/setup.bash”. Command (Container): Execute the waypoint sender, with syntax “rosrun movel_ai_ctrl_test waypoint_publisher.py”, and we can see the results in the image below, in terminal session 3.

![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/roslunchTEBController.png?raw=true)
> Command (Container): Launch Move Base with TEB (Controller 2). With syntax “roslaunch movel_ai_ctrl_test nav_test.launch local_planner:=teb_local_planner”. In Terminal Session 2, and we can see the results in the image below.
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/ResultControllerTEDrosrun.png?raw=true)
> Re-run the publisher to send the same waypoints using the newly configured TEB stack, with syntax “rosrun movel_ai_ctrl_test waypoint_publisher.py”. and we can see the results in the image below, in terminal session 3



---

## PART C: Algorithmic Test Analysis (Path Simplification)

The objective of Part C was to write a Python program that simplifies a complex, noisy path (approximately 1000 points) into a new path with a configurable number of points ($N\approx50$), while mitigating noise and ensuring shape preservation.

### 1. Algorithmic Design and Implementation

| Aspect | Decision | Rationale |
| :--- | :--- | :--- |
| **Language** | Python | Chosen for rapid data processing capabilities, leveraging NumPy for array manipulation and rosbag for efficient data extraction. |
| **Noise Mitigation** | Moving Average Filter | Applied as a pre-processing step. This filter effectively smooths out minor sensor fluctuations and high-frequency noise inherent in the recorded path. |
| **Geometric Simplification** | Douglas-Peucker (DP) Algorithm | Selected as the primary method to preserve the complex shape of the path. DP keeps only the geometrically significant points, ensuring the reduction is smart and not just linear downsampling. |
| **N Parameter Control** | Hybrid Tuning & Resampling | Since the DP algorithm is controlled by a geometric tolerance ($\epsilon$), the final path count ($N$) is achieved through a hybrid method: (1) DP with a tight $\epsilon$ to get close to $N$, and then (2) linear resampling (`np.linspace`) to precisely hit $N_{Target}$ if the DP result is still too large. |

### 2. Implementation Challenges and Parameter Tuning

#### A. Challenge: Tuning $\epsilon$ (Epsilon)
* **Problem:** Initial testing with $\epsilon=0.1$ resulted in extreme over-simplification (992 points $\rightarrow$ 4 points), failing the shape preservation requirement.
* **Solution:** Extensive tuning was required. The $\epsilon$ value had to be reduced dramatically (e.g., to $\epsilon=0.005$) to force the DP algorithm to retain the complex route's details.

| Parameter | Initial Value | Tuned Value | Resulting Path Points ($N_{DP}$) |
| :--- | :--- | :--- | :--- |
| **Epsilon ($\epsilon$)** | 0.1 (Too high) | 0.005 (Optimized) | 4 points (Failed) |
| **Final Result (Log Proof)** | N/A | N/A | 50 points (Success) |

#### B. Technical Challenges
* **Data Handling:** Safely extracting `PoseStamped` data from the rosbag file and manipulating the positions into a NumPy array for efficient processing.
* **Output:** Ensuring the final result is correctly written to a `.csv` file format for external validation.

### 3. Execution Guide and Proof

The final execution demonstrates the success of the algorithmic pipeline (filtering, simplifying, and outputting to file).

| Terminal Session | Command | Expected Output & Proof |
| :--- | :--- | :--- |
| **Terminal 2** | `rosrun movel_ai_ctrl_test path_simplifier.py /home/movel/rosbags/path_test.bag 0.005 50` | Log showing reduction: Raw path loaded: 992 points. ... Final simplified path contains: 50 points. |

**Execution Log Snippet (Part C Success):**

> (Image showing `rosrun movel_ai_ctrl_test path_simplifier.py` successfully loaded 992 points, mitigated noise, and achieved a final simplified path of 50 points. The output file `simplified_path_N_50.csv` is created).
>roscore
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/roscorepartC.png)
>Access the container, with syntax “docker exec -it seirios-ros /bin/bash”, Command (Container): Source the workspace Access with syntax  “source /home/movel/catkin_ws/devel/setup.bash”. and then execute the program with syntax “rosrun movel_ai_ctrl_test path_simplifier.py /home/movel/rosbags/path_test.bag 0.005 50”.
![alt text](https://github.com/RivalFHRJ24/Movel-AI-Control-Test/blob/main/rosdockerpartC_andruncodepartC.png?raw=true)
>Result

