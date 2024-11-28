# ROS Turtle Control

This ROS (Robot Operating System) project controls two turtles in the `turtlesim` simulation using a user-friendly terminal interface built with the `ncurses` C++ library. The system allows real-time control of the turtles' linear and angular velocities, while also ensuring they avoid collisions and stay within the boundaries of the simulation environment.

This project includes two solutions to maintain safety during turtle movement:

- **Solution 1**: A manual safety check where proposed velocities are validated before being published.
- **Solution 2**: A dynamic safety mechanism where the turtles' velocities are directly controlled, and their movement is stopped automatically if they come too close to each other or the boundaries.

## Features

- **User-friendly interface**: A terminal-based interface for controlling the turtles' velocities using the `ncurses` library.
- **Collision avoidance**: Automatically stops turtles if they come too close to each other or the boundaries.
- **Boundary protection**: Ensures that turtles stay within the simulation's defined area.
- **Real-time velocity adjustment**: Users can update turtle velocities in real-time, with automatic safety checks.

## Dependencies

This project relies on the following dependencies:

- **ROS (Robot Operating System)**
- **`turtlesim` package**
- **`ncurses` library** for creating the user interface

## File Overview

### Solution 1: Manual Safety Check (Files: `ui1.cpp` and `distance1.cpp`)

#### 1. `ui1.cpp`

This file creates a user interface (UI) for controlling the turtles using the `ncurses` library. It allows the user to interact with the turtles by entering linear and angular velocities, which are then validated and published.

- **Key Functions**:
  - **`killExistingTurtle`**: Kills any existing turtles before spawning new ones.
  - **`spawnTurtle`**: Spawns two turtles in the simulation at predefined positions.
  - **`drawBox`**: Draws a control box for each turtle's control panel in the terminal.
  - **`updateVelocities`**: Displays the current linear and angular velocities for each turtle.
  - **`editVelocities`**: Allows the user to enter new velocity values for the active turtle.
  - **`main`**: Initializes the ROS node, handles user input, switches between turtles, and publishes velocity commands to the turtles.

- **How it works**: 
  - The UI allows switching between Turtle 1 and Turtle 2 using the left or right arrow keys.
  - When the user presses `Enter`, the UI allows for the input of new velocity values for the active turtle.
  - The velocities are then published to `/proposed_vel_turtle1` or `/proposed_vel_turtle2` topics, depending on the active turtle.

#### 2. `distance1.cpp`

This file contains logic to ensure that the turtles' velocities are safe by checking their projected paths. It listens to the turtles' pose information and proposed velocity commands and decides whether it is safe to apply those velocities.

- **Key Functions**:
  - **`turtle1PoseCallback`** and **`turtle2PoseCallback`**: These callbacks subscribe to the pose topics of Turtle 1 and Turtle 2 (`/turtle1/pose` and `/turtle2/pose`), updating their respective pose data.
  - **`isSafe`**: Determines if a proposed velocity will result in a collision or boundary violation by simulating the turtle's future position over a small time interval.
  - **`proposedVelocityTurtle1Callback`** and **`proposedVelocityTurtle2Callback`**: These functions receive proposed velocities for Turtle 1 and Turtle 2, check if they are safe using the `isSafe` function, and if so, publish them to the appropriate velocity topic (`/turtle1/cmd_vel` or `/turtle2/cmd_vel`).

- **How it works**:
  - The `distance1.cpp` file works in the background, ensuring that only safe velocities are published to the turtles.
  - It listens to the proposed velocity topics (`/proposed_vel_turtle1` and `/proposed_vel_turtle2`) and validates whether the new velocities would lead to a collision with the other turtle or a boundary violation.
  - If the velocity is deemed safe, it publishes the velocity command to the respective turtle's velocity topic.

---

### Solution 2: Dynamic Safety Check (Files: `ui2.cpp` and `distance2.cpp`)

#### 1. `ui2.cpp`

This file is an alternative UI implementation for controlling the turtles. Similar to the first solution, it uses `ncurses` for the terminal-based interface but with a different approach to safety.

- **Key Functions**:
  - **`killExistingTurtle`**: Kills any existing turtles before spawning new ones.
  - **`spawnTurtle`**: Spawns two turtles in the simulation at predefined positions.
  - **`drawBox`**: Draws a control box for each turtle's control panel in the terminal.
  - **`updateVelocities`**: Displays the current linear and angular velocities for each turtle.
  - **`editVelocities`**: Allows the user to enter new velocity values for the active turtle.
  - **`main`**: Initializes the ROS node, handles user input, and publishes velocity commands to the turtles.

- **How it works**: 
  - Similar to Solution 1, the user can switch between Turtle 1 and Turtle 2 and edit their velocities in real-time.
  - However, in this solution, the proposed velocities are directly sent to the `/turtle1/cmd_vel` or `/turtle2/cmd_vel` topics without pre-validation. If a turtle comes too close to the other, the movement will be stopped automatically.

#### 2. `distance2.cpp`

This file monitors the turtles' positions and ensures that the turtles' velocities are safe by stopping the turtles if they come too close to each other or the simulation's boundaries.

- **Key Functions**:
  - **`isSafeWithBoundaries`**: Checks if a turtle is approaching the boundaries of the simulation and will cause a collision.
  - **`confirmIsSafe`**: Verifies if the distance between the turtles is safe and whether each turtle is within the simulation boundaries.
  - **`publishDistance`**: Calculates the distance between the two turtles and publishes it.
  - **`turtle1PoseCallback`** and **`turtle2PoseCallback`**: These callbacks update the poses of Turtle 1 and Turtle 2 and trigger safety checks.
  - **`main`**: Runs a loop to check the safety status of the turtles and stops them if necessary.

- **How it works**:
  - The `distance2.cpp` file checks the distance between the two turtles and validates their positions in real-time.
  - If the turtles come too close to each other (below the minimum safety threshold) or approach the boundaries, they are stopped automatically by publishing a `Twist` message with zero velocities.

## How to Run

1. **Start ROS environment**:
   Run this command to ensure the project runs successfully
   ```
   roscore
   ```
2. **Start the turtlesim environment**:
   Run the following commands to launch the `turtlesim` simulation:
   ```
   rosrun turtlesim turtlesim_node
   ```

3. **Run Solution 1**:
   For the manual safety check solution, run the following commands:
   ```
   rosrun assignment1 ui1_node
   rosrun assignment1 distance1_node
   ```

4. **Run Solution 2**:
   For the dynamic safety check solution, run the following commands:
   ```
   rosrun assignment1 ui2_node
   rosrun assignment1 distance2_node
   ```

## Notes

- Ensure that your ROS environment is set up properly before running the above commands.
- The turtles' velocities are adjustable via the terminal UI, but safety checks will prevent unsafe movement, either by pre-validation (Solution 1) or automatic stopping (Solution 2).
  
## License

This project is licensed under the MIT License - see the [LICENSE](rt1/my_ros/src/assignment1_rt/src/LICENSE) file for details.
