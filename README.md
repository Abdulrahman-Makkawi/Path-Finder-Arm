3-DOF Robot Arm Path Tracker:

This MATLAB project simulates and visualizes a 3-Degree-of-Freedom (3-DOF) robot arm. It allows users to set multiple target goals in a 2D environment and attempts to reach each goal sequentially while avoiding predefined circular obstacles. The arm's movement is controlled using a basic gradient descent-like approach on its joint angles, coupled with collision detection.

Features:

    -Interactive Goal Setting: Click anywhere on the plot to dynamically add multiple target goals for the robot arm.

    -Multi-Goal Path Planning: The arm attempts to reach each set goal one after another.

    -Real-time Visualization: See the robot arm's movement and its attempts to reach the goals in real-time.

    -Configurable Arm Parameters: Easily adjust the lengths of each arm segment (L1, L2, L3) directly from the UI.

    -Obstacle Avoidance: The arm is programmed to detect and avoid circular obstacles in its path. Obstacles can be customized.

    -Adjustable Algorithm Parameters: Fine-tune the goal threshold, learning rate (alpha), and the number of epochs for movement.

    -Visual Feedback: Goals turn green once successfully reached.

Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

Prerequisites

You need MATLAB installed on your system to run this project.

Installation

    -Download the files: Save the robotArm3DOF_MultiGoalPath.m file to a directory on your computer.

    -Launch MATLAB.

    -Navigate to the directory: In MATLAB's Current Folder browser, navigate to the directory where you saved the files.

How to Run

    -Run the main function: In the MATLAB Command Window, type: "robotArm3DOF_MultiGoalPath" and press Enter.

    -Set Goals: A figure window will appear. Click anywhere within the axes to set desired target goals for the robot arm. You'll see red 'x' markers appear at each goal location.

    -Adjust Parameters (Optional): Modify the values for L1, L2, L3, Threshold, Obstacles, Alpha, and Epochs in the UI panel on the left if you wish.

    -Obstacles: Enter obstacles as a matrix, where each row is [x_center, y_center, radius]. For example, [1.5, 1.5, 0.3; -2.0, 0.5, 0.4] defines two obstacles.

    -Start Simulation: Click the "Start" button.

    -Observe: The robot arm will begin moving towards the first goal, then subsequent goals. Reached goals will turn green.

How it Works

The project uses a simple iterative approach to move the robot arm:

    Forward Kinematics: Calculates the end-effector position (x, y) and the positions of all joints based on the current joint angles (theta).

    Goal Seeking: In each epoch, the arm attempts to adjust its joint angles (theta) slightly. If the new configuration moves the end-effector closer to the goal and does not result in a collision, the new angles are adopted.

    Collision Detection: Before accepting new joint angles, the checkCollision function verifies if any segment of the arm intersects with any circular obstacle.

    Multi-Goal Handling: The startCallback function iterates through the list of user-defined goals, calling moveToGoal for each one.

    Visualization: The plot is updated in real-time to show the arm's current configuration, the goals, and the obstacles.

Customization

You can easily customize various aspects of the simulation:

    Arm Lengths: Change L1, L2, L3 values in the UI.

    Goal Threshold: Adjust Threshold to define how close the arm needs to get to a goal to consider it "reached."

    Learning Rate (Alpha): Controls the step size for angle adjustments. A larger alpha might lead to faster but potentially less stable movement.

    Epochs: The maximum number of iterations the arm will attempt to reach a single goal.

    Obstacles: Modify the Obstacles input in the UI or directly in the code to add, remove, or change the position and size of circular obstacles. Each obstacle is defined by its [x_center, y_center, radius].

Contributing

Feel free to fork this repository and contribute! Whether it's improving the movement algorithm, adding different types of obstacles, or enhancing the UI, all contributions are welcome.

License

This project is open-source and available under the MIT License.

Contact

If you have any questions or suggestions, feel free to reach out!
