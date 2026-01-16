# Autonomous Navigation System for TurtleBot4 using ROS2
A modular ROS2-based software architecture for TurtleBot4, integrating perception, localization, path planning, and navigation. The system leverages an OAK-D PRO camera with YOLOv8 for cone detection, AMCL for localization, and Nav2 with A* planning to achieve reliable autonomous indoor navigation in dynamic environments.

# How To Run

To correctly run the project, carefully follow the steps below:

1. **Navigate to the project workspace** 
   Open a terminal and move to the main directory of the project workspace.

2. **Power on the TurtleBot4** 
   Make sure the TurtleBot4 robot is turned on and properly connected to the network. 
   If the robot's lights do not turn on at startup, it's likely that the services have not started correctly. 
   In this case, run the following command:
   ```bash
   sudo bash turtlebot4_restart_services.bash 192.168.0.100
   ```

3. **Build the project** 
   Run the build script located in the main project folder with the following command:
   ```bash
   ./build.bash
   ```

4. **Load the camera configuration** 
   Use `rqt` to load the YAML configuration file for the camera, located at:
   ```
   config/camera_conf.yaml
   ```
   This file enables the depth camera and correctly sets its parameters.

5. **Restart the camera** 
   To apply the parameters, restart the camera using the following commands:
   ```bash
   ros2 service call /oakd/stop_camera std_srvs/srv/Trigger "{}"
   ros2 service call /oakd/start_camera std_srvs/srv/Trigger "{}"
   ```

6. **Start the simulation** 
   Run the following script:
   ```bash
   ./start.bash
   ```

   This bash script automatically opens several terminal tabs and launches the main ROS2 components of the system. 
   The script performs the following actions:
   - **localization**: starts the localization node, which allows the robot to estimate its position in the environment.
   - **navigation Server**: starts the `nav2` navigation server, responsible for path planning.
   - **perception**: starts the perception nodes, which process information from sensors and cameras.
   - **navigation**: starts the control nodes responsible for robot navigation.
   - **TurtleBot4 Rviz**: launches the TurtleBot4 visualization tool, useful for real-time monitoring of the system and sensors via RViz.

By following these steps, the entire system will be correctly initialized and ready for execution in a simulated environment.
