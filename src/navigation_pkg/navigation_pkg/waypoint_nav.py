import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions, TaskResult
import time
import os
from math import sqrt
from collections import deque
from custom_interfaces_pkg.msg import Waypoint
from custom_interfaces_pkg.srv import PointTransformer
from irobot_create_msgs.msg import KidnapStatus
from math import atan2
from math import sin, cos
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import math
from enum import Enum, auto
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav2_msgs.srv import ClearEntireCostmap

# Configurable parameters
DEBUG = False  # Enable debug logging

SERVICE_TIMEOUT_MS = 50  # Timeout for service calls, in milliseconds
SERVICE_WAIT_TIMEOUT_SEC = 120.0  # Timeout for waiting for service availability, in seconds
INITIAL_WAIT_TIME = 3.0  # Initial wait time before processing (e.g., for receiving initial waypoints), in seconds
MAX_TIME_WAYPOINTS = 15.0  # Maximum time to consider a waypoint as valid, in seconds

# Navigation parameters
MAXLEN_WAYPOINTS = 10  # Maximum number of waypoints to store in the queue
LOOP_PERIOD = 0.5  # Main loop execution period, in seconds
NUM_MIN_WAYPOINTS = 2  # Minimum number of waypoints required to start navigation
WAYPOINT_SIMILARITY_TOLERANCE = 0.03  # Distance threshold (in meters) to consider a waypoint as similar to the previous goal (prevents re-sending close goals)
OUTLIER_DISTANCE_THRESHOLD = 2.0  # Threshold to detect outlier waypoints based on distance, in meters
DEGREES_TO_MOVE_AROUND = 15  # Degrees to rotate when exploring surroundings at a waypoint
TIME_TO_MOVE_AROUND = 0.5  # Time to rotate for each step during "look around" behavior, in seconds
VIEW_AROUND = False  # If True, enables rotation to look around for cones after reaching a waypoint
RADIUS = 0.2  # Radius (in meters) used localiation state

def quaternion_from_euler(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
    qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
    qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
    qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
    return (qx, qy, qz, qw)
 
def compute_orientation_towards(current_pose: PoseStamped, target_pose: PoseStamped):
    dx = target_pose.pose.position.x - current_pose.pose.position.x
    dy = target_pose.pose.position.y - current_pose.pose.position.y
    yaw = atan2(dy, dx)
    q = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion()
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    return orientation

DIRECTIONS_MAP = {
    'TurtleBot4Directions.NORTH': TurtleBot4Directions.NORTH,
    'TurtleBot4Directions.SOUTH': TurtleBot4Directions.SOUTH,
    'TurtleBot4Directions.EAST': TurtleBot4Directions.EAST,
    'TurtleBot4Directions.WEST': TurtleBot4Directions.WEST,
}

def parse_config_file(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Config file {filepath} not found.")
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    if len(lines) < 2:
        raise ValueError("Config file must contain at least two lines.")
    def parse_line(line):
        parts = [p.strip() for p in line.split(',')]
        if len(parts) != 3:
            raise ValueError(f"Invalid config line: {line}")
        x, y = float(parts[0]), float(parts[1])
        direction = DIRECTIONS_MAP.get(parts[2])
        if direction is None:
            raise ValueError(f"Invalid direction in config: {parts[2]}")
        return x, y, direction
    start_x, start_y, start_dir = parse_line(lines[0])
    goal_x, goal_y, goal_dir = parse_line(lines[1])
    return (start_x, start_y, start_dir), (goal_x, goal_y, goal_dir)

class NavState(Enum):
    GO_TO_FINAL_GOAL = auto()
    FINAL_GOAL_REACHED = auto()
    GO_TO_WAYPOINT = auto()
    WAYPOINT_REACHED = auto()
    WAYPOINT_FAILED = auto()
    VIEW_AROUND = auto()
    KIDNAPPED = auto()
    LOCALIZE = auto()

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_interruptible_navigator')
        self.navigator = TurtleBot4Navigator()

        # State variables
        self.waypoint_service_processing = False
        self.distance = None
        self.pending_waypoints = deque(maxlen=MAXLEN_WAYPOINTS)
        self.current_waypoint = None
        self.goal_pose = None
        self.first_loop = True
        self.rotation_start_time = None
        self.waypoint_timer = None
        self.kidnapped = False
        self.accepting_waypoint = True
        self.costmap_service_busy = False
        angular_velocity = math.radians(DEGREES_TO_MOVE_AROUND) / TIME_TO_MOVE_AROUND
        self.rotation_sequence = [
            (0.0, TIME_TO_MOVE_AROUND),                    # 0. STOP 
            (-angular_velocity, TIME_TO_MOVE_AROUND),      # 1. RIGHT DEGREES_TO_MOVE_AROUND°
            (0.0, 2 * TIME_TO_MOVE_AROUND),                    # 2. STOP
            (2 * angular_velocity, TIME_TO_MOVE_AROUND),   # 3. LEFT 2*DEGREES_TO_MOVE_AROUND°
            (0.0, 2 * TIME_TO_MOVE_AROUND),                    # 4. STOP
            (-angular_velocity, TIME_TO_MOVE_AROUND),      # 5. Right DEGREES_TO_MOVE_AROUND°
            (0.0, TIME_TO_MOVE_AROUND)                     # 6. STOP
        ]
        
        # Initial parsing config
        start, goal = parse_config_file('./config/path_config.txt')
        self.initial_pose = self.navigator.getPoseStamped([start[0], start[1]], start[2])
        self.goal_pose = self.navigator.getPoseStamped([goal[0], goal[1]], goal[2])
        self.current_orientation = self.initial_pose.pose.orientation

        # Navigation nav2
        self.get_logger().info("Waiting Nav2...")
        self.navigator.waitUntilNav2Active()

        # Setting initial pose
        self.get_logger().info("Init navigation...")
        self.navigator.setInitialPose(self.initial_pose)

        # Service compute_waypoint -> client
        self.transformer_client = self.create_client(PointTransformer, 'transform_waypoint')
        if not self._wait_for_service(self.transformer_client, 'transform_waypoint'):
            self.destroy_node()
            rclpy.shutdown()
            return
        
        # Service clear local costmap -> client
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        if not self._wait_for_service(self.clear_costmap_client, '/local_costmap/clear_entirely_local_costmap'):
            self.destroy_node()
            rclpy.shutdown()
            return

        # Cmd vel publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscription waypoint topic
        self.create_subscription(Waypoint, '/goal_waypoint', self.waypoint_callback, 10)

        # Subscription kidnap topic
        qos = QoSProfile(reliability=ReliabilityPolicy. BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(KidnapStatus, '/kidnap_status', self.kidnap_callback, qos)
        
        # If is docked -> undock
        if self.navigator.getDockedStatus():
            self.navigator.undock()

        # Start loop
        self.get_logger().info("Nav2 active...starting navigation.")
        self.state = NavState.GO_TO_FINAL_GOAL # Initial state
        self.clear_local_costmap()
        self.create_timer(LOOP_PERIOD, self.navigation_loop)
        self.initial_time = time.time()

    ####################### SERVICE FUNCTIONS ###############################
    def _wait_for_service(self, client, service_name):
        "Waits for the availability of a service with enhanced logging and a global timeout"
        self.get_logger().info(f'Waiting service {service_name}...')
        start_time = time.time()

        while not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            elapsed = time.time() - start_time
            self.get_logger().warn(f'Service {service_name} not avaiable, retry...')
            if elapsed >= SERVICE_WAIT_TIMEOUT_SEC - 1:
                self.get_logger().error(f'Service {service_name} not avaiable after {SERVICE_WAIT_TIMEOUT_SEC - 1} seconds. Stopping node...')
                return False

        self.get_logger().info(f'Service {service_name} avaiable')
        return True
    
    def handle_service_response(self, future):
        try:
            # if the timer has expired
            if future.cancelled():
                self.get_logger().error("Detection service deleted")
                self.waypoint_service_processing = False
                return
            
            response = future.result()
            if response.success:
                tp = response.transformed_point
                if DEBUG:
                    self.get_logger().info(
                        f"Transformed point: x={tp.x:.2f}, y={tp.y:.2f}, z={tp.z:.2f}"
                    )

                # Salva nella coda dei pending waypoint
                self.pending_waypoints.append((tp, self.distance))

            else:
                self.get_logger().warn("Transformation failed, waypoint ignored.")
            
            self.waypoint_service_processing = False

        except Exception as e:
            self.get_logger().error(f"Error during service call: {e}")
            self.waypoint_service_processing = False

    ####################### TOPIC CALLBACK FUNCTIONS ###############################
    def kidnap_callback(self, msg: KidnapStatus):
        self.kidnapped = msg.is_kidnapped

    def waypoint_callback(self, msg: Waypoint):
        if DEBUG:
            self.get_logger().info("Waypoint received")

        if not self.accepting_waypoint:
            if DEBUG:
                self.get_logger().info("Waypoint ignored")
            return

        if not self.transformer_client.wait_for_service(timeout_sec=1.0):
            if DEBUG:
                self.get_logger().warn("Transformation service not available")
            return
                    
        self.waypoint_service_processing = True
        self.distance = msg.distance
        
        # Prepare the request
        request = PointTransformer.Request()
        request.point = msg.point

        future = self.transformer_client.call_async(request)

        # Init timeout service
        def timeout_callback():
            if not future.done():
                self.get_logger().error(f"TIMEOUT detection service ({SERVICE_TIMEOUT_MS}ms)")
                future.cancel()
                self.waypoint_service_processing = False

        # Crea timer per timeout (converti ms in secondi)
        timeout_timer = self.create_timer(SERVICE_TIMEOUT_MS / 1000.0, timeout_callback)

        def cleanup_and_callback(fut):
            timeout_timer.cancel()  # Cancella il timer
            self.handle_service_response(fut)
        
        future.add_done_callback(cleanup_and_callback)

    ####################### NAVIGATION LOOP ###############################
    def navigation_loop(self):
        self.get_logger().info(f"Current state: {self.state.name}")
        now = time.time()

        # Time needed to collect any initial waypoints.
        if self.first_loop and now - self.initial_time <= INITIAL_WAIT_TIME: 
            return
        else:
            self.first_loop = False
        
        # KIDNAPPED
        if self.state == NavState.KIDNAPPED:
            self.handle_kidnapped_state()
            return

        # GO TO FINAL GOAL
        if self.state == NavState.GO_TO_FINAL_GOAL:
            self.handle_go_to_final_goal()
            return
        
        # FINAL_GOAL_REACHED
        if self.state == NavState.FINAL_GOAL_REACHED:
            self.handle_final_goal_reached()
            return

        # GO TO WAYPOINT
        if self.state == NavState.GO_TO_WAYPOINT:
            self.handle_go_to_waypoint()
            return

        # WAYPOINT REACHED
        if self.state == NavState.WAYPOINT_REACHED:
            self.handle_waypoint_reached()
            return

        # WAYPOINT FAILED
        if self.state == NavState.WAYPOINT_FAILED:
            self.handle_waypoint_failed()
            return

        # VIEW AROUND
        if self.state == NavState.VIEW_AROUND:
            self.handle_view_around()
            return

        # LOCALIZE
        if self.state == NavState.LOCALIZE:
            self.handle_localize()
            return

    ####################### STATE HANDLERS #######################

    def handle_go_to_final_goal(self):
        self.accepting_waypoint = True

        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            self.state = NavState.KIDNAPPED
            self.is_navigating_to_goal = False
            return  
        
        # If I have waypoints, go to the next waypoint
        if len(self.pending_waypoints) >= NUM_MIN_WAYPOINTS:
            self.state = NavState.GO_TO_WAYPOINT
            self.is_navigating_to_goal = False
            return
        
        # If the task is completed
        if self.navigator.isTaskComplete():
            result = self.check_results()
            if result == TaskResult.SUCCEEDED and self.is_navigating_to_goal:
                self.state = NavState.FINAL_GOAL_REACHED
                self.is_navigating_to_goal = False
                return
            else:
                self.goToFinalPose()
                self.is_navigating_to_goal = True


        # Otherwise, go to the final goal
        if not self.is_navigating_to_goal:
            self.goToFinalPose()
            self.is_navigating_to_goal = True
            return
    

    def handle_final_goal_reached(self):
        self.accepting_waypoint = False
        self.get_logger().info("Final goal reached. Stopping navigation")
        self.destroy_node()
        rclpy.shutdown()  
        
    
    def handle_go_to_waypoint(self):
        self.accepting_waypoint = True

        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            self.state = NavState.KIDNAPPED
            self.cancel_waypoint_timer()
            self.current_waypoint = None
            return   
        
        # Task completed: deciding how to proceed, failure or success.
        if self.navigator.isTaskComplete():
            result = self.check_results()
            if result == TaskResult.SUCCEEDED:
                if not self.is_navigating_to_goal and self.current_waypoint is not None:
                    if DEBUG:
                        self.get_logger().info("Waypoint reached")
                    self.state = NavState.WAYPOINT_REACHED
                    self.cancel_waypoint_timer()
                    self.current_waypoint = None
                    return
            elif result in [TaskResult.FAILED, TaskResult.CANCELED]:
                if DEBUG:
                    self.get_logger().info("Navigation failed")
                self.state = NavState.WAYPOINT_FAILED
                self.cancel_waypoint_timer()
                self.current_waypoint = None
                return
                

        # If I don’t have enough waypoints, go to the final goal
        if len(self.pending_waypoints) < NUM_MIN_WAYPOINTS:
            self.state = NavState.GO_TO_FINAL_GOAL
            self.cancel_waypoint_timer()
            self.current_waypoint = None
        else:
            # Go to the found waypoint
            result = self.get_filtered_average_pose()
            if not result:
                return
            best_pose, avg_dist = result
            if self.current_waypoint and self.is_similar_pose(self.current_waypoint, best_pose):
                return  # Similar waypoint, ignore it
            
            # START NAVIGATION TOWARDS WAYPOINT
            self.start_navigation_to(best_pose)

            # If there is already an active timer, cancel it
            self.cancel_waypoint_timer()

            def on_waypoint_timeout():
                if DEBUG:
                    self.get_logger().warn("⏱️ Timeout raggiunto per il waypoint! waypoint fallito.")
                self.waypoint_timer = None
                self.state = NavState.WAYPOINT_FAILED
                self.current_waypoint = None

            # Create a new timer that calls the timeout function after MAX_TIME_WAYPOINTS seconds
            self.waypoint_timer = self.create_timer(MAX_TIME_WAYPOINTS, on_waypoint_timeout)

    def handle_waypoint_reached(self):
        self.pending_waypoints.clear()
        self.accepting_waypoint = True
        if DEBUG:
            self.get_logger().info("Waypoint reached")

        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            self.state = NavState.KIDNAPPED
            return
        
        # If I have new waypoints, go to them
        if len(self.pending_waypoints) >= NUM_MIN_WAYPOINTS:
            self.state = NavState.GO_TO_WAYPOINT
            return
        
        # Look around or go to the final goal
        if VIEW_AROUND:
            self.state = NavState.VIEW_AROUND
        else:
            self.state = NavState.GO_TO_FINAL_GOAL

    def handle_waypoint_failed(self):
        self.pending_waypoints.clear()
        self.accepting_waypoint = True

        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            self.state = NavState.KIDNAPPED
        else:
            # If I have new waypoints, go to them
            if len(self.pending_waypoints) >= NUM_MIN_WAYPOINTS:
                self.state = NavState.GO_TO_WAYPOINT
                return
            else:
                self.state = NavState.GO_TO_FINAL_GOAL

    def handle_view_around(self):
        self.accepting_waypoint = True

        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            self.state = NavState.KIDNAPPED
            # Reset per la prossima volta
            if hasattr(self, 'rotation_step'):
                delattr(self, 'rotation_step')
            return   
        
        # Se ho nuovi waypoint vado al waypoint
        if len(self.pending_waypoints) >= NUM_MIN_WAYPOINTS:
            self.state = NavState.GO_TO_WAYPOINT
            # Reset per la prossima volta
            if hasattr(self, 'rotation_step'):
                delattr(self, 'rotation_step')
            return
        
        # If I have new waypoints, go to them
        if not hasattr(self, 'rotation_step'):
            self.rotation_step = 0
            self.rotation_start_time = time.time()
            if DEBUG:
                self.get_logger().info("Starting rotation sequence to look around")
        
        current_time = time.time()
        elapsed_time = current_time - self.rotation_start_time
        
        
        if self.rotation_step < len(self.rotation_sequence):
            angular_vel, duration = self.rotation_sequence[self.rotation_step]
            
            # If enough time has passed for this step
            if elapsed_time >= duration:
                # Move to the next step
                self.rotation_step += 1
                self.rotation_start_time = current_time
                
                # If the sequence is finished
                if self.rotation_step >= len(self.rotation_sequence):
                    # Stop the robot
                    twist = Twist()
                    twist.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist)
                    
                    # Reset for next time
                    delattr(self, 'rotation_step')
                    
                    if DEBUG:
                        self.get_logger().info("Rotation sequence completed")
                    
                    # Go to final goal
                    self.state = NavState.GO_TO_FINAL_GOAL
                    return
            
            # Publish angular velocity command
            twist = Twist()
            twist.linear.x = 0.0 
            twist.angular.z = angular_vel
            self.cmd_vel_publisher.publish(twist)
            
            if DEBUG and elapsed_time < 0.1:  # Log only at the start of each step
                direction = "right" if angular_vel < 0 else "left" if angular_vel > 0 else "still"
                degrees = abs(angular_vel * TIME_TO_MOVE_AROUND * 180 / math.pi)
                self.get_logger().info(f"Step {self.rotation_step}: {direction} ({degrees:.0f}°)")
        
        else:
            # Fallback
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            
            if hasattr(self, 'rotation_step'):
                delattr(self, 'rotation_step')
            
            self.state = NavState.GO_TO_FINAL_GOAL
        

    def handle_kidnapped_state(self):
        self.accepting_waypoint = False

        # If placed back, clear the costmap and localize
        if not self.kidnapped:
            if not self.costmap_service_busy:
                self.clear_local_costmap()
                self.state = NavState.LOCALIZE
        else:
            if DEBUG:
                self.get_logger().info("ROBOT LIFTED!")
            # Clear waypoints
            self.pending_waypoints.clear()
            self.navigator.cancelTask()
            self.current_waypoint = None
            self.is_navigating_to_goal = False 

    def handle_localize(self):
        self.accepting_waypoint = False
        if DEBUG:
                self.get_logger().info("LOCALIZATION!")  
        
        # If the robot is lifted, switch to kidnapped state
        if self.kidnapped:
            # Reset localization attribute
            if hasattr(self, 'localize_start_time'):
                delattr(self, 'localize_start_time')
            self.state = NavState.KIDNAPPED
            return
        
        # Initialize localization if not already started
        if not hasattr(self, 'localize_start_time'):
            self.localize_start_time = time.time()
            if DEBUG:
                self.get_logger().info("Starting localization - rotating in place")
        
        current_time = time.time()
        elapsed_time = current_time - self.localize_start_time
        
        # Time and angular speed to complete the circle with linear speed = 0.2 m/s
        total_time = (2 * math.pi * RADIUS) / 0.2
        angular_speed = 2 * math.pi / total_time
        
        if elapsed_time < total_time:
            # Continue circular movement
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = angular_speed
            self.cmd_vel_publisher.publish(twist)
            
            if DEBUG and elapsed_time < 0.1:  # Initial log
                self.get_logger().info(f"Localization: circle radius {RADIUS}m, total time {total_time:.1f}s")
        
        # Mi fermo per vedere waypoints
        elif total_time < elapsed_time and elapsed_time < (total_time + total_time/3.0):
            # Stop to observe waypoints
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            if DEBUG:
                self.get_logger().info("Stopping to observe waypoints")
            
        else:
            if DEBUG:
                self.get_logger().info("Localization completed")
            # Return to goal finale
            delattr(self, 'localize_start_time') # Reset for the next localization
            self.state = NavState.GO_TO_FINAL_GOAL

        
    ####################### UTILS FUNCTIONS ###############################
    def goToFinalPose(self):
        self.navigator.goToPose(self.goal_pose)
        self.current_waypoint = None

    def start_navigation_to(self, target_pose: PoseStamped):
        self.navigator.goToPose(target_pose)
        self.current_waypoint = target_pose
        self.is_navigating_to_goal = False
        if DEBUG:
            self.get_logger().info(f"Navigating to new waypoint:({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})")

    def cancel_waypoint_timer(self):
        # If there's an active timer, cancel it
        if hasattr(self, 'waypoint_timer') and self.waypoint_timer is not None:
            self.waypoint_timer.cancel()
            self.waypoint_timer = None

    def clear_local_costmap(self):
        self.costmap_service_busy = True
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            if DEBUG:
                self.get_logger().info("Local costmap successfully cleared")
            self.costmap_service_busy = False
        else:
            if DEBUG:
                self.get_logger().warn("Timeout or error while clearing local costmap")
            self.costmap_service_busy = False

    def get_filtered_average_pose(self):
        # Computes the average position of the filtered waypoints, excluding outliers
        # based on their distance from the mean position. Then builds a PoseStamped 
        # pointing from the current pose toward the computed average position.
        if not self.pending_waypoints:
            return None

        x_vals = [point.x for point, _ in self.pending_waypoints]
        y_vals = [point.y for point, _ in self.pending_waypoints]
        mean_x, mean_y = sum(x_vals)/len(x_vals), sum(y_vals)/len(y_vals)

        filtered = [(point,dist) for point, dist in self.pending_waypoints
                    if sqrt((point.x - mean_x)**2 + (point.y - mean_y)**2) < OUTLIER_DISTANCE_THRESHOLD]

        if not filtered:
            return None

        avg_x = sum(point.x for point, _ in filtered) / len(filtered)
        avg_y = sum(point.y for point, _ in filtered) / len(filtered)
        avg_dist = sum(dist for _, dist in filtered) / len(filtered)

        avg_pose = PoseStamped()
        avg_pose.header.frame_id = "map"
        avg_pose.header.stamp = self.get_clock().now().to_msg()
        avg_pose.pose.position.x = avg_x
        avg_pose.pose.position.y = avg_y

        current_pose = self.get_current_pose()
        avg_pose.pose.orientation = compute_orientation_towards(current_pose, avg_pose)

        return avg_pose, avg_dist

    def get_current_pose(self) -> PoseStamped:
        feedback = self.navigator.getFeedback()
        if feedback and hasattr(feedback, 'current_pose'):
            return feedback.current_pose
        return self.initial_pose  # fallback

    def is_similar_pose(self, pose1: PoseStamped, pose2: PoseStamped):
        dx = abs(pose1.pose.position.x - pose2.pose.position.x)
        dy = abs(pose1.pose.position.y - pose2.pose.position.y)
        return dx < WAYPOINT_SIMILARITY_TOLERANCE and dy < WAYPOINT_SIMILARITY_TOLERANCE

    def check_results(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if DEBUG: 
                self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            if DEBUG:
                self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            if DEBUG:
                self.get_logger().info('Goal failed!')
        else:
            if DEBUG:
                self.get_logger().info('Goal has an invalid return status!')
        return result

    def destroy_node(self):
        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Errore durante la cancellazione del task: {e}")
        try:
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Errore nel publish di stop: {e}")

        # Stop timers
        try:
            if hasattr(self, 'waypoint_timer') and self.waypoint_timer:
                self.waypoint_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Errore nel cancellare il timer: {e}")

        # Clear waypoints
        self.pending_waypoints.clear()

        # Distruggi esplicitamente il nodo
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()