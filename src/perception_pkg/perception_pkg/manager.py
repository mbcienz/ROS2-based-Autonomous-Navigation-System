import rclpy
from rclpy.node import Node
from custom_interfaces_pkg.msg import ImageAcquisition, VisualInformation, Waypoint
from custom_interfaces_pkg.srv import DetectCones, ComputeWaypoint

# CONST
DEBUG = True
SERVICE_TIMEOUT_MS = 50             # Timeout for services in milliseconds. 
SERVICE_WAIT_TIMEOUT_SEC = 10.0     # Timeout waiting for availability services.


class ManagerNode(Node):
    def __init__(self):
        super().__init__('manager_node')
        
        self.processing_detector = False    # true if the detector is already processing an image.
        self.processing_waypoint = False    # true if the compute waypoint is processing a waypoint.
        self.images_to_process = None       # images that are being processed.
        self.images_to_send = None          # images to send to the compute waypoint node.
        self.detections_result = None       # detector result.
        self.detections_to_send = None      # detection to send to the compute waypoint node.
        
        # debug statistics.
        self.stats = {
            'total_images': 0,
            'discarded_images': 0,
            'successful_detections': 0,
            'failed_detections': 0,
            'successful_waypoints': 0,
            'failed_waypoints': 0,
            'timeout_errors': 0
        }

        # subscriber to the image_acquisition topic.
        self.image_sub = self.create_subscription(
            ImageAcquisition, 
            'image_acquisition', 
            self.image_callback, 
            10
        )

        # service detection client.
        self.detector_client = self.create_client(DetectCones, 'detect_cones')
        self._wait_for_service(self.detector_client, 'detect_cones')

        # compute waypoint service client.
        self.waypoint_client = self.create_client(ComputeWaypoint, 'compute_waypoint')
        self._wait_for_service(self.waypoint_client, 'compute_waypoint')

        # publish waypoints on the goal_waypoint topic.
        self.waypoint_pub = self.create_publisher(Waypoint, '/goal_waypoint', 10)
        self.get_logger().info('Active publisher on the /goal_waypoint topic.')

        # publish visualization information on the visual_info topic.
        self.visual_info_pub = self.create_publisher(VisualInformation, '/visual_info', 10)
        self.get_logger().info('Active publisher on the /visual_info topic.')

        self.get_logger().info('Manager node correctly started.')

    # waiting for the serviec
    def _wait_for_service(self, client, service_name):
        self.get_logger().info(f'Waiting service: {service_name}...')
        while not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            self.get_logger().warn(f'Service {service_name} not available, retrying...')
        self.get_logger().info(f'Service {service_name} available.')

    def image_callback(self, msg):
        self.stats['total_images'] += 1
        
        # if an image is already processing we descard the new one. Remember: we wait at max 50ms (we ensure a smooth 
        # computations up to 20 fps) before cancel the service.
        if self.processing_detector:
            self.stats['discarded_images'] += 1
            if DEBUG:
                self.get_logger().debug(f"Image #{self.stats['total_images']} discarded - detection in progress.")
            return
        
        if DEBUG:
            self.get_logger().info(f"Processing image #{self.stats['total_images']}")
        
        self.images_to_process = msg
        self._start_detection()

    def _start_detection(self):
        # Prepare the request.
        self.processing_detector = True
        request = DetectCones.Request()
        request.rgb_image = self.images_to_process.rgb_image
        
        if DEBUG:
            self.get_logger().info("Sending detection request...")
        
        # call the asynchronous detection service.
        future = self.detector_client.call_async(request)
        
        # set the timeout
        def timeout_callback():
            if not future.done():
                self.stats['timeout_errors'] += 1
                self.get_logger().error(f"TIMEOUT detection service ({SERVICE_TIMEOUT_MS}ms)")
                self.processing_detector = False
                future.cancel()
        
        # set timer for timeout (convert milliseconds in seconds).
        timeout_timer = self.create_timer(SERVICE_TIMEOUT_MS / 1000.0, timeout_callback)
        
        def cleanup_and_callback(fut):
            timeout_timer.cancel()
            self.detection_response_callback(fut)
        
        future.add_done_callback(cleanup_and_callback)

    def detection_response_callback(self, future):
        try:
            # if service timout
            if future.cancelled():
                self.get_logger().error("Detection service deleted.")
                self.processing_detector = False
                return
                
            response = future.result()
            
            if response.success:
                self.stats['successful_detections'] += 1
                if DEBUG:
                    self.get_logger().info(f"Detection OK")
                
                # Process the detection
                self.detections_result = response.detections
                self.process_detections()
                
            else:
                self.stats['failed_detections'] += 1
                self.get_logger().error(f"Failed detection")
                self.processing_detector = False
                
        except Exception as e:
            self.stats['failed_detections'] += 1
            self.get_logger().error(f' Detection callbakc error: {e}')
            self.processing_detector = False

    def process_detections(self):
        #  if detector doesn't detect cones, send the image for the visualization node.
        if len(self.detections_result) == 0:      
            self.publish_visual_info(self.images_to_process, self.detections_result, [])
            if DEBUG:
                self.get_logger().info("No detected cone.")
            self.processing_detector = False
        else:
            # if the compute waypoint service is not processing, send a new request, otherwise discard the image and go on.
            if not self.processing_waypoint:
                self.images_to_send = self.images_to_process # save the image to send to the compute waypoint node.
                self.detections_to_send = self.detections_result # save the detection to send to the compute waypoint
                # the above steps are necessary because if a new callback is executed we want to keep the in progess data. 
                self.processing_waypoint=True
                self.processing_detector=False
                self._start_waypoint_computation()
            else:
                self.processing_detector=False # service detector free.

    def _start_waypoint_computation(self):
        # send the request to the compute waypoint node.
        request = ComputeWaypoint.Request()
        request.depth_image = self.images_to_send.depth_image
        request.detections = self.detections_to_send

        if DEBUG:
            self.get_logger().info("Send the compute waypoint request...")

        # call the asynchronous service with the tiemout.
        future = self.waypoint_client.call_async(request)
        
        # set timeout
        def timeout_callback():
            if not future.done():
                self.stats['timeout_errors'] += 1
                self.get_logger().error(f"TIMEOUT waypoint service ({SERVICE_TIMEOUT_MS}ms)")
                self.processing_waypoint = False
                future.cancel()
        
        # create timer for timeout
        timeout_timer = self.create_timer(SERVICE_TIMEOUT_MS / 1000.0, timeout_callback)
        
        def cleanup_and_callback(fut):
            timeout_timer.cancel()  # Cancella il timer
            self.waypoint_response_callback(fut)
        
        future.add_done_callback(cleanup_and_callback)

    def waypoint_response_callback(self, future):
        try:
            if future.cancelled():
                self.get_logger().error("Waypoint service canceled.")
                return
                
            response = future.result()
            
            if response.success:
                self.stats['successful_waypoints'] += 1
                
                # Invia le informazioni sui topic
                self.publish_visual_info(self.images_to_send, self.detections_to_send, response.points)
                self.publish_waypoint(response.waypoint, response.distance)
                if DEBUG:
                    self.get_logger().info(f"Computed waypoint distance: {response.distance:.2f}m")
            else:
                self.publish_visual_info(self.images_to_send, self.detections_to_send, response.points)
                self.stats['failed_waypoints'] += 1
                self.get_logger().error(f"No computed waypoint.")
            
            self.processing_waypoint = False  # now i can accept new images.

        except Exception as e:
            self.processing_waypoint = False  # now i can accept new images.
            self.stats['failed_waypoints'] += 1
            self.get_logger().error(f"waypoint callback error: {e}")

    def publish_visual_info(self, images, boxes, points):
        if images is None:
            self.get_logger().warn("Attempting to post on visual_info topic without images.")
            return
            
        visual_info = VisualInformation()
        visual_info.image = images.rgb_image
        visual_info.boxes = boxes
        visual_info.points = points
        self.visual_info_pub.publish(visual_info)
        
        if DEBUG:
            self.get_logger().debug(f"Visual info published: {len(boxes)} boxes, {len(points)} points.")

    def publish_waypoint(self, waypoint, distance):
        if waypoint is None:
            self.get_logger().warn("Attempting to publish a none waypoint.")
            return
            
        msg = Waypoint()
        msg.point = waypoint
        msg.distance = distance
        
        self.waypoint_pub.publish(msg)
        
        if DEBUG:
            self.get_logger().info(f"Published waypoint: x={waypoint.x:.2f}, y={waypoint.y:.2f}, z={waypoint.z:.2f}, dist={distance:.2f}m")

    def print_statistics(self):
        success_rate_detection = 0
        success_rate_waypoint = 0
        
        total_detections = self.stats['successful_detections'] + self.stats['failed_detections']
        if total_detections > 0:
            success_rate_detection = (self.stats['successful_detections'] / total_detections) * 100
            
        total_waypoints = self.stats['successful_waypoints'] + self.stats['failed_waypoints']
        if total_waypoints > 0:
            success_rate_waypoint = (self.stats['successful_waypoints'] / total_waypoints) * 100

        self.get_logger().info(
            f"\n === STATISTICHE COMPUTE NODE ===\n"
            f"Total images: {self.stats['total_images']}\n"
            f"Discarded images: {self.stats['discarded_images']}\n"
            f"Successful detections: {self.stats['successful_detections']}\n"
            f"Failed detections: {self.stats['failed_detections']}\n"
            f"Success rate detection: {success_rate_detection:.1f}%\n"
            f"Successful waypoints: {self.stats['successful_waypoints']}\n"
            f"Failed waypoints: {self.stats['failed_waypoints']}\n"
            f"Success rate waypoint: {success_rate_waypoint:.1f}%\n"
            f"Timeout errors: {self.stats['timeout_errors']}\n"
            f"==============================="
        )

    def destroy_node(self):
        self.get_logger().info("Closing node...")
        
        if DEBUG:
            self.print_statistics()
        
        super().destroy_node()
        self.get_logger().info("Node closed correctly.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Execution error: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()