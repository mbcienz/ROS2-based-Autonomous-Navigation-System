import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces_pkg.msg import ImageAcquisition
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from rclpy.time import Time
import cv2
from collections import deque
import time

MAX_DELAY = 500             # maximum delay (in milliseconds) between RGB and Depth frame
DEBUG = False               # flag for enable debug prints
SHOW = False                # flag for image visualization
STATS_INTERVAL = 10.0       # time inteval (in seconds) to print statistics about the visualization node (i.e. delays)

# First version: acquisition node with synchronization based on timestamp and MAX_DELAY. Publish RGB-Depth frames only when two frame are synchronized
# within a MAX_DELAY time.
class Acquisition_Synchronized(Node):
    def __init__(self):
        super().__init__('acquisition_node')

        # subscribe to camera (RGB and Depth) frame topics.
        self.rgb_sub = Subscriber(self, Image, '/oakd/rgb/preview/image_raw')
        self.depth_sub = Subscriber(self, Image, '/oakd/stereo/image_raw')

        # synchronizer between RGB and Depth based on frame timestamp. Synchronization is expressed in seconds (set to 0.5s).
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=100, slop=(MAX_DELAY / 1000)) 
        self.ts.registerCallback(self.sync_callback)

        # publisher on the image_acquisition topic.
        self.image_pub = self.create_publisher(ImageAcquisition, 'image_acquisition', 10)
        self.get_logger().info("Publisher attivo su /image_acquisition")

        # useful to convert the frame into Image type.
        self.bridge = CvBridge()
        self.i = 0          # counter used to compute statistics.

        # map for statistics based on delays. 
        self.delay_stats = {
            'rgb_vs_depth': deque(maxlen=100),
            'rgb_vs_now': deque(maxlen=100),
            'depth_vs_now': deque(maxlen=100)
        }
        
        # printing statistics. this is called every STATS_INTERVAL time.
        if DEBUG:
            self.stats_timer = self.create_timer(STATS_INTERVAL, self.print_statistics)
            self.last_stats_time = time.time()

    # callback when two frame are synchronized (within the MAX_DELAY time).
    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        # create the message ImageAcquisition and send it to the /image_acquisition topic.
        images_to_send = ImageAcquisition()
        images_to_send.rgb_image = rgb_msg
        images_to_send.depth_image = depth_msg
        self.image_pub.publish(images_to_send)
        
        if DEBUG:
            # compute and store delays.
            self.calculate_and_store_delays(rgb_msg, depth_msg)
            self.i += 1

        # display images if needed.
        if SHOW:
            self.display_images(rgb_msg, depth_msg)

    def calculate_and_store_delays(self, rgb_msg, depth_msg):
        now = self.get_clock().now()
    
        # convert timestamp into Time objects.
        rgb_time = Time.from_msg(rgb_msg.header.stamp)
        depth_time = Time.from_msg(depth_msg.header.stamp)

        # computes delay in milliseconds.
        delay_rgb_vs_depth = abs((rgb_time - depth_time).nanoseconds * 1e-6)
        delay_rgb_now = (now - rgb_time).nanoseconds * 1e-6
        delay_depth_now = (now - depth_time).nanoseconds * 1e-6

        # save dealys for statistics.
        self.delay_stats['rgb_vs_depth'].append(delay_rgb_vs_depth)
        self.delay_stats['rgb_vs_now'].append(delay_rgb_now)
        self.delay_stats['depth_vs_now'].append(delay_depth_now)

    def display_images(self, rgb_msg, depth_msg):
        try:
            # convert and show RGB image.
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv2.imshow("RGB Image", rgb_image)
            
            # convert and show Depth image.
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cv2.imshow("Depth Image", depth_normalized)
            
            cv2.waitKey(1)
            self.windows_created = True
            
        except Exception as e:
            self.get_logger().error(f"Image error visualization: {str(e)}")

    def print_statistics(self):
        if not self.delay_stats['rgb_vs_depth']:
            return
        
        # compute delay means.
        avg_rgb_vs_depth = sum(self.delay_stats['rgb_vs_depth']) / len(self.delay_stats['rgb_vs_depth'])
        avg_rgb_now = sum(self.delay_stats['rgb_vs_now']) / len(self.delay_stats['rgb_vs_now'])
        avg_depth_now = sum(self.delay_stats['depth_vs_now']) / len(self.delay_stats['depth_vs_now'])
        
        # compute min and max delay.
        min_rgb_vs_depth = min(self.delay_stats['rgb_vs_depth'])
        max_rgb_vs_depth = max(self.delay_stats['rgb_vs_depth'])
        
        current_time = time.time()
        elapsed = current_time - self.last_stats_time
        
        self.get_logger().info(
            f"\n=== STATISTIC DELAY (lasts {elapsed:.1f}s, {len(self.delay_stats['rgb_vs_depth'])} messages) ===\n"
            f"Avarage delay RGB vs Depth: {avg_rgb_vs_depth:.2f} ms (min: {min_rgb_vs_depth:.2f}, max: {max_rgb_vs_depth:.2f})\n"
            f"Avarage delay RGB vs Now: {avg_rgb_now:.2f} ms\n"
            f"Avarage delay Depth vs Now: {avg_depth_now:.2f} ms\n"
            f"Total messages processed: {self.i}\n"
            f"=============================================="
        )
        
        self.last_stats_time = current_time

    def destroy_node(self):
        self.get_logger().info("Closing the node...")
        
        # closinw openCV windows.
        if SHOW:
            cv2.destroyAllWindows()
            self.get_logger().info("OpenCV window closed.")
        
        # print final statistics.
        if DEBUG:
            if self.delay_stats['rgb_vs_depth']:
                self.get_logger().info("Printing final statistics...")
                self.print_statistics()
        
        # call destroy_node from upper class.
        super().destroy_node()
        self.get_logger().info("Node close correctly.")



# Second version: everytime a RGB frame is received, associate the last depth frame received and publish on /image_acquisition topic.
class Acquisition(Node):
    def __init__(self):
        super().__init__('acquisition_node')

        # subscribe to camera (RGB and Depth) frame topics.
        self.rgb_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/oakd/stereo/image_raw', self.depth_callback, 10)

        # publisher on the image_acquisition topic.
        self.image_pub = self.create_publisher(ImageAcquisition, 'image_acquisition', 20)
        self.get_logger().info("Active publisher on image_acquisition topic.")

        self.bridge = CvBridge()
        self.i = 0
        # used to save last depth frame received.
        self.last_depth_msg = None

    def depth_callback(self, msg):
        # save the depth frame.
        self.last_depth_msg = msg

    def rgb_callback(self, rgb_msg):
        now = self.get_clock().now()
        rgb_time = Time.from_msg(rgb_msg.header.stamp)

        # if no depth frame is available, skip the frame.
        if self.last_depth_msg is None:
            if DEBUG:
                self.get_logger().warn("No depth messages avaialble")
            return

        depth_msg = self.last_depth_msg
        depth_time = Time.from_msg(depth_msg.header.stamp)

        # compute delays.
        delay_rgb_vs_depth = abs((rgb_time - depth_time).nanoseconds * 1e-6)
        if DEBUG:
            delay_rgb_now = (now - rgb_time).nanoseconds * 1e-6
            delay_depth_now = (now - depth_time).nanoseconds * 1e-6
            self.get_logger().info(
                f"Delay RGB vs Depth: {delay_rgb_vs_depth:.2f} ms\n"
                f"Delay RGB vs Now: {delay_rgb_now:.2f} ms\n"
                f"Delay Depth vs Now: {delay_depth_now:.2f} ms"
            )

        # create the message ImageAcquisition and send it to the /image_acquisition topic.
        msg_to_send = ImageAcquisition()
        msg_to_send.rgb_image = rgb_msg
        msg_to_send.depth_image = depth_msg
        self.image_pub.publish(msg_to_send)
        if DEBUG:
            self.get_logger().info(f"Synchronized message sended {self.i}")
            self.i += 1


def main(args=None):
    rclpy.init(args=args)
    #node = Acquisition()
    node = Acquisition_Synchronized()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
