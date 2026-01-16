import rclpy
from rclpy.node import Node
from custom_interfaces_pkg.msg import VisualInformation
import cv2
from cv_bridge import CvBridge

DEBUG = False


def is_valid_point(pt):
    return pt.x >= 0 and pt.y >= 0 and (pt.x, pt.y) != (-1, -1)


# returns the RGB encoding of the color given its string representation. 
def get_color_array(color="red"):
    return {
        "red": (0, 0, 255),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255),
        "green": (0, 255, 0),
        "unknow": (255, 255, 255)           # white colore if the cone is of unlisted color.
,    }.get(color, (0, 0, 255))

class Visualization(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(VisualInformation, 'visual_info', self.listener_callback, 10)
        self.get_logger().info('Publisher active on the /visual_info topic.')

    def listener_callback(self, msg: VisualInformation):
        if DEBUG:
            self.get_logger().info('Received image.')
        # convert RGB image
        rgb_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')

        #if a bounding box exists.
        if len(msg.boxes) != 0:
            for box in msg.boxes:
                bbox = box.bbox
                color = box.color
                label = f'{color} Cone'

                # draw label and bounding box.
                pt1 = (int(bbox[0]), int(bbox[1]))
                pt2 = (int(bbox[2]), int(bbox[3]))
                cv2.rectangle(rgb_image, pt1, pt2, get_color_array(color), 1)
                cv2.putText(rgb_image, label, (pt1[0], pt1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, get_color_array(color), 1)
            
            # point visualization.
            points = msg.points
            # draw white circles for the first 3 points (bottom center of bounding boxes and midpoint) except point 4 (direction). 
            for i, pt in enumerate(points[:3]):
                if is_valid_point(pt):
                    cv2.circle(rgb_image, (int(pt.x), int(pt.y)), 3, (255, 255, 255), -1)

            # draw line between point1 and point2 if valid.
            if len(points) >= 2 and is_valid_point(points[0]) and is_valid_point(points[1]):
                start = (int(points[0].x), int(points[0].y))
                end = (int(points[1].x), int(points[1].y))
                cv2.line(rgb_image, start, end, (255, 255, 255), 1)

            # draw an arrow from point3 and point4 (middle point and direction) if valid.
            if len(points) >= 4 and is_valid_point(points[2]) and is_valid_point(points[3]):
                start = (int(points[2].x), int(points[2].y))
                end = (int(points[3].x), int(points[3].y))
                cv2.arrowedLine(rgb_image, start, end, (255, 255, 255), 2, tipLength=0.3)
        
        # show the annotated image.
        cv2.imshow("RGB image", rgb_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Visualization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
