import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from pathlib import Path
import numpy as np
from ultralytics import YOLO
from custom_interfaces_pkg.msg import ConeDetection
from custom_interfaces_pkg.srv import DetectCones

DEBUG = False
DETECTOR_CONF = 0.5         # detection theshold confidence.

# compute the cone color based on the dominant color in the bounding box.
def get_dominant_color(x, img):
    try:
        mid_y = int((x[1] + x[3]) / 2)
        box = img[mid_y:int(x[3]), int(x[0]):int(x[2])]
        
        # check the bounding box size.
        if box.size == 0:
            return 'red', 0
            
        data = np.reshape(box, (-1, 3)).astype(np.float32)
        
        # verify if there are enough pixels.
        if len(data) < 1:
            return 'red', 0

        # use the k-means to assign color.
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        _, _, centers = cv2.kmeans(data, 1, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        dominant = centers[0].astype(np.int32)
        hsv = cv2.cvtColor(np.uint8([[dominant]]), cv2.COLOR_BGR2HSV)
        h = hsv[0, 0, 0]

        if h < 16:
            return 'red', h
        elif h < 35:
            return 'yellow', h
        elif h < 92:
            return 'green', h
        elif h < 130:
            return 'blue', h
        return 'red', h    # unknown is red
        
    except Exception as e:
        # in error cases the default color is red.
        return 'red', 0

class DetectorServiceNode(Node):
    def __init__(self):
        super().__init__('detector_service_node')
        self.bridge = CvBridge()
        
        # load the YOLO model.
        try:
            model_path = str(Path(__file__).parent / 'models' / 'Cone.pt')
            self.model = YOLO(model_path)
            self.confidence = DETECTOR_CONF
            self.get_logger().info(f"YOLO model loaded from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Model loading error: {e}")
            raise

        # Ccreate the service.
        self.srv = self.create_service(
            DetectCones, 
            'detect_cones', 
            self.detect_cones_callback
        )
        self.get_logger().info("Service /detect_cones active.")
        
        # monitoring statistics.
        self.detection_count = 0
        self.error_count = 0

    def detect_cones_callback(self, request, response):
        try:
            # convert the RGB image from ROS to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(request.rgb_image, desired_encoding='bgr8')
            
            if DEBUG:
                self.get_logger().info("Starting YOLO inference...")
            
            # compute inference.
            results = self.model(cv_image, conf=self.confidence, verbose=DEBUG)[0]

            detections = []
            
            # Process all the detections
            for box in results.boxes:
                confidence = box.conf.item()
                
                if DEBUG:
                    self.get_logger().info(f"Detection with confidence: {confidence:.3f}")
                
                # extract bounding box coordinates.
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, xyxy)
                
                # check if the bounding box is valid.
                h, w = cv_image.shape[:2]
                if x1 >= 0 and y1 >= 0 and x2 <= w and y2 <= h and x2 > x1 and y2 > y1:
                    # get the cone color
                    color, _ = get_dominant_color([x1, y1, x2, y2], cv_image)
                    
                    # create the ConeDetection message.
                    detection = ConeDetection()
                    detection.bbox = [x1, y1, x2, y2]
                    detection.color = color
                    
                    detections.append(detection)
                    
                    if DEBUG:
                        self.get_logger().info(f"{color} cone detected: [{x1}, {y1}, {x2}, {y2}]")
                else:
                    if DEBUG:
                        self.get_logger().warn(f"Invalid Bounding box, ignored: [{x1}, {y1}, {x2}, {y2}]")

            # prepare the response.
            response.success = True
            response.detections = detections
            
            # update statistics.
            self.detection_count += 1
            
            if DEBUG:
                self.get_logger().info(f'Detection #{self.detection_count}: {len(detections)} cone detected.')
                
            return response

        except Exception as e:
            # handling errors.
            self.error_count += 1
            error_msg = f'Detection error: {str(e)}'
            self.get_logger().error(error_msg)
            
            # error response
            response.success = False
            response.detections = []  # empty bounding box list.
            
            return response

    def get_statistics(self):
        return {
            'total_detections': self.detection_count,
            'total_errors': self.error_count,
            'success_rate': (self.detection_count / max(1, self.detection_count + self.error_count)) * 100
        }

    def destroy_node(self):
        stats = self.get_statistics()
        self.get_logger().info(
            f"Detector statistics service: "
            f"Detection: {stats['total_detections']}, "
            f"Errors: {stats['total_errors']}, "
            f"Success rate: {stats['success_rate']:.1f}%"
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = DetectorServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()