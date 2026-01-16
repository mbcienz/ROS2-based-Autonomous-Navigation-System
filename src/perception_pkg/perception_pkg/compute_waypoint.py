import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
from custom_interfaces_pkg.msg import PixelPoint
from custom_interfaces_pkg.srv import ComputeWaypoint

# color configuration.
LEFT_COLOR = "red"
RIGHT_COLOR = "yellow"
RIGHT_COLOR2 = "blue"

# frame size.
SIZE_RGB = (400, 400)
SIZE_STEREO = (1280, 720)

# camera parameters.
FOCAL_LENGTH_M = 0.00386434     # focal length in meters.
SENSOR_SIZE = (0.0048, 0.0036)  # sensor width and height in meters (4.8mm x 3.6mm).

# project paramters.
AREA_MAX = 15000                # filter detections with an area bigger than 15000 pixels (it corresponds to a cone detected around 70cm away with 400x400 frame size).
AREA_MIN = 150                  # filter detections with an area lower than 150 pixels (it corresponds to a cone detected around 8m away with 400x400 frame size).
HEIGHT_MAX = 0                  # filtes detectsion upper the camera (on the z axes).
OFFSET_DIST = 0.1               # reduce the distance in order to deal with the camera delay (respect the real time) while moving.
FACTOR_OFFSET_SINGLE_CONE = 6   # multiplication offset factor used to generate a fake cone when only one is detected. 
DEBUG = False

# validation parameters
MIN_DISTANCE = 0.75  # we can not accept distance lower than 0.75 cm. 
MAX_DISTANCE = 12.0  # we can not accet distance higher than 12 m.
MIN_ROI_SIZE = 1     # minimum pixel for the ROI (Region Of Interest), used to compute the avarage distance.


def scale_point(x_small, y_small):
    # scale points from the RGB frame to the STEREO frame.
    x_big = x_small * (SIZE_STEREO[0] / SIZE_RGB[0])
    y_big = y_small * (SIZE_STEREO[1] / SIZE_RGB[1])
    return int(x_big), int(y_big)


def clip_point(x=None, y=None):
    # sesures that the coordinates are within the image bounds.
    if x is not None:
        x = max(0, min(SIZE_RGB[0] - 1, x))
    if y is not None:
        y = max(0, min(SIZE_RGB[1] - 1, y))
    return (x, y)


def to_pixel_point(point):
    # convert a point (tuple) into PixelPoint.
    if point is None or point == (-1, -1):
        return PixelPoint(x=-1, y=-1)
    x, y = point
    return PixelPoint(x=int(round(x)), y=int(round(y)))


def get_points(left_bb, right_bb):
    # get the visualization points (waypoint and  direction).
    offset_dir = 50             # arrow dimension.
    none_point = (-1, -1)       # fake point for "no info".

    if left_bb is None and right_bb is None:
        return [none_point, none_point, none_point, none_point]

    # if both cone are presents. we don't need y1, because we take the bottom center of the bounding box.
    x1_l, _, x2_l, y2_l = left_bb
    x1_r, _, x2_r, y2_r = right_bb

    # create waypoint (midpoint between cones) and clip it within the frame bounds. 
    center_left = clip_point((x1_l + x2_l) // 2, y2_l)
    center_right = clip_point((x1_r + x2_r) // 2, y2_r)
    mid_point = clip_point(
        (center_left[0] + center_right[0]) // 2, 
        (center_left[1] + center_right[1]) // 2
    )

    # check if the cones are in the right order.
    if x1_l < x1_r:
        direction_point = clip_point(mid_point[0], mid_point[1] - offset_dir)
    else:
        direction_point = clip_point(mid_point[0], mid_point[1] + offset_dir)

    return [center_left, center_right, mid_point, direction_point]


class ComputeWaypointNode(Node):
    def __init__(self):
        super().__init__('compute_waypoint_node')
        self.bridge = CvBridge()
        
        # init variables for images and statistics. 
        self.current_depth_image = None
        self.processing_stats = {
            'total_requests': 0,
            'successful_computations': 0,
            'failed_computations': 0,
            'invalid_depths': 0,
            'single_cone_cases': 0
        }
        
        # create the service.
        self.srv = self.create_service(
            ComputeWaypoint, 
            'compute_waypoint', 
            self.compute_waypoint_callback
        )
        
        # Timer for statistic debug (every 30s).
        if DEBUG:
            self.stats_timer = self.create_timer(30.0, self.print_stats)
        
        self.get_logger().info('Service compute_waypoint active.')

    def compute_waypoint_callback(self, request, response):
        self.processing_stats['total_requests'] += 1
        
        try:
            # check for input validation. if not valid return success = False
            if not self._validate_request(request):
                response.success = False
                self.processing_stats['failed_computations'] += 1
                return response

            # if valid, convert the depth image.
            depth_image = request.depth_image
            self.current_depth_image = self.bridge.imgmsg_to_cv2(
                depth_image, 
                desired_encoding='passthrough'
            )

            # process the detection
            left_cones, right_cones = self._categorize_detections(request.detections)
            
            if DEBUG:
                self.get_logger().debug(
                    f"Detections: {len(left_cones)} left cones, {len(right_cones)} right cones."
                )

            # find the closest cones.
            closest_left, distance_left, closest_right, distance_right = (
                self._find_closest_cones(left_cones, right_cones)
            )

            # no cones are valid. so can't compute the waypoint.
            if closest_left is None and closest_right is None:
                response.success = False
                response.points = [to_pixel_point((-1, -1))] * 4
                # is a successful computation even if cones are not visibile.
                self.processing_stats['successful_computations'] += 1
                return response

            # compute points for visualization purpose.
            points_tuples = get_points(closest_left, closest_right)
            response.points = [to_pixel_point(p) for p in points_tuples]

            # compute coordinate into the camera frame 
            left_coordinates = self._compute_frame_coordinates(distance_left, closest_left)
            right_coordinates = self._compute_frame_coordinates(distance_right, closest_right)

            # compute the final waypoint.
            waypoint, distance = self._compute_midpoint(left_coordinates, right_coordinates)
            
            # if the waypoint is computed.
            if waypoint is not None:
                response.waypoint = waypoint
                response.distance = distance
                response.success = True
                self.processing_stats['successful_computations'] += 1
                
                if DEBUG:
                    self.get_logger().info(
                        f"Waypoint: x={waypoint.x:.2f}, y={waypoint.y:.2f}, "
                        f"z={waypoint.z:.2f}, dist={distance:.2f}m"
                    )
            else:
                # service worked but is not possible to compute the waypoint.
                response.success = False
                response.points = [to_pixel_point((-1, -1))] * 4
                self.processing_stats['successful_computations'] += 1

        except Exception as e:
            self.get_logger().error(f"Error in compute_waypoint service: {e}")
            response.success = False
            self.processing_stats['failed_computations'] += 1

        return response

    def _validate_request(self, request):
        # validate the input request.
        if not hasattr(request, 'depth_image') or not hasattr(request, 'detections'):
            return False
        
        if request.depth_image is None:
            self.get_logger().warn("Missing image in the request.")
            return False
            
        if len(request.detections) == 0:
            self.get_logger().debug("No detection available.")
            return False
            
        return True

    def _categorize_detections(self, detections):
        # categorize detection by their color.
        left_cones = []
        right_cones = []
        
        for detection in detections:
            if detection.color == LEFT_COLOR:
                left_cones.append(detection.bbox)
            elif detection.color in [RIGHT_COLOR, RIGHT_COLOR2]:
                right_cones.append(detection.bbox)
            elif DEBUG:
                # if the color is unexpected we ignore it.
                self.get_logger().debug(f"Ignored color: {detection.color}")
                
        return left_cones, right_cones

    def _find_closest_cones(self, left_cones, right_cones):
        # find the closest left and closest right cones.
        distance_left, closest_left = self._find_closest_cone(left_cones)
        distance_right, closest_right = self._find_closest_cone(right_cones)

        # handle cases.
        if distance_left is not None and distance_right is not None:
            # both cones visible.
            pass
        elif distance_left is not None:
            # only left cone visible.
            closest_right = self._create_virtual_cone(closest_left, left=True)
            distance_right = distance_left
            self.processing_stats['single_cone_cases'] += 1
            if DEBUG:
                self.get_logger().info("Virtual right cone created.")
        elif distance_right is not None:
            # only right cone visible.
            closest_left = self._create_virtual_cone(closest_right, left=False)
            distance_left = distance_right
            self.processing_stats['single_cone_cases'] += 1
            if DEBUG:
                self.get_logger().info("Virtual left cone created.")
        else:
            # no visible (or valid) cone.
            return None, None, None, None

        if DEBUG:
            self.get_logger().debug(
                f"Distances: left={distance_left:.2f}m, right={distance_right:.2f}m"
            )

        return closest_left, distance_left, closest_right, distance_right

    def _find_closest_cone(self, boxes):
        # find the closest cone given a list of detections.
        min_dist = float('inf')
        closest_box = None
        # for eveery bb, compute the distance. the one with the lower distance is the closest.
        for box in boxes:
            avg_depth = self._get_average_depth_in_box(box)
            if avg_depth is not None and avg_depth < min_dist:
                min_dist = avg_depth
                closest_box = box

        if min_dist == float('inf'):
            return None, None
        
        return min_dist, closest_box

    def _get_average_depth_in_box(self, bbox):
        # compute the avarage depth of a cone (the depth is avareged whitin a box).
        if self.current_depth_image is None:
            self.get_logger().warn("Depth image not available.")
            return None

        x1, y1, x2, y2 = bbox
        base = (x2 - x1)
        altezza = (y2 - y1)

        # check aspect ratio (only detection with the height higher than the width are valid).
        if (base/altezza) >= 1.0:
            if DEBUG:
                self.get_logger().info(f"Invalid aspect ratio.")
            return None
        
        area = base * altezza
        
        # area check. Detections are valid only if the bounding box are is within a lower and upper bound.
        if not (AREA_MIN < area < AREA_MAX):
            if DEBUG:
                self.get_logger().info(f"Invalid area: {area}")
            return None
            
        # invalid bounding box. It is out the frame.
        if y2 >= SIZE_RGB[1]:
            if DEBUG:
                self.get_logger().info("Bounding box out from the frame margin.")
            return None

        try:
            # center coordinate of the RGB.
            cx_rgb = int((x1 + x2) / 2)
            cy_rgb = int((y1 + y2) / 2)

            # convert RGB center coordinate into depth coordinate.
            cx_depth, cy_depth = scale_point(cx_rgb, cy_rgb)

            # compute ROI dimension.
            width = max(MIN_ROI_SIZE, int((x2 - x1) / 8))
            height = max(MIN_ROI_SIZE, int((y2 - y1) / 8))
            
            # scale dimensions.
            width = max(MIN_ROI_SIZE, int(width * (SIZE_STEREO[0] / SIZE_RGB[0])))
            height = max(MIN_ROI_SIZE, int(height * (SIZE_STEREO[1] / SIZE_RGB[1])))

            # compute bounds and clipping.
            x_start = max(0, cx_depth - width // 2)
            x_end = min(self.current_depth_image.shape[1], cx_depth + width // 2)
            y_start = max(0, cy_depth - height // 2)
            y_end = min(self.current_depth_image.shape[0], cy_depth + height // 2)

            # ensure a minimum roi.
            x_start, x_end = self._ensure_minimum_roi(x_start, x_end, self.current_depth_image.shape[1])
            y_start, y_end = self._ensure_minimum_roi(y_start, y_end, self.current_depth_image.shape[0])

            # extract roi.
            roi = self.current_depth_image[y_start:y_end, x_start:x_end]

            if roi.size == 0:
                if DEBUG:
                    self.get_logger().info("Empty ROI.")
                return None

            # Filter valid pixel 
            valid_pixels = roi[(roi > 0) & (~np.isnan(roi)) & (roi < 65535)]

            if valid_pixels.size == 0:
                if DEBUG:
                    self.get_logger().info("None pixel depth valid.")
                self.processing_stats['invalid_depths'] += 1
                return None

            # compute mean distance.
            avg_depth_m = np.mean(valid_pixels) / 1000.0  # from millimeters to meters.

            # validate distance.
            if not (MIN_DISTANCE <= avg_depth_m <= MAX_DISTANCE):
                if DEBUG:
                    self.get_logger().info(f"Invalid distance: {avg_depth_m:.2f}m")
                return None

            return max(0, avg_depth_m - OFFSET_DIST)

        except Exception as e:
            if DEBUG:
                self.get_logger().info(f"Depth error: {e}")
            return None

    def _ensure_minimum_roi(self, start, end, max_dim):
        # ensure that the roi has at least 1 pixel.
        if start == end:
            if end < max_dim - 1:
                end += 1
            elif start > 0:
                start -= 1
        return start, end

    def _create_virtual_cone(self, bbox, left=True):
        # create a virtual cone when only one is detected.
        x1, y1, x2, y2 = bbox
        width = x2 - x1

        # compute new position.
        if left:
            new_x1 = x1 + FACTOR_OFFSET_SINGLE_CONE * width
            new_x2 = x2 + FACTOR_OFFSET_SINGLE_CONE * width
        else:
            new_x1 = x1 - FACTOR_OFFSET_SINGLE_CONE * width
            new_x2 = x2 - FACTOR_OFFSET_SINGLE_CONE * width

        # Clipping. Can not create a bounding box out of the frame.
        new_x1, _ = clip_point(new_x1)
        new_x2, _ = clip_point(new_x2)

        # final validation.
        if new_x1 >= new_x2:
            if left:
                new_x1 = max(0, new_x1 - 1)
            else:
                new_x2 = min(SIZE_RGB[0] - 1, new_x2 + 1)

        return (new_x1, y1, new_x2, y2)

    def _compute_midpoint(self, left_coordinates, right_coordinates):
        # compute the midpoint between two coordinates.
        if not (left_coordinates and right_coordinates):
            return None, None

        # compute middle point.
        mid_x = (left_coordinates[0] + right_coordinates[0]) / 2.0
        mid_y = (left_coordinates[1] + right_coordinates[1]) / 2.0
        mid_z = (left_coordinates[2] + right_coordinates[2]) / 2.0

        # filter on the height.
        if mid_z < HEIGHT_MAX:
            midpoint = Point()
            midpoint.x = mid_x
            midpoint.y = mid_y
            midpoint.z = -0.24  # camera is fixed always to 0.24m from the ground. So, the z-axis value of the waypoint is -0.24m, placed on the ground.

            # compute 2D distance.
            distance = np.linalg.norm([mid_x, mid_y])
            
            return midpoint, distance

        return None, None

    def _compute_frame_coordinates(self, depth, bbox):
        # compute the 3d coordinate into the camera frame.
        if bbox is None or depth is None:
            return None

        # center of the boounding box.
        xmin, ymin, xmax, ymax = bbox
        u = (xmin + xmax) / 2.0
        v = (ymin + ymax) / 2.0

        # camera intrinsic parameters.
        img_w, img_h = SIZE_RGB
        sensor_w, sensor_h = SENSOR_SIZE
        fx = FOCAL_LENGTH_M * img_w / sensor_w
        fy = FOCAL_LENGTH_M * img_h / sensor_h
        cx = img_w / 2.0
        cy = img_h / 2.0

        # coordinate depth camera.
        x = depth                           # forward: positive. 
        y = ((u - cx) * depth / fx) * -1    # left: positive, right: negative
        z = ((v - cy) * depth / fy) * -1    # up: positive, down: negative.

        return (x, y, z)

    def print_stats(self):
        # print periodic statistics.
        if self.processing_stats['total_requests'] == 0:
            return
            
        success_rate = (
            self.processing_stats['successful_computations'] / 
            self.processing_stats['total_requests'] * 100
        )
        
        self.get_logger().info(
            f"\n=== WAYPOINT NODE STATISTICS ===\n"
            f"Total requests: {self.processing_stats['total_requests']}\n"
            f"Successful computations: {self.processing_stats['successful_computations']}\n"
            f"Failed computations: {self.processing_stats['failed_computations']}\n"
            f"Success rate: {success_rate:.1f}%\n"
            f"Invalid Depths: {self.processing_stats['invalid_depths']}\n"
            f"Single cone cases: {self.processing_stats['single_cone_cases']}\n"
            f"====================================="
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ComputeWaypointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Keyboard interrupt.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Execution error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()