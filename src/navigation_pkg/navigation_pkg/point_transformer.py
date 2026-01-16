import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from custom_interfaces_pkg.srv import PointTransformer

DEBUG= False
FRAME_ID = "oakd_left_camera_frame"


class PointTransformerService(Node):
    def __init__(self):
        super().__init__('point_transformer_service')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Control the tf is avaiable
        while not self.tf_buffer.can_transform('map',FRAME_ID,Time(),Duration(seconds=0.5)):
            self.get_logger().warn("Waiting for tf. Service not avaiable")
            rclpy.spin_once(self, timeout_sec=0.1)
                
        self.srv = self.create_service(PointTransformer, 'transform_waypoint', self.transform_callback)

        self.get_logger().info("Service 'transform_waypoint' ready.")

    def transform_callback(self, request, response):
        now = self.get_clock().now().to_msg()
        point_in_camera = PointStamped()
        point_in_camera.header.stamp = now
        point_in_camera.header.frame_id = FRAME_ID
        point_in_camera.point = request.point

        try:
            if not self.tf_buffer.can_transform(
                'map',
                point_in_camera.header.frame_id,
                Time(),
                Duration(seconds=0.5)
            ):
                self.get_logger().warn("Transformation not avaiable.")
                response.success = False
                return response

            transform = self.tf_buffer.lookup_transform(
                'map',
                point_in_camera.header.frame_id,
                Time(),
                timeout=Duration(seconds=1.0)
            )

            transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
            response.transformed_point = transformed_point_stamped.point
            response.success = True

            if DEBUG:
                self.get_logger().info(
                    f"Transformed point: x={response.transformed_point.x:.2f}, "
                    f"y={response.transformed_point.y:.2f}, z={response.transformed_point.z:.2f}"
                )
            return response

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF ERROR: {str(e)}")
            response.success = False
            return response
        except Exception as e:
            self.get_logger().error(f"ERROR: {str(e)}")
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
