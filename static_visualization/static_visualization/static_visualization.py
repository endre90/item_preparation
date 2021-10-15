import os
import rclpy
import json
import time
import tf2_ros
import threading

from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA


class StaticVisualization(Node):
    def __init__(self):
        self.node_name = "static_visualization"
        super().__init__(self.node_name)

        self.tfBuffer = tf2_ros.Buffer()
        self.lf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.parameter_keys = ["item", "big_file_paths"]

        self.declared_parameters = [
            self.declare_parameter(x, "default_value") for x in self.parameter_keys
        ]

        self.parameters = {
            x: self.get_parameter(x).get_parameter_value().string_value
            for x in self.parameter_keys
        }

        while rclpy.ok():
            try:
                with open(self.parameters["big_file_paths"]) as jsonfile:
                    self.big_file_paths = json.load(jsonfile)
            except Exception as e:
                self.get_logger().info("failed")
                time.sleep(1)
            else:
                self.get_logger().info("opened")
                break

        self.static_items_jsons = [json.load(open(self.parameters["item"]))]

        self.static_items = [
            x for x in self.static_items_jsons if x["visualization"]["show_mesh"]
        ]

        self.get_logger().info(
            "added {nr} non-interactive items".format(nr=str(len(self.static_items)))
        )

        self.marker_array_publisher = self.create_publisher(
            MarkerArray, "static_markers", 10
        )

        self.individual_markers = [[x, TransformStamped()] for x in self.static_items]

        self.color = ColorRGBA()

        self.marker_timer_period = 0.01
        self.marker_timer = self.create_timer(
            self.marker_timer_period, self.publish_markers
        )

        self.tf_lookup()

    def tf_lookup(self):
        def tf_lookup_callback():

            # wait a bit for the tf to become alive
            time.sleep(5)

            rate = self.create_rate(100)
            while rclpy.ok():
                try:
                    for x in self.individual_markers:
                        x[1] = self.tfBuffer.lookup_transform(
                            x[0]["parent_frame"],
                            x[0]["child_frame"],
                            tf2_ros.Time(seconds=0, nanoseconds=0),
                        )
                    rate.sleep()
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as e:
                    self.get_logger().error(
                        e,
                        "failed to lookup tf for: {parent} to {child}".format(
                            parent=x[0]["parent_frame"], child=x[0]["child_frame"]
                        ),
                    )
                    continue

        t = threading.Thread(target=tf_lookup_callback)
        t.daemon = True
        t.start()

    def publish_markers(self):
        msg = MarkerArray()
        markers = []
        id = 0
        for x in self.individual_markers:
            id = id + 1
            indiv_marker = Marker()
            indiv_marker.header.frame_id = x[0]["parent_frame"]
            indiv_marker.header.stamp = Time()
            indiv_marker.ns = ""
            indiv_marker.id = id
            indiv_marker.type = 10
            indiv_marker.action = 0
            indiv_marker.pose.position.x = x[1].transform.translation.x
            indiv_marker.pose.position.y = x[1].transform.translation.y
            indiv_marker.pose.position.z = x[1].transform.translation.z
            indiv_marker.pose.orientation.x = x[1].transform.rotation.x
            indiv_marker.pose.orientation.y = x[1].transform.rotation.y
            indiv_marker.pose.orientation.z = x[1].transform.rotation.z
            indiv_marker.pose.orientation.w = x[1].transform.rotation.w
            indiv_marker.scale.x = x[0]["visualization"]["scale_x"]
            indiv_marker.scale.y = x[0]["visualization"]["scale_y"]
            indiv_marker.scale.z = x[0]["visualization"]["scale_z"]
            indiv_marker.color.a = x[0]["visualization"]["color_a"]
            indiv_marker.color.r = x[0]["visualization"]["color_r"]
            indiv_marker.color.g = x[0]["visualization"]["color_g"]
            indiv_marker.color.b = x[0]["visualization"]["color_b"]
            indiv_marker.mesh_resource = "file://" + os.path.join(
                self.big_file_paths["big_file_paths"], "items",
                x[0]["visualization"]["mesh"],
            )
            markers.append(indiv_marker)
            msg.markers = markers
        self.marker_array_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    viz = StaticVisualization()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
