import os
import rclpy
import json
import time
import datetime

from rclpy.node import Node
from interactive_markers import InteractiveMarkerServer

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback


class InteractiveVisualization(Node):
    def __init__(self):
        self.node_name = "interactive_visualization"
        super().__init__(self.node_name)

        self.interactive_markers_server = InteractiveMarkerServer(
            self, "interactive_markers"
        )

        self.parameter_keys = ["item", "tool", "big_file_paths", "scale"]

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

        self.interactive_items = [json.load(open(self.parameters["item"]))]

        # self.interactive_items = [x for x in self.interactive_items_jsons]

        self.get_logger().info(
            "added {nr} interactive controller".format(
                nr=str(len(self.interactive_items))
            )
        )

        for x in self.interactive_items:
            self.create_interactive_marker(
                x,
                x["visualization"]["fixed_marker"],
                InteractiveMarkerControl.NONE,
                True,
            )

        self.interactive_markers_server.applyChanges()

    def create_marker_control(self, msg, item):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_marker())
        msg.controls.append(control)
        return control

    def normalize_quaternion(self, quaternion_msg):
        norm = (
            quaternion_msg.x ** 2
            + quaternion_msg.y ** 2
            + quaternion_msg.z ** 2
            + quaternion_msg.w ** 2
        )
        s = norm ** (-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def make_marker(self):
        marker = Marker()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        if self.parameters["tool"] == "arrow":
            point_1 = Point()
            point_2 = Point()
            point_1.x = 0.0
            point_1.y = 0.0
            point_1.z = 0.0
            point_2.x = 0.0
            point_2.y = 0.0
            point_2.z = 0.025
            marker.type = 0
            marker.points = [point_1, point_2]
            marker.scale.x = 0.005
            marker.scale.y = 0.01
            marker.scale.z = 0.01
        else:
            marker.type = 10
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.mesh_resource = "file://" + os.path.join(
                self.big_file_paths["big_file_paths"],
                "tools",
                "{tool}_tcp.stl".format(tool=self.parameters["tool"]),
            )
        return marker

    def process_feedback(self, feedback):
        log_prefix = f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"

        log_mouse = ""
        if feedback.mouse_point_valid:
            log_mouse = (
                f"{feedback.mouse_point.x}, {feedback.mouse_point.y}, "
                f"{feedback.mouse_point.z} in frame {feedback.header.frame_id}"
            )

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info(f"{log_prefix}: button click at {log_mouse}")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.get_logger().info(
                f"{log_prefix}: menu item {feedback.menu_entry_id} clicked at {log_mouse}"
            )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.get_logger().info(
                f"{log_prefix}: pose changed\n"
                f"position: "
                f"{feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z}\n"
                f"orientation: "
                f"{feedback.pose.orientation.x}, {feedback.pose.orientation.y}, "
                f"{feedback.pose.orientation.z}, {feedback.pose.orientation.w}\n"
                f"frame: {feedback.header.frame_id} "
                f"time: {feedback.header.stamp.sec} sec, "
                f"{feedback.header.stamp.nanosec} nsec"
            )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.get_logger().info(f"{log_prefix}: mouse down at {log_mouse}")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().info(f"{log_prefix}: mouse up at {log_mouse}")

    def create_interactive_marker(self, item, fixed, interaction_mode, show_6dof=False):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = item["child_frame"]
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = item["visualization"]["marker_scale"]*float(self.parameters["scale"])
        int_marker.name = "new_frame"
        int_marker.description = item["child_frame"]

        self.create_marker_control(int_marker, item)
        int_marker.controls[0].interaction_mode = interaction_mode
        int_marker.controls[0].always_visible = True

        if fixed:
            int_marker.name += "_fixed"
            int_marker.description += "\n(fixed orientation)"

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = {
                InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
                InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
                InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D",
                InteractiveMarkerControl.FIXED: "FIXED",
                InteractiveMarkerControl.INHERIT: "INHERIT",
                InteractiveMarkerControl.MOVE_ROTATE: "MOVE_ROTATE",
            }
            int_marker.name += "_" + control_modes_dict[interaction_mode]
            int_marker.description = "3D Control"
            if show_6dof:
                int_marker.description += " + 6-DOF controls"
            int_marker.description += "\n" + control_modes_dict[interaction_mode]

        if show_6dof:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.always_visible = True
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            control.always_visible = True
            self.normalize_quaternion(control.orientation)
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

        self.interactive_markers_server.insert(
            int_marker, feedback_callback=self.process_feedback
        )


def main(args=None):
    rclpy.init(args=args)

    viz = InteractiveVisualization()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
