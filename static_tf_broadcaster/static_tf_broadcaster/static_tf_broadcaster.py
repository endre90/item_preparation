import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import tf2_ros

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")

        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.scene_parameters_path = self.declare_parameter(
            "item", "default value"
        )

        self.create_timer(1.0, self.static_tf_broadcaster_timer_callback)
        
        self.item = (
            self.get_parameter("item")
            .get_parameter_value()
            .string_value
        )

        self.included_items = [json.load(open(self.item))]

        self.static_transforms = []

        for item in self.included_items:
            tf = TransformStamped()
            tf.header.frame_id = item["parent_frame"]
            tf.header.stamp = Time()
            tf.child_frame_id = item["child_frame"]
            tf.transform = self.make_transform_from_json(item)
            self.static_transforms.append(tf)

        # also show the secondary transformations
        for item in self.included_items:
            i = 1
            for st in item["secondary_transforms"]:
                tf = TransformStamped()
                tf.header.frame_id = item["child_frame"]
                tf.header.stamp = Time()
                tf.child_frame_id = "{frame}_{i}".format(frame=item["child_frame"], i=i)
                tf.transform = self.make_transform_from_json(st)
                self.static_transforms.append(tf)
                i += 1


    def make_transform_from_json(self, json):
        tf = TransformStamped()
        tf.transform.translation.x = json["transform"]["translation"]["x"]
        tf.transform.translation.y = json["transform"]["translation"]["y"]
        tf.transform.translation.z = json["transform"]["translation"]["z"]
        tf.transform.rotation.x = json["transform"]["rotation"]["x"]
        tf.transform.rotation.y = json["transform"]["rotation"]["y"]
        tf.transform.rotation.z = json["transform"]["rotation"]["z"]
        tf.transform.rotation.w = json["transform"]["rotation"]["w"]
        return tf.transform

    def static_tf_broadcaster_timer_callback(self):
        try:
            for tf in self.static_transforms:
                self.static_tf_broadcaster.sendTransform(tf)
        finally:
            pass

def main(args=None):
    rclpy.init(args=args)

    viz = StaticTFBroadcaster()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()