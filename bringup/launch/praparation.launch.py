import os
import sys
from launch import LaunchDescription

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

class Panic(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

def generate_launch_description():

    # there has to be a better way to write this xD
    # something like this?

    # use structopt::StructOpt;

    # #[derive(StructOpt, Debug)]
    # #[structopt(name = "basic")]
    # pub struct ArgsCLI {
    #     /// Dimacs instance name
    #     #[structopt(long, short = "i", default_value = "NONE")]
    #     pub instance: String,
    #     /// Format (dimacs or predicate)
    #     #[structopt(long, short = "f", default_value = "predicate")]
    #     pub format: String,
    #     /// Decision heuristic
    #     #[structopt(long, short = "d", default_value = "ran")]
    #     pub decision_heuristic: String,
    #     /// Find all assignments (predicate format only)
    #     #[structopt(long, short = "a", parse(try_from_str), default_value = "false")]
    #     pub all: bool,
    # }   

    # pub struct Args {
    #     pub instance: String,
    #     pub format: String,
    #     pub decision_heuristic: String,
    #     pub all: bool
    # }   

    # pub fn handle_args() -> Args {
    #     let args = ArgsCLI::from_args();
    #     Args {
    #         instance: args.instance,
    #         format: args.format,
    #         decision_heuristic: args.decision_heuristic,
    #         all: args.all
    #     }
    # }


    if len(sys.argv) == 7:
        split_4 = sys.argv[4].split(":=")
        split_5 = sys.argv[5].split(":=")
        split_6 = sys.argv[6].split(":=")
        if split_4[0] == "item":
            item = split_4[1]
            if split_5[0] == "tool":
                tool = split_5[1]
                if split_6[0] == "scale":
                    scale = split_6[1]
                else:
                    raise Panic("Argument: {arg} has to be 'scale'".format(arg = split_6[0]))
            else:
                raise Panic("Argument: {arg} has to be 'tool'".format(arg = split_5[0]))
        else:
            raise Panic("Argument: {arg} has to be 'item'".format(arg = split_4[0]))
    elif len(sys.argv) == 6:
        split_4 = sys.argv[4].split(":=")
        split_5 = sys.argv[5].split(":=")
        if split_4[0] == "item":
            item = split_4[1]
            if split_5[0] == "tool":
                tool = split_5[1]
                scale = "1"
            else:
                raise Panic("Argument: {arg} has to be 'tool'".format(arg = split_5[0]))
        else:
            raise Panic("Argument: {arg} has to be 'item'".format(arg = split_4[0]))
    elif len(sys.argv) == 5:
        split_4 = sys.argv[4].split(":=")
        if split_4[0] == "item":
            item = split_4[1]
            tool = "arrow"
            scale = "1"
        else:
            raise Panic("Argument: {arg} has to be 'item'".format(arg = split_4[0]))
    elif len(sys.argv) > 6:
        raise Panic("Invalid number of arguments")
    else: 
        item = "cube"
        tool = "arrow"
        scale = "1"

    bringup_dir = FindPackageShare("bringup").find("bringup")
    item_parameters_dir = FindPackageShare("item_parameters").find("item_parameters")

    rviz_config_file = os.path.join(bringup_dir, "config", "preparation.rviz")

    parameters = {
        "scale": scale,
        "tool": tool,
        "item": os.path.join(item_parameters_dir, "parameters", "{item}.json".format(item = item)),
        "big_file_paths": os.path.join(bringup_dir, "config", "paths.json")
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="rita",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    static_tf_broadcaster_node = Node(
        package="static_tf_broadcaster",
        executable="static_tf_broadcaster",
        namespace="rita",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    static_visualization_node = Node(
        package="static_visualization",
        executable="static_visualization",
        namespace="rita",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    interactive_visualization_node = Node(
        package="interactive_visualization",
        executable="interactive_visualization",
        namespace="rita",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    nodes_to_start = [
        rviz_node,
        static_tf_broadcaster_node,
        static_visualization_node,
        interactive_visualization_node,
    ]

    return LaunchDescription(nodes_to_start)
