import numpy as np
import argparse
import json
import time

from isaac import Application, Cask
from packages.pyalice.Composite import create_composite_message
'''
Moves a robot arm based on joint waypoints to pickup and dropoff a box between two pre-defined
locations repeatedly. This app assumes a vacuum pump is connected through the digital
io interfaces. This is tested with a robot, without a gripper connected.
'''


def create_composite_waypoint(name, quantities, values):
    '''Creates a CompositeProto message with name as uuid'''
    msg = create_composite_message(quantities, values)
    msg.uuid = name
    return msg


def create_composite_atlas(cask_root, joints):
    '''Creates composite atlas cask with waypoints for ur10.'''
    if len(joints) != 6:
        raise ValueError("UR10 should have 6 joints, got {}".format(len(joints)))

    cask = Cask(cask_root, writable=True)
    # at joint waypoints
    quantities = [[x, "position", 1] for x in joints]

    CART_OBSERVE_WAYPOINT = np.array(
        [0.000, -1.571, -1.571, -1.571, 1.571, 0.000], dtype=np.dtype("float64"))
    CART_ALIGN_WAYPOINT = np.array(
        [0.259, -1.783, -1.834, -1.089, 1.575, 0.280], dtype=np.dtype("float64"))
    CART_DROPOFF_WAYPOINT = np.array(
        [0.274, -1.869, -1.967, -0.835, 1.581, 0.289], dtype=np.dtype("float64"))

    DOLLY_OBSERVE_WAYPOINT = np.array(
        [0.000, -1.571, -1.571, -1.571, 1.571, 0.000], dtype=np.dtype("float64"))
    DOLLY_ALIGN_WAYPOINT = np.array(
        [-0.137, -1.833, -1.833, -1.012, 1.553, -0.147], dtype=np.dtype("float64"))
    DOLLY_DROPOFF_WAYPOINT = np.array(
        [-0.137, -1.940, -1.846, -0.961, 1.581, -0.147], dtype=np.dtype("float64"))

    cask.write_message(create_composite_waypoint("cart_observe", quantities, CART_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_align", quantities, CART_ALIGN_WAYPOINT))
    cask.write_message(create_composite_waypoint("cart_dropoff", quantities, CART_DROPOFF_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_observe", quantities, DOLLY_OBSERVE_WAYPOINT))
    cask.write_message(create_composite_waypoint("dolly_align", quantities, DOLLY_ALIGN_WAYPOINT))
    cask.write_message(
        create_composite_waypoint("dolly_dropoff", quantities, DOLLY_DROPOFF_WAYPOINT))

    quantities = [[x, "none", 1] for x in ["open", "close"]]
    OPEN_WAYPOINT = np.array([1.0, 0.0], dtype=np.dtype("float64"))
    CLOSE_WAYPOINT = np.array([0.0, 1.0], dtype=np.dtype("float64"))
    cask.write_message(create_composite_waypoint("open", quantities, OPEN_WAYPOINT))
    cask.write_message(create_composite_waypoint("close", quantities, CLOSE_WAYPOINT))

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description="Sortbot Demo")
    parser.add_argument("--cask", help="Path to output atlas", default="/tmp/shuffle_box_waypoints")
    parser.add_argument("--generation", help="Robot generation.", choices=["cb3", "e-series"], default="e-series")
    parser.add_argument("--robot_ip", help="robot ip", default="192.168.0.100")
    parser.add_argument("--headless_mode", help="start driver with headless mode enabled or not",
                        default=False, type=lambda x: (str(x).lower() == 'true'))
    args = parser.parse_args()

    # get kinematic file and joints
    kinematic_file = "apps/assets/kinematic_trees/ur10.kinematic.json"
    joints = []
    with open(kinematic_file, 'r') as fd:
        kt = json.load(fd)
        for link in kt['links']:
            if 'motor' in link and link['motor']['type'] != 'constant':
                joints.append(link['name'])

    # create composite atlas
    create_composite_atlas(args.cask, joints)

    # Create and start the app
    app = Application(name="Shuffle Box Hardware", modules=["sight"])
    # load bebavior subgraph. this contains the sequency behavior to move the arm between
    # waypoints and control gripper digital output
    app.load("packages/universal_robots/apps/shuffle_box_behavior_hardware.subgraph.json", prefix="behavior")
    behavior_interface = app.nodes["behavior.interface"]["subgraph"]
    app.nodes["behavior.atlas"]["CompositeAtlas"].config.cask = args.cask
    app.load("packages/planner/apps/multi_joint_lqr_control.subgraph.json", prefix="lqr")
    app.load("packages/navsim/apps/navsim_tcp.subgraph.json", "simulation")

    # Load driver subgraph
    generation = args.generation
    if generation == "e-series":
        app.load("packages/universal_robots/ur_robot_driver/apps/ur_eseries_robot.subgraph.json", prefix="ur")
    elif generation == "cb3":
        app.load("packages/universal_robots/ur_robot_driver/apps/ur_cb3_robot.subgraph.json", prefix="ur")
    else:
        raise Exception("Unknown robot generation")

    # Load components
    ur_interface = app.nodes["ur.subgraph"]["interface"]
    ur_controller = app.nodes["ur.controller"]["ScaledMultiJointController"]
    ur_driver = app.nodes["ur.universal_robots"]["UniversalRobots"]

    lqr_interface = app.nodes["lqr.subgraph"]["interface"]
    kinematic_tree = app.nodes["lqr.kinematic_tree"]["KinematicTree"]
    lqr_planner = app.nodes["lqr.local_plan"]["MultiJointLqrPlanner"]
    sim_in = app.nodes["simulation.interface"]["input"]
    sim_out = app.nodes["simulation.interface"]["output"]

    # load multi joint lqr control subgraph
    app.connect(behavior_interface, "joint_target", lqr_interface, "joint_target")
    kinematic_tree.config.kinematic_file = kinematic_file
    lqr_planner.config.speed_min = [-1.5] + [-1.0] * (len(joints) - 1)
    lqr_planner.config.speed_max = [1.5] + [1.0] * (len(joints) - 1)
    lqr_planner.config.acceleration_min = [-1.5] + [-1.0] * (len(joints) - 1)
    lqr_planner.config.acceleration_max = [1.5] + [1.0] * (len(joints) - 1)

    # Configs
    ur_controller.config.control_mode = "joint position"
    ur_driver.config.control_mode = "joint position"
    ur_driver.config.robot_ip = args.robot_ip
    ur_driver.config.headless_mode = args.headless_mode
    ur_driver.config.tool_digital_out_names = ["open", "close"]

    # Edges
    app.connect(ur_interface, "arm_state", behavior_interface, "joint_state")
    app.connect(ur_interface, "io_state", behavior_interface, "io_state")
    app.connect(behavior_interface, "io_command", ur_interface, "io_command")
    app.connect(behavior_interface, "joint_target", ur_interface, "joint_target")

    app.connect(sim_out, "joint_state", lqr_interface, "joint_state")
    app.connect(sim_out, "joint_state", behavior_interface, "joint_state")
    app.connect(sim_out, "io_state", behavior_interface, "io_state")
    app.connect(lqr_interface, "joint_command", sim_in, "joint_position")
    app.connect(behavior_interface, "io_command", sim_in, "io_command")

    # Sequence nodes
    sequence_behavior = app.nodes["behavior.sequence_behavior"]
    repeat_behavior = app.nodes["behavior.repeat_behavior"]
    nodes_stopped = True

    # Enable sight
    widget = app.add("sight").add(app.registry.isaac.sight.SightWidget, "IO")
    widget.config.type = "plot"
    widget.config.channels = [
      {
        "name": "ur.universal_robots/UniversalRobots/open"
      },
      {
        "name": "ur.universal_robots/UniversalRobots/close"
      }
    ]

    # run app
    app.start()

    try:
        while True:
            # Make sure urcap is running when starting the nodes
            program_running = app.receive("ur.subgraph", "interface", "robot_program_running")
            if program_running is None:
                time.sleep(1)
            else:
                if program_running.proto.flag == True and nodes_stopped:
                    sequence_behavior.start()
                    repeat_behavior.start()
                    nodes_stopped = False
                elif program_running.proto.flag == False and nodes_stopped == False:
                    sequence_behavior.stop()
                    repeat_behavior.stop()
                    nodes_stopped = True

    except KeyboardInterrupt:
        if nodes_stopped == False:
            sequence_behavior.stop()
            repeat_behavior.stop()
        app.stop()