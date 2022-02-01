load("//bzl:py.bzl", "isaac_jupyter_app", "isaac_py_app")
load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "ur5_move_lego_hardware_subgraph",
    modules = [
        "behavior_tree",
        "composite",
    ],
    subgraph = "ur5_move_lego_hardware.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_jupyter_app(
    name = "simple_joint_control",
    data = [
        "//apps/assets/kinematic_trees",
        "//apps/assets/lula_assets",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_eseries_robot_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_cb3_robot_subgraph",
    ],
    modules = [
        "sight",
        "universal_robots",
    ],
    notebook = "simple_joint_control.ipynb",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "ur5_move_lego_hardware",
    srcs = ["ur5_move_lego_hardware.py"],
    data = [
        "//apps/assets/kinematic_trees",
        "//apps/assets/lula_assets",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
        "//packages/universal_robots/digital_twin_app:ur5_move_lego_hardware_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_eseries_robot_subgraph",
        "//packages/universal_robots/ur_robot_driver/apps:ur_cb3_robot_subgraph",
    ],
    modules = [
        "behavior_tree",
        "planner",
        "sight",
        "universal_robots",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_subgraph(
    name = "shuffle_box_behavior_subgraph",
    modules = [
        "behavior_tree",
        "composite",
    ],
    subgraph = "shuffle_box_behavior.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "UR5_sim",
    srcs = ["UR5_sim.py"],
    data = [
        "//apps/samples/manipulation:config",
        ":shuffle_box_behavior_subgraph",
        "//apps/assets/kinematic_trees",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "behavior_tree",
        "ml",
        "object_pose_estimation",
        "planner",
        "sight",
        "viewers",
    ],
)