from ros2_control_py import rclcpp
from ros2_control_py.controller_manager import ControllerManager
from ros2_control_py.ros2_control_test_assets import (
    valid_urdf_ros2_control_system_one_interface,
)
from time import sleep

default_urdf = valid_urdf_ros2_control_system_one_interface


def main(argv):
    rclcpp.init(rclcpp.VectorString(argv))

    executor = rclcpp.SingleThreadedExecutor()
    manager_node_name = "controller_manager"

    cm = ControllerManager(executor, manager_node_name)

    print(f"update rate is {cm.get_update_rate()} Hz")

    executor.add_node(cm)  # Should be cm.
    executor.spin_some()

    # for calculating sleep time
    period = 1_000_000_000 / cm.get_update_rate()
    cm_now_ns = cm.now().nanoseconds()
    cm_now = cm.now()
    next_iteration_time = cm_now
    next_iteration_time_ns = cm_now.nanoseconds()

    # for calculating the measured period of the loop
    previous_time = cm.now()
    previous_time_ns = cm.now().nanoseconds()

    while rclcpp.ok():
        # calculate measured period
        current_time = cm.now()
        current_time_ns = cm.now().nanoseconds()
        measured_period = current_time - previous_time
        previous_time = current_time
        previous_time_ns = current_time_ns

        # execute update loop
        cm.read(cm.now(), measured_period)
        cm.update(cm.now(), measured_period)
        cm.write(cm.now(), measured_period)

        # wait until we hit the end of the period
        next_iteration_time_ns += period
        executor.spin_all(next_iteration_time_ns - cm.now().nanoseconds())
        sleeping_time = (
            next_iteration_time_ns - cm.now().nanoseconds()
        ) / 1_000_000_000
        print(sleeping_time)
        sleep(sleeping_time)

    executor.spin()
    rclcpp.shutdown()


if __name__ == "__main__":
    from sys import argv

    main(
        [
            argv[0],
            "--ros-args",
            "-p",
            f"robot_description:={default_urdf}",
            "-p",
            "update_rate:=10",
        ]
    )
