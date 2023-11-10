from ros2_control_py import rclcpp
from ros2_control_py.controller_manager import ControllerManager
from ros2_control_py.ros2_control_test_assets import (
    valid_urdf_ros2_control_system_one_interface,
)
from time import sleep

default_urdf = valid_urdf_ros2_control_system_one_interface


def main(argv):
    rclcpp.init(rclcpp.VectorString(argv))

    executor = rclcpp.StaticSingleThreadedExecutor()
    manager_node_name = "controller_manager"

    cm = ControllerManager(executor, manager_node_name)

    print(f"update rate is {cm.get_update_rate()} Hz")

    executor.add_node(cm)
    executor.spin_some()

    # for calculating sleep time
    period = 1_000_000_000 / cm.get_update_rate()
    cm_now = cm.now().nanoseconds()
    next_iteration_time = cm_now

    # for calculating the measured period of the loop
    previous_time = cm.now()

    while rclcpp.ok():
        # calculate measured period
        current_time = cm.now()
        measured_period = current_time - previous_time
        previous_time = current_time

        # execute update loop
        cm.read(cm.now(), measured_period)
        cm.update(cm.now(), measured_period)
        cm.write(cm.now(), measured_period)

        # wait until we hit the end of the period
        next_iteration_time += period
        executor.spin_all((next_iteration_time - cm.now()).nanoseconds())
        sleep((next_iteration_time - cm.now()).nanoseconds() / 1_000_000_000)

    executor.spin()
    rclcpp.shutdown()


if __name__ == "__main__":
    from sys import argv

    main(
        [argv[0], "--ros-args", f"robot_description:={default_urdf}", "update_rate:=10"]
    )
