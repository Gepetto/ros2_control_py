from ros2_control_py.hardware_interface import HardwareInfo
from os import getenv

# Humble / Rolling name diff
ROS_DISTRO = getenv("ROS_DISTRO")
assert ROS_DISTRO in ("humble", "rolling")
hardware_plugin_name = (
    "hardware_class_type"
    if hasattr(HardwareInfo, "hardware_class_type")
    else "hardware_plugin_name"
)
