from ros2_control_py.hardware_interface import (
    HW_IF_EFFORT,
    HW_IF_POSITION,
    HW_IF_VELOCITY,
    parse_control_resources_from_urdf,
    HardwareInfo,
)
from ros2_control_py import ros2_control_test_assets
import pytest
from math import isclose, pi

hardware_plugin_name = (
    "hardware_class_type"
    if hasattr(HardwareInfo, "hardware_class_type")
    else "hardware_plugin_name"
)


def test_empty_string_throws_error():
    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf("")


def test_empty_urdf_throws_error():
    empty_urdf = (
        '<?xml version="1.0"?><robot name="robot" xmlns="http://www.ros.org"></robot>'
    )
    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(empty_urdf)


def test_string_robot_not_root_throws_error():
    broken_xml_string = """
        <?xml version=\"1.0\"?><ros2_control name=\"robot\">><robot name=\"robot\" xmlns=\"http://www.ros.org\"></robot></ros2_control>
    """
    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(broken_xml_string)


def test_invalid_child_throws_error():
    broken_urdf_string = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf_ros2_control_invalid_child
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(broken_urdf_string)


def test_missing_attribute_throws_error():
    broken_urdf_string = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf_ros2_control_missing_attribute
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(broken_urdf_string)


def test_parameter_missing_name_throws_error():
    broken_urdf_string = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf_ros2_control_parameter_missing_name
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(broken_urdf_string)


def test_component_interface_type_empty_throws_error():
    broken_urdf_string = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf_ros2_control_component_interface_type_empty
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(broken_urdf_string)


def test_successfully_parse_valid_urdf_system_one_interface():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_one_interface
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemPositionOnly"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "2"

    assert len(hardware_info.joints) == 2

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 1
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_POSITION
    assert hardware_info.joints[0].command_interfaces[0].min == "-1"
    assert hardware_info.joints[0].command_interfaces[0].max == "1"
    assert len(hardware_info.joints[0].state_interfaces) == 1
    assert hardware_info.joints[0].state_interfaces[0].name == HW_IF_POSITION

    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"
    assert len(hardware_info.joints[1].command_interfaces) == 1
    assert hardware_info.joints[1].command_interfaces[0].name == HW_IF_POSITION
    assert hardware_info.joints[1].command_interfaces[0].min == "-1"
    assert hardware_info.joints[1].command_interfaces[0].max == "1"
    assert len(hardware_info.joints[1].state_interfaces) == 1
    assert hardware_info.joints[1].state_interfaces[0].name == HW_IF_POSITION


def test_successfully_parse_valid_urdf_system_multi_interface():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_multi_interface
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemMultiInterface"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemMultiInterfaceHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "2"
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "2"

    assert len(hardware_info.joints) == 2

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 3
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_POSITION
    assert hardware_info.joints[0].command_interfaces[0].initial_value == "1.2"
    assert hardware_info.joints[0].command_interfaces[1].initial_value == "3.4"
    assert len(hardware_info.joints[0].state_interfaces) == 3
    assert hardware_info.joints[0].state_interfaces[1].name == HW_IF_VELOCITY

    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"
    assert len(hardware_info.joints[1].command_interfaces) == 1
    assert len(hardware_info.joints[1].state_interfaces) == 3
    assert hardware_info.joints[1].state_interfaces[2].name == HW_IF_EFFORT


def test_successfully_parse_valid_urdf_system_robot_with_sensor():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_robot_with_sensor
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemWithSensor"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemWithSensorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "2"

    assert len(hardware_info.joints) == 2

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"

    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"

    assert len(hardware_info.sensors) == 1

    assert hardware_info.sensors[0].name == "tcp_fts_sensor"
    assert hardware_info.sensors[0].type == "sensor"
    assert len(hardware_info.sensors[0].state_interfaces) == 6
    assert len(hardware_info.sensors[0].command_interfaces) == 0
    assert hardware_info.sensors[0].state_interfaces[0].name == "fx"
    assert hardware_info.sensors[0].state_interfaces[1].name == "fy"
    assert hardware_info.sensors[0].state_interfaces[2].name == "fz"
    assert hardware_info.sensors[0].state_interfaces[3].name == "tx"
    assert hardware_info.sensors[0].state_interfaces[4].name == "ty"
    assert hardware_info.sensors[0].state_interfaces[5].name == "tz"

    assert len(hardware_info.sensors[0].parameters) == 3
    assert hardware_info.sensors[0].parameters["frame_id"] == "kuka_tcp"
    assert hardware_info.sensors[0].parameters["lower_limits"] == "-100"
    assert hardware_info.sensors[0].parameters["upper_limits"] == "100"


def test_successfully_parse_valid_urdf_system_robot_with_external_sensor():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_robot_with_external_sensor
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 2
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemPositionOnlyWithExternalSensor"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "2"

    assert len(hardware_info.joints) == 2

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"

    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"

    assert len(hardware_info.sensors) == 0

    hardware_info = control_hardware[1]

    assert hardware_info.name == "RRBotForceTorqueSensor2D"
    assert hardware_info.type == "sensor"
    assert len(hardware_info.hardware_parameters) == 1
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "0.43"

    assert len(hardware_info.sensors) == 1
    assert hardware_info.sensors[0].name == "tcp_fts_sensor"
    assert hardware_info.sensors[0].type == "sensor"
    assert hardware_info.sensors[0].parameters["frame_id"] == "kuka_tcp"


def test_successfully_parse_valid_urdf_actuator_modular_robot():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_actuator_modular_robot
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 2
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotModularJoint1"
    assert hardware_info.type == "actuator"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/PositionActuatorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "1.23"

    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"

    hardware_info = control_hardware[1]

    assert hardware_info.name == "RRBotModularJoint2"
    assert hardware_info.type == "actuator"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/PositionActuatorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "3"

    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint2"
    assert hardware_info.joints[0].type == "joint"


def test_successfully_parse_valid_urdf_actuator_modular_robot_with_sensors():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_actuator_modular_robot_sensors
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 4
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotModularJoint1"
    assert hardware_info.type == "actuator"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/VelocityActuatorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "1.23"

    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 1
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_VELOCITY

    assert len(hardware_info.transmissions) == 1
    assert hardware_info.transmissions[0].name == "transmission1"
    assert (
        hardware_info.transmissions[0].type
        == "transmission_interface/SimpleTansmission"
    )
    assert len(hardware_info.transmissions[0].joints) == 1
    assert hardware_info.transmissions[0].joints[0].name == "joint1"
    assert isclose(
        hardware_info.transmissions[0].joints[0].mechanical_reduction,
        1024.0 / pi,
        abs_tol=0.01,
    )
    assert len(hardware_info.transmissions[0].actuators) == 1
    assert hardware_info.transmissions[0].actuators[0].name == "actuator1"

    hardware_info = control_hardware[1]

    assert hardware_info.name == "RRBotModularJoint2"
    assert hardware_info.type == "actuator"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/VelocityActuatorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "3"

    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint2"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 1
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_VELOCITY
    assert hardware_info.joints[0].command_interfaces[0].min == "-1"
    assert hardware_info.joints[0].command_interfaces[0].max == "1"

    hardware_info = control_hardware[2]

    assert hardware_info.name == "RRBotModularPositionSensorJoint1"
    assert hardware_info.type == "sensor"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/PositionSensorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 1
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "2"

    assert len(hardware_info.sensors) == 0
    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 0
    assert len(hardware_info.joints[0].state_interfaces) == 1
    assert hardware_info.joints[0].state_interfaces[0].name == HW_IF_POSITION

    hardware_info = control_hardware[3]

    assert hardware_info.name == "RRBotModularPositionSensorJoint2"
    assert hardware_info.type == "sensor"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/PositionSensorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 1
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "2"

    assert len(hardware_info.sensors) == 0
    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint2"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 0
    assert len(hardware_info.joints[0].state_interfaces) == 1
    assert hardware_info.joints[0].state_interfaces[0].name == HW_IF_POSITION


def test_successfully_parse_valid_urdf_system_multi_joints_transmission():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_multi_joints_transmission
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotModularWrist"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/ActuatorHardwareMultiDOF"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "1.23"

    assert len(hardware_info.joints) == 2
    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"

    assert len(hardware_info.transmissions) == 1
    assert hardware_info.transmissions[0].name == "transmission1"
    assert (
        hardware_info.transmissions[0].type
        == "transmission_interface/DifferentialTransmission"
    )
    assert len(hardware_info.transmissions[0].joints) == 2
    assert hardware_info.transmissions[0].joints[0].name == "joint1"
    assert hardware_info.transmissions[0].joints[0].role == "joint1"
    assert hardware_info.transmissions[0].joints[0].mechanical_reduction == 10.0
    assert hardware_info.transmissions[0].joints[0].offset == 0.5
    assert hardware_info.transmissions[0].joints[1].name == "joint2"
    assert hardware_info.transmissions[0].joints[1].role == "joint2"
    assert hardware_info.transmissions[0].joints[1].mechanical_reduction == 50.0
    assert hardware_info.transmissions[0].joints[1].offset == 0.0

    assert len(hardware_info.transmissions[0].actuators) == 2
    assert hardware_info.transmissions[0].actuators[0].name == "joint1_motor"
    assert hardware_info.transmissions[0].actuators[0].role == "actuator1"
    assert hardware_info.transmissions[0].actuators[1].name == "joint2_motor"
    assert hardware_info.transmissions[0].actuators[1].role == "actuator2"


def test_successfully_parse_valid_urdf_sensor_only():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_sensor_only
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "CameraWithIMU"
    assert hardware_info.type == "sensor"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/CameraWithIMUSensor"
    )
    assert len(hardware_info.hardware_parameters) == 1
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "2"

    assert len(hardware_info.sensors) == 2
    assert hardware_info.sensors[0].name == "sensor1"
    assert hardware_info.sensors[0].type == "sensor"
    assert len(hardware_info.sensors[0].state_interfaces) == 3
    assert hardware_info.sensors[0].state_interfaces[0].name == "roll"
    assert hardware_info.sensors[0].state_interfaces[1].name == "pitch"
    assert hardware_info.sensors[0].state_interfaces[2].name == "yaw"

    assert hardware_info.sensors[1].name == "sensor2"
    assert hardware_info.sensors[1].type == "sensor"
    assert len(hardware_info.sensors[1].state_interfaces) == 1
    assert hardware_info.sensors[1].state_interfaces[0].name == "image"


def test_successfully_parse_valid_urdf_actuator_only():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_actuator_only
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "ActuatorModularJoint1"
    assert hardware_info.type == "actuator"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/VelocityActuatorHardware"
    )
    assert len(hardware_info.hardware_parameters) == 2
    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == "1.13"

    assert len(hardware_info.joints) == 1
    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 1
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_VELOCITY
    assert hardware_info.joints[0].command_interfaces[0].min == "-1"
    assert hardware_info.joints[0].command_interfaces[0].max == "1"
    assert len(hardware_info.joints[0].state_interfaces) == 1
    assert hardware_info.joints[0].state_interfaces[0].name == HW_IF_VELOCITY

    assert len(hardware_info.transmissions) == 1
    transmission = hardware_info.transmissions[0]
    assert transmission.name == "transmission1"
    assert transmission.type == "transmission_interface/RotationToLinerTansmission"
    assert len(transmission.joints) == 1
    joint = transmission.joints[0]
    assert joint.name == "joint1"
    assert joint.role == "joint1"
    assert "velocity" in joint.state_interfaces
    assert "velocity" in joint.command_interfaces
    assert joint.mechanical_reduction == 325.949
    assert joint.offset == 0.0
    assert len(transmission.actuators) == 1
    actuator = transmission.actuators[0]
    assert actuator.name == "actuator1"
    assert actuator.role == "actuator1"
    assert actuator.state_interfaces == joint.state_interfaces
    assert actuator.command_interfaces == joint.command_interfaces
    assert actuator.offset == 0.0
    assert len(transmission.parameters) == 1
    assert transmission.parameters["additional_special_parameter"] == "1337"


def test_successfully_parse_valid_urdf_system_robot_with_gpio():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_robot_with_gpio
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemWithGPIO"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemWithGPIOHardware"
    )

    assert len(hardware_info.joints) == 2

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"

    assert hardware_info.joints[1].name == "joint2"
    assert hardware_info.joints[1].type == "joint"

    assert len(hardware_info.gpios) == 2

    assert hardware_info.gpios[0].name == "flange_analog_IOs"
    assert hardware_info.gpios[0].type == "gpio"
    assert len(hardware_info.gpios[0].state_interfaces) == 3
    assert len(hardware_info.gpios[0].command_interfaces) == 1
    assert hardware_info.gpios[0].state_interfaces[0].name == "analog_output1"
    assert hardware_info.gpios[0].state_interfaces[1].name == "analog_input1"
    assert hardware_info.gpios[0].state_interfaces[2].name == "analog_input2"

    assert hardware_info.gpios[1].name == "flange_vacuum"
    assert hardware_info.gpios[1].type == "gpio"
    assert len(hardware_info.gpios[1].state_interfaces) == 1
    assert len(hardware_info.gpios[1].command_interfaces) == 1
    assert hardware_info.gpios[1].state_interfaces[0].name == "vacuum"
    assert hardware_info.gpios[1].command_interfaces[0].name == "vacuum"

    assert len(hardware_info.transmissions) == 0


def test_successfully_parse_valid_urdf_system_with_size_and_data_type():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_system_robot_with_size_and_data_type
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "RRBotSystemWithSizeAndDataType"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/RRBotSystemWithSizeAndDataType"
    )

    assert len(hardware_info.joints) == 1

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert len(hardware_info.joints[0].command_interfaces) == 1
    assert hardware_info.joints[0].command_interfaces[0].name == HW_IF_POSITION
    assert hardware_info.joints[0].command_interfaces[0].data_type == "double"
    assert hardware_info.joints[0].command_interfaces[0].size == 1
    assert len(hardware_info.joints[0].state_interfaces) == 1
    assert hardware_info.joints[0].state_interfaces[0].name == HW_IF_POSITION
    assert hardware_info.joints[0].state_interfaces[0].data_type == "double"
    assert hardware_info.joints[0].state_interfaces[0].size == 1

    assert len(hardware_info.gpios) == 1

    assert hardware_info.gpios[0].name == "flange_IOS"
    assert hardware_info.gpios[0].type == "gpio"
    assert len(hardware_info.gpios[0].command_interfaces) == 1
    assert hardware_info.gpios[0].command_interfaces[0].name == "digital_output"
    assert hardware_info.gpios[0].command_interfaces[0].data_type == "bool"
    assert hardware_info.gpios[0].command_interfaces[0].size == 2
    assert len(hardware_info.gpios[0].state_interfaces) == 2
    assert hardware_info.gpios[0].state_interfaces[0].name == "analog_input"
    assert hardware_info.gpios[0].state_interfaces[0].data_type == "double"
    assert hardware_info.gpios[0].state_interfaces[0].size == 3
    assert hardware_info.gpios[0].state_interfaces[1].name == "image"
    assert hardware_info.gpios[0].state_interfaces[1].data_type == "cv::Mat"
    assert hardware_info.gpios[0].state_interfaces[1].size == 1


def test_successfully_parse_parameter_empty():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.valid_urdf_ros2_control_parameter_empty
        + ros2_control_test_assets.urdf_tail
    )

    control_hardware = parse_control_resources_from_urdf(urdf_to_test)
    assert len(control_hardware) == 1
    hardware_info = control_hardware[0]

    assert hardware_info.name == "2DOF_System_Robot_Position_Only"
    assert hardware_info.type == "system"
    assert (
        getattr(hardware_info, hardware_plugin_name)
        == "ros2_control_demo_hardware/2DOF_System_Hardware_Position_Only"
    )

    assert len(hardware_info.joints) == 1

    assert hardware_info.joints[0].name == "joint1"
    assert hardware_info.joints[0].type == "joint"
    assert hardware_info.joints[0].command_interfaces[0].name == "position"

    assert hardware_info.hardware_parameters["example_param_write_for_sec"] == ""
    assert hardware_info.hardware_parameters["example_param_read_for_sec"] == "2"


def test_negative_size_throws_error():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf2_ros2_control_illegal_size
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(urdf_to_test)


def test_noninteger_size_throws_error():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf2_ros2_control_illegal_size2
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(urdf_to_test)


def test_transmission_and_component_joint_mismatch_throws_error():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf2_hw_transmission_joint_mismatch
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(urdf_to_test)


def test_transmission_given_too_many_joints_throws_error():
    urdf_to_test = (
        ros2_control_test_assets.urdf_head
        + ros2_control_test_assets.invalid_urdf2_transmission_given_too_many_joints
        + ros2_control_test_assets.urdf_tail
    )

    with pytest.raises(RuntimeError):
        parse_control_resources_from_urdf(urdf_to_test)
