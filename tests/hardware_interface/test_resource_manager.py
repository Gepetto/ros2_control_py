from ros2_control_py import ros2_control_test_assets
from ros2_control_py.ros2_control_test_assets import (
    TEST_ACTUATOR_HARDWARE_CLASS_TYPE,
    TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
    TEST_ACTUATOR_HARDWARE_NAME,
    TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
    TEST_ACTUATOR_HARDWARE_TYPE,
    TEST_SENSOR_HARDWARE_CLASS_TYPE,
    TEST_SENSOR_HARDWARE_COMMAND_INTERFACES,
    TEST_SENSOR_HARDWARE_NAME,
    TEST_SENSOR_HARDWARE_STATE_INTERFACES,
    TEST_SENSOR_HARDWARE_TYPE,
    TEST_SYSTEM_HARDWARE_CLASS_TYPE,
    TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
    TEST_SYSTEM_HARDWARE_NAME,
    TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
    TEST_SYSTEM_HARDWARE_TYPE,
)
from ros2_control_py.hardware_interface import (
    ResourceManager,
    ActuatorInterface,
    StateInterface,
    CommandInterface,
    HardwareInfo,
    UNCONFIGURED,
    ACTIVE,
    INACTIVE,
    FINALIZED,
    return_type,
    State,
    VectorString,
    Time,
    Duration,
    FloatRef,
)
from lifecycle_msgs import msg as lifecycle_msgs_msg
import pytest
from os import getenv

ROS_DISTRO = getenv("ROS_DISTRO")
PLUGIN_NAME = (
    "CLASS_TYPE" if hasattr(HardwareInfo, "hardware_class_type") else "PLUGIN_NAME"
)


def ASSERT_FALSE(a):
    assert not a


def EXPECT_FALSE(a):
    ASSERT_FALSE(a)


def ASSERT_TRUE(a):
    assert a


def EXPECT_TRUE(a):
    ASSERT_TRUE(a)


def ASSERT_EQ(a, b):
    assert a == b


def EXPECT_EQ(a, b):
    ASSERT_EQ(a, b)


def ASSERT_NE(a, b):
    assert a != b


def EXPECT_NE(a, b):
    ASSERT_NE(a, b)


class TestableResourceManager(ResourceManager):
    __test__ = False

    def __init__(
        self,
        urdf: str = None,
        validate_interfaces: bool = True,
        activate_all: bool = False,
    ):
        if urdf is None:
            super().__init__()
        else:
            super().__init__(urdf, validate_interfaces, activate_all)


def set_components_state(
    rm: TestableResourceManager,
    components: VectorString,
    state_id: int,
    state_name: str,
) -> list[return_type]:
    if len(components) == 0:
        components = VectorString(
            ["TestActuatorHardware", "TestSensorHardware", "TestSystemHardware"]
        )
    results = []
    for component in components:
        state = State(state_id, state_name)
        result = rm.set_component_state(component, state)
        results.append(result)
    return results


def configure_components(rm: TestableResourceManager, components: VectorString = None):
    if components is None:
        components = []
    return set_components_state(
        rm, components, lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE, INACTIVE
    )


def activate_components(rm: TestableResourceManager, components: VectorString = None):
    if components is None:
        components = []
    return set_components_state(
        rm, components, lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE, ACTIVE
    )


def deactivate_components(rm: TestableResourceManager, components: VectorString = None):
    if components is None:
        components = []
    return set_components_state(
        rm, components, lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE, INACTIVE
    )


def cleanup_components(rm: TestableResourceManager, components: VectorString = None):
    if components is None:
        components = []
    return set_components_state(
        rm,
        components,
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
        UNCONFIGURED,
    )


def shutdown_components(rm: TestableResourceManager, components: VectorString = None):
    if components is None:
        components = []
    return set_components_state(
        rm, components, lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED, FINALIZED
    )


def test_initialization_empty():
    with pytest.raises(RuntimeError):
        TestableResourceManager("")


def test_initialization_with_urdf():
    TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)


def test_post_initialization_with_urdf():
    TestableResourceManager().load_urdf(ros2_control_test_assets.minimal_robot_urdf)


def test_initialization_with_urdf_manual_validation():
    # we validate the results manually
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf, False)

    EXPECT_EQ(1, rm.actuator_components_size())
    EXPECT_EQ(1, rm.sensor_components_size())
    EXPECT_EQ(1, rm.system_components_size())

    state_interface_keys = rm.state_interface_keys()
    ASSERT_EQ(11, len(state_interface_keys))
    EXPECT_TRUE(rm.state_interface_exists("joint1/position"))
    EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"))
    EXPECT_TRUE(rm.state_interface_exists("sensor1/velocity"))
    EXPECT_TRUE(rm.state_interface_exists("joint2/position"))
    EXPECT_TRUE(rm.state_interface_exists("joint3/position"))

    command_interface_keys = rm.command_interface_keys()
    ASSERT_EQ(6, len(command_interface_keys))
    EXPECT_TRUE(rm.command_interface_exists("joint1/position"))
    EXPECT_TRUE(rm.command_interface_exists("joint2/velocity"))
    EXPECT_TRUE(rm.command_interface_exists("joint3/velocity"))


def test_initialization_with_wrong_urdf():
    # missing state keys
    with pytest.raises(Exception):
        TestableResourceManager(
            ros2_control_test_assets.minimal_robot_missing_state_keys_urdf
        )
    # missing command keys
    with pytest.raises(Exception):
        TestableResourceManager(
            ros2_control_test_assets.minimal_robot_missing_command_keys_urdf
        )


def test_initialization_with_urdf_unclaimed():
    # we validate the results manually
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    command_interface_keys = rm.command_interface_keys()
    for key in command_interface_keys:
        EXPECT_FALSE(rm.command_interface_is_claimed(key))
    # state interfaces don't have to be locked, hence any arbitrary key
    # should return False.
    state_interface_keys = rm.state_interface_keys()
    for key in state_interface_keys:
        EXPECT_FALSE(rm.command_interface_is_claimed(key))


def test_no_load_urdf_function_called():
    if ROS_DISTRO != "rolling":
        return
    rm = TestableResourceManager()
    ASSERT_FALSE(rm.is_urdf_already_loaded())


def test_load_urdf_called_if_urdf_is_invalid():
    if ROS_DISTRO != "rolling":
        return
    rm = TestableResourceManager()
    with pytest.raises(Exception):
        rm.load_urdf(ros2_control_test_assets.minimal_robot_missing_state_keys_urdf)
    ASSERT_TRUE(rm.is_urdf_already_loaded())


def test_load_urdf_called_if_urdf_is_valid():
    if ROS_DISTRO != "rolling":
        return
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)
    ASSERT_TRUE(rm.is_urdf_already_loaded())


def test_can_load_urdf_later():
    if ROS_DISTRO != "rolling":
        return
    rm = TestableResourceManager()
    ASSERT_FALSE(rm.is_urdf_already_loaded())
    rm.load_urdf(ros2_control_test_assets.minimal_robot_urdf)
    ASSERT_TRUE(rm.is_urdf_already_loaded())


def test_resource_claiming():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)
    # Activate components to get all interfaces available
    activate_components(rm)

    key = "joint1/position"
    EXPECT_TRUE(rm.command_interface_is_available(key))
    EXPECT_FALSE(rm.command_interface_is_claimed(key))

    position_command_interface = rm.claim_command_interface(key)
    EXPECT_TRUE(rm.command_interface_is_available(key))
    EXPECT_TRUE(rm.command_interface_is_claimed(key))
    with pytest.raises(RuntimeError):
        rm.claim_command_interface(key)
    EXPECT_TRUE(rm.command_interface_is_available(key))
    del position_command_interface
    EXPECT_TRUE(rm.command_interface_is_available(key))
    EXPECT_FALSE(rm.command_interface_is_claimed(key))

    # command interfaces can only be claimed once
    for key in (
        "joint1/position",
        "joint1/position",
        "joint1/position",
        "joint2/velocity",
        "joint3/velocity",
    ):
        interface = rm.claim_command_interface(key)
        EXPECT_TRUE(rm.command_interface_is_available(key))
        EXPECT_TRUE(rm.command_interface_is_claimed(key))
        with pytest.raises(RuntimeError):
            rm.claim_command_interface(key)
        EXPECT_TRUE(rm.command_interface_is_available(key))
        del interface
        EXPECT_TRUE(rm.command_interface_is_available(key))
        EXPECT_FALSE(rm.command_interface_is_claimed(key))

    # TODO(destogl): This claim test is not True.... can not be...
    # state interfaces can be claimed multiple times
    for key in [
        "joint1/position",
        "joint1/velocity",
        "sensor1/velocity",
        "joint2/position",
        "joint3/position",
    ]:
        EXPECT_TRUE(rm.state_interface_is_available(key))
        interface = rm.claim_state_interface(key)
        EXPECT_TRUE(rm.state_interface_is_available(key))
        rm.claim_state_interface(key)
        del interface

    interfaces = []
    interface_names = ["joint1/position", "joint2/velocity", "joint3/velocity"]
    for key in interface_names:
        EXPECT_TRUE(rm.command_interface_is_available(key))
        interfaces.append(rm.claim_command_interface(key))
    for key in interface_names:
        EXPECT_TRUE(rm.command_interface_is_available(key))
        EXPECT_TRUE(rm.command_interface_is_claimed(key))
    interfaces.clear()
    for key in interface_names:
        EXPECT_TRUE(rm.command_interface_is_available(key))
        EXPECT_FALSE(rm.command_interface_is_claimed(key))


class ExternalComponent(ActuatorInterface):
    def __init__(self):
        super().__init__()

    def export_state_interfaces(self) -> list[StateInterface]:
        return [
            StateInterface("external_joint", "external_state_interface"),
        ]

    def export_command_interfaces(self) -> list[CommandInterface]:
        return [
            CommandInterface("external_joint", "external_command_interface"),
        ]

    def get_name(self) -> str:
        return "ExternalComponent"

    def read(self, time: Time, period: Duration) -> return_type:
        return return_type.OK

    def write(self, time: Time, period: Duration) -> return_type:
        return return_type.OK


def test_post_initialization_add_components():
    # we validate the results manually
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf, False)
    # Activate components to get all interfaces available
    activate_components(rm)

    EXPECT_EQ(1, rm.actuator_components_size())
    EXPECT_EQ(1, rm.sensor_components_size())
    EXPECT_EQ(1, rm.system_components_size())

    ASSERT_EQ(11, len(rm.state_interface_keys()))
    ASSERT_EQ(6, len(rm.command_interface_keys()))

    external_component_hw_info = HardwareInfo()
    external_component_hw_info.name = "ExternalComponent"
    external_component_hw_info.type = "actuator"
    if hasattr(HardwareInfo, "is_async"):
        external_component_hw_info.is_async = False
    rm.import_component(ExternalComponent(), external_component_hw_info)
    EXPECT_EQ(2, rm.actuator_components_size())

    ASSERT_EQ(12, len(rm.state_interface_keys()))
    EXPECT_TRUE(rm.state_interface_exists("external_joint/external_state_interface"))
    ASSERT_EQ(7, len(rm.command_interface_keys()))
    EXPECT_TRUE(
        rm.command_interface_exists("external_joint/external_command_interface")
    )

    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map["ExternalComponent"].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )

    configure_components(rm, ["ExternalComponent"])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map["ExternalComponent"].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )

    activate_components(rm, ["ExternalComponent"])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map["ExternalComponent"].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )

    rm.claim_state_interface("external_joint/external_state_interface")
    rm.claim_command_interface("external_joint/external_command_interface")


def test_default_prepare_perform_switch():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)
    # Activate components to get all interfaces available
    activate_components(rm)

    EXPECT_TRUE(rm.prepare_command_mode_switch(VectorString([""]), VectorString([""])))
    EXPECT_TRUE(rm.perform_command_mode_switch(VectorString([""]), VectorString([""])))


hardware_resources_command_modes = """
    <ros2_control name="TestSystemCommandModes" type="system">
        <hardware>
            <plugin>test_hardware_components/TestSystemCommandModes</plugin>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
"""

command_mode_urdf = (
    ros2_control_test_assets.urdf_head
    + hardware_resources_command_modes
    + ros2_control_test_assets.urdf_tail
)


def test_custom_prepare_perform_switch():
    rm = TestableResourceManager(command_mode_urdf)
    # Scenarios defined by example criteria
    empty_keys = VectorString([])
    irrelevant_keys = VectorString(["elbow_joint/position", "should_joint/position"])
    illegal_single_key = VectorString(["joint1/position"])
    legal_keys_position = VectorString(["joint1/position", "joint2/position"])
    legal_keys_velocity = VectorString(["joint1/velocity", "joint2/velocity"])
    # Default behavior for empty key lists
    EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, empty_keys))

    # Default behavior when given irrelevant keys
    EXPECT_TRUE(rm.prepare_command_mode_switch(irrelevant_keys, irrelevant_keys))
    EXPECT_TRUE(rm.prepare_command_mode_switch(irrelevant_keys, empty_keys))
    EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, irrelevant_keys))

    # The test hardware interface has a criteria that both joints must change mode
    EXPECT_FALSE(rm.prepare_command_mode_switch(illegal_single_key, illegal_single_key))
    EXPECT_FALSE(rm.prepare_command_mode_switch(illegal_single_key, empty_keys))
    EXPECT_FALSE(rm.prepare_command_mode_switch(empty_keys, illegal_single_key))

    # Test legal start keys
    EXPECT_TRUE(
        rm.prepare_command_mode_switch(legal_keys_position, legal_keys_position)
    )
    EXPECT_TRUE(
        rm.prepare_command_mode_switch(legal_keys_velocity, legal_keys_velocity)
    )
    EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_position, empty_keys))
    EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, legal_keys_position))
    EXPECT_TRUE(rm.prepare_command_mode_switch(legal_keys_velocity, empty_keys))
    EXPECT_TRUE(rm.prepare_command_mode_switch(empty_keys, legal_keys_velocity))

    # Test rejection from perform_command_mode_switch, test hardware rejects empty start sets
    EXPECT_TRUE(
        rm.perform_command_mode_switch(legal_keys_position, legal_keys_position)
    )
    EXPECT_FALSE(rm.perform_command_mode_switch(empty_keys, empty_keys))
    EXPECT_FALSE(rm.perform_command_mode_switch(empty_keys, legal_keys_position))


def test_resource_status():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    status_map = rm.get_components_status()

    # name
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].name, TEST_ACTUATOR_HARDWARE_NAME)
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].name, TEST_SENSOR_HARDWARE_NAME)
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].name, TEST_SYSTEM_HARDWARE_NAME)
    # type
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].type, TEST_ACTUATOR_HARDWARE_TYPE)
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].type, TEST_SENSOR_HARDWARE_TYPE)
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].type, TEST_SYSTEM_HARDWARE_TYPE)
    # plugin_name
    EXPECT_EQ(
        getattr(status_map[TEST_ACTUATOR_HARDWARE_NAME], PLUGIN_NAME.lower()),
        globals()["TEST_ACTUATOR_HARDWARE_" + PLUGIN_NAME],
    )
    EXPECT_EQ(
        getattr(status_map[TEST_SENSOR_HARDWARE_NAME], PLUGIN_NAME.lower()),
        globals()["TEST_SENSOR_HARDWARE_" + PLUGIN_NAME],
    )
    EXPECT_EQ(
        getattr(status_map[TEST_SYSTEM_HARDWARE_NAME], PLUGIN_NAME.lower()),
        globals()["TEST_SYSTEM_HARDWARE_" + PLUGIN_NAME],
    )
    # state
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)

    def check_interfaces(
        registered_interfaces: VectorString, interface_names: VectorString
    ):
        for interface in interface_names:
            EXPECT_TRUE(interface in registered_interfaces)

    check_interfaces(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].command_interfaces,
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
    )
    EXPECT_EQ(len(status_map[TEST_SENSOR_HARDWARE_NAME].command_interfaces), 0)
    check_interfaces(
        status_map[TEST_SYSTEM_HARDWARE_NAME].command_interfaces,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
    )

    check_interfaces(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces,
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
    )
    EXPECT_TRUE(
        "joint1/some_unlisted_interface"
        in status_map[TEST_ACTUATOR_HARDWARE_NAME].state_interfaces
    )
    check_interfaces(
        status_map[TEST_SENSOR_HARDWARE_NAME].state_interfaces,
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
    )
    check_interfaces(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state_interfaces,
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
    )


def test_lifecycle_all_resources():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    # All resources start as UNCONFIGURED
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)
    del status_map

    ASSERT_TRUE(all(c == return_type.OK for c in configure_components(rm)))
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), INACTIVE)
    del status_map

    ASSERT_TRUE(all(c == return_type.OK for c in activate_components(rm)))
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), ACTIVE)
    del status_map

    ASSERT_TRUE(all(c == return_type.OK for c in deactivate_components(rm)))
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), INACTIVE)
    del status_map

    ASSERT_TRUE(all(c == return_type.OK for c in cleanup_components(rm)))
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)
    del status_map

    ASSERT_TRUE(all(c == return_type.OK for c in shutdown_components(rm)))
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), FINALIZED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), FINALIZED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), FINALIZED)
    del status_map


def test_lifecycle_individual_resources():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    # All resources start as UNCONFIGURED
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)
    del status_map

    configure_components(rm, [TEST_ACTUATOR_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)
    del status_map

    activate_components(rm, [TEST_ACTUATOR_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), UNCONFIGURED)
    del status_map

    configure_components(rm, [TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), INACTIVE)
    del status_map

    activate_components(rm, [TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), ACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), ACTIVE)
    del status_map

    deactivate_components(rm, [TEST_ACTUATOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), ACTIVE)
    del status_map

    cleanup_components(rm, [TEST_SENSOR_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), INACTIVE)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), ACTIVE)
    del status_map

    shutdown_components(rm, [TEST_ACTUATOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), FINALIZED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), UNCONFIGURED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), FINALIZED)
    del status_map

    shutdown_components(rm, [TEST_SENSOR_HARDWARE_NAME])
    status_map = rm.get_components_status()
    EXPECT_EQ(
        status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_ACTUATOR_HARDWARE_NAME].state.label(), FINALIZED)
    EXPECT_EQ(
        status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_SENSOR_HARDWARE_NAME].state.label(), FINALIZED)
    EXPECT_EQ(
        status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
        lifecycle_msgs_msg.State.PRIMARY_STATE_FINALIZED,
    )
    EXPECT_EQ(status_map[TEST_SYSTEM_HARDWARE_NAME].state.label(), FINALIZED)
    del status_map


def test_resource_availability_and_claiming_in_lifecycle():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    def check_interfaces(
        interface_names: VectorString, check_method, expected_result: bool
    ):
        for interface in interface_names:
            EXPECT_EQ(check_method(interface), expected_result)

    def check_interface_claiming(
        state_interface_names: VectorString,
        command_interface_names: VectorString,
        expected_result: bool,
    ):
        states = []
        commands = []

        if expected_result:
            for key in state_interface_names:
                states.append(rm.claim_state_interface(key))
            for key in command_interface_names:
                commands.append(rm.claim_command_interface(key))
        else:
            for key in state_interface_names:
                with pytest.raises(RuntimeError):
                    states.append(rm.claim_state_interface(key))
            for key in command_interface_names:
                with pytest.raises(RuntimeError):
                    commands.append(rm.claim_command_interface(key))

        check_interfaces(
            command_interface_names,
            lambda arg1: rm.command_interface_is_claimed(arg1),
            expected_result,
        )

    # All resources start as UNCONFIGURED - All interfaces are imported but not available
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        False,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )

    # Nothing can be claimed
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        False,
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], False)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        False,
    )

    # When actuator is configured all interfaces become available
    configure_components(rm, {TEST_ACTUATOR_HARDWARE_NAME})
    check_interfaces(
        ["joint1/position"], lambda arg1: rm.command_interface_is_available(arg1), True
    )
    check_interfaces(
        ["joint1/max_velocity"],
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        False,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )

    # Can claim Actuator's interfaces
    check_interface_claiming([], ["joint1/position"], True)
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, ["joint1/max_velocity"], True
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], False)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        False,
    )

    # When actuator is activated all state- and command- interfaces become available
    activate_components(rm, {TEST_ACTUATOR_HARDWARE_NAME})
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        False,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )

    # Can claim all Actuator's state interfaces and command interfaces
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        True,
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], False)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        False,
    )

    # Check if all interfaces still exits
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )

    # When Sensor and System are configured their state-
    # and command- interfaces are available
    configure_components(rm, [TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME])
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        ["joint2/velocity", "joint3/velocity"],
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        ["joint2/max_acceleration", "configuration/max_tcp_jerk"],
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )

    # Can claim:
    # - all Actuator's state interfaces and command interfaces
    # - sensor's state interfaces
    # - system's state and command interfaces
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        True,
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], True)
    check_interface_claiming([], ["joint2/velocity", "joint3/velocity"], True)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        ["joint2/max_acceleration", "configuration/max_tcp_jerk"],
        True,
    )

    # All active - everything available
    activate_components(rm, [TEST_SENSOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME])
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )

    # Can claim everything
    # - actuator's state interfaces and command interfaces
    # - sensor's state interfaces
    # - system's state and non-moving command interfaces
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        True,
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], True)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        True,
    )

    # When deactivated - movement interfaces are not available anymore
    deactivate_components(rm, [TEST_ACTUATOR_HARDWARE_NAME, TEST_SENSOR_HARDWARE_NAME])
    check_interfaces(
        ["joint1/position"], lambda arg1: rm.command_interface_is_available(arg1), True
    )
    check_interfaces(
        ["joint1/max_velocity"],
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )

    # Can claim everything
    # - actuator's state and command interfaces
    # - sensor's state interfaces
    # - system's state and command interfaces
    check_interface_claiming([], ["joint1/position"], True)
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, ["joint1/max_velocity"], True
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], True)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        True,
    )

    # When sensor is cleaned up the interfaces are not available anymore
    cleanup_components(rm, {TEST_SENSOR_HARDWARE_NAME})
    check_interfaces(
        ["joint1/position"], lambda arg1: rm.command_interface_is_available(arg1), True
    )
    check_interfaces(
        ["joint1/max_velocity"],
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_is_available(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        False,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_is_available(arg1),
        True,
    )

    # Can claim everything
    # - actuator's state and command interfaces
    # - no sensor's interface
    # - system's state and command interfaces
    check_interface_claiming([], ["joint1/position"], True)
    check_interface_claiming(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES, ["joint1/max_velocity"], True
    )
    check_interface_claiming(TEST_SENSOR_HARDWARE_STATE_INTERFACES, [], False)
    check_interface_claiming(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        True,
    )

    # Check if all interfaces still exits
    check_interfaces(
        TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES,
        lambda arg1: rm.command_interface_exists(arg1),
        True,
    )

    check_interfaces(
        TEST_ACTUATOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SENSOR_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )
    check_interfaces(
        TEST_SYSTEM_HARDWARE_STATE_INTERFACES,
        lambda arg1: rm.state_interface_exists(arg1),
        True,
    )


def test_managing_controllers_reference_interfaces():
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf)

    CONTROLLER_NAME = "test_controller"
    REFERENCE_INTERFACE_NAMES = VectorString(["input1", "input2", "input3"])
    FULL_REFERENCE_INTERFACE_NAMES = VectorString(
        [
            CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[0],
            CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[1],
            CONTROLLER_NAME + "/" + REFERENCE_INTERFACE_NAMES[2],
        ]
    )

    reference_interfaces = []
    reference_interface_values = [FloatRef(1.0), FloatRef(2.0), FloatRef(3.0)]

    for name, value in zip(REFERENCE_INTERFACE_NAMES, reference_interface_values):
        reference_interfaces.append(CommandInterface(CONTROLLER_NAME, name, value))

    rm.import_controller_reference_interfaces(CONTROLLER_NAME, reference_interfaces)

    ASSERT_EQ(
        rm.get_controller_reference_interface_names(CONTROLLER_NAME),
        FULL_REFERENCE_INTERFACE_NAMES,
    )

    # check that all interfaces are imported properly
    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_TRUE(rm.command_interface_exists(interface))
        EXPECT_FALSE(rm.command_interface_is_available(interface))
        EXPECT_FALSE(rm.command_interface_is_claimed(interface))

    # make interface available
    rm.make_controller_reference_interfaces_available(CONTROLLER_NAME)
    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_TRUE(rm.command_interface_exists(interface))
        EXPECT_TRUE(rm.command_interface_is_available(interface))
        EXPECT_FALSE(rm.command_interface_is_claimed(interface))

    # try to make interfaces available from unknown controller
    with pytest.raises(IndexError):
        rm.make_controller_reference_interfaces_available("unknown_controller")

    # claim interfaces in a scope that deletes them after
    claimed_itf1 = rm.claim_command_interface(FULL_REFERENCE_INTERFACE_NAMES[0])
    claimed_itf3 = rm.claim_command_interface(FULL_REFERENCE_INTERFACE_NAMES[2])

    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_TRUE(rm.command_interface_exists(interface))
        EXPECT_TRUE(rm.command_interface_is_available(interface))
    EXPECT_TRUE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[0]))
    EXPECT_FALSE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[1]))
    EXPECT_TRUE(rm.command_interface_is_claimed(FULL_REFERENCE_INTERFACE_NAMES[2]))

    # access interface value
    EXPECT_EQ(claimed_itf1.get_value(), 1.0)
    EXPECT_EQ(claimed_itf3.get_value(), 3.0)

    claimed_itf1.set_value(11.1)
    claimed_itf3.set_value(33.3)
    EXPECT_EQ(claimed_itf1.get_value(), 11.1)
    EXPECT_EQ(claimed_itf3.get_value(), 33.3)

    EXPECT_EQ(reference_interface_values[0], 11.1)
    EXPECT_EQ(reference_interface_values[1], 2.0)
    EXPECT_EQ(reference_interface_values[2], 33.3)

    del claimed_itf1
    del claimed_itf3

    # interfaces should be released now, but still managed by resource manager
    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_TRUE(rm.command_interface_exists(interface))
        EXPECT_TRUE(rm.command_interface_is_available(interface))
        EXPECT_FALSE(rm.command_interface_is_claimed(interface))

    # make interfaces unavailable
    rm.make_controller_reference_interfaces_unavailable(CONTROLLER_NAME)
    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_TRUE(rm.command_interface_exists(interface))
        EXPECT_FALSE(rm.command_interface_is_available(interface))
        EXPECT_FALSE(rm.command_interface_is_claimed(interface))

    # try to make interfaces unavailable from unknown controller
    with pytest.raises(IndexError):
        rm.make_controller_reference_interfaces_unavailable("unknown_controller")

    # Last written values should stay
    EXPECT_EQ(reference_interface_values[0], 11.1)
    EXPECT_EQ(reference_interface_values[1], 2.0)
    EXPECT_EQ(reference_interface_values[2], 33.3)

    # remove reference interfaces from resource manager
    rm.remove_controller_reference_interfaces(CONTROLLER_NAME)

    # they should not exist in resource manager
    for interface in FULL_REFERENCE_INTERFACE_NAMES:
        EXPECT_FALSE(rm.command_interface_exists(interface))
        EXPECT_FALSE(rm.command_interface_is_available(interface))

    # try to remove interfaces from unknown controller
    with pytest.raises(IndexError):
        rm.make_controller_reference_interfaces_unavailable("unknown_controller")


class TestResourceManagerReadWriteError:
    def setup_method(self, method):
        self.rm = TestableResourceManager()
        self.claimed_itfs = []
        self.time = Time(0)
        self.duration = Duration(0, 10000000)
        # values to set to hardware to simulate failure on read and write

    def setup_resource_manager_and_do_initial_checks(self):
        self.rm = TestableResourceManager(
            ros2_control_test_assets.minimal_robot_urdf, False
        )
        activate_components(self.rm)

        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SENSOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )

        self.claimed_itfs.append(
            self.rm.claim_command_interface(
                TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES[0]
            )
        )
        self.claimed_itfs.append(
            self.rm.claim_command_interface(TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES[0])
        )

        self.check_if_interface_available(True, True)
        # with default values read and write should run without any problems
        ok, failed_hardware_names = self.rm.read(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        ok, failed_hardware_names = self.rm.write(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

    # check if all interfaces are available
    def check_if_interface_available(
        self, actuator_interfaces: bool, system_interfaces: bool
    ):
        for interface in TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES:
            EXPECT_EQ(
                self.rm.command_interface_is_available(interface), actuator_interfaces
            )
        for interface in TEST_ACTUATOR_HARDWARE_STATE_INTERFACES:
            EXPECT_EQ(
                self.rm.state_interface_is_available(interface), actuator_interfaces
            )
        for interface in TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES:
            EXPECT_EQ(
                self.rm.command_interface_is_available(interface), system_interfaces
            )
        for interface in TEST_SYSTEM_HARDWARE_STATE_INTERFACES:
            EXPECT_EQ(
                self.rm.state_interface_is_available(interface), system_interfaces
            )

    def check_read_or_write_failure(
        self, method_that_fails, other_method, fail_value: bool
    ):
        # define state to set components to
        state_active = State(lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE, ACTIVE)

        # read failure for TEST_ACTUATOR_HARDWARE_NAME
        self.claimed_itfs[0].set_value(fail_value)
        self.claimed_itfs[1].set_value(fail_value - 10.0)
        ok, failed_hardware_names = method_that_fails(self.time, self.duration)
        EXPECT_FALSE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        ASSERT_EQ(failed_hardware_names, VectorString([TEST_ACTUATOR_HARDWARE_NAME]))
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(False, True)
        self.rm.set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

        # read failure for TEST_SYSTEM_HARDWARE_NAME
        self.claimed_itfs[0].set_value(fail_value - 10.0)
        self.claimed_itfs[1].set_value(fail_value)
        ok, failed_hardware_names = method_that_fails(self.time, self.duration)
        EXPECT_FALSE(ok)
        EXPECT_NE(len(failed_hardware_names), 0)
        ASSERT_EQ(failed_hardware_names, VectorString([TEST_SYSTEM_HARDWARE_NAME]))
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
        )
        self.check_if_interface_available(True, False)
        self.rm.set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

        # read failure for both, TEST_ACTUATOR_HARDWARE_NAME and TEST_SYSTEM_HARDWARE_NAME
        self.claimed_itfs[0].set_value(fail_value)
        self.claimed_itfs[1].set_value(fail_value)
        ok, failed_hardware_names = method_that_fails(self.time, self.duration)
        EXPECT_FALSE(ok)
        EXPECT_NE(len(failed_hardware_names), 0)
        ASSERT_EQ(
            failed_hardware_names,
            VectorString([TEST_ACTUATOR_HARDWARE_NAME, TEST_SYSTEM_HARDWARE_NAME]),
        )
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_UNCONFIGURED,
        )
        self.check_if_interface_available(False, False)
        self.rm.set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active)
        self.rm.set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

    def check_read_or_write_deactivate(
        self, method_that_deactivates, other_method, deactivate_value: bool
    ):
        # define state to set components to
        state_active = State(lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE, ACTIVE)

        # deactivate for TEST_ACTUATOR_HARDWARE_NAME
        self.claimed_itfs[0].set_value(deactivate_value)
        self.claimed_itfs[1].set_value(deactivate_value - 10.0)
        # deactivate on error
        ok, failed_hardware_names = method_that_deactivates(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)

        # reactivate
        self.rm.set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        del status_map
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

        # deactivate for TEST_SYSTEM_HARDWARE_NAME
        self.claimed_itfs[0].set_value(deactivate_value - 10.0)
        self.claimed_itfs[1].set_value(deactivate_value)
        # deactivate on flag
        ok, failed_hardware_names = method_that_deactivates(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
        )
        self.check_if_interface_available(True, True)
        # re-activate
        self.rm.set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        del status_map
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

        # deactivate both, TEST_ACTUATOR_HARDWARE_NAME and TEST_SYSTEM_HARDWARE_NAME
        self.claimed_itfs[0].set_value(deactivate_value)
        self.claimed_itfs[1].set_value(deactivate_value)
        # deactivate on flag
        ok, failed_hardware_names = method_that_deactivates(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_INACTIVE,
        )
        self.check_if_interface_available(True, True)
        # re-activate
        self.rm.set_component_state(TEST_ACTUATOR_HARDWARE_NAME, state_active)
        self.rm.set_component_state(TEST_SYSTEM_HARDWARE_NAME, state_active)
        status_map = self.rm.get_components_status()
        EXPECT_EQ(
            status_map[TEST_ACTUATOR_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        EXPECT_EQ(
            status_map[TEST_SYSTEM_HARDWARE_NAME].state.id(),
            lifecycle_msgs_msg.State.PRIMARY_STATE_ACTIVE,
        )
        self.check_if_interface_available(True, True)
        del status_map
        # write is sill OK
        ok, failed_hardware_names = other_method(self.time, self.duration)
        EXPECT_TRUE(ok)
        EXPECT_EQ(len(failed_hardware_names), 0)
        self.check_if_interface_available(True, True)

    def test_handle_error_on_hardware_read(self):
        if ROS_DISTRO != "rolling":
            return
        self.setup_resource_manager_and_do_initial_checks()

        # check read methods failures
        self.check_read_or_write_failure(
            lambda time, duration: rm.read(time, duration),
            lambda time, duration: rm.write(time, duration),
            test_constants.READ_FAIL_VALUE,
        )

    def test_handle_error_on_hardware_write(self):
        if ROS_DISTRO != "rolling":
            return
        self.setup_resource_manager_and_do_initial_checks()

        # check write methods failures
        self.check_read_or_write_failure(
            lambda time, duration: rm.write(time, duration),
            lambda time, duration: rm.read(time, duration),
            test_constants.WRITE_FAIL_VALUE,
        )

    def test_handle_deactivate_on_hardware_read(self):
        if ROS_DISTRO != "rolling":
            return
        self.setup_resource_manager_and_do_initial_checks()

        # check read methods failures
        self.check_read_or_write_deactivate(
            lambda time, duration: rm.read(time, duration),
            lambda time, duration: rm.write(time, duration),
            test_constants.READ_DEACTIVATE_VALUE,
        )

    def test_handle_deactivate_on_hardware_write(self):
        if ROS_DISTRO != "rolling":
            return
        self.setup_resource_manager_and_do_initial_checks()

        # check write methods failures
        self.check_read_or_write_deactivate(
            lambda time, duration: rm.write(time, duration),
            lambda time, duration: rm.read(time, duration),
            test_constants.WRITE_DEACTIVATE_VALUE,
        )


def test_test_caching_of_controllers_to_hardware():
    if ROS_DISTRO != "rolling":
        return
    rm = TestableResourceManager(ros2_control_test_assets.minimal_robot_urdf, False)
    activate_components(rm)

    TEST_CONTROLLER_ACTUATOR_NAME = "test_controller_actuator"
    TEST_CONTROLLER_SYSTEM_NAME = "test_controller_system"
    TEST_BROADCASTER_ALL_NAME = "test_broadcaster_all"
    TEST_BROADCASTER_SENSOR_NAME = "test_broadcaster_sensor"

    rm.cache_controller_to_hardware(
        TEST_CONTROLLER_ACTUATOR_NAME, TEST_ACTUATOR_HARDWARE_COMMAND_INTERFACES
    )
    rm.cache_controller_to_hardware(
        TEST_BROADCASTER_ALL_NAME, TEST_ACTUATOR_HARDWARE_STATE_INTERFACES
    )

    rm.cache_controller_to_hardware(
        TEST_CONTROLLER_SYSTEM_NAME, TEST_SYSTEM_HARDWARE_COMMAND_INTERFACES
    )
    rm.cache_controller_to_hardware(
        TEST_BROADCASTER_ALL_NAME, TEST_SYSTEM_HARDWARE_STATE_INTERFACES
    )

    rm.cache_controller_to_hardware(
        TEST_BROADCASTER_SENSOR_NAME, TEST_SENSOR_HARDWARE_STATE_INTERFACES
    )
    rm.cache_controller_to_hardware(
        TEST_BROADCASTER_ALL_NAME, TEST_SENSOR_HARDWARE_STATE_INTERFACES
    )

    controllers = rm.get_cached_controllers_to_hardware(TEST_ACTUATOR_HARDWARE_NAME)
    ASSERT_EQ(
        controllers,
        VectorString([TEST_CONTROLLER_ACTUATOR_NAME, TEST_BROADCASTER_ALL_NAME]),
    )

    controllers = rm.get_cached_controllers_to_hardware(TEST_SYSTEM_HARDWARE_NAME)
    ASSERT_EQ(
        controllers,
        VectorString([TEST_CONTROLLER_SYSTEM_NAME, TEST_BROADCASTER_ALL_NAME]),
    )

    controllers = rm.get_cached_controllers_to_hardware(TEST_SENSOR_HARDWARE_NAME)
    ASSERT_EQ(
        controllers,
        VectorString([TEST_BROADCASTER_SENSOR_NAME, TEST_BROADCASTER_ALL_NAME]),
    )
