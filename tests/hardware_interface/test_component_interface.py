from ros2_control_py.hardware_interface import (
    Actuator,
    ActuatorInterface,
    CommandInterface,
    StateInterface,
    HardwareInfo,
    HW_IF_POSITION,
    HW_IF_VELOCITY,
    UNCONFIGURED,
    ACTIVE,
    INACTIVE,
    FINALIZED,
    return_type,
    FloatRefProp,
    CallbackReturn,
    Time,
    Duration,
    VectorString,
)
from lifecycle_msgs.msg import State
from math import isnan, nan

TIME = Time(0)
PERIOD = Duration(0, 10000000)
TRIGGER_READ_WRITE_ERROR_CALLS = 10000


class DummyActuator(ActuatorInterface):
    position_state_ = FloatRefProp(nan)
    velocity_state_ = FloatRefProp(nan)
    velocity_command_ = FloatRefProp(0.0)

    def __init__(self):
        super().__init__()
        self.read_calls_ = 0
        self.write_calls_ = 0
        self.recoverable_error_happened_ = False

    def on_init(self, info):
        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state):
        self.position_state_ = 0.0
        self.velocity_state_ = 0.0
        if self.recoverable_error_happened_:
            velocity_command_ = 0.0
        self.read_calls_ = 0
        self.write_calls_ = 0
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self):
        state_interfaces = []
        state_interfaces.append(
            StateInterface("joint1", HW_IF_POSITION, self.position_state_)
        )
        state_interfaces.append(
            StateInterface("joint1", HW_IF_VELOCITY, self.velocity_state_)
        )
        return state_interfaces

    def export_command_interfaces(self):
        # We can command in velocity
        command_interfaces = []
        command_interfaces.append(
            CommandInterface("joint1", HW_IF_VELOCITY, self.velocity_command_)
        )
        return command_interfaces

    def get_name(self):
        return "DummyActuator"

    def read(self, time, period):
        self.read_calls_ += 1
        if self.read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        # no-op, state is getting propagated within write.
        return return_type.OK

    def write(self, time, period):
        self.write_calls_ += 1
        if self.write_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        self.position_state_ += self.velocity_command_
        self.velocity_state_ = self.velocity_command_
        return return_type.OK

    def on_shutdown(self, previous_state):
        self.velocity_state_ = 0
        return CallbackReturn.SUCCESS

    def on_error(self, previous_state):
        if not self.recoverable_error_happened_:
            self.recoverable_error_happened_ = True
            return CallbackReturn.SUCCESS
        else:
            return CallbackReturn.ERROR
        return CallbackReturn.FAILURE


def ASSERT_TRUE(a):
    assert a


def ASSERT_EQ(a, b):
    assert a == b


def EXPECT_EQ(a, b):
    ASSERT_EQ(a, b)


def test_dummy_actuator():
    dummy_actuator_hw = DummyActuator()
    actuator_hw = Actuator(dummy_actuator_hw)

    mock_hw_info = HardwareInfo()
    state = actuator_hw.initialize(mock_hw_info)
    EXPECT_EQ(State.PRIMARY_STATE_UNCONFIGURED, state.id())
    EXPECT_EQ(UNCONFIGURED, state.label())

    state_interfaces = actuator_hw.export_state_interfaces()
    ASSERT_EQ(2, len(state_interfaces))
    EXPECT_EQ("joint1/position", state_interfaces[0].get_name())
    EXPECT_EQ(HW_IF_POSITION, state_interfaces[0].get_interface_name())
    EXPECT_EQ("joint1", state_interfaces[0].get_prefix_name())
    EXPECT_EQ("joint1/velocity", state_interfaces[1].get_name())
    EXPECT_EQ(HW_IF_VELOCITY, state_interfaces[1].get_interface_name())
    EXPECT_EQ("joint1", state_interfaces[1].get_prefix_name())

    command_interfaces = actuator_hw.export_command_interfaces()
    ASSERT_EQ(1, len(command_interfaces))
    EXPECT_EQ("joint1/velocity", command_interfaces[0].get_name())
    EXPECT_EQ(HW_IF_VELOCITY, command_interfaces[0].get_interface_name())
    EXPECT_EQ("joint1", command_interfaces[0].get_prefix_name())

    velocity_value = 1.0
    command_interfaces[0].set_value(velocity_value)
    ASSERT_EQ(return_type.ERROR, actuator_hw.write(TIME, PERIOD))

    # Noting should change because it is UNCONFIGURED
    for step in range(10):
        ASSERT_EQ(return_type.ERROR, actuator_hw.read(TIME, PERIOD))

        ASSERT_TRUE(isnan(state_interfaces[0].get_value()))  # position value
        ASSERT_TRUE(isnan(state_interfaces[1].get_value()))  # velocity

        ASSERT_EQ(return_type.ERROR, actuator_hw.write(TIME, PERIOD))

    state = actuator_hw.configure()
    EXPECT_EQ(State.PRIMARY_STATE_INACTIVE, state.id())
    EXPECT_EQ(INACTIVE, state.label())

    # Read and Write are working because it is INACTIVE
    for step in range(10):
        ASSERT_EQ(return_type.OK, actuator_hw.read(TIME, PERIOD))

        EXPECT_EQ(step * velocity_value, state_interfaces[0].get_value())
        # position value
        EXPECT_EQ(velocity_value if step else 0, state_interfaces[1].get_value())
        # velocity

        ASSERT_EQ(return_type.OK, actuator_hw.write(TIME, PERIOD))

    state = actuator_hw.activate()
    EXPECT_EQ(State.PRIMARY_STATE_ACTIVE, state.id())
    EXPECT_EQ(ACTIVE, state.label())

    # Read and Write are working because it is ACTIVE
    for step in range(10):
        ASSERT_EQ(return_type.OK, actuator_hw.read(TIME, PERIOD))

        EXPECT_EQ((10 + step) * velocity_value, state_interfaces[0].get_value())
        # position value
        EXPECT_EQ(velocity_value, state_interfaces[1].get_value())
        # velocity

        ASSERT_EQ(return_type.OK, actuator_hw.write(TIME, PERIOD))

    state = actuator_hw.shutdown()
    EXPECT_EQ(State.PRIMARY_STATE_FINALIZED, state.id())
    EXPECT_EQ(FINALIZED, state.label())

    # Noting should change because it is FINALIZED
    for step in range(10):
        ASSERT_EQ(return_type.ERROR, actuator_hw.read(TIME, PERIOD))

        EXPECT_EQ(20 * velocity_value, state_interfaces[0].get_value())
        # position value
        EXPECT_EQ(0, state_interfaces[1].get_value())
        # velocity

        ASSERT_EQ(return_type.ERROR, actuator_hw.write(TIME, PERIOD))

    EXPECT_EQ(
        return_type.OK,
        actuator_hw.prepare_command_mode_switch(VectorString([""]), VectorString([""])),
    )
    EXPECT_EQ(
        return_type.OK,
        actuator_hw.perform_command_mode_switch(VectorString([""]), VectorString([""])),
    )
