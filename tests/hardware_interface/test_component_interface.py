from ros2_control_py.hardware_interface import (
    Actuator,
    Sensor,
    System,
    ActuatorInterface,
    SensorInterface,
    SystemInterface,
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
    CallbackReturn,
    Time,
    Duration,
    FloatRefProp,
    FloatRef,
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

    def __init__(self) -> None:
        super().__init__()
        self.read_calls_ = 0
        self.write_calls_ = 0
        self.recoverable_error_happened_ = False

    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state: State) -> CallbackReturn:
        self.position_state_ = 0.0
        self.velocity_state_ = 0.0
        if self.recoverable_error_happened_:
            velocity_command_ = 0.0
        self.read_calls_ = 0
        self.write_calls_ = 0
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self) -> list[StateInterface]:
        return [
            StateInterface("joint1", HW_IF_POSITION, self.position_state_),
            StateInterface("joint1", HW_IF_VELOCITY, self.velocity_state_),
        ]

    def export_command_interfaces(self) -> list[CommandInterface]:
        # We can command in velocity
        return [CommandInterface("joint1", HW_IF_VELOCITY, self.velocity_command_)]

    def get_name(self) -> str:
        return "DummyActuator"

    def read(self, time: Time, period: Duration) -> return_type:
        self.read_calls_ += 1
        if self.read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        # no-op, state is getting propagated within write.
        return return_type.OK

    def write(self, time: Time, period: Duration) -> return_type:
        self.write_calls_ += 1
        if self.write_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        self.position_state_ += self.velocity_command_
        self.velocity_state_ = self.velocity_command_
        return return_type.OK

    def on_shutdown(self, previous_state: State) -> CallbackReturn:
        self.velocity_state_ = 0
        return CallbackReturn.SUCCESS

    def on_error(self, previous_state: State) -> CallbackReturn:
        if not self.recoverable_error_happened_:
            self.recoverable_error_happened_ = True
            return CallbackReturn.SUCCESS
        else:
            return CallbackReturn.ERROR
        return CallbackReturn.FAILURE


class DummySensor(SensorInterface):
    voltage_level_ = FloatRefProp(nan)

    def __init__(self) -> None:
        super().__init__()
        self.voltage_level_hw_value_ = 0x666
        # Helper variables to initiate error on read
        self.read_calls_ = 0
        self.recoverable_error_happened_ = False

    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        # We hardcode the info
        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state: State) -> CallbackReturn:
        self.voltage_level_ = 0.0
        self.read_calls_ = 0
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self) -> list[StateInterface]:
        # We can read some voltage level
        return [StateInterface("joint1", "voltage", self.voltage_level_)]

    def get_name(self) -> str:
        return "DummySensor"

    def read(self, time: Time, period: Duration) -> return_type:
        self.read_calls_ += 1
        if self.read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        # no-op, static value
        self.voltage_level_ = self.voltage_level_hw_value_
        return return_type.OK

    def on_error(self, previous_state: State) -> CallbackReturn:
        if not self.recoverable_error_happened_:
            self.recoverable_error_happened_ = True
            return CallbackReturn.SUCCESS
        else:
            return CallbackReturn.ERROR
        return CallbackReturn.FAILURE


class DummySystem(SystemInterface):
    def __init__(self):
        super().__init__()
        self.position_state_ = [FloatRef(nan), FloatRef(nan), FloatRef(nan)]
        self.velocity_state_ = [FloatRef(nan), FloatRef(nan), FloatRef(nan)]
        self.velocity_command_ = [FloatRef(0), FloatRef(0), FloatRef(0)]
        # Helper variables to initiate error on read
        self.read_calls_ = 0
        self.write_calls_ = 0
        self.recoverable_error_happened_ = False

    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        # We hardcode the info
        return CallbackReturn.SUCCESS

    def on_configure(self, previous_state: State) -> CallbackReturn:
        for p, v in zip(self.position_state_, self.velocity_state_):
            p @= 0
            v @= 0
        # reset command only if error is initiated
        if self.recoverable_error_happened_:
            for v in self.velocity_command_:
                v @= 0
        self.read_calls_ = 0
        self.write_calls_ = 0
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self) -> list[StateInterface]:
        # We can read a position and a velocity
        return [
            StateInterface("joint1", HW_IF_POSITION, self.position_state_[0]),
            StateInterface("joint1", HW_IF_VELOCITY, self.velocity_state_[0]),
            StateInterface("joint2", HW_IF_POSITION, self.position_state_[1]),
            StateInterface("joint2", HW_IF_VELOCITY, self.velocity_state_[1]),
            StateInterface("joint3", HW_IF_POSITION, self.position_state_[2]),
            StateInterface("joint3", HW_IF_VELOCITY, self.velocity_state_[2]),
        ]

    def export_command_interfaces(self) -> list[CommandInterface]:
        # We can command in velocity
        return [
            CommandInterface("joint1", HW_IF_VELOCITY, self.velocity_command_[0]),
            CommandInterface("joint2", HW_IF_VELOCITY, self.velocity_command_[1]),
            CommandInterface("joint3", HW_IF_VELOCITY, self.velocity_command_[2]),
        ]

    def get_name(self) -> str:
        return "DummySystem"

    def read(self, time: Time, period: Duration) -> return_type:
        self.read_calls_ += 1
        if self.read_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        # no-op, state is getting propagated within write.
        return return_type.OK

    def write(self, time: Time, period: Duration) -> return_type:
        self.write_calls_ += 1
        if self.write_calls_ == TRIGGER_READ_WRITE_ERROR_CALLS:
            return return_type.ERROR
        for p, v in zip(self.position_state_, self.velocity_state_):
            p += self.velocity_command_[0]
            v @= self.velocity_command_[0]
        return return_type.OK

    def on_shutdown(self, previous_state: State) -> CallbackReturn:
        for v in self.velocity_state_:
            v @= 0
        return CallbackReturn.SUCCESS

    def on_error(self, previous_state: State) -> CallbackReturn:
        if not self.recoverable_error_happened_:
            self.recoverable_error_happened_ = True
            return CallbackReturn.SUCCESS
        else:
            return CallbackReturn.ERROR
        return CallbackReturn.FAILURE


class DummySystemPreparePerform(SystemInterface):
    def __init__(self):
        super().__init__()

    # Override the pure virtual functions with default behavior
    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        # We hardcode the info
        return CallbackReturn.SUCCESS

    def export_state_interfaces(self) -> list[StateInterface]:
        return []

    def export_command_interfaces(self) -> list[CommandInterface]:
        return []

    def get_name(self) -> str:
        return "DummySystemPreparePerform"

    def read(self, time: Time, period: Duration) -> return_type:
        return return_type.OK

    def write(self, time: Time, period: Duration) -> return_type:
        return return_type.OK

    # Custom prepare/perform functions
    def prepare_command_mode_switch(
        self, start_interfaces: VectorString, stop_interfaces: VectorString
    ) -> return_type:
        # Criteria to test against
        if len(start_interfaces) != 1:
            return return_type.ERROR
        if len(stop_interfaces) != 2:
            return return_type.ERROR
        return return_type.OK

    def perform_command_mode_switch(
        self, start_interfaces: VectorString, stop_interfaces: VectorString
    ) -> return_type:
        # Criteria to test against
        if len(start_interfaces) != 1:
            return return_type.ERROR
        if len(stop_interfaces) != 2:
            return return_type.ERROR
        return return_type.OK


def test_dummy_actuator():
    actuator_hw = Actuator(DummyActuator())

    mock_hw_info = HardwareInfo()
    state = actuator_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = actuator_hw.export_state_interfaces()
    assert 2 == len(state_interfaces)
    assert "joint1/position" == state_interfaces[0].get_name()
    assert HW_IF_POSITION == state_interfaces[0].get_interface_name()
    assert "joint1" == state_interfaces[0].get_prefix_name()
    assert "joint1/velocity" == state_interfaces[1].get_name()
    assert HW_IF_VELOCITY == state_interfaces[1].get_interface_name()
    assert "joint1" == state_interfaces[1].get_prefix_name()

    command_interfaces = actuator_hw.export_command_interfaces()
    assert 1 == len(command_interfaces)
    assert "joint1/velocity" == command_interfaces[0].get_name()
    assert HW_IF_VELOCITY == command_interfaces[0].get_interface_name()
    assert "joint1" == command_interfaces[0].get_prefix_name()

    velocity_value = 1.0
    command_interfaces[0].set_value(velocity_value)
    assert return_type.ERROR == actuator_hw.write(TIME, PERIOD)

    # Noting should change because it is UNCONFIGURED
    for step in range(10):
        assert return_type.ERROR == actuator_hw.read(TIME, PERIOD)

        assert isnan(state_interfaces[0].get_value())  # position value
        assert isnan(state_interfaces[1].get_value())  # velocity

        assert return_type.ERROR == actuator_hw.write(TIME, PERIOD)

    state = actuator_hw.configure()
    assert State.PRIMARY_STATE_INACTIVE == state.id()
    assert INACTIVE == state.label()

    # Read and Write are working because it is INACTIVE
    for step in range(10):
        assert return_type.OK == actuator_hw.read(TIME, PERIOD)

        assert step * velocity_value == state_interfaces[0].get_value()
        # position value
        assert velocity_value if step else 0 == state_interfaces[1].get_value()
        # velocity

        assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    state = actuator_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    # Read and Write are working because it is ACTIVE
    for step in range(10):
        assert return_type.OK == actuator_hw.read(TIME, PERIOD)

        assert (10 + step) * velocity_value == state_interfaces[0].get_value()
        # position value
        assert velocity_value == state_interfaces[1].get_value()
        # velocity

        assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    state = actuator_hw.shutdown()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # Noting should change because it is FINALIZED
    for step in range(10):
        assert return_type.ERROR == actuator_hw.read(TIME, PERIOD)

        assert 20 * velocity_value == state_interfaces[0].get_value()
        # position value
        assert 0 == state_interfaces[1].get_value()
        # velocity

        assert return_type.ERROR == actuator_hw.write(TIME, PERIOD)

    assert return_type.OK == actuator_hw.prepare_command_mode_switch(
        VectorString([""]), VectorString([""])
    )
    assert return_type.OK == actuator_hw.perform_command_mode_switch(
        VectorString([""]), VectorString([""])
    )


def test_dummy_sensor():
    sensor_hw = Sensor(DummySensor())

    mock_hw_info = HardwareInfo()
    state = sensor_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = sensor_hw.export_state_interfaces()
    assert 1 == len(state_interfaces)
    assert "joint1/voltage" == state_interfaces[0].get_name()
    assert "voltage" == state_interfaces[0].get_interface_name()
    assert "joint1" == state_interfaces[0].get_prefix_name()
    assert isnan(state_interfaces[0].get_value())

    # Not updated because is is UNCONFIGURED
    sensor_hw.read(TIME, PERIOD)
    assert isnan(state_interfaces[0].get_value())

    # Updated because is is INACTIVE
    state = sensor_hw.configure()
    assert State.PRIMARY_STATE_INACTIVE == state.id()
    assert INACTIVE == state.label()
    assert 0 == state_interfaces[0].get_value()

    # It can read now
    sensor_hw.read(TIME, PERIOD)
    assert 0x666 == state_interfaces[0].get_value()


def test_dummy_system():
    system_hw = System(DummySystem())

    mock_hw_info = HardwareInfo()
    state = system_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = system_hw.export_state_interfaces()
    assert 6 == len(state_interfaces)
    assert "joint1/position" == state_interfaces[0].get_name()
    assert HW_IF_POSITION == state_interfaces[0].get_interface_name()
    assert "joint1" == state_interfaces[0].get_prefix_name()
    assert "joint1/velocity" == state_interfaces[1].get_name()
    assert HW_IF_VELOCITY == state_interfaces[1].get_interface_name()
    assert "joint1" == state_interfaces[1].get_prefix_name()
    assert "joint2/position" == state_interfaces[2].get_name()
    assert HW_IF_POSITION == state_interfaces[2].get_interface_name()
    assert "joint2" == state_interfaces[2].get_prefix_name()
    assert "joint2/velocity" == state_interfaces[3].get_name()
    assert HW_IF_VELOCITY == state_interfaces[3].get_interface_name()
    assert "joint2" == state_interfaces[3].get_prefix_name()
    assert "joint3/position" == state_interfaces[4].get_name()
    assert HW_IF_POSITION == state_interfaces[4].get_interface_name()
    assert "joint3" == state_interfaces[4].get_prefix_name()
    assert "joint3/velocity" == state_interfaces[5].get_name()
    assert HW_IF_VELOCITY == state_interfaces[5].get_interface_name()
    assert "joint3" == state_interfaces[5].get_prefix_name()

    command_interfaces = system_hw.export_command_interfaces()
    assert 3 == len(command_interfaces)
    assert "joint1/velocity" == command_interfaces[0].get_name()
    assert HW_IF_VELOCITY == command_interfaces[0].get_interface_name()
    assert "joint1" == command_interfaces[0].get_prefix_name()
    assert "joint2/velocity" == command_interfaces[1].get_name()
    assert HW_IF_VELOCITY == command_interfaces[1].get_interface_name()
    assert "joint2" == command_interfaces[1].get_prefix_name()
    assert "joint3/velocity" == command_interfaces[2].get_name()
    assert HW_IF_VELOCITY == command_interfaces[2].get_interface_name()
    assert "joint3" == command_interfaces[2].get_prefix_name()

    velocity_value = 1.0
    command_interfaces[0].set_value(velocity_value)  # velocity
    command_interfaces[1].set_value(velocity_value)  # velocity
    command_interfaces[2].set_value(velocity_value)  # velocity
    assert return_type.ERROR == system_hw.write(TIME, PERIOD)

    # Noting should change because it is UNCONFIGURED
    for step in range(10):
        assert return_type.ERROR == system_hw.read(TIME, PERIOD)

        assert isnan(state_interfaces[0].get_value())  # position value
        assert isnan(state_interfaces[1].get_value())  # velocity
        assert isnan(state_interfaces[2].get_value())  # position value
        assert isnan(state_interfaces[3].get_value())  # velocity
        assert isnan(state_interfaces[4].get_value())  # position value
        assert isnan(state_interfaces[5].get_value())  # velocity

        assert return_type.ERROR == system_hw.write(TIME, PERIOD)

    state = system_hw.configure()
    assert State.PRIMARY_STATE_INACTIVE == state.id()
    assert INACTIVE == state.label()

    # Read and Write are working because it is INACTIVE
    for step in range(10):
        assert return_type.OK == system_hw.read(TIME, PERIOD)

        assert (
            step * velocity_value == state_interfaces[0].get_value()
        )  # position value
        assert (
            velocity_value if step else 0 == state_interfaces[1].get_value()
        )  # velocity
        assert (
            step * velocity_value == state_interfaces[2].get_value()
        )  # position value
        assert (
            velocity_value if step else 0 == state_interfaces[3].get_value()
        )  # velocity
        assert (
            step * velocity_value == state_interfaces[4].get_value()
        )  # position value
        assert (
            velocity_value if step else 0 == state_interfaces[5].get_value()
        )  # velocity

        assert return_type.OK == system_hw.write(TIME, PERIOD)

    state = system_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    # Read and Write are working because it is ACTIVE
    for step in range(10):
        assert return_type.OK == system_hw.read(TIME, PERIOD)

        assert (10 + step) * velocity_value == state_interfaces[
            0
        ].get_value()  # position value
        assert velocity_value == state_interfaces[1].get_value()  # velocity
        assert (10 + step) * velocity_value == state_interfaces[
            2
        ].get_value()  # position value
        assert velocity_value == state_interfaces[3].get_value()  # velocity
        assert (10 + step) * velocity_value == state_interfaces[
            4
        ].get_value()  # position value
        assert velocity_value == state_interfaces[5].get_value()  # velocity

        assert return_type.OK == system_hw.write(TIME, PERIOD)

    state = system_hw.shutdown()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # Noting should change because it is FINALIZED
    for step in range(10):
        assert return_type.ERROR == system_hw.read(TIME, PERIOD)

        assert 20 * velocity_value == state_interfaces[0].get_value()  # position value
        assert 0 == state_interfaces[1].get_value()  # velocity
        assert 20 * velocity_value == state_interfaces[2].get_value()  # position value
        assert 0 == state_interfaces[3].get_value()  # velocity
        assert 20 * velocity_value == state_interfaces[4].get_value()  # position value
        assert 0 == state_interfaces[5].get_value()  # velocity

        assert return_type.ERROR == system_hw.write(TIME, PERIOD)

    assert return_type.OK == system_hw.prepare_command_mode_switch(
        VectorString([""]), VectorString([""])
    )
    assert return_type.OK == system_hw.perform_command_mode_switch(
        VectorString([""]), VectorString([""])
    )


def test_dummy_command_mode_system():
    system_hw = System(DummySystemPreparePerform())

    mock_hw_info = HardwareInfo()
    state = system_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    one_key = VectorString(["joint1/position"])
    two_keys = VectorString(["joint1/position", "joint1/velocity"])

    # Only calls with (one_key, two_keys) should return OK
    assert return_type.ERROR, system_hw.prepare_command_mode_switch(one_key == one_key)
    assert return_type.ERROR, system_hw.perform_command_mode_switch(one_key == one_key)
    assert return_type.OK == system_hw.prepare_command_mode_switch(one_key, two_keys)
    assert return_type.OK == system_hw.perform_command_mode_switch(one_key, two_keys)
    assert return_type.ERROR, system_hw.prepare_command_mode_switch(two_keys == one_key)
    assert return_type.ERROR, system_hw.perform_command_mode_switch(two_keys == one_key)


def test_dummy_actuator_read_error_behavior():
    actuator_hw = Actuator(DummyActuator())

    mock_hw_info = HardwareInfo()
    state = actuator_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = actuator_hw.export_state_interfaces()
    command_interfaces = actuator_hw.export_command_interfaces()
    state = actuator_hw.configure()
    state = actuator_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    # Initiate error on write (this is first time therefore recoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.ERROR == actuator_hw.read(TIME, PERIOD)

    state = actuator_hw.get_state()
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    # activate again and expect reset values
    state = actuator_hw.configure()
    assert state_interfaces[0].get_value() == 0
    assert command_interfaces[0].get_value() == 0

    state = actuator_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    # Initiate error on write (this is the second time therefore unrecoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.ERROR == actuator_hw.read(TIME, PERIOD)

    state = actuator_hw.get_state()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # can not change state anymore
    state = actuator_hw.configure()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()


def test_dummy_actuator_write_error_behavior():
    actuator_hw = Actuator(DummyActuator())

    mock_hw_info = HardwareInfo()
    state = actuator_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = actuator_hw.export_state_interfaces()
    command_interfaces = actuator_hw.export_command_interfaces()
    state = actuator_hw.configure()
    state = actuator_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    # Initiate error on write (this is first time therefore recoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == actuator_hw.write(TIME, PERIOD)
    assert return_type.ERROR == actuator_hw.write(TIME, PERIOD)

    state = actuator_hw.get_state()
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    # activate again and expect reset values
    state = actuator_hw.configure()
    assert state_interfaces[0].get_value() == 0
    assert command_interfaces[0].get_value() == 0

    state = actuator_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == actuator_hw.read(TIME, PERIOD)
    assert return_type.OK == actuator_hw.write(TIME, PERIOD)

    # Initiate error on write (this is the second time therefore unrecoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == actuator_hw.write(TIME, PERIOD)
    assert return_type.ERROR == actuator_hw.write(TIME, PERIOD)

    state = actuator_hw.get_state()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # can not change state anymore
    state = actuator_hw.configure()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()


def test_dummy_sensor_read_error_behavior():
    sensor_hw = Sensor(DummySensor())

    mock_hw_info = HardwareInfo()
    state = sensor_hw.initialize(mock_hw_info)

    state_interfaces = sensor_hw.export_state_interfaces()
    # Updated because is is INACTIVE
    state = sensor_hw.configure()
    state = sensor_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == sensor_hw.read(TIME, PERIOD)

    # Initiate recoverable error - call read 99 times OK and on 100-time will return error
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == sensor_hw.read(TIME, PERIOD)
    assert return_type.ERROR == sensor_hw.read(TIME, PERIOD)

    state = sensor_hw.get_state()
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    # activate again and expect reset values
    state = sensor_hw.configure()
    assert state_interfaces[0].get_value() == 0

    state = sensor_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    # Initiate unrecoverable error - call read 99 times OK and on 100-time will return error
    for i in range(1, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == sensor_hw.read(TIME, PERIOD)
    assert return_type.ERROR == sensor_hw.read(TIME, PERIOD)

    state = sensor_hw.get_state()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # can not change state anymore
    state = sensor_hw.configure()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()


def test_dummy_system_read_error_behavior():
    system_hw = System(DummySystem())

    mock_hw_info = HardwareInfo()
    state = system_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = system_hw.export_state_interfaces()
    command_interfaces = system_hw.export_command_interfaces()
    state = system_hw.configure()
    state = system_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.OK == system_hw.write(TIME, PERIOD)

    # Initiate error on write (this is first time therefore recoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.ERROR == system_hw.read(TIME, PERIOD)

    state = system_hw.get_state()
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    # activate again and expect reset values
    state = system_hw.configure()
    for index in range(6):
        assert state_interfaces[index].get_value() == 0
    for index in range(3):
        assert command_interfaces[index].get_value() == 0
    state = system_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.OK == system_hw.write(TIME, PERIOD)

    # Initiate error on write (this is the second time therefore unrecoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.ERROR == system_hw.read(TIME, PERIOD)

    state = system_hw.get_state()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # can not change state anymore
    state = system_hw.configure()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()


def test_dummy_system_write_error_behavior():
    system_hw = System(DummySystem())

    mock_hw_info = HardwareInfo()
    state = system_hw.initialize(mock_hw_info)
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    state_interfaces = system_hw.export_state_interfaces()
    command_interfaces = system_hw.export_command_interfaces()
    state = system_hw.configure()
    state = system_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.OK == system_hw.write(TIME, PERIOD)

    # Initiate error on write (this is first time therefore recoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == system_hw.write(TIME, PERIOD)
    assert return_type.ERROR == system_hw.write(TIME, PERIOD)

    state = system_hw.get_state()
    assert State.PRIMARY_STATE_UNCONFIGURED == state.id()
    assert UNCONFIGURED == state.label()

    # activate again and expect reset values
    state = system_hw.configure()
    for index in range(6):
        assert state_interfaces[index].get_value() == 0
    for index in range(3):
        assert command_interfaces[index].get_value() == 0
    state = system_hw.activate()
    assert State.PRIMARY_STATE_ACTIVE == state.id()
    assert ACTIVE == state.label()

    assert return_type.OK == system_hw.read(TIME, PERIOD)
    assert return_type.OK == system_hw.write(TIME, PERIOD)

    # Initiate error on write (this is the second time therefore unrecoverable)
    for i in range(2, TRIGGER_READ_WRITE_ERROR_CALLS):
        assert return_type.OK == system_hw.write(TIME, PERIOD)
    assert return_type.ERROR == system_hw.write(TIME, PERIOD)

    state = system_hw.get_state()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()

    # can not change state anymore
    state = system_hw.configure()
    assert State.PRIMARY_STATE_FINALIZED == state.id()
    assert FINALIZED == state.label()
