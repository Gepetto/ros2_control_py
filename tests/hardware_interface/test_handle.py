from ros2_control_py.hardware_interface import (
    CommandInterface,
    StateInterface,
    FloatRef,
)
import pytest

JOINT_NAME = "joint_1"
FOO_INTERFACE = "FooInterface"


def test_command_interface():
    value = FloatRef(1.337)
    interface = CommandInterface(JOINT_NAME, FOO_INTERFACE, value)
    assert interface.get_value() == value.value()
    interface.set_value(0.0)
    assert interface.get_value() == 0.0


def test_state_interface():
    value = FloatRef(1.337)
    interface = StateInterface(JOINT_NAME, FOO_INTERFACE, value)
    assert interface.get_value() == value.value()
    assert not hasattr(interface, "set_value")


def test_name_getters_work():
    handle = StateInterface(JOINT_NAME, FOO_INTERFACE)
    assert handle.get_name() == JOINT_NAME + "/" + FOO_INTERFACE
    assert handle.get_interface_name() == FOO_INTERFACE
    assert handle.get_prefix_name() == JOINT_NAME


def test_value_methods_throw_for_nullptr():
    handle = CommandInterface(JOINT_NAME, FOO_INTERFACE)
    with pytest.raises(RuntimeError):
        handle.get_value()
    with pytest.raises(RuntimeError):
        handle.set_value(0.0)


def test_value_methods_work_on_non_nullptr():
    value = FloatRef(1.337)
    handle = CommandInterface(JOINT_NAME, FOO_INTERFACE, value)
    assert handle.get_value() == value.value()
    handle.set_value(0.0)
    assert handle.get_value() == 0.0
