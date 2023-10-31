
###############
ros2_control_py
###############

Parsed Modules
==============

* hardware_interface
* controller_interface
* controller_manager
* ros2_control_test_assets

Tested Features
===============

hardware_interface
------------------

* parse_control_resources_from_urdf
* StateInterface / CommandInterface (see FloatRef_ and FloatRefProp_)
* Actuator / Sensor / System
* ActuatorInterface / SensorInterface / SystemInterface

rclcpp
------

* FloatRef_

New Interface
=============

rclcpp
------

| Because rclpy and rclcpp are bindings over rclc and the fact that ros2 control is written over rclcpp and not rclc,
| We need to provide some bindings over rclcpp and rclcpp_lifecycle for this to work.

* Time / Duration
* State
* LifecycleNodeInterface / CallbackReturn
* FloatRef_ / FloatRefProp_
* VectorString / VectorDouble (see StlBindings_)

.. _FloatRef:

FloatRef
^^^^^^^^

| FloatRef is an owning reference to a ``double`` that behaves as a float-like object in Python.
| In C++ it decays into a ``double`` or a ``double*`` for interfaces that require it.
| It's purpose is to be used with StateInterface/CommandInterface.

Warning:
	| Although you can use assignment operators like +=,
	| you **cannot** assign to a FloatRef with =.
	| To do that see FloatRefProp_ of use @ / @= / set_value.

Usage:

.. code:: python

	from ros2_control_py.hardware_interface import CommandInterface, HW_IF_VELOCITY
	from ros2_control_py.rclcpp import FloatRef
	fr = FloatRef(5)
	assert fr == 5
	fr += 3
	assert fr == 8
	fr.set_value(5 / 2)
	assert fr == 2.5
	fr @ 8
	assert fr == 8
	fr @= 7
	assert fr == 7
	ci = CommandInterface("name", HW_IF_VELOCITY, fr)
	assert ci.get_value() == 7
	fr @= 4
	assert ci.get_value() == 4
	ci.set_value(5)
	assert fr == 5

.. _FloatRefProp:

FloatRefProp
^^^^^^^^^^^^

FloatRefProp is a ``property`` like descriptor that handles instance wide FloatRef so that you can assign to them like normal floats.

Usage:

.. code:: python

	from ros2_control_py.hardware_interface import CommandInterface, HW_IF_VELOCITY
	from ros2_control_py.rclcpp import FloatRefProp
	class Dummy:
		fr = FloatRefProp(5)
		def __init__(self):
			self.ci = CommandInterface("name", HW_IF_VELOCITY, self.fr)

	d = Dummy()
	assert d.fr == 5
	assert d.ci.get_value() == 5
	d.fr += 3
	assert d.fr == 8
	d.fr = 5 / 2
	assert d.fr == 2.5
	d.fr = 4
	assert d.ci.get_value() == 4
	d.ci.set_value(5)
	assert d.fr == 5

.. _StlBindings:

StlBindings
^^^^^^^^^^^

| When using stl containers (``std::vector``, ``std::map``, ``std::set``, etc...) in the python interface,
| we need to use a specialized binding for changes to go both ways.
| This is only needed for some cases, mainly containers of string/double,
| In other cases use a simple list but beware: it will be copied/moved out when passes to a C++ interface
| (For these types you cannot have a reference to the container but merely a copy).
| All these bindings are located in the rclcpp module in PascaleCase.
| (ex: ``std::vector<std::string>`` => ``VectorString``).
