
###############
ros2_control_py
###############

Parsed Modules
==============

* hardware_interface
* controller_interface
* ros2_control_test_assets

Tested Features
===============

hardware_interface
------------------

* parse_control_resources_from_urdf
* StateInterface / CommandInterface (see FloatRef_ and FloatRefProp_)
* Actuator / Sensor / System
* ActuatorInterface / SensorInterface / SystemInterface
* FloatRef_

New Interface
=============

hardware_interface
------------------

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

	from ros2_control_py.hardware_interface import FloatRef, CommandInterface, HW_IF_VELOCITY
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

	from ros2_control_py.hardware_interface import FloatRefProp, CommandInterface, HW_IF_VELOCITY
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
