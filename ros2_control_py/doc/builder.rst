
#######
Builder
#######

Internal documentation for the executable that parses C++ and outputs pybind11 bindings.

Coding Conventions
=================

This project is written in C++20.

There is only one translation unit (.cpp file) which is main.cpp.

Each part of the program is in its own header.

Each header is split in a .hpp for the declarations and a .hxx for definitions.

Project Structure
=================

* ``utils/``: folder providing all utilities that are not in ``utils.hxx``.

  * ``hash.hpp``: boost's ``combine_hash`` function and ``std::hash`` for ``std::pair`` and ``std::tuple``.

  * ``ptr_iter.hpp``: ``ptr_iter`` function that takes a range of pointer-like objects and returns a range of the pointed objects by reference.

  * ``sep.hpp``: ``Sep`` struct to easily output ranges, supports separator object and element projection function.

* ``main.cpp``: Coordinates the program and handles the post-parsing step.

* ``parse.hpp``: Handles the pre-parsing and parsing steps.

* ``structs.hpp``: A collection of structs that store info on the parsed program.

* ``utils.hpp``: Provides ``ASSERT`` and ``ASSERT_DIR`` macros, string handling functions and all the content from the ``utils/`` folder.

* ``write.hpp``: Writes the pybind11 bindings from the gathered data.

Program Steps
=============

1. PreParsing_: Removes C++ features that are not handled by the next step.

2. Parsing_: CppParser parses the modified source code and we regroup the collected info in data-structures.

3. PostParsing_: This step compute the info necessary for the bindings like class hierarchy and adds rclcpp bindings.

4. Writing_: We then write the bindings from the gathered data.

.. _PreParsing:

PreParsing
----------

The ``remove_attributes`` function from ``utils.hpp`` is called to remove C++ features not supported by the parser.

This function removes:

* `attributes <https://en.cppreference.com/w/cpp/language/attributes>`_: aka ``[[...]]``
* digit separators: aka ``d'ddd'ddd`` with ``d`` for digits, the number is preserved by this operation.
* `raw string literals <https://en.cppreference.com/w/cpp/language/string_literal>`_: aka ``R"(...)"``, replaced by empty strings.
* template arguments: aka ``template <...>``, replaced with ``template <>``

The template arguments are removed because CppParser does not handle `non-type template parameters <https://en.cppreference.com/w/cpp/language/template_parameters>`_.

.. _Parsing:

Parsing
-------

| It is called `CppParser <https://github.com/Gepetto/cppparser>`_ and is a reworked fork of `salehjg's CppParser <https://github.com/salehjg/cppparser/tree/reform-boost-systemwide>`_,
| which itself is a fork of the original `satya-das' CppParser <https://github.com/satya-das/cppparser>`_.

| `salehjg <https://github.com/salehjg>`_ removed the internal copy of boost from the project.
| In our fork we fixed the CMakeLists and reworked the code
| so it would not produce warnings.

.. _PostParsing:

PostParsing
-----------

This part takes data-structures from last step and modifies them in the following ways:

1. Find stl containers that should be bound.

2. Load rclcpp bindings.

3. Find all mother classes and find out which header needs other headers.

4. Inherit virtual members, etc...

5. Find out which function / member function are overloaded.

6. Compute the order in which headers should be bound.

.. _Writing:

Writing
-------

TODO.
