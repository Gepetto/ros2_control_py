from ros2_control_py.hardware_interface import FloatRef
from math import nan, inf, isnan


def cpp_div(a, b):
    if b != 0:
        return a / b
    if a == 0:
        return nan
    if a < 0:
        return -inf
    if a > 0:
        return inf
    return nan


def assert_same_float(a, b):
    assert a == b or (isnan(a) and isnan(b))


def assert_cmp(a, b, ta, tb):
    fa, fb = ta(a), tb(b)
    assert (a == b) == (fa == fb)
    assert (a != b) == (fa != fb)
    assert (a >= b) == (fa >= fb)
    assert (a > b) == (fa > fb)
    assert (a <= b) == (fa <= fb)
    assert (a < b) == (fa < fb)


def assert_arith(a, b, ta, tb):
    fa, fb = ta(a), tb(b)
    r = fa + fb
    assert type(r) is float
    assert (a + b) == r
    r = fa - fb
    assert type(r) is float
    assert (a - b) == r
    r = fa * fb
    assert type(r) is float
    assert (a * b) == r
    r = fa / fb
    assert type(r) is float
    assert_same_float(cpp_div(a, b), r)


def assert_self_arith(a, b, tb):
    fb = tb(b)
    r = FloatRef(a)
    r += fb
    assert type(r) is FloatRef
    assert (a + b) == r.get_value()
    r = FloatRef(a)
    r -= fb
    assert type(r) is FloatRef
    assert (a - b) == r.get_value()
    r = FloatRef(a)
    r *= fb
    assert type(r) is FloatRef
    assert (a * b) == r.get_value()
    r = FloatRef(a)
    r /= fb
    assert type(r) is FloatRef
    assert_same_float(cpp_div(a, b), r.get_value())


def assert_cmpi(a, b, ta, tb):
    assert_cmp(int(a), int(b), ta, tb)


def assert_arithi(a, b, ta, tb):
    assert_arith(int(a), int(b), ta, tb)


def assert_self_arithi(a, b, tb):
    assert_self_arith(int(a), int(b), tb)


def assert_cmps(a, b):
    assert_cmp(a, b, FloatRef, FloatRef)
    assert_cmp(a, b, FloatRef, float)
    assert_cmp(a, b, float, FloatRef)
    assert_cmpi(a, b, FloatRef, int)
    assert_cmpi(a, b, int, FloatRef)


def assert_ariths(a, b):
    assert_arith(a, b, FloatRef, FloatRef)
    assert_arith(a, b, FloatRef, float)
    assert_arith(a, b, float, FloatRef)
    assert_arithi(a, b, FloatRef, int)
    assert_arithi(a, b, int, FloatRef)


def assert_self_ariths(a, b):
    assert_self_arith(a, b, FloatRef)
    assert_self_arith(a, b, float)
    assert_self_arithi(a, b, int)


def test_cmp():
    for i in range(-10, 11):
        for j in range(-10, 11):
            assert_cmps(i * 1.5, j / 2)


def test_arith():
    for i in range(-10, 11):
        for j in range(-10, 11):
            assert_ariths(i * 1.5, j / 2)


def test_self_arith():
    for i in range(-10, 11):
        for j in range(-10, 11):
            assert_self_ariths(i * 1.5, j / 2)


def test_isnan():
    assert isnan(FloatRef(nan))
