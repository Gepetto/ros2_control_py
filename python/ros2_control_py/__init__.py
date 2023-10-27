import pkgutil
import importlib

loader = None
module_name = None
is_pkg = None

__all__ = []
for loader, module_name, is_pkg in pkgutil.walk_packages(__path__):
    __all__.append(module_name)
    globals()[module_name] = importlib.import_module(f"ros2_control_py.{module_name}")

del pkgutil
del importlib
del loader
del module_name
del is_pkg
