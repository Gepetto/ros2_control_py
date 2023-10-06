import pkgutil

__all__ = []
for loader, module_name, is_pkg in pkgutil.walk_packages(__path__):
    __all__.append(module_name)
    globals()[module_name] = loader.find_module(module_name).load_module(module_name)

del pkgutil
del loader
del module_name
del is_pkg
