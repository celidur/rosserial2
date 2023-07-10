import sys
import importlib
import rosserial2 as ros2


def load_message(package, message):
    if package == 'std_msgs' and message == "Time":
        package = "builtin_interfaces"
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)


def import_module(module_name, module_dir=None):
    """
        desc
    """
    if module_dir:
        sys.path.insert(0, module_dir)
    try:
        module = importlib.import_module(module_name)
    except:
        raise "File not found"
    finally:
        if module_dir:
            sys.path.pop(0)
        return module


def load_pkg_module(package, directory):
    # check if its in the python path
    path = sys.path
    try:
        import_module(package)
    except ImportError:
        pass
    try:
        m = __import__(package + '.' + directory)
    except ImportError:
        ros2._logger.error("Cannot import package : %s" % package)
        ros2._logger.error("sys.path was " + str(path))
        return None
    return m

