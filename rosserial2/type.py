import sys
import importlib
import rosserial2.std_msgs as std_msgs
import rosserial2 as ros2


def load_message(package, message):
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


def getRosType(package, typeName):
    try:
        return eval(package + "." + typeName + "()")
    except AttributeError:
        ros2._logger.error('The module "%s" was not found' % (package + "." + typeName))
        return None


class rosType:
    def __init__(self, package, typeName):
        self.message_serialized = getRosType(package, typeName)
        if self.message_serialized is None:
            ros2._logger.error("Message not found")
            raise
        self.message_type = load_message(package, typeName)
        self._type = package + "/" + typeName

    def serialize(self, message):
        return self.message_serialized.serialize(message)

    def deserialize(self, data):
        self.message_serialized.deserialize(data)
        message_type = self.message_type()
        return self.set_message(message_type, self.message_serialized.__dict__())

    def set_message(self, message_type, message, name=None):
        for i in message:
            if isinstance(message[i], dict):
                if name is None:
                    name = i
                else:
                    name = name + "." + i
                self.set_message(message_type, message[i], name)
            else:
                if name is None:
                    message_type.__setattr__(i, message[i])
                else:
                    setattr(eval("message_type." + name), i, message[i])
                    ros2._logger.info("2 : message_type." + name + "." + i + "=" + str(message[i]))
        return message_type

    def type(self):
        return self._type

    def __hash__(self):
        return hash(self.message_serialized)
