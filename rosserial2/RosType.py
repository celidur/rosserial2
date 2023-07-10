import rosserial2 as ros2
from .LoadMsg import load_message
import rosserial2.rosserial_sensor_msgs as sensor_msgs
import rosserial2.rosserial_std_msgs as std_msgs

def getRosType(package, typeName):
    try:
        return eval(package + "." + typeName + "()")
    except AttributeError:
        ros2._logger.error('The module "%s" was not found' % (package + "." + typeName))
        return None

class RosType:
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
                    name_ = i
                else:
                    name_ = name + "." + i
                self.set_message(message_type, message[i], name_)
            else:
                if name is None:
                    message_type.__setattr__(i, message[i])
                else:
                    setattr(eval("message_type." + name), i, message[i])
        return message_type

    def type(self):
        return self._type

    def __hash__(self):
        return hash(self.message_serialized)
