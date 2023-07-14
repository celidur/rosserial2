from ..fuction import *


class Log:
    ROSDEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4

    def __init__(self):
        self.level: int = 0
        self.msg: str = ""

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint8(self.level) + serialization_string(self.msg)

    def deserialize(self, data, offset=0):
        offset, self.level = deserialization_uint8(data, offset)
        offset, self.msg = deserialization_string(data, offset)
        return offset

    def __dict__(self):
        return {'level': self.level,
                'msg': self.msg}

    def set(self, value):
        self.level = value.level
        self.msg = value.msg

    def __hash__(self):
        return 0x11abd731c25933261cd6183bd12d6295
