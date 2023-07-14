from ..fuction import *


class UInt16:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint16(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_uint16(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x1df79edf208b629fe6b81923a544552d
