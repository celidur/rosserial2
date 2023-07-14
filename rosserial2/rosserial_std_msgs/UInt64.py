from ..fuction import *


class UInt64:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint64(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_uint64(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x1b2a79973e8bf53d7b53acb71299cb57
