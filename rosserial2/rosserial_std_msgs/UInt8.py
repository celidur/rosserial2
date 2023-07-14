from ..fuction import *


class UInt8:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint8(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_uint8(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x7c8164229e7d2c17eb95e9231617fdee
