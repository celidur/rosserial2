from ..fuction import *


class UInt32:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_uint32(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_uint32(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x304a39449588c7f8ce2df6e8001c5fce
