from ..fuction import *


class Float32:
    def __init__(self):
        self.data: float = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_float32(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_float32(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x73fcbf46b49191e672908e50842a83d4
