from ..fuction import *


class Int64:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int64(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_int64(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0x34add168574510e6e17f5d23ecc077ef
