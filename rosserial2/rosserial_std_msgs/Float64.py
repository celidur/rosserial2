from ..fuction import *


class Float64:
    def __init__(self):
        self.data: float = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_float64(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_float64(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value.data

    def __hash__(self):
        return 0xfdb28210bfa9d7c91146260178d9a584
