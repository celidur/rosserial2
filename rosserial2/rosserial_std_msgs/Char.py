from ..fuction import *


class Char:
    def __init__(self):
        self.data: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int8(self.data)

    def deserialize(self, data, offset=0):
        offset, self.data = deserialization_int8(data, offset)
        return offset

    def __dict__(self):
        return {'data': self.data}

    def set(self, value):
        self.data = value

    def __hash__(self):
        return 0x1bf77f25acecdedba0e224b162199717
